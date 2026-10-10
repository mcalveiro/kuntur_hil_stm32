"""
main.py -- Orquestación de Kuntur-Bridge.

Fork HIL (kuntur_cadac_hil): sin componentes 'px4', 'fcs' ni 'mission'
-- Pixhawk no participa de este proyecto, el FCS Python alternativo
(kuntur_fcs/falcon_fcs) no se usa (el autopiloto corre dentro de
RealFalcon6Adapter o, cuando esté integrado, en el STM32) y Kuntur-
Mission nunca se usó. KSE corre exclusivamente como proceso Rust
separado (kse_server/).
"""
from __future__ import annotations

import signal
import threading

from .config import ExecMode
from .config_file import KunturSimSettings, load_settings
from .geodesy import SACOLocalFrame
from .kse_client import KSEClient
from . import logging_setup
from .telnet_client import calibrate_origin

log = logging_setup.get_logger("main")

THREAD_JOIN_TIMEOUT_S = 3.0


def _kse_client_kwargs(settings: KunturSimSettings) -> dict:
    kwargs = dict(
        simulink_ip=settings.network.simulink_ip,
        telemetry_port=settings.network.simulink_telemetry_port,
        command_port=settings.network.simulink_command_port,
        exec_mode=settings.kse.exec_mode,
        rendezvous_timeout_s=settings.rates.fcs_rendezvous_timeout_s,
        stale_warning_every_n_cycles=settings.rates.fcs_stale_warning_every_n_cycles,
        socket_path=settings.kse.socket_path,
        # Fork HIL: sin 'fcs'/'px4' -- nadie va a confirmar actuadores
        # por CHANNEL ACTUATORS_READY, así que kse_server no debe
        # reportarlo como advertencia (ver kse_client.py /
        # kse_server/src/main.rs).
        expect_actuator_client=False,
    )
    if settings.kse.server_binary:
        kwargs["server_binary"] = settings.kse.server_binary
    return kwargs


def _auto_reset_supervisor(kse: KSEClient, threshold: int, stop_event: threading.Event) -> None:
    supervisor_log = logging_setup.get_logger("fdir_supervisor")
    already_reset = False
    while not stop_event.wait(timeout=0.5):
        try:
            stale = kse.control_staleness_cycles()
        except OSError:
            supervisor_log.warning("conexión con KSE perdida -- terminando el hilo")
            return
        if stale >= threshold and not already_reset:
            supervisor_log.warning(
                f"control_staleness_cycles={stale} >= umbral {threshold} -- llamando kse.reset()",
                extra={"stale_cycles": stale, "threshold": threshold},
            )
            kse.reset()
            already_reset = True
        elif stale == 0:
            already_reset = False


def run(
    exec_mode: ExecMode = ExecMode.ENGINE_DRIVEN,
    config_path: str | None = None,
    no_flightgear: bool = False,
    **overrides,
) -> None:
    session_id, log_path = logging_setup.configure()
    log.info("logging inicializado", extra={"session_id": session_id, "log_path": log_path})

    settings = load_settings(config_path) if config_path else KunturSimSettings()

    if exec_mode != ExecMode.ENGINE_DRIVEN:
        settings.kse.exec_mode = exec_mode
    if no_flightgear and "flightgear" in settings.components.enabled:
        settings.components.enabled.remove("flightgear")
    for key, value in overrides.items():
        for section in (settings.network, settings.rates, settings.kse, settings.fdir, settings.origin_fallback):
            if hasattr(section, key):
                setattr(section, key, value)
                break

    enabled = settings.components.enabled
    log.info(f"componentes habilitados (orden de arranque): {enabled}", extra={"enabled": enabled})
    log.info(
        f"KSE: proceso kse_server / exec_mode={settings.kse.exec_mode.name}",
        extra={"exec_mode": settings.kse.exec_mode.name, "socket_path": settings.kse.socket_path},
    )
    # Configuración efectiva ya resuelta (defaults < .ini < overrides) --
    # sin esto, con tres niveles de superposición, es fácil no saber qué
    # valor terminó realmente en uso.
    log.debug("configuración efectiva resuelta", extra={
        "config_path": config_path,
        "network": vars(settings.network),
        "rates": vars(settings.rates),
        "fdir": vars(settings.fdir),
    })

    if "flightgear" in enabled:
        local_frame, prop_tree = calibrate_origin(
            fg_ip=settings.network.fg_ip,
            fg_telnet_port=settings.network.fg_telnet_port,
            fallback_lat=settings.origin_fallback.lat,
            fallback_lon=settings.origin_fallback.lon,
            fallback_alt=settings.origin_fallback.alt_m,
        )
    else:
        # Sin FlightGear (headless): SACOLocalFrame igual se instancia
        # con el origen de fallback -- es la autoridad única de la
        # proyección WGS-84 para el resto de los componentes.
        local_frame = SACOLocalFrame(
            settings.origin_fallback.lat, settings.origin_fallback.lon, settings.origin_fallback.alt_m
        )
        prop_tree = None

    kse = KSEClient(**_kse_client_kwargs(settings))
    kse.start()

    shutdown_event = threading.Event()
    threads: list[threading.Thread] = []

    for component in enabled:
        if component == "executive":
            continue
        if component == "flightgear":
            from .flightgear_worker import flightgear_worker
            t = threading.Thread(
                target=flightgear_worker,
                args=(kse, local_frame, prop_tree),
                kwargs=dict(
                    fg_ip=settings.network.fg_ip,
                    fg_udp_port=settings.network.fg_udp_port,
                    fdm_rate_hz=settings.rates.fg_fdm_rate_hz,
                    terrain_poll_every_n=settings.rates.fg_terrain_poll_every_n,
                    stop_event=shutdown_event,
                ),
                daemon=True, name="FlightGearWorker",
            )
            t.start()
            threads.append(t)

            # Canal ICD auxiliar de instrumentación (KUNTUR_AUX, ver
            # icd_schema/kuntur_ic_aux_v1.json) -- hilo aparte, con SU
            # PROPIA conexión Telnet (ver telnet_client.connect_property_tree),
            # no la de flightgear_worker/prop_tree (FGPropertyTree no es
            # thread-safe). plant_driver.py le envía directo, sin pasar
            # por KSE -- este hilo no necesita a kse para nada.
            from .aux_instrument_worker import aux_instrument_worker
            t = threading.Thread(
                target=aux_instrument_worker,
                kwargs=dict(
                    fg_ip=settings.network.fg_ip,
                    fg_telnet_port=settings.network.fg_telnet_port,
                    aux_port=settings.network.kuntur_aux_port,
                    stop_event=shutdown_event,
                ),
                daemon=True, name="AuxInstrumentWorker",
            )
        elif component == "telemetry_stream":
            from .telemetry_stream_worker import telemetry_stream_worker
            t = threading.Thread(
                target=telemetry_stream_worker,
                args=(kse, local_frame),
                kwargs=dict(
                    host=settings.network.telemetry_http_host,
                    port=settings.network.telemetry_http_port,
                    rate_hz=settings.rates.telemetry_rate_hz,
                    waypoints_file=settings.scenario.waypoints_file,
                    stop_event=shutdown_event,
                ),
                daemon=True, name="TelemetryStream",
            )
        elif component == "gcs_api":
            from .gcs_api_worker import gcs_api_worker
            t = threading.Thread(
                target=gcs_api_worker,
                kwargs=dict(
                    host=settings.network.gcs_api_http_host,
                    port=settings.network.gcs_api_http_port,
                    plant_mode_ip=settings.network.plant_mode_ip,
                    plant_mode_port=settings.network.plant_mode_port,
                    stop_event=shutdown_event,
                ),
                daemon=True, name="GcsApiWorker",
            )
        else:
            raise AssertionError(f"componente sin manejador: {component!r}")
        t.start()
        threads.append(t)

    if settings.fdir.auto_reset_after_stale_cycles > 0:
        log.info(
            f"auto-reset de KSE habilitado: umbral = {settings.fdir.auto_reset_after_stale_cycles} ciclos consecutivos",
            extra={"auto_reset_threshold": settings.fdir.auto_reset_after_stale_cycles},
        )
        supervisor = threading.Thread(
            target=_auto_reset_supervisor,
            args=(kse, settings.fdir.auto_reset_after_stale_cycles, shutdown_event),
            daemon=True, name="FDIRSupervisor",
        )
        supervisor.start()
        threads.append(supervisor)

    def _handle_signal(signum, _frame) -> None:
        # Python re-entra al intérprete en un punto seguro entre
        # bytecodes para ejecutar el handler -- a diferencia del
        # manejador de señal en C de kse_server, acá SÍ es seguro llamar
        # código Python normal, incluido el logger.
        name = signal.Signals(signum).name
        log.warning(f"señal {name} recibida -- iniciando apagado ordenado", extra={"signal": name})
        shutdown_event.set()

    signal.signal(signal.SIGINT, _handle_signal)
    signal.signal(signal.SIGTERM, _handle_signal)

    shutdown_event.wait()

    log.info("esperando a que los hilos terminen limpiamente")
    for t in threads:
        t.join(timeout=THREAD_JOIN_TIMEOUT_S)
        if t.is_alive():
            log.warning(
                f"{t.name} no terminó dentro de {THREAD_JOIN_TIMEOUT_S}s",
                extra={"thread": t.name, "timeout_s": THREAD_JOIN_TIMEOUT_S},
            )

    log.info("deteniendo KSE")
    kse.stop()
    log.info("Kuntur-Bridge finalizado")
    logging_setup.shutdown()


def _cli() -> None:
    import argparse

    parser = argparse.ArgumentParser(description="Kuntur-Bridge -- orquestación de Kuntur-SIM (fork HIL).")
    parser.add_argument("--config", dest="config_path", default=None)
    parser.add_argument("--no-flightgear", action="store_true")
    args = parser.parse_args()
    run(config_path=args.config_path, no_flightgear=args.no_flightgear)


if __name__ == "__main__":
    _cli()
