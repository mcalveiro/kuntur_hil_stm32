"""
plant_driver.py -- Proceso "planta" del fork HIL. Reemplaza a Simulink +
kuntur_ic.cpp como contraparte de kse_server, hablando el mismo
protocolo (ver protocol.py) y delegando la física a un PlantAdapter
(plane_adapter.py).

Este proceso NO se lanza desde kuntur_bridge/main.py -- así como hoy
Simulink corre aparte y KSE simplemente escucha su UDP, este proceso
corre aparte de Kuntur-Bridge. main.py no necesita saber que existe.

Paceo en tiempo real: nada en el protocolo Kuntur-IC pacea el lado
planta -- es la planta la que manda el ritmo (ver docstring de
protocol.py). Este driver dweite a tiempo real con time.sleep()
compensando el tiempo de cómputo, igual que un solver de paso fijo.

Fork HIL: sin canal KUNTUR_CTL (CtlLink) -- el retasking/reemplazo de
ruta se retiró del backend (ver plan-hil-stm32-portado-autopiloto.md
§9/§14). RealFalcon6Adapter.retask()/replace_route() quedan intactos
(no se llaman desde este driver, pero no se borran -- si el canal CTL
vuelve post-tesis, la lógica de aplicación ya está lista).

En cambio sí hay un canal nuevo, ModeLink (KUNTUR_MODE): gcs_api_worker.py
(kuntur_bridge, proceso aparte) le avisa a este proceso cuándo el
operador activó Manual -- ver cadac_plant/protocol.py::ModeLink y
plane_adapter.py::RealFalcon6Adapter.set_manual_mode()/_apply_manual_bypass().

Y, si --stm32-port está presente, el puente HIL hacia el STM32F767 (ver
kuntur_bridge/stm32_bridge.py) -- cada HIL_DECIMATION pasos (100 Hz,
decimación x10 del paso físico de 1 kHz, ver §5 del plan) se empaqueta
KUNTUR_PLANT_STATE y se manda por USB CDC; la respuesta
(KUNTUR_SURFACE_CMD) o su ausencia (timeout/sin hardware conectado) se
resuelve en RealFalcon6Adapter._apply_hil_bypass() -- este driver solo
reenvía, no decide fallback.
"""
from __future__ import annotations

import argparse
import logging
import time

from .plane_adapter import ActuatorCommand, MockFalcon6Adapter, PlantAdapter, RealFalcon6Adapter
from .protocol import AuxTelemetryLink, ModeLink, PlantLink
from kuntur_bridge.config import KUNTUR_AUX_PORT, KUNTUR_MODE_PORT
from kuntur_bridge.stm32_bridge import Stm32Bridge

log = logging.getLogger("cadac_plant.driver")

# Canal ICD auxiliar de instrumentación (ver icd_schema/kuntur_ic_aux_v1.json
# y protocol.AuxTelemetryLink): decimación de entero exacto 1000/25 = 40,
# fija -- asume int_step=0.001s (1 kHz, el nativo de RealFalcon6Adapter con
# f16c11_7.asc). Con otro int_step la tasa efectiva del canal auxiliar deja
# de ser 25 Hz (ver el warning en run() más abajo) -- no se recalcula
# dinámicamente porque el valor 40 es una decisión de diseño explícita
# (evita jitter de fase), no un cociente genérico.

# Decimación del canal HIL (ver icd_schema/kuntur_ic_plant_state_v1.json)
# -- 1000/100 = 10, misma asunción de int_step=0.001s que AUX_DECIMATION.
HIL_DECIMATION = 10
AUX_DECIMATION = 40


def run(
    adapter: PlantAdapter,
    int_step_s: float | None = None,
    kse_ip: str = "127.0.0.1",
    telemetry_port: int = 6000,
    command_port: int = 6001,
    aux_bridge_ip: str | None = None,
    aux_port: int | None = None,
    mode_bind_ip: str = "0.0.0.0",
    mode_port: int = KUNTUR_MODE_PORT,
    stm32_port: str | None = None,
    stm32_baud: int = 115200,
    autopilot_rate_hz: float | None = None,
    north0: float = 0.0,
    east0: float = 0.0,
    down0: float = -600.0,
    psi0: float = 0.0,
    ias0: float = 60.0,
    max_steps: int | None = None,
    realtime: bool = True,
) -> None:
    """int_step_s=None (default) usa el paso NATIVO del adaptador
    (adapter.int_step_s, si lo expone -- RealFalcon6Adapter sí, con el
    int_step real del *.asc de origen, p.ej. 0.001s/1000Hz para
    f16c11_7.asc; MockFalcon6Adapter no lo expone y cae a 0.01s/100Hz).
    Pasar un valor explícito solo tiene sentido para MockFalcon6Adapter
    -- RealFalcon6Adapter RECHAZA en step() cualquier dt distinto de su
    propio int_step_s (ver plane_adapter.py), así que forzar acá un valor
    que no coincida termina en excepción al primer paso, no en una
    integración silenciosamente incorrecta."""
    if int_step_s is None:
        int_step_s = getattr(adapter, "int_step_s", 0.01)

    if abs(int_step_s - 0.001) > 1e-9:
        log.warning(
            "canal ICD auxiliar decimado con factor fijo %d asumiendo int_step=0.001s "
            "(1kHz) -- con int_step=%.6fs la tasa efectiva del canal auxiliar es %.2f Hz, "
            "no 25 Hz (ver icd_schema/kuntur_ic_aux_v1.json, campo 'transport.decimation')",
            AUX_DECIMATION, int_step_s, 1.0 / (int_step_s * AUX_DECIMATION),
        )

    link = PlantLink(kse_ip=kse_ip, telemetry_port=telemetry_port, command_port=command_port)
    aux_link = AuxTelemetryLink(
        bridge_ip=aux_bridge_ip if aux_bridge_ip is not None else "127.0.0.1",
        aux_port=aux_port if aux_port is not None else KUNTUR_AUX_PORT,
    )
    mode_link = ModeLink(bind_ip=mode_bind_ip, mode_port=mode_port)

    stm32_bridge: Stm32Bridge | None = None
    if stm32_port is not None:
        if autopilot_rate_hz is not None:
            raise ValueError(
                "plant_driver.run(): --stm32-port y --autopilot-rate-hz son "
                "mutuamente excluyentes -- el HIL real ya decima el autopiloto "
                "por su cuenta (HIL_DECIMATION, vía stm32_bridge.py); "
                "--autopilot-rate-hz es para el comparativo Fase 2 en SIL puro "
                "(sin STM32, ver set_autopilot_decimation() en plane_adapter.py)."
            )
        stm32_bridge = Stm32Bridge(port=stm32_port, baudrate=stm32_baud)
        adapter.set_hil_mode(True)

    if autopilot_rate_hz is not None:
        decimation = round(1.0 / (autopilot_rate_hz * int_step_s))
        if decimation < 1:
            raise ValueError(
                f"plant_driver.run(): --autopilot-rate-hz={autopilot_rate_hz} "
                f"con int_step={int_step_s}s da una decimación < 1 (el autopiloto "
                f"no puede correr MÁS rápido que la física)."
            )
        actual_rate_hz = 1.0 / (decimation * int_step_s)
        if abs(actual_rate_hz - autopilot_rate_hz) > 1e-6:
            log.warning(
                "plant_driver: --autopilot-rate-hz=%.3f no es un divisor entero de "
                "1/int_step=%.1fHz -- decimación redondeada a n=%d, tasa EFECTIVA "
                "%.3fHz (no la pedida)",
                autopilot_rate_hz, 1.0 / int_step_s, decimation, actual_rate_hz,
            )
        adapter.set_autopilot_decimation(decimation)

    adapter.reset(north0, east0, down0, psi0, ias0)

    actuator = ActuatorCommand(elevator=0.0, aileron=0.0, rudder=0.0, throttle=0.5)
    ground_elev_m = 0.0
    seq_num = 0
    t_sim_us = 0
    stale_streak = 0
    manual_active = False

    log.info(
        "plant_driver iniciado -- kse=%s tx(states)->:%d rx(cmd)<-:%d int_step=%.4fs "
        "-- canal aux->%s:%d (decimación x%d) -- canal modo (TCP) escuchando en :%d"
        "%s",
        kse_ip, telemetry_port, command_port, int_step_s,
        aux_link.bridge_ip, aux_link.aux_port, AUX_DECIMATION, mode_port,
        (f" -- STM32 en {stm32_port}@{stm32_baud} (HIL activo, decimación x{HIL_DECIMATION})"
         if stm32_port is not None else "")
        + (f" -- autopiloto decimado a {autopilot_rate_hz}Hz (Fase 2, SIL puro)"
           if autopilot_rate_hz is not None else ""),
    )

    t_wall_start = time.monotonic()
    try:
        while max_steps is None or seq_num < max_steps:
            state = adapter.step(int_step_s, actuator)
            link.send_states(t_sim_us=t_sim_us, seq_num=seq_num, state=state)

            if seq_num % AUX_DECIMATION == 0:
                aux_link.send_aux(t_sim_us=t_sim_us, seq_num=seq_num, aux=adapter.aux_telemetry())

            # Canal KUNTUR_MODE -- no bloqueante, igual criterio que el
            # resto (consumido en el mismo hilo que step(), sin locks).
            mode_msg = mode_link.poll()
            if mode_msg is not None:
                manual_active = mode_msg.mode == 1
                adapter.set_manual_mode(manual_active)
                mode_link.send_mode_ack(seq=mode_msg.seq)

            # Canal HIL (KUNTUR_PLANT_STATE/KUNTUR_SURFACE_CMD) -- decimado
            # a 100 Hz, ver §5 del plan. Síncrono (espera la respuesta del
            # STM32 con timeout corto, ver Stm32Bridge) -- a propósito:
            # es la planta la que manda el ritmo acá también, igual que
            # con KSE/AUX/MODE; el timeout de Stm32Bridge (25 ms default)
            # ya deja margen bajo el presupuesto de 10 ms por paso x10.
            if stm32_bridge is not None and seq_num % HIL_DECIMATION == 0:
                aux = adapter.aux_telemetry()
                ai = adapter.autopilot_inputs()
                result = stm32_bridge.send_plant_state(
                    t_sim_us=t_sim_us, seq_num=seq_num,
                    flags=(1 if seq_num == 0 else 0) | 2,  # bit0=init, bit1=active
                    sn=state.north, se=state.east, sd=state.down,
                    vn=state.vn, ve=state.ve, vd=state.vd,
                    phi=state.phi, theta=state.theta, psi=state.psi,
                    p=state.p, q=state.q, r=state.r,
                    ax=state.ax, ay=state.ay, az=state.az,
                    pdynmc=aux.pdynmc, vmach=aux.vmach, dvbe=aux.dvbe, grav=ai.grav,
                    alpha=state.alpha, beta=state.beta,
                    dla=ai.dla, dlde=ai.dlde, dma=ai.dma, dmq=ai.dmq, dmde=ai.dmde,
                    dyb=ai.dyb, dydr=ai.dydr, dnb=ai.dnb, dnr=ai.dnr, dndr=ai.dndr,
                    dllp=ai.dllp, dllda=ai.dllda,
                    gmax=ai.gmax, gminx=ai.gminx,
                    swel1=ai.swel1, swel2=ai.swel2, swel3=ai.swel3,
                    psiflx=ai.psiflx_deg, thtflx=ai.thtflx_deg, altcom=ai.altcom,
                    mguid=ai.mguid, maut=ai.maut,
                    manual_active=1 if manual_active else 0,
                )
                if result is not None:
                    adapter.apply_stm32_surface_cmd(
                        delax_rad=result.delax_rad, delex_rad=result.delex_rad,
                        delrx_rad=result.delrx_rad, throttle=result.throttle,
                    )

            cmd = link.poll_command(current_t_sim_us=t_sim_us)
            if cmd is not None:
                actuator = ActuatorCommand(
                    elevator=cmd.elevator, aileron=cmd.aileron,
                    rudder=cmd.rudder, throttle=cmd.throttle,
                )
                ground_elev_m = cmd.ground_elev_m  # noqa: F841 -- ver TODO #6 en plane_adapter.py
                stale_streak = 0
            else:
                stale_streak += 1
                if stale_streak % 200 == 0:
                    log.warning("sin KUNTUR_COMMAND fresco hace %d pasos consecutivos", stale_streak)

            seq_num += 1
            t_sim_us += int(int_step_s * 1_000_000)

            if realtime:
                target_wall = t_wall_start + seq_num * int_step_s
                sleep_s = target_wall - time.monotonic()
                if sleep_s > 0:
                    time.sleep(sleep_s)
    finally:
        link.close()
        aux_link.close()
        mode_link.close()
        if stm32_bridge is not None:
            stm32_bridge.close()
        log.info("plant_driver detenido en seq_num=%d (t_sim=%.3fs)", seq_num, t_sim_us / 1e6)


def _cli() -> None:
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(name)s: %(message)s")
    parser = argparse.ArgumentParser(description="cadac_plant -- planta del fork HIL.")
    parser.add_argument("--kse-ip", default="127.0.0.1")
    parser.add_argument("--telemetry-port", type=int, default=6000)
    parser.add_argument("--command-port", type=int, default=6001)
    parser.add_argument(
        "--aux-ip", default=None,
        help="IP de kuntur_bridge para el canal ICD auxiliar de instrumentación "
             "(KUNTUR_AUX, ver icd_schema/kuntur_ic_aux_v1.json). Default: 127.0.0.1.",
    )
    parser.add_argument(
        "--aux-port", type=int, default=None,
        help=f"Puerto UDP del canal ICD auxiliar. Default: {KUNTUR_AUX_PORT} "
             "(kuntur_bridge.config.KUNTUR_AUX_PORT).",
    )
    parser.add_argument(
        "--mode-port", type=int, default=KUNTUR_MODE_PORT,
        help=f"Puerto TCP del canal de modo KUNTUR_MODE (Automático/Manual, "
             f"ver ModeLink). Default: {KUNTUR_MODE_PORT}.",
    )
    parser.add_argument(
        "--stm32-port", default=None,
        help="Puerto serie USB CDC del STM32 (p.ej. /dev/ttyACM0). Si se omite "
             "(default), el HIL queda deshabilitado -- el autopiloto nativo "
             "Python gobierna siempre (comportamiento SIL puro, bit-exacto).",
    )
    parser.add_argument(
        "--stm32-baud", type=int, default=115200,
        help="Baudrate del puerto serie. USB CDC (virtual COM) lo ignora en la "
             "práctica, pero pyserial igual lo pide. Default: 115200.",
    )
    parser.add_argument(
        "--autopilot-rate-hz", type=float, default=None,
        help="Fase 2 del plan (SIL puro, sin STM32): decima 'guidance'/"
             "'control'/'actuator' a esta tasa (p.ej. 100 para 100Hz) en vez "
             "de correrlos cada paso físico -- ver set_autopilot_decimation() "
             "en plane_adapter.py. Mutuamente excluyente con --stm32-port "
             "(el HIL real decima por su cuenta). Default: None (nativo, sin "
             "decimar).",
    )
    parser.add_argument(
        "--int-step", type=float, default=None,
        help="Paso de integración (s). Default: nativo del adaptador (ver run()). "
             "Con --adapter real, no tiene efecto útil salvo que coincida con el "
             "int_step del *.asc -- se rechaza si no coincide.",
    )
    parser.add_argument("--max-steps", type=int, default=None)
    parser.add_argument("--no-realtime", action="store_true")
    parser.add_argument("--adapter", choices=["mock", "real"], default="mock")
    parser.add_argument(
        "--asc", default=None,
        help="Solo con --adapter real: ruta a un *.asc de FALCON6 (default: "
             "cadac_py/CADAC/FALCON6/f16c11_7.asc, vendorizado en el fork).",
    )
    parser.add_argument(
        "--waypoints", default=None,
        help="Solo con --adapter real: archivo de waypoints (lat_deg,lon_deg,alt_msl_m) "
             "para sobrescribir swel1/swel2/swel3 del *.asc -- ver waypoints_file.py. "
             "Si se omite, se usan los waypoints originales del *.asc.",
    )
    parser.add_argument("--origin-lat", type=float, default=-31.31)
    parser.add_argument("--origin-lon", type=float, default=-64.21)
    parser.add_argument("--origin-alt", type=float, default=589.0)
    parser.add_argument("--north0", type=float, default=None)
    parser.add_argument("--east0", type=float, default=None)
    parser.add_argument("--down0", type=float, default=None)
    parser.add_argument("--psi0-deg", type=float, default=None)
    parser.add_argument("--ias0", type=float, default=None)
    args = parser.parse_args()

    if args.adapter == "mock":
        adapter: PlantAdapter = MockFalcon6Adapter()
        # Condición inicial de crucero razonable para el mock (60 m/s).
        ic_defaults = dict(north0=0.0, east0=0.0, down0=-600.0, psi0=0.0, ias0=60.0)
    else:
        import os

        from kuntur_bridge.geodesy import SACOLocalFrame
        from kuntur_bridge.waypoints_file import load_waypoints_list

        asc_path = args.asc or os.path.join(
            os.path.dirname(__file__), "..", "cadac_py", "CADAC", "FALCON6", "f16c11_7.asc",
        )
        frame = SACOLocalFrame(args.origin_lat, args.origin_lon, args.origin_alt)
        waypoints = load_waypoints_list(args.waypoints, frame) if args.waypoints else None
        adapter = RealFalcon6Adapter(asc_path, waypoints=waypoints)
        adapter.set_frame(frame)
        # Condición inicial DEL *.asc de origen (f16c11_7.asc: sbel3=-1000,
        # dvbe=180, psiblx=0) -- NO la del mock (60 m/s está muy por debajo
        # del régimen de vuelo con el que FALCON6 fue trimado/validado).
        ic_defaults = dict(north0=0.0, east0=0.0, down0=-1000.0, psi0=0.0, ias0=180.0)

    ic = dict(ic_defaults)
    if args.north0 is not None:
        ic["north0"] = args.north0
    if args.east0 is not None:
        ic["east0"] = args.east0
    if args.down0 is not None:
        ic["down0"] = args.down0
    if args.psi0_deg is not None:
        import math as _math
        ic["psi0"] = _math.radians(args.psi0_deg)
    if args.ias0 is not None:
        ic["ias0"] = args.ias0

    run(
        adapter,
        int_step_s=args.int_step,
        kse_ip=args.kse_ip,
        telemetry_port=args.telemetry_port,
        command_port=args.command_port,
        aux_bridge_ip=args.aux_ip,
        aux_port=args.aux_port,
        mode_port=args.mode_port,
        stm32_port=args.stm32_port,
        stm32_baud=args.stm32_baud,
        autopilot_rate_hz=args.autopilot_rate_hz,
        max_steps=args.max_steps,
        realtime=not args.no_realtime,
        **ic,
    )


if __name__ == "__main__":
    _cli()
