"""
config_file.py -- Configuración de Kuntur-SIM vía archivos .ini.

Fork HIL (kuntur_cadac_hil): sin secciones [px4]/[fcs], sin 'mission' --
ese backend/componente no corre en este fork (ver §9 del plan HIL:
Pixhawk no participa, Kuntur-Mission nunca se usó). Componentes válidos:
'executive' (obligatorio), 'flightgear', 'telemetry_stream', 'gcs_api'
(mode switch Automático/Manual + estado del STM32) y, cuando se agregue
en fases posteriores, 'manual_joystick' (FS-i6).
"""
from __future__ import annotations

import configparser
from dataclasses import dataclass, field
from pathlib import Path

from .config import (
    SIMULINK_IP, SIMULINK_TELEMETRY_PORT, SIMULINK_COMMAND_PORT,
    FG_IP, FG_UDP_PORT, FG_TELNET_PORT, FG_FDM_RATE_HZ, FG_TERRAIN_POLL_EVERY_N,
    KUNTUR_AUX_PORT, KUNTUR_MODE_PORT,
    FALLBACK_LAT, FALLBACK_LON, FALLBACK_ALT,
    FCS_RENDEZVOUS_TIMEOUT_S, FCS_STALE_WARNING_EVERY_N_CYCLES,
    ExecMode,
)

DEFAULT_TELEMETRY_RATE_HZ = 30.0
DEFAULT_SOCKET_PATH = "/tmp/kse.sock"
VALID_COMPONENTS = ("executive", "flightgear", "telemetry_stream", "gcs_api")


class ConfigError(Exception):
    pass


@dataclass
class NetworkSettings:
    simulink_ip: str = SIMULINK_IP
    simulink_telemetry_port: int = SIMULINK_TELEMETRY_PORT
    simulink_command_port: int = SIMULINK_COMMAND_PORT
    fg_ip: str = FG_IP
    fg_udp_port: int = FG_UDP_PORT
    fg_telnet_port: int = FG_TELNET_PORT
    # Puerto UDP donde kuntur_bridge escucha el canal ICD auxiliar de
    # instrumentación (KUNTUR_AUX, ver icd_schema/kuntur_ic_aux_v1.json) --
    # NO es un puerto de FlightGear, es propio de Kuntur-Bridge;
    # plant_driver.py le envía directo, sin pasar por kse_server.
    kuntur_aux_port: int = KUNTUR_AUX_PORT
    # Relay SSE de telemetría hacia Kuntur-GCS (componente 'telemetry_stream').
    telemetry_http_host: str = "0.0.0.0"
    telemetry_http_port: int = 8765
    # API REST hacia Kuntur-GCS (componente 'gcs_api', ver
    # gcs_api_worker.py): mode switch Automático/Manual + estado del
    # STM32. Fork HIL: sin ctl_ip/ctl_port -- el canal KUNTUR_CTL
    # (retasking) se retiró del backend (ver §9/§14 del plan).
    gcs_api_http_host: str = "0.0.0.0"
    gcs_api_http_port: int = 8766
    # Canal KUNTUR_MODE (ver config.KUNTUR_MODE_PORT / ModeLink en
    # cadac_plant/protocol.py): gcs_api_worker.py es CLIENTE TCP de
    # plant_driver.py (proceso aparte) para propagarle el modo vigente.
    plant_mode_ip: str = "127.0.0.1"
    plant_mode_port: int = KUNTUR_MODE_PORT


@dataclass
class RateSettings:
    fg_fdm_rate_hz: float = FG_FDM_RATE_HZ
    fg_terrain_poll_every_n: int = FG_TERRAIN_POLL_EVERY_N
    telemetry_rate_hz: float = DEFAULT_TELEMETRY_RATE_HZ
    fcs_rendezvous_timeout_s: float = FCS_RENDEZVOUS_TIMEOUT_S
    fcs_stale_warning_every_n_cycles: int = FCS_STALE_WARNING_EVERY_N_CYCLES


@dataclass
class KseSettings:
    exec_mode: ExecMode = ExecMode.ENGINE_DRIVEN
    socket_path: str = DEFAULT_SOCKET_PATH
    server_binary: str | None = None


@dataclass
class ComponentsSettings:
    """Default del fork HIL: 'executive' + 'flightgear' + 'telemetry_stream'
    + 'gcs_api' -- este último habilitado por default (a diferencia del
    fork CADAC-SIL original, donde dependía de plant_driver.py --adapter
    real corriendo aparte con el canal KUNTUR_CTL activo): acá gcs_api
    solo expone mode switch + estado, no requiere ningún canal nuevo del
    lado de la planta para arrancar."""
    enabled: list[str] = field(
        default_factory=lambda: ["executive", "flightgear", "telemetry_stream", "gcs_api"]
    )


@dataclass
class FdirSettings:
    auto_reset_after_stale_cycles: int = 0


@dataclass
class OriginFallbackSettings:
    lat: float = FALLBACK_LAT
    lon: float = FALLBACK_LON
    alt_m: float = FALLBACK_ALT


@dataclass
class ScenarioSettings:
    """Ruta del archivo de waypoints del escenario -- SOLO para que
    telemetry_stream_worker.py publique la ruta a Kuntur-GCS (/route,
    ver load_route()). Independiente de los waypoints que de verdad usa
    RealFalcon6Adapter (eso lo decide --waypoints de plant_driver.py,
    proceso aparte): este campo es puramente de visualización."""
    waypoints_file: str | None = "scenarios/waypoints_cordoba.txt"


@dataclass
class KunturSimSettings:
    network: NetworkSettings = field(default_factory=NetworkSettings)
    rates: RateSettings = field(default_factory=RateSettings)
    kse: KseSettings = field(default_factory=KseSettings)
    components: ComponentsSettings = field(default_factory=ComponentsSettings)
    fdir: FdirSettings = field(default_factory=FdirSettings)
    origin_fallback: OriginFallbackSettings = field(default_factory=OriginFallbackSettings)
    scenario: ScenarioSettings = field(default_factory=ScenarioSettings)


def _split_list(raw: str) -> list[str]:
    return [x.strip() for x in raw.split(",") if x.strip()]


def load_settings(path: str | Path) -> KunturSimSettings:
    path = Path(path)
    if not path.exists():
        raise ConfigError(f"No existe el archivo de configuración: {path}")

    parser = configparser.ConfigParser()
    parser.read(path)
    s = KunturSimSettings()

    if parser.has_section("network"):
        n = parser["network"]
        s.network.simulink_ip = n.get("simulink_ip", s.network.simulink_ip)
        s.network.simulink_telemetry_port = n.getint("simulink_telemetry_port", s.network.simulink_telemetry_port)
        s.network.simulink_command_port = n.getint("simulink_command_port", s.network.simulink_command_port)
        s.network.fg_ip = n.get("fg_ip", s.network.fg_ip)
        s.network.fg_udp_port = n.getint("fg_udp_port", s.network.fg_udp_port)
        s.network.fg_telnet_port = n.getint("fg_telnet_port", s.network.fg_telnet_port)
        s.network.kuntur_aux_port = n.getint("kuntur_aux_port", s.network.kuntur_aux_port)
        s.network.telemetry_http_host = n.get("telemetry_http_host", s.network.telemetry_http_host)
        s.network.telemetry_http_port = n.getint("telemetry_http_port", s.network.telemetry_http_port)
        s.network.gcs_api_http_host = n.get("gcs_api_http_host", s.network.gcs_api_http_host)
        s.network.gcs_api_http_port = n.getint("gcs_api_http_port", s.network.gcs_api_http_port)
        s.network.plant_mode_ip = n.get("plant_mode_ip", s.network.plant_mode_ip)
        s.network.plant_mode_port = n.getint("plant_mode_port", s.network.plant_mode_port)

    if parser.has_section("rates"):
        r = parser["rates"]
        s.rates.fg_fdm_rate_hz = r.getfloat("fg_fdm_rate_hz", s.rates.fg_fdm_rate_hz)
        s.rates.fg_terrain_poll_every_n = r.getint("fg_terrain_poll_every_n", s.rates.fg_terrain_poll_every_n)
        s.rates.telemetry_rate_hz = r.getfloat("telemetry_rate_hz", s.rates.telemetry_rate_hz)
        s.rates.fcs_rendezvous_timeout_s = r.getfloat("fcs_rendezvous_timeout_ms", s.rates.fcs_rendezvous_timeout_s * 1000) / 1000
        s.rates.fcs_stale_warning_every_n_cycles = r.getint("fcs_stale_warning_every_n_cycles", s.rates.fcs_stale_warning_every_n_cycles)
        if s.rates.fg_fdm_rate_hz <= 0 or s.rates.telemetry_rate_hz <= 0:
            raise ConfigError("Las frecuencias en [rates] deben ser > 0 Hz.")
        if s.rates.fcs_rendezvous_timeout_s <= 0:
            raise ConfigError("fcs_rendezvous_timeout_ms debe ser > 0.")

    if parser.has_section("kse"):
        k = parser["kse"]
        exec_mode_raw = k.get("exec_mode", "engine_driven").strip().lower()
        if exec_mode_raw in ("engine_driven", "0"):
            s.kse.exec_mode = ExecMode.ENGINE_DRIVEN
        elif exec_mode_raw in ("authority_driven", "1"):
            s.kse.exec_mode = ExecMode.AUTHORITY_DRIVEN
        else:
            raise ConfigError(f"[kse] exec_mode={exec_mode_raw!r} inválido.")
        s.kse.socket_path = k.get("socket_path", s.kse.socket_path)
        server_binary = k.get("server_binary", "").strip()
        s.kse.server_binary = server_binary or None

    if parser.has_section("components"):
        c = parser["components"]
        enabled = _split_list(c.get("enabled", ",".join(s.components.enabled)))
        for name in enabled:
            if name not in VALID_COMPONENTS:
                raise ConfigError(f"[components] componente desconocido: {name!r} -- opciones: {VALID_COMPONENTS}")
        if "executive" not in enabled:
            raise ConfigError("[components] 'executive' es obligatorio.")
        s.components.enabled = enabled

    if parser.has_section("fdir"):
        f = parser["fdir"]
        s.fdir.auto_reset_after_stale_cycles = f.getint("auto_reset_after_stale_cycles", s.fdir.auto_reset_after_stale_cycles)
        if s.fdir.auto_reset_after_stale_cycles < 0:
            raise ConfigError("[fdir] auto_reset_after_stale_cycles no puede ser negativo.")

    if parser.has_section("origin_fallback"):
        o = parser["origin_fallback"]
        s.origin_fallback.lat = o.getfloat("lat", s.origin_fallback.lat)
        s.origin_fallback.lon = o.getfloat("lon", s.origin_fallback.lon)
        s.origin_fallback.alt_m = o.getfloat("alt_m", s.origin_fallback.alt_m)

    if parser.has_section("scenario"):
        sc = parser["scenario"]
        s.scenario.waypoints_file = sc.get("waypoints_file", s.scenario.waypoints_file or "").strip() or None

    return s


DEFAULT_INI_TEMPLATE = f"""\
; kuntur_sim.ini -- Configuración de Kuntur-Bridge / KSE (fork HIL).

[network]
simulink_ip = {SIMULINK_IP}
simulink_telemetry_port = {SIMULINK_TELEMETRY_PORT}
simulink_command_port = {SIMULINK_COMMAND_PORT}
fg_ip = {FG_IP}
fg_udp_port = {FG_UDP_PORT}
fg_telnet_port = {FG_TELNET_PORT}
; Canal ICD auxiliar de instrumentacion (KUNTUR_AUX) -- NO es un puerto de
; FlightGear, kuntur_bridge lo escucha directo desde plant_driver.py, sin
; pasar por kse_server (ver icd_schema/kuntur_ic_aux_v1.json).
kuntur_aux_port = {KUNTUR_AUX_PORT}
; Relay SSE de telemetria hacia Kuntur-GCS (componente 'telemetry_stream').
telemetry_http_host = 0.0.0.0
telemetry_http_port = 8765
; API REST hacia Kuntur-GCS (componente 'gcs_api', ver gcs_api_worker.py):
; mode switch Automatico/Manual + estado del STM32.
gcs_api_http_host = 0.0.0.0
gcs_api_http_port = 8766
; Canal KUNTUR_MODE: gcs_api_worker.py (cliente TCP) -> plant_driver.py
; (servidor, ModeLink) -- propaga el modo vigente al proceso aparte que
; corre la planta real.
plant_mode_ip = 127.0.0.1
plant_mode_port = {KUNTUR_MODE_PORT}

[rates]
fg_fdm_rate_hz = {FG_FDM_RATE_HZ}
fg_terrain_poll_every_n = {FG_TERRAIN_POLL_EVERY_N}
telemetry_rate_hz = {DEFAULT_TELEMETRY_RATE_HZ}
fcs_rendezvous_timeout_ms = {FCS_RENDEZVOUS_TIMEOUT_S * 1000:.1f}
fcs_stale_warning_every_n_cycles = {FCS_STALE_WARNING_EVERY_N_CYCLES}

[kse]
exec_mode = engine_driven
socket_path = {DEFAULT_SOCKET_PATH}
server_binary =

[components]
; Fork HIL: 'executive' (obligatorio, habla con kse_server),
; 'flightgear' (visualizacion), 'telemetry_stream' (SSE hacia
; Kuntur-GCS), 'gcs_api' (mode switch Automatico/Manual + estado STM32).
enabled = executive, flightgear, telemetry_stream, gcs_api

[fdir]
auto_reset_after_stale_cycles = 0

[origin_fallback]
lat = {FALLBACK_LAT}
lon = {FALLBACK_LON}
alt_m = {FALLBACK_ALT}

[scenario]
; Solo para que telemetry_stream_worker publique la ruta a Kuntur-GCS
; (/route) -- no es lo que usa plant_driver.py --adapter real (ver
; --waypoints de ese proceso, aparte).
waypoints_file = scenarios/waypoints_cordoba.txt
"""


def write_default_ini(path: str | Path) -> None:
    Path(path).write_text(DEFAULT_INI_TEMPLATE)
