"""
telemetry_stream_worker.py -- Relay de telemetría hacia Kuntur-GCS (SSE).

Un navegador no puede leer UDP ni el socket Unix de KSE, así que este
hilo hace de puente: muestrea KSEClient.snapshot() a `rate_hz`, lo
convierte a unidades de operador (lat/lon/alt geodésicos vía
SACOLocalFrame, ángulos en grados) y lo publica como Server-Sent Events
en http://<host>:<port>/telemetry. También expone /health (JSON).

Solo lectura: no escribe nada hacia KSE ni hacia la planta. Sin
dependencias fuera de la biblioteca estándar.
"""
from __future__ import annotations

import json
import math
import threading
import time
from pathlib import Path
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

from .geodesy import SACOLocalFrame
from .kse_client import KSEClient
from . import logging_setup

log = logging_setup.get_logger("telemetry_stream")

# Un cliente SSE sin datos nuevos recibe un comentario de keepalive cada
# tanto, para que proxies/navegador no cierren la conexión ociosa.
KEEPALIVE_S = 5.0


G0 = 9.80665

# Modo de vuelo que reporta el vehículo. La planta CADAC/FALCON6 es un
# piloto automático cerrado que recorre en bucle los waypoints del
# escenario: no tiene Despegue/Loiter/Aterrizaje/Manual, así que el único
# modo real es Patrullaje mientras lleguen estados.
VEHICLE_MODE = "PATRULLAJE"


def load_route(path: str | None) -> list[dict]:
    """Waypoints del escenario (mismo archivo que usa plant_driver):
    'lat_deg, lon_deg, alt_msl_m, radio_captura_m', '#' comenta."""
    if not path:
        return []
    route = []
    for line in Path(path).read_text().splitlines():
        line = line.split("#", 1)[0].strip()
        if not line:
            continue
        lat, lon, alt, radius = (float(x) for x in line.split(",")[:4])
        route.append({"lat": lat, "lon": lon, "alt_m": alt, "capture_m": radius})
    return route


class _TelemetryHub:
    """Último frame publicado + condición para despertar a los clientes SSE."""

    def __init__(self) -> None:
        self._cond = threading.Condition()
        self._frame: dict | None = None
        self._version = 0
        self.route: list[dict] = []

    def publish(self, frame: dict) -> None:
        with self._cond:
            self._frame = frame
            self._version += 1
            self._cond.notify_all()

    def latest(self) -> dict | None:
        with self._cond:
            return self._frame

    def wait_next(self, last_version: int, timeout: float) -> tuple[int, dict | None]:
        with self._cond:
            if self._version == last_version:
                self._cond.wait(timeout=timeout)
            if self._version == last_version:
                return last_version, None
            return self._version, self._frame


def _make_handler(hub: _TelemetryHub, stop_event: threading.Event):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, fmt, *args):  # silencia el access log de stdlib
            pass

        def _cors(self) -> None:
            self.send_header("Access-Control-Allow-Origin", "*")

        def do_OPTIONS(self):
            self.send_response(204)
            self._cors()
            self.end_headers()

        def do_GET(self):
            if self.path.startswith("/health"):
                frame = hub.latest()
                body = json.dumps({"ok": True, "has_data": frame is not None}).encode()
                self.send_response(200)
                self._cors()
                self.send_header("Content-Type", "application/json")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)
                return
            if self.path.startswith("/route"):
                body = json.dumps({"route": hub.route, "loop": True}).encode()
                self.send_response(200)
                self._cors()
                self.send_header("Content-Type", "application/json")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)
                return
            if not self.path.startswith("/telemetry"):
                self.send_error(404)
                return

            self.send_response(200)
            self._cors()
            self.send_header("Content-Type", "text/event-stream")
            self.send_header("Cache-Control", "no-cache")
            self.send_header("Connection", "keep-alive")
            self.end_headers()
            log.info("cliente GCS conectado", extra={"client": self.client_address[0]})
            version = 0
            last_write = time.monotonic()
            try:
                while not stop_event.is_set():
                    version, frame = hub.wait_next(version, timeout=1.0)
                    if frame is not None:
                        self.wfile.write(f"data: {json.dumps(frame)}\n\n".encode())
                        self.wfile.flush()
                        last_write = time.monotonic()
                    elif time.monotonic() - last_write >= KEEPALIVE_S:
                        self.wfile.write(b": keepalive\n\n")
                        self.wfile.flush()
                        last_write = time.monotonic()
            except (BrokenPipeError, ConnectionResetError):
                pass
            finally:
                log.info("cliente GCS desconectado", extra={"client": self.client_address[0]})

    return Handler


def _snapshot_to_frame(snap, local_frame: SACOLocalFrame) -> dict:
    lat_rad, lon_rad, alt_m = local_frame.ned_to_geodetic(snap.north, snap.east, snap.down)
    gs = math.hypot(snap.vn, snap.ve)
    # Razón de giro del rumbo (psi_dot) desde las tasas de cuerpo, no la
    # tasa de guiñada r a secas: es lo que muestra el turn coordinator.
    turn_rate = (snap.q * math.sin(snap.phi) + snap.r * math.cos(snap.phi)) / max(math.cos(snap.theta), 0.1)
    return {
        "mode": VEHICLE_MODE,
        "turn_rate_dps": math.degrees(turn_rate),
        "ay_g": snap.ay / G0,  # fuerza específica lateral: 0 = giro coordinado (bola centrada)
        "t_sim_s": snap.t_sim_us / 1e6,
        "seq_num": snap.seq_num,
        "lat_deg": math.degrees(lat_rad),
        "lon_deg": math.degrees(lon_rad),
        "alt_msl_m": alt_m,
        "roll_deg": math.degrees(snap.phi),
        "pitch_deg": math.degrees(snap.theta),
        "heading_deg": math.degrees(snap.psi) % 360.0,
        "ias_mps": snap.ias,
        "gs_mps": gs,
        "vs_mps": -snap.vd,  # vd es hacia abajo (NED); variometro positivo = ascenso
        "alpha_deg": math.degrees(snap.alpha),
        "beta_deg": math.degrees(snap.beta),
        "p_dps": math.degrees(snap.p),
        "q_dps": math.degrees(snap.q),
        "r_dps": math.degrees(snap.r),
        "status": snap.status,
    }


def telemetry_stream_worker(
    runtime: KSEClient,
    local_frame: SACOLocalFrame,
    host: str = "0.0.0.0",
    port: int = 8765,
    rate_hz: float = 10.0,
    waypoints_file: str | None = None,
    stop_event: threading.Event | None = None,
) -> None:
    if stop_event is None:
        stop_event = threading.Event()

    hub = _TelemetryHub()
    try:
        hub.route = load_route(waypoints_file)
    except (OSError, ValueError) as e:
        log.warning(f"no se pudo leer la ruta {waypoints_file!r}: {e}", extra={"error": str(e)})
    server = ThreadingHTTPServer((host, port), _make_handler(hub, stop_event))
    server.daemon_threads = True
    server_thread = threading.Thread(target=server.serve_forever, daemon=True, name="TelemetrySSE")
    server_thread.start()
    log.info(
        f"telemetría para Kuntur-GCS en http://{host}:{port}/telemetry ({rate_hz:.0f} Hz)",
        extra={"host": host, "port": port, "rate_hz": rate_hz},
    )

    interval = 1.0 / rate_hz
    last_seq = -1
    try:
        while not stop_event.is_set():
            t0 = time.monotonic()
            try:
                snap = runtime.snapshot()
            except OSError as e:
                log.warning(f"conexión con KSE perdida ({e}) -- terminando el hilo", extra={"error": str(e)})
                return
            except Exception as e:
                log.error(f"error leyendo snapshot: {e}", extra={"error": str(e)})
                snap = None

            # Sin paquete nuevo (planta detenida) no se republica: el GCS
            # detecta la falta de frames y marca el enlace como caído.
            if snap is not None and snap.seq_num != last_seq:
                last_seq = snap.seq_num
                hub.publish(_snapshot_to_frame(snap, local_frame))

            elapsed = time.monotonic() - t0
            if elapsed < interval:
                stop_event.wait(timeout=interval - elapsed)
    finally:
        server.shutdown()
        server.server_close()
        log.info("relay de telemetría terminado")
