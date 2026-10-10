"""
gcs_api_worker.py -- Puente REST entre Kuntur-GCS (navegador) y el modo
de control del vehículo (Automático/Manual).

Fork HIL: reemplaza por completo al gcs_api_worker.py del fork CADAC-SIL
original, que exponía retasking/reemplazo de ruta sobre el canal
KUNTUR_CTL (retirado del backend, ver plan-hil-stm32-portado-autopiloto.md
§9/§14). Lo único que este componente necesita hoy es:

  - POST /mode   {"mode": "automatic" | "manual"} -- cambia el modo.
  - GET  /status -- devuelve el modo actual y el estado del STM32.

El modo no se aplica solo acá: `ModeClient` lo propaga por el canal
KUNTUR_MODE (TCP, ver icd_schema/kuntur_ic_mode_v1.json y
cadac_plant/protocol.py::ModeLink) hasta plant_driver.py -- el único
proceso que puede aplicar el bypass de superficies
(RealFalcon6Adapter.set_manual_mode(), ver plan §8). POST /mode solo
confirma éxito a Kuntur-GCS si plant_driver.py efectivamente lo recibió
y lo confirmó (ACK) -- un modo "aplicado" que en realidad no llegó a la
planta sería peor que un error visible.
"""
from __future__ import annotations

import json
import socket
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

from .config import KUNTUR_MODE_PORT
from .icd_mode import pack_mode
from .icd_mode_ack import unpack_mode_ack, MODE_ACK_SIZE, ICDFrameError as _ModeAckFrameError
from . import logging_setup

log = logging_setup.get_logger("gcs_api")

VALID_MODES = ("automatic", "manual")
_MODE_TO_WIRE = {"automatic": 0, "manual": 1}


class PlantUnreachable(Exception):
    """plant_driver.py no está arriba, o la conexión se cortó a mitad de
    un pedido -- el handler HTTP lo traduce a 503, no cuelga al navegador."""


class ModeClient:
    """Cliente TCP persistente hacia plant_driver.py:KUNTUR_MODE. Mismo
    patrón que el CtlClient original (conexión perezosa, timeout corto,
    serializado con un lock), reducido al mínimo: un solo campo de
    estado para propagar, sin geodesia de por medio."""

    CONNECT_TIMEOUT_S = 1.0
    RECV_TIMEOUT_S = 2.0

    def __init__(self, plant_ip: str, plant_port: int):
        self._plant_ip = plant_ip
        self._plant_port = plant_port
        self._sock: socket.socket | None = None
        self._lock = threading.Lock()
        self._seq = 0

    def close(self) -> None:
        with self._lock:
            self._drop()

    def _ensure_connected(self) -> None:
        if self._sock is not None:
            return
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(self.CONNECT_TIMEOUT_S)
        try:
            sock.connect((self._plant_ip, self._plant_port))
        except OSError as e:
            sock.close()
            raise PlantUnreachable(
                f"no se pudo conectar a KUNTUR_MODE {self._plant_ip}:{self._plant_port}: {e}"
            ) from e
        sock.settimeout(self.RECV_TIMEOUT_S)
        self._sock = sock
        log.info(f"conectado a KUNTUR_MODE en {self._plant_ip}:{self._plant_port}")

    def _drop(self) -> None:
        if self._sock is not None:
            try:
                self._sock.close()
            except OSError:
                pass
        self._sock = None

    def send_mode(self, mode: str) -> None:
        with self._lock:
            try:
                self._ensure_connected()
                self._seq += 1
                seq = self._seq
                self._sock.sendall(pack_mode(seq=seq, mode=_MODE_TO_WIRE[mode]))
                raw = self._sock.recv(MODE_ACK_SIZE)
                if len(raw) != MODE_ACK_SIZE:
                    raise PlantUnreachable("KUNTUR_MODE_ACK incompleto (conexión cerrada a mitad de pedido)")
                ack = unpack_mode_ack(raw)
                if ack.seq != seq:
                    raise PlantUnreachable(f"KUNTUR_MODE_ACK fuera de secuencia: esperado {seq}, recibido {ack.seq}")
            except (OSError, socket.timeout, _ModeAckFrameError) as e:
                self._drop()
                raise PlantUnreachable(str(e)) from e


def _make_handler(state: "_ModeState", mode_client: ModeClient):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, fmt, *args):  # silencia el access log de stdlib
            pass

        def _cors(self) -> None:
            self.send_header("Access-Control-Allow-Origin", "*")
            self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
            self.send_header("Access-Control-Allow-Headers", "Content-Type")

        def do_OPTIONS(self):
            self.send_response(204)
            self._cors()
            self.end_headers()

        def _send_json(self, status_code: int, body: dict) -> None:
            raw = json.dumps(body).encode()
            self.send_response(status_code)
            self._cors()
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(raw)))
            self.end_headers()
            self.wfile.write(raw)

        def do_GET(self):
            if self.path == "/status":
                self._send_json(200, state.snapshot())
                return
            self.send_error(404)

        def do_POST(self):
            if self.path == "/mode":
                try:
                    length = int(self.headers.get("Content-Length", "0"))
                    raw = self.rfile.read(length) if length else b"{}"
                    body = json.loads(raw)
                    mode = str(body["mode"])
                    if mode not in VALID_MODES:
                        raise ValueError(f"modo inválido: {mode!r} -- opciones: {VALID_MODES}")
                except (KeyError, ValueError, TypeError, json.JSONDecodeError) as e:
                    self._send_json(400, {"status": "bad_request", "detail": str(e)})
                    return

                try:
                    mode_client.send_mode(mode)
                except PlantUnreachable as e:
                    state.set_plant_reachable(False)
                    self._send_json(503, {"status": "plant_unreachable", "detail": str(e)})
                    return

                state.set_plant_reachable(True)
                state.set_mode(mode)
                self._send_json(200, state.snapshot())
                return
            self.send_error(404)

    return Handler


class _ModeState:
    """Estado de modo + estado del STM32 (informativo, hoy siempre
    desconectado -- lo actualizará stm32_bridge cuando exista, Fase 5/6
    del plan), expuesto por GET /status. Thread-safe -- ThreadingHTTPServer
    atiende cada request en su propio hilo."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self.mode = "automatic"
        self.plant_reachable: bool | None = None  # None = sin intentos aún
        self.stm32_connected = False
        self.stm32_fallback_count = 0

    def set_mode(self, mode: str) -> None:
        with self._lock:
            if mode != self.mode:
                log.info(f"modo cambiado: {self.mode} -> {mode}", extra={"mode": mode})
            self.mode = mode

    def set_plant_reachable(self, reachable: bool) -> None:
        with self._lock:
            self.plant_reachable = reachable

    def snapshot(self) -> dict:
        with self._lock:
            return {
                "mode": self.mode,
                "plant_reachable": self.plant_reachable,
                "stm32_connected": self.stm32_connected,
                "stm32_fallback_count": self.stm32_fallback_count,
            }


def gcs_api_worker(
    host: str = "0.0.0.0",
    port: int = 8766,
    plant_mode_ip: str = "127.0.0.1",
    plant_mode_port: int = KUNTUR_MODE_PORT,
    stop_event: threading.Event | None = None,
) -> None:
    if stop_event is None:
        stop_event = threading.Event()

    state = _ModeState()
    mode_client = ModeClient(plant_mode_ip, plant_mode_port)
    server = ThreadingHTTPServer((host, port), _make_handler(state, mode_client))
    server.daemon_threads = True
    server_thread = threading.Thread(target=server.serve_forever, daemon=True, name="GcsApiHTTP")
    server_thread.start()
    log.info(
        f"API REST para Kuntur-GCS en http://{host}:{port} (POST /mode, GET /status) "
        f"-- KUNTUR_MODE -> {plant_mode_ip}:{plant_mode_port}",
        extra={"host": host, "port": port, "plant_mode_ip": plant_mode_ip, "plant_mode_port": plant_mode_port},
    )
    try:
        stop_event.wait()
    finally:
        mode_client.close()
        server.shutdown()
        server.server_close()
        log.info("gcs_api_worker terminado")
