"""
logging_setup.py -- Configuración central de logging de kuntur_bridge.

Diseño (ver sistema_de_logs_kuntur_sim.md, requisitos NF1-NF7):

- NO bloqueante en el camino caliente: cada llamada a logger.info(...) desde
  cualquier hilo (fcs_worker, px4_bridge_worker, flightgear_worker, etc.)
  solo empuja el record a una cola en memoria (QueueHandler) -- barato, sin
  I/O. Un único hilo separado (QueueListener) drena esa cola y hace el I/O
  real (archivo + consola), fuera del camino caliente.
- Cola acotada con política de desborde explícita: si se llena, se
  DESCARTA el record más nuevo (no se bloquea el hilo que loggea, no crece
  memoria sin límite) -- ver _BoundedQueueHandler.enqueue().
- Dos salidas desde el mismo QueueListener: consola (texto legible, lo que
  ya existía como print()) y archivo JSONL (estructurado, para
  correlación/análisis vía `run_sim.py logs` o jq/pandas directo).
- Correlación básica: session_id (una por sesión, generada acá si no vino
  por env de run_sim.py) + seq_num (cuando el evento está asociado a un
  ciclo del ICD -- se pasa como kwarg `extra={"seq_num": ...}`).
"""
from __future__ import annotations

import datetime
import json
import logging
import logging.handlers
import os
import queue
import random
import socket
import string
import sys
import threading

LOGS_DIR_DEFAULT = "/tmp/kuntur_sim_logs"
QUEUE_MAXSIZE = 10_000  # NF2: acotada -- ver _BoundedQueueHandler

_listener: logging.handlers.QueueListener | None = None
_dropped_count = 0
_dropped_lock = threading.Lock()


def generate_session_id() -> str:
    """`YYYYMMDD-HHMMSS-XXXX` -- ordenable, legible, sin necesidad de UUID
    completo (rig de una sola máquina, no hace falta esa unicidad)."""
    ts = datetime.datetime.now().strftime("%Y%m%d-%H%M%S")
    suffix = "".join(random.choices(string.hexdigits.lower()[:16], k=4))
    return f"{ts}-{suffix}"


class _BoundedQueueHandler(logging.handlers.QueueHandler):
    """QueueHandler estándar, pero con política de desborde explícita
    (NF2): si la cola está llena, se descarta el record silenciosamente
    (contado, no impreso -- imprimir en el propio hilo que está
    desbordado sería exactamente el bloqueo que este diseño evita) en vez
    de bloquear (put() normal) o de dejar que Handler.handleError()
    imprima un traceback a stderr por cada record perdido."""

    def enqueue(self, record: logging.LogRecord) -> None:
        try:
            self.queue.put_nowait(record)
        except queue.Full:
            global _dropped_count
            with _dropped_lock:
                _dropped_count += 1


def dropped_count() -> int:
    """Cuántos records se descartaron por desborde de la cola desde que
    arrancó el proceso -- útil para saber si el log está siendo fiel."""
    with _dropped_lock:
        return _dropped_count


class _JsonlFormatter(logging.Formatter):
    """Una línea JSON por record. Mismo esquema de campos que el lado
    Rust (kse_server) para que ambos sean correlacionables en el mismo
    archivo: ts, level, session_id, component, seq_num, t_sim_us,
    message, + cualquier campo estructurado extra pasado por `extra=`."""

    _RESERVED = set(logging.LogRecord(
        "", 0, "", 0, "", (), None
    ).__dict__.keys()) | {"message", "asctime"}

    def __init__(self, session_id: str):
        super().__init__()
        self._session_id = session_id

    def format(self, record: logging.LogRecord) -> str:
        payload = {
            "ts": datetime.datetime.fromtimestamp(
                record.created, tz=datetime.timezone.utc
            ).isoformat(timespec="microseconds"),
            "level": record.levelname,
            "session_id": self._session_id,
            "component": record.name,
            "message": record.getMessage(),
        }
        for key, value in record.__dict__.items():
            if key not in self._RESERVED and key not in payload:
                payload[key] = value
        return json.dumps(payload, default=str)


class _ConsoleFormatter(logging.Formatter):
    """Texto legible para la terminal -- el mismo estilo que ya tenían
    los print() (`[main] mensaje`), sin el ruido de session_id/JSON."""

    def format(self, record: logging.LogRecord) -> str:
        prefix = f"[{record.name}]"
        if record.levelno >= logging.ERROR:
            prefix = f"[{record.name}] ERROR:"
        elif record.levelno >= logging.WARNING:
            prefix = f"[{record.name}] ADVERTENCIA:"
        return f"{prefix} {record.getMessage()}"


def configure(
    session_id: str | None = None,
    log_path: str | None = None,
    console_level: int = logging.INFO,
    file_level: int = logging.DEBUG,
) -> tuple[str, str]:
    """Configura el logging de todo el proceso. Llamar UNA vez, al
    arrancar (main.py:run(), antes de lanzar cualquier hilo/subproceso).
    Devuelve (session_id, log_path) resueltos.

    Cada proceso (kuntur_bridge, kse_server) escribe su PROPIO archivo
    dentro de la misma sesión -- `{session_id}.kuntur_bridge.jsonl` acá,
    `{session_id}.kse_server.jsonl` del lado Rust -- en vez de compartir
    un único archivo entre dos procesos de dos runtimes distintos, lo que
    evitaría tener que garantizar escritura atómica cruzada entre ellos.
    `run_sim.py logs` combina ambos archivos al leer (ver run_sim.py).

    session_id: si no se pasa, se toma de KUNTUR_SESSION_ID (así
    run_sim.py puede fijarlo antes de lanzar el subproceso); si tampoco
    está seteada, se genera acá -- kuntur_bridge funciona standalone sin
    run_sim.py. Se escribe de vuelta a os.environ para que KSEClient lo
    propague a kse_server sin cambios adicionales (mismo mecanismo que
    KSE_SIMULINK_IP, etc.).
    """
    global _listener

    session_id = session_id or os.environ.get("KUNTUR_SESSION_ID") or generate_session_id()
    if log_path is None:
        os.makedirs(LOGS_DIR_DEFAULT, exist_ok=True)
        log_path = os.path.join(LOGS_DIR_DEFAULT, f"{session_id}.kuntur_bridge.jsonl")
    else:
        os.makedirs(os.path.dirname(log_path) or ".", exist_ok=True)

    q: queue.Queue = queue.Queue(maxsize=QUEUE_MAXSIZE)

    os.environ["KUNTUR_SESSION_ID"] = session_id

    root = logging.getLogger("kuntur_bridge")
    root.setLevel(logging.DEBUG)
    root.handlers.clear()
    root.addHandler(_BoundedQueueHandler(q))
    root.propagate = False

    console_handler = logging.StreamHandler(sys.stdout)
    console_handler.setLevel(console_level)
    console_handler.setFormatter(_ConsoleFormatter())

    file_handler = logging.FileHandler(log_path, mode="a")
    file_handler.setLevel(file_level)
    file_handler.setFormatter(_JsonlFormatter(session_id))

    _listener = logging.handlers.QueueListener(
        q, console_handler, file_handler, respect_handler_level=True
    )
    _listener.start()

    return session_id, log_path


def shutdown() -> None:
    """Drena y detiene el QueueListener -- llamar al final del apagado
    ordenado (después de que los workers ya terminaron, antes de que el
    proceso salga), para no perder los últimos eventos (NF7)."""
    global _listener
    if _listener is not None:
        _listener.stop()
        _listener = None


def get_logger(name: str) -> logging.Logger:
    """Logger hijo de 'kuntur_bridge' -- ej. get_logger('main'),
    get_logger('px4_bridge'). El nombre completo (`kuntur_bridge.px4_bridge`)
    queda como campo `component` en el JSONL."""
    return logging.getLogger(f"kuntur_bridge.{name}")
