#!/usr/bin/env python3
"""
run_sim.py -- Orquestador de ciclo de vida de Kuntur-SIM.

FlightGear y kuntur_bridge se lanzan cada uno en su PROPIA sesión de
sistema operativo (start_new_session=True) -- ninguno recibe señales de
la terminal directamente. run_sim.py es el ÚNICO proceso que reacciona a
Ctrl+C, y decide el orden y la forma de apagar todo lo demás.

Subcomandos:
    python3 run_sim.py start [--config kuntur_sim.ini] [--no-flightgear]
                              [--runfg-script runfg.sh] [--aircraft 757|c172p|rascal|f16]
    python3 run_sim.py stop
    python3 run_sim.py status
    python3 run_sim.py clean
    python3 run_sim.py logs [-f] [--level INFO] [--component px4_bridge] [--seq 4821]

FlightGear se lanza invocando runfg.sh (no se reimplementa acá su lógica
de selección de aeronave/paths). Qué FCS corre (Kuntur-FCS o PX4) es una
decisión que vive enteramente en el .ini de kuntur_bridge
([components].enabled) -- run_sim.py es agnóstico a eso, solo orquesta
el proceso.

Logs: cada proceso (kuntur_bridge, kse_server) escribe su propio archivo
JSONL bajo el mismo session_id -- `{session_id}.kuntur_bridge.jsonl` y
`{session_id}.kse_server.jsonl` en LOGS_DIR (ver sistema_de_logs_kuntur_sim.md).
`run_sim.py logs` los combina, ordena por timestamp, filtra y colorea.
"""
from __future__ import annotations

import argparse
import datetime
import glob
import json
import os
import random
import re
import signal
import socket
import string
import subprocess
import sys
import time
from dataclasses import dataclass, asdict
from pathlib import Path

ROOT = Path(__file__).resolve().parent
SESSION_FILE = ROOT / ".run_sim_session.json"
DEFAULT_SOCKET_PATH = "/tmp/kse.sock"
DEFAULT_RUNFG_SCRIPT = ROOT / "runfg.sh"
LOGS_DIR = "/tmp/kuntur_sim_logs"

SHUTDOWN_GRACE_S = 5.0
STARTUP_GUARD_S = 1.0
DEFAULT_TELNET_PORT = 5000       # debe coincidir con TELNET_PORT de runfg.sh
FLIGHTGEAR_TELNET_TIMEOUT_S = 90.0


def _generate_session_id() -> str:
    """Mismo esquema que kuntur_bridge.logging_setup.generate_session_id()
    -- generado acá y no importado para no atar run_sim.py al paquete
    kuntur_bridge (hoy solo lo invoca como subproceso)."""
    ts = datetime.datetime.now().strftime("%Y%m%d-%H%M%S")
    suffix = "".join(random.choices(string.hexdigits.lower()[:16], k=4))
    return f"{ts}-{suffix}"


@dataclass
class SessionState:
    kuntur_bridge_pid: int | None
    fgfs_pid: int | None
    socket_path: str
    started_at: float
    session_id: str = ""

    def save(self) -> None:
        SESSION_FILE.write_text(json.dumps(asdict(self), indent=2))

    @staticmethod
    def load() -> "SessionState | None":
        if not SESSION_FILE.exists():
            return None
        try:
            data = json.loads(SESSION_FILE.read_text())
        except (json.JSONDecodeError, OSError):
            return None
        return SessionState(**data)

    @staticmethod
    def clear() -> None:
        SESSION_FILE.unlink(missing_ok=True)


def _pid_alive(pid: int | None) -> bool:
    if pid is None:
        return False
    try:
        os.kill(pid, 0)
    except ProcessLookupError:
        return False
    except PermissionError:
        return True
    return True


def _socket_reachable(socket_path: str) -> bool:
    if not os.path.exists(socket_path):
        return False
    s = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    try:
        s.connect(socket_path)
        return True
    except OSError:
        return False
    finally:
        s.close()


def _terminate_pid(pid: int, name: str, grace_s: float = SHUTDOWN_GRACE_S) -> None:
    if not _pid_alive(pid):
        return
    print(f"[run_sim] Deteniendo {name} (PID {pid})...")
    try:
        os.kill(pid, signal.SIGTERM)
    except ProcessLookupError:
        return
    deadline = time.time() + grace_s
    while time.time() < deadline:
        if not _pid_alive(pid):
            print(f"[run_sim] {name} terminó limpiamente.")
            return
        time.sleep(0.1)
    print(f"[run_sim] {name} no respondió a SIGTERM en {grace_s}s -- forzando con SIGKILL.")
    try:
        os.kill(pid, signal.SIGKILL)
    except ProcessLookupError:
        pass


def _pattern_kill(pattern: str) -> int:
    result = subprocess.run(["pgrep", "-f", pattern], capture_output=True, text=True)
    pids = [p for p in result.stdout.split() if p.isdigit()]
    if pids:
        subprocess.run(["pkill", "-9", "-f", pattern], capture_output=True)
    return len(pids)


def _launch_flightgear(runfg_script: Path, aircraft: str | None) -> subprocess.Popen:
    if not runfg_script.exists():
        raise FileNotFoundError(
            f"No se encontró {runfg_script}. Pasar --runfg-script <path> "
            "o usar --no-flightgear para correr headless."
        )
    env = os.environ.copy()
    if aircraft:
        env["KUNTUR_AIRCRAFT"] = aircraft
    print(f"[run_sim] Lanzando FlightGear ({runfg_script}, KUNTUR_AIRCRAFT={aircraft or env.get('KUNTUR_AIRCRAFT', '757')})...")
    return subprocess.Popen(
        ["bash", str(runfg_script)], env=env, start_new_session=True,
    )


def _wait_for_flightgear_telnet(
    port: int, fgfs_proc: subprocess.Popen,
    timeout_s: float = FLIGHTGEAR_TELNET_TIMEOUT_S, poll_interval_s: float = 1.0,
) -> bool:
    """Espera a que el Telnet de FlightGear (Property Tree) acepte
    conexiones antes de lanzar kuntur_bridge.

    Reemplaza el sleep(STARTUP_GUARD_S) fijo de 1s que había antes acá:
    si kuntur_bridge arranca e intenta calibrate_origin() (ver
    telnet_client.py) antes de que ese socket exista, agota sus propios
    reintentos (5 x 2s) y cae al origen de FALLBACK -- lo que deja
    prop_tree=None y, con eso, a FlightGear pausado para siempre
    (--enable-freeze de runfg.sh), porque flightgear_worker.py solo
    manda el `/sim/freeze/master 0` que despausa cuando SÍ hay prop_tree.
    1s alcanza con escena ya cacheada en una máquina rápida, pero no en
    una carga en frío real -- de ahí el bug reportado (FlightGear queda
    "trabado" sin ningún error visible, solo un warning fácil de perder
    en el log de kuntur_bridge)."""
    print(f"[run_sim] Esperando a que FlightGear acepte conexiones Telnet en el puerto {port}...")
    t0 = time.time()
    deadline = t0 + timeout_s
    while time.time() < deadline:
        if fgfs_proc.poll() is not None:
            print(f"[run_sim] ADVERTENCIA: fgfs terminó (código {fgfs_proc.returncode}) mientras se esperaba su Telnet.")
            return False
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.settimeout(1.0)
        try:
            s.connect(("127.0.0.1", port))
        except OSError:
            time.sleep(poll_interval_s)
            continue
        finally:
            s.close()
        print(f"[run_sim] Telnet de FlightGear respondiendo (tras {time.time() - t0:.1f}s) -- lanzando kuntur_bridge.")
        return True

    print(
        f"[run_sim] ADVERTENCIA: Telnet de FlightGear no respondió en {timeout_s:.0f}s -- "
        "lanzando kuntur_bridge de todos modos (probablemente caiga al origen de fallback y "
        "FlightGear quede pausado; puede que necesites presionar 'p' en su ventana a mano)."
    )
    return False


def _launch_kuntur_bridge(config_path: str | None, no_flightgear: bool, session_id: str) -> subprocess.Popen:
    env = os.environ.copy()
    env["PYTHONPATH"] = str(ROOT) + os.pathsep + env.get("PYTHONPATH", "")
    env["KUNTUR_SESSION_ID"] = session_id
    argv = [sys.executable, "-m", "kuntur_bridge.main"]
    if config_path:
        argv += ["--config", config_path]
    if no_flightgear:
        argv += ["--no-flightgear"]
    print(f"[run_sim] Lanzando kuntur_bridge ({' '.join(argv)})...")
    return subprocess.Popen(
        argv, cwd=str(ROOT), env=env, start_new_session=True,
    )


def cmd_clean(args: argparse.Namespace) -> None:
    print("[run_sim] Limpiando procesos y sockets residuales...")

    state = SessionState.load()
    socket_paths = {DEFAULT_SOCKET_PATH}
    if state:
        socket_paths.add(state.socket_path)
        if _pid_alive(state.kuntur_bridge_pid):
            _terminate_pid(state.kuntur_bridge_pid, "kuntur_bridge", grace_s=1.0)
        if _pid_alive(state.fgfs_pid):
            _terminate_pid(state.fgfs_pid, "fgfs", grace_s=1.0)

    n_kse = _pattern_kill("kse_server")
    n_fgfs = _pattern_kill("fgfs")
    print(f"[run_sim] Procesos kse_server encontrados y terminados: {n_kse}")
    print(f"[run_sim] Procesos fgfs encontrados y terminados: {n_fgfs}")

    for path in socket_paths:
        if os.path.exists(path):
            os.unlink(path)
            print(f"[run_sim] Socket residual eliminado: {path}")

    SessionState.clear()
    print("[run_sim] Limpieza completa.")


def cmd_status(args: argparse.Namespace) -> None:
    state = SessionState.load()
    if state is None:
        print("[run_sim] Sin sesión registrada (no se corrió 'start', o ya se hizo 'stop'/'clean').")
        return

    kb_alive = _pid_alive(state.kuntur_bridge_pid)
    fg_alive = _pid_alive(state.fgfs_pid)
    sock_ok = _socket_reachable(state.socket_path)

    print("[run_sim] Estado de la sesión:")
    print(f"  kuntur_bridge (PID {state.kuntur_bridge_pid}): {'vivo' if kb_alive else 'MUERTO (residual)'}")
    print(f"  fgfs          (PID {state.fgfs_pid}): "
          f"{'vivo' if fg_alive else ('MUERTO (residual)' if state.fgfs_pid else 'no lanzado (--no-flightgear)')}")
    print(f"  socket KSE    ({state.socket_path}): {'responde' if sock_ok else 'NO responde'}")

    if not kb_alive and not fg_alive:
        print("[run_sim] Ningún proceso rastreado sigue vivo -- correr 'clean' para descartar la sesión.")


def cmd_stop(args: argparse.Namespace) -> None:
    state = SessionState.load()
    if state is None:
        print("[run_sim] Sin sesión registrada -- nada que detener.")
        return
    _shutdown_session(state)
    SessionState.clear()
    print("[run_sim] Sesión detenida.")


def _shutdown_session(state: SessionState) -> None:
    _terminate_pid(state.kuntur_bridge_pid, "kuntur_bridge")
    _terminate_pid(state.fgfs_pid, "fgfs")
    if os.path.exists(state.socket_path):
        os.unlink(state.socket_path)
        print(f"[run_sim] Socket residual eliminado: {state.socket_path}")


def cmd_start(args: argparse.Namespace) -> None:
    existing = SessionState.load()
    if existing and (_pid_alive(existing.kuntur_bridge_pid) or _pid_alive(existing.fgfs_pid)):
        print("[run_sim] Ya hay una sesión corriendo. Usar 'status' para verla o 'stop' antes de arrancar otra.")
        sys.exit(1)

    cmd_clean(args)

    session_id = _generate_session_id()
    print(f"[run_sim] session_id: {session_id} -- logs en {LOGS_DIR}/{session_id}.*.jsonl")

    fgfs_proc = None
    if not args.no_flightgear:
        fgfs_proc = _launch_flightgear(Path(args.runfg_script), args.aircraft)
        time.sleep(STARTUP_GUARD_S)
        if fgfs_proc.poll() is not None:
            print(f"[run_sim] ADVERTENCIA: runfg.sh terminó casi de inmediato (código {fgfs_proc.returncode}) "
                  "-- revisar el log de FlightGear arriba. Continuando de todos modos.")
        else:
            _wait_for_flightgear_telnet(args.telnet_port, fgfs_proc)

    kb_proc = _launch_kuntur_bridge(args.config, args.no_flightgear, session_id)
    time.sleep(STARTUP_GUARD_S)
    if kb_proc.poll() is not None:
        print(f"[run_sim] ERROR: kuntur_bridge terminó casi de inmediato (código {kb_proc.returncode}).")
        if fgfs_proc is not None:
            _terminate_pid(fgfs_proc.pid, "fgfs", grace_s=2.0)
        sys.exit(1)

    state = SessionState(
        kuntur_bridge_pid=kb_proc.pid,
        fgfs_pid=fgfs_proc.pid if fgfs_proc else None,
        socket_path=args.socket_path,
        started_at=time.time(),
        session_id=session_id,
    )
    state.save()
    print(f"[run_sim] Sesión activa. kuntur_bridge PID={kb_proc.pid}"
          + (f", fgfs PID={fgfs_proc.pid}" if fgfs_proc else " (sin FlightGear)"))
    print("[run_sim] Ctrl+C para detener todo de forma ordenada.")

    shutdown_requested = {"flag": False}

    def _handle_signal(signum, _frame):
        name = signal.Signals(signum).name
        print(f"\n[run_sim] Señal {name} recibida -- deteniendo la sesión.")
        shutdown_requested["flag"] = True

    signal.signal(signal.SIGINT, _handle_signal)
    signal.signal(signal.SIGTERM, _handle_signal)

    try:
        while not shutdown_requested["flag"]:
            if kb_proc.poll() is not None:
                print(f"[run_sim] kuntur_bridge terminó por su cuenta (código {kb_proc.returncode}).")
                break
            if fgfs_proc is not None and fgfs_proc.poll() is not None:
                print(f"[run_sim] FlightGear terminó por su cuenta (código {fgfs_proc.returncode}).")
                break
            time.sleep(1.0)
    finally:
        _shutdown_session(state)
        SessionState.clear()
        print("[run_sim] Sesión finalizada.")


_LEVEL_COLOR = {
    "ERROR": "\033[31m",   # rojo
    "WARN": "\033[33m",    # amarillo
    "WARNING": "\033[33m",
    "INFO": "\033[0m",     # sin color
    "DEBUG": "\033[2m",    # tenue
}
_COLOR_RESET = "\033[0m"


def _use_color() -> bool:
    return sys.stdout.isatty() and not os.environ.get("NO_COLOR")


def _normalize_record(raw: dict, source_file: str) -> dict | None:
    """Acepta los dos esquemas JSONL del proyecto sin forzar a ninguno de
    los dos lados a imitar al otro (ver logging_setup.py y
    kse_server/src/logging_setup.rs):

      - Python (logging_setup.py):  ts, level, session_id, component,
        message, + campos extra planos.
      - Rust (tracing-subscriber):  timestamp, level, target, fields
        (message + extra), span (session_id ahí adentro).

    Devuelve un dict común: ts (datetime), level, component, message,
    session_id, seq_num, extra (dict), source_file. None si la línea no
    matchea ninguno de los dos esquemas (se descarta silenciosamente --
    una línea de log corrupta no debería tirar abajo el visor)."""
    try:
        if "timestamp" in raw and "fields" in raw:
            fields = dict(raw.get("fields", {}))
            message = fields.pop("message", "")
            span = raw.get("span") or {}
            ts = datetime.datetime.fromisoformat(raw["timestamp"].replace("Z", "+00:00"))
            return {
                "ts": ts, "level": raw.get("level", "INFO"),
                "component": raw.get("target", "?"),
                "message": message,
                "session_id": span.get("session_id"),
                "seq_num": fields.get("seq_num"),
                "extra": fields, "source_file": source_file,
            }
        elif "ts" in raw:
            reserved = {"ts", "level", "session_id", "component", "message"}
            extra = {k: v for k, v in raw.items() if k not in reserved}
            ts = datetime.datetime.fromisoformat(raw["ts"])
            return {
                "ts": ts, "level": raw.get("level", "INFO"),
                "component": raw.get("component", "?"),
                "message": raw.get("message", ""),
                "session_id": raw.get("session_id"),
                "seq_num": raw.get("seq_num"),
                "extra": extra, "source_file": source_file,
            }
    except (KeyError, ValueError):
        return None
    return None


def _read_all_records(paths: list[str]) -> list[dict]:
    records = []
    for path in paths:
        try:
            with open(path) as f:
                for line in f:
                    line = line.strip()
                    if not line:
                        continue
                    try:
                        raw = json.loads(line)
                    except json.JSONDecodeError:
                        continue
                    rec = _normalize_record(raw, os.path.basename(path))
                    if rec:
                        records.append(rec)
        except OSError:
            continue
    return records


def _format_record(rec: dict, color: bool) -> str:
    ts_str = rec["ts"].strftime("%H:%M:%S.%f")[:-3]
    level = rec["level"].upper()
    prefix = f"{ts_str} {level:5s} [{rec['component']}]"
    if color:
        c = _LEVEL_COLOR.get(level, "")
        prefix = f"{c}{prefix}{_COLOR_RESET}"
    extra_str = ""
    shown_extra = {k: v for k, v in rec["extra"].items() if k != "seq_num"}
    if shown_extra:
        extra_str = " " + " ".join(f"{k}={v}" for k, v in shown_extra.items())
    seq_str = f" seq={rec['seq_num']}" if rec.get("seq_num") is not None else ""
    return f"{prefix} {rec['message']}{seq_str}{extra_str}"


def _matches_filters(rec: dict, args: argparse.Namespace) -> bool:
    if args.level:
        levels_order = ["DEBUG", "INFO", "WARN", "WARNING", "ERROR"]
        min_idx = levels_order.index(args.level.upper()) if args.level.upper() in levels_order else 0
        rec_idx = levels_order.index(rec["level"].upper()) if rec["level"].upper() in levels_order else 0
        if rec_idx < min_idx:
            return False
    if args.component and args.component not in rec["component"]:
        return False
    if args.seq is not None and rec.get("seq_num") != args.seq:
        return False
    return True


def cmd_logs(args: argparse.Namespace) -> None:
    state = SessionState.load()
    session_id = args.session_id or (state.session_id if state else None)
    if not session_id:
        print("[run_sim] Sin session_id -- no hay sesión registrada y no se pasó --session-id.")
        sys.exit(1)

    pattern = os.path.join(LOGS_DIR, f"{session_id}.*.jsonl")
    paths = sorted(glob.glob(pattern))
    if not paths:
        print(f"[run_sim] Sin archivos de log para session_id={session_id} en {LOGS_DIR}.")
        sys.exit(1)

    color = _use_color()
    seen_offsets = {p: os.path.getsize(p) for p in paths}

    records = _read_all_records(paths)
    records.sort(key=lambda r: r["ts"])
    for rec in records:
        if _matches_filters(rec, args):
            print(_format_record(rec, color))

    follow = args.follow
    if follow is None:
        follow = bool(state and _pid_alive(state.kuntur_bridge_pid))
    if not follow:
        return

    print(f"[run_sim] --- siguiendo en vivo (session_id={session_id}, Ctrl+C para salir) ---")
    buffers = {p: "" for p in paths}
    try:
        while True:
            time.sleep(0.2)
            for p in paths:
                try:
                    size = os.path.getsize(p)
                except OSError:
                    continue
                if size <= seen_offsets[p]:
                    continue
                with open(p) as f:
                    f.seek(seen_offsets[p])
                    chunk = f.read(size - seen_offsets[p])
                seen_offsets[p] = size
                buffers[p] += chunk
                lines = buffers[p].split("\n")
                buffers[p] = lines[-1]
                for line in lines[:-1]:
                    line = line.strip()
                    if not line:
                        continue
                    try:
                        raw = json.loads(line)
                    except json.JSONDecodeError:
                        continue
                    rec = _normalize_record(raw, os.path.basename(p))
                    if rec and _matches_filters(rec, args):
                        print(_format_record(rec, color))
    except KeyboardInterrupt:
        print("\n[run_sim] fin del seguimiento.")


def main() -> None:
    parser = argparse.ArgumentParser(description="Orquestador de ciclo de vida de Kuntur-SIM.")
    sub = parser.add_subparsers(dest="command", required=True)

    p_start = sub.add_parser("start", help="Lanzar FlightGear + kuntur_bridge.")
    p_start.add_argument("--config", default=None, help="Ruta al .ini de kuntur_bridge.")
    p_start.add_argument("--no-flightgear", action="store_true", help="No lanzar FlightGear (headless).")
    p_start.add_argument("--runfg-script", default=str(DEFAULT_RUNFG_SCRIPT), help="Ruta a runfg.sh.")
    p_start.add_argument("--aircraft", default=None, help="KUNTUR_AIRCRAFT: 757 | c172p | rascal | f16.")
    p_start.add_argument("--socket-path", default=DEFAULT_SOCKET_PATH, help="Path del socket Unix de KSE.")
    p_start.add_argument(
        "--telnet-port", type=int, default=DEFAULT_TELNET_PORT,
        help="Puerto Telnet de FlightGear a esperar antes de lanzar kuntur_bridge "
             "(debe coincidir con TELNET_PORT de runfg.sh).",
    )
    p_start.set_defaults(func=cmd_start)

    p_stop = sub.add_parser("stop", help="Detener la sesión activa (desde otra terminal).")
    p_stop.set_defaults(func=cmd_stop)

    p_status = sub.add_parser("status", help="Ver el estado de la sesión activa.")
    p_status.set_defaults(func=cmd_status)

    p_clean = sub.add_parser("clean", help="Limpieza de emergencia: mata procesos residuales y borra sockets huérfanos.")
    p_clean.set_defaults(func=cmd_clean)

    p_logs = sub.add_parser("logs", help="Ver el log de diagnóstico de la sesión (kuntur_bridge + kse_server combinados).")
    p_logs.add_argument("--session-id", default=None, help="Sesión a inspeccionar (default: la sesión activa).")
    p_logs.add_argument("-f", "--follow", dest="follow", action="store_true", default=None, help="Seguir en vivo.")
    p_logs.add_argument("--no-follow", dest="follow", action="store_false", help="No seguir -- volcar y salir.")
    p_logs.add_argument("--level", default=None, help="Nivel mínimo: DEBUG | INFO | WARN | ERROR.")
    p_logs.add_argument("--component", default=None, help="Filtrar por componente (substring, ej. px4_bridge).")
    p_logs.add_argument("--seq", type=int, default=None, help="Filtrar por seq_num exacto (correlación).")
    p_logs.set_defaults(func=cmd_logs)

    args = parser.parse_args()
    args.func(args)


if __name__ == "__main__":
    main()
