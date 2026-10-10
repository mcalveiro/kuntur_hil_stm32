"""
kse_client.py -- Cliente Python de KSE (Kuntur Simulation Executive).

Único cliente de KSE en todo el proyecto -- lanza kse_server como
subproceso, en su propia sesión de sistema operativo, y habla su
protocolo de control por socket Unix. ICD v1.3: snapshot() ahora incluye
accel_body (fuerza específica) y vel_ned, y euler llega en radianes.
"""
from __future__ import annotations

import os
import socket
import subprocess
import threading
import time
from dataclasses import dataclass
from pathlib import Path

from .config import ExecMode
from . import logging_setup

log = logging_setup.get_logger("kse_client")

DEFAULT_SERVER_BINARY = Path(__file__).resolve().parent.parent / "kse_server" / "target" / "release" / "kse_server"
DEFAULT_SOCKET_PATH = "/tmp/kse.sock"


@dataclass(frozen=True)
class AircraftStateSnapshot:
    """Copia inmutable del último KUNTUR_STATES válido (ICD v1.3).

    phi/theta/psi en RADIANES (deroga el hallazgo histórico "ago_18" que
    documentaba grados -- ver icd_schema/kuntur_ic_v1_3.json). ax/ay/az
    ya son fuerza específica (f_body), transformada por la S-Function --
    no requieren ninguna transformación adicional acá.
    """
    t_sim_us: int
    seq_num: int
    north: float
    east: float
    down: float
    phi: float
    theta: float
    psi: float
    p: float
    q: float
    r: float
    ax: float
    ay: float
    az: float
    vn: float
    ve: float
    vd: float
    ias: float
    alpha: float
    beta: float
    status: int


class RemoteEvent:
    def __init__(self, sock: socket.socket):
        self._sock = sock
        self._file = sock.makefile("rw", buffering=1)
        self._lock = threading.Lock()

    def _cmd(self, line: str) -> str:
        with self._lock:
            self._file.write(line + "\n")
            self._file.flush()
            resp = self._file.readline()
        return resp.strip()

    def wait(self, timeout: float = 3600.0) -> bool:
        return self._cmd(f"WAIT {timeout}") == "SET"

    def set(self) -> None:
        self._cmd("SET")

    def clear(self) -> None:
        self._cmd("CLEAR")

    def is_set(self) -> bool:
        return self._cmd("IS_SET") == "1"


class KSEClient:
    def __init__(
        self,
        socket_path: str = DEFAULT_SOCKET_PATH,
        server_binary: str | Path | None = None,
        simulink_ip: str = "127.0.0.1",
        telemetry_port: int = 6000,
        command_port: int = 6001,
        exec_mode: ExecMode = ExecMode.ENGINE_DRIVEN,
        rendezvous_timeout_s: float = 0.003,
        stale_warning_every_n_cycles: int = 10,
        expect_actuator_client: bool = True,
        spawn: bool = True,
    ):
        self._socket_path = socket_path
        self._proc: subprocess.Popen | None = None

        if spawn:
            binary = str(server_binary or DEFAULT_SERVER_BINARY)
            if not Path(binary).exists():
                raise FileNotFoundError(
                    f"kse_server no encontrado en {binary}. Compilar con "
                    "'cargo build --release' dentro de kse_server/."
                )
            if os.path.exists(socket_path):
                log.warning(
                    f"socket residual encontrado en {socket_path} -- eliminando",
                    extra={"socket_path": socket_path},
                )
                os.unlink(socket_path)

            env = os.environ.copy()
            env["KSE_SIMULINK_IP"] = simulink_ip
            env["KSE_TELEMETRY_PORT"] = str(telemetry_port)
            env["KSE_COMMAND_PORT"] = str(command_port)
            env["KSE_RENDEZVOUS_MS"] = str(max(1, round(rendezvous_timeout_s * 1000)))
            env["KSE_STALE_WARNING_EVERY_N"] = str(max(1, stale_warning_every_n_cycles))
            # Fork CADAC-SIL: en SIL puro (sin 'fcs' ni 'px4' habilitados) nadie
            # confirma actuadores nunca -- eso es esperado, no una falla, así
            # que le avisamos a kse_server para que no repita el warning de
            # staleness por consola indefinidamente (ver main.rs).
            env["KSE_EXPECT_ACTUATOR_CLIENT"] = "1" if expect_actuator_client else "0"
            self._proc = subprocess.Popen(
                [binary, socket_path], env=env, start_new_session=True,
            )
            self._wait_for_socket(socket_path)

        self._cmd_sock = self._connect("CMD")
        self._cmd_file = self._cmd_sock.makefile("rw", buffering=1)
        self._cmd_lock = threading.Lock()

        self._ready = RemoteEvent(self._connect("READY"))
        self._new_state = RemoteEvent(self._connect("NEW_STATE"))
        self._actuators_ready = RemoteEvent(self._connect("ACTUATORS_READY"))

        if exec_mode != ExecMode.ENGINE_DRIVEN:
            self.set_exec_mode(exec_mode)

    def _wait_for_socket(self, path: str, timeout: float = 5.0) -> None:
        deadline = time.time() + timeout
        while time.time() < deadline:
            if os.path.exists(path):
                return
            if self._proc is not None and self._proc.poll() is not None:
                raise RuntimeError(f"kse_server terminó antes de crear el socket (código {self._proc.returncode}).")
            time.sleep(0.02)
        raise TimeoutError(f"kse_server no creó el socket {path} en {timeout} s.")

    def _connect(self, channel: str) -> socket.socket:
        s = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        try:
            s.connect(self._socket_path)
        except ConnectionRefusedError as e:
            raise ConnectionRefusedError(
                f"No se pudo conectar a kse_server en {self._socket_path} ({e}). "
                "Probablemente un socket residual -- ejecutar 'python3 run_sim.py clean'."
            ) from e
        s.sendall(f"CHANNEL {channel}\n".encode())
        return s

    def _cmd(self, line: str) -> str:
        with self._cmd_lock:
            self._cmd_file.write(line + "\n")
            self._cmd_file.flush()
            return self._cmd_file.readline().strip()

    def start(self) -> None:
        pass

    def stop(self, timeout_s: float = 2.0) -> None:
        try:
            self._cmd("STOP")
        except OSError:
            pass
        if self._proc is None:
            return
        try:
            self._proc.wait(timeout=timeout_s)
            log.info("kse_server terminó vía STOP por protocolo")
            return
        except subprocess.TimeoutExpired:
            log.warning(
                f"kse_server no respondió a STOP en {timeout_s}s -- escalando a SIGTERM",
                extra={"timeout_s": timeout_s},
            )
        self._proc.terminate()
        try:
            self._proc.wait(timeout=timeout_s)
            log.info("kse_server terminó vía SIGTERM")
            return
        except subprocess.TimeoutExpired:
            log.warning(
                f"kse_server no respondió a SIGTERM en {timeout_s}s -- escalando a SIGKILL",
                extra={"timeout_s": timeout_s},
            )
        self._proc.kill()
        self._proc.wait(timeout=timeout_s)
        log.warning("kse_server terminado vía SIGKILL (sin garantía de limpieza del socket)")

    def reset(self) -> None:
        self._cmd("RESET")

    def snapshot(self) -> AircraftStateSnapshot | None:
        resp = self._cmd("SNAPSHOT")
        parts = resp.split()
        if not parts or parts[0] == "NONE":
            return None
        (_, t_sim_us, seq_num, north, east, down, phi, theta, psi,
         p, q, r, ax, ay, az, vn, ve, vd, ias, alpha, beta, status) = parts
        return AircraftStateSnapshot(
            t_sim_us=int(t_sim_us), seq_num=int(seq_num),
            north=float(north), east=float(east), down=float(down),
            phi=float(phi), theta=float(theta), psi=float(psi),
            p=float(p), q=float(q), r=float(r),
            ax=float(ax), ay=float(ay), az=float(az),
            vn=float(vn), ve=float(ve), vd=float(vd),
            ias=float(ias), alpha=float(alpha), beta=float(beta),
            status=int(status),
        )

    def set_actuators(self, elevator: float, aileron: float, rudder: float, throttle: float) -> None:
        self._cmd(f"SET_ACTUATORS {elevator} {aileron} {rudder} {throttle}")

    def set_ground_elevation(self, elev_m: float) -> None:
        self._cmd(f"SET_GROUND_ELEV {elev_m}")

    def set_exec_mode(self, mode: ExecMode) -> None:
        self._cmd(f"SET_EXEC_MODE {int(mode)}")

    def control_staleness_cycles(self) -> int:
        return int(self._cmd("STALENESS").split()[1])

    def ready_event(self) -> RemoteEvent:
        return self._ready

    def new_state_event(self) -> RemoteEvent:
        return self._new_state

    def actuators_ready_event(self) -> RemoteEvent:
        return self._actuators_ready
