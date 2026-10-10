"""
stm32_bridge.py -- puente USB CDC <-> STM32F767 para el autopiloto HIL
(ver plan-hil-stm32-portado-autopiloto.md §2/§5/§11 Fase 5-6).

Vive en el paquete kuntur_bridge (ver §9 del plan, árbol de
"Conservar") pero se importa DIRECTO desde cadac_plant/plant_driver.py
-- NO es un worker/thread de kuntur_bridge/main.py. Mismo patrón que
kuntur_bridge.geodesy/config/waypoints_file, que plant_driver.py ya
importa directo pese a correr en un proceso aparte: kuntur_bridge es,
para este fork, tanto el paquete que orquesta sus propios workers
(main.py) COMO una biblioteca compartida de la que plant_driver.py toma
prestado. Acá aplica el mismo criterio, y además es la única opción
razonable: plant_driver.py necesita la respuesta SÍNCRONA del STM32
antes de seguir con el paso siguiente (cada 10 pasos = 100 Hz, ver §5
del plan) -- cruzar a un hilo de kuntur_bridge obligaría a inventar
otro canal IPC solo para volver a cruzar el proceso, sin ganar nada.

Dos modos explícitos, sin estados intermedios:
  - CONECTADO: el puerto serie abrió con éxito. send_plant_state()
    manda KUNTUR_PLANT_STATE y espera KUNTUR_SURFACE_CMD con timeout.
  - FALLBACK SILENCIOSO: el puerto no existe, se desconectó, la
    respuesta no llegó a tiempo, o el frame es inválido --
    send_plant_state() devuelve None. plant_driver.py no necesita hacer
    nada especial: RealFalcon6Adapter._apply_hil_bypass() ya maneja
    "sin respuesta fresca" dejando el cálculo nativo tal cual (ver
    plane_adapter.py).

pyserial es la única dependencia de terceros de este módulo -- excepción
deliberada y acotada, mismo criterio que pymavlink tenía para
px4_bridge.py en el fork original (ver requirements-stm32.txt). Si no
está instalado, o el puerto no abre, el bridge opera en fallback
permanente sin crashear -- plant_driver.py debe poder correr sin
hardware conectado.
"""
from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Callable

try:
    import serial  # pyserial
except ImportError:  # pragma: no cover -- entorno sin pyserial instalado
    serial = None  # type: ignore[assignment]

from .icd_plant_state import pack_plant_state, PLANT_STATE_SIZE
from .icd_surface_cmd import unpack_surface_cmd, SURFACE_CMD_SIZE, ICDFrameError
from . import logging_setup

log = logging_setup.get_logger("stm32_bridge")

# Entre un fallo de apertura/E-S y el próximo intento de reconexión --
# sin esto, con el puerto desconectado, send_plant_state() intentaría
# reabrir el serial en CADA llamada (100 Hz = 100 intentos/s de abrir un
# dispositivo que no está).
RECONNECT_INTERVAL_S = 1.0


@dataclass(frozen=True)
class SurfaceCmdResult:
    """KUNTUR_SURFACE_CMD ya decodificado -- ver icd_surface_cmd.py."""
    delax_rad: float
    delex_rad: float
    delrx_rad: float
    throttle: float
    status: int
    round_trip_s: float


class Stm32Bridge:
    """Cliente del canal USB CDC hacia el STM32. `serial_factory` es
    inyectable (default: serial.Serial) para poder testear la lógica de
    fallback/timeout sin hardware real -- ver
    firmware/kuntur_autopilot/test/host/ para el test que la ejercita
    con un doble de prueba."""

    def __init__(
        self,
        port: str,
        baudrate: int = 115200,
        timeout_s: float = 0.025,
        serial_factory: Callable[..., object] | None = None,
    ):
        self._port = port
        self._baudrate = baudrate
        self._timeout_s = timeout_s
        self._serial_factory = serial_factory if serial_factory is not None else (
            serial.Serial if serial is not None else None
        )
        self._conn = None
        self._last_connect_attempt_s = 0.0
        self.fallback_count = 0
        self._connect()

    @property
    def connected(self) -> bool:
        return self._conn is not None

    def _connect(self) -> None:
        self._last_connect_attempt_s = time.monotonic()
        if self._serial_factory is None:
            log.warning("stm32_bridge: pyserial no está instalado -- modo fallback permanente")
            return
        try:
            self._conn = self._serial_factory(self._port, self._baudrate, timeout=self._timeout_s)
            log.info(f"stm32_bridge: conectado a {self._port} @ {self._baudrate}")
        except OSError as e:
            log.warning(f"stm32_bridge: no se pudo abrir {self._port}: {e} -- modo fallback")
            self._conn = None

    def _maybe_reconnect(self) -> None:
        if self._serial_factory is None:
            return
        if (time.monotonic() - self._last_connect_attempt_s) >= RECONNECT_INTERVAL_S:
            self._connect()

    def close(self) -> None:
        if self._conn is not None:
            try:
                self._conn.close()
            except OSError:
                pass
            self._conn = None

    def send_plant_state(self, **plant_state_fields: object) -> SurfaceCmdResult | None:
        """`plant_state_fields` son exactamente los kwargs de
        icd_plant_state.pack_plant_state() (t_sim_us, seq_num, flags,
        sn, se, ..., manual_active) -- el llamador (plant_driver.py) ya
        tiene PlantState/AuxTelemetry/AutopilotInputs en scope y arma el
        payload; este módulo no conoce esas dataclasses (mismo criterio
        de capas que el resto del fork: cadac_plant depende de
        kuntur_bridge, no al revés)."""
        if self._conn is None:
            self._maybe_reconnect()
            if self._conn is None:
                return None

        t0 = time.monotonic()
        try:
            raw = pack_plant_state(**plant_state_fields)
            assert len(raw) == PLANT_STATE_SIZE
            self._conn.write(raw)
            response = self._conn.read(SURFACE_CMD_SIZE)
        except OSError as e:
            log.warning(f"stm32_bridge: error de E/S ({e}) -- desconectando, fallback")
            self.close()
            self.fallback_count += 1
            return None

        if len(response) != SURFACE_CMD_SIZE:
            # timeout del puerto serie (pyserial.read() con timeout_s
            # devuelve lo que haya llegado, puede ser b"" o un frame
            # parcial) -- no es un error de E/S, es "no contestó a tiempo".
            self.fallback_count += 1
            return None

        try:
            pkt = unpack_surface_cmd(response)
        except ICDFrameError as e:
            log.warning(f"stm32_bridge: KUNTUR_SURFACE_CMD inválido: {e}")
            self.fallback_count += 1
            return None

        return SurfaceCmdResult(
            delax_rad=pkt.delax, delex_rad=pkt.delex, delrx_rad=pkt.delrx,
            throttle=pkt.throttle, status=pkt.status,
            round_trip_s=time.monotonic() - t0,
        )


def _dummy_plant_state_kwargs() -> dict:
    """41 campos de pack_plant_state (todo en 0.0/0 salvo el magic
    implícito), suficiente para un smoke test -- no le importa a
    FakeSerial qué hay adentro, solo que el tamaño sea PLANT_STATE_SIZE."""
    names = [
        "t_sim_us", "seq_num", "flags",
        "sn", "se", "sd", "vn", "ve", "vd", "phi", "theta", "psi", "p", "q", "r",
        "ax", "ay", "az", "pdynmc", "vmach", "dvbe", "grav", "alpha", "beta",
        "dla", "dlde", "dma", "dmq", "dmde", "dyb", "dydr", "dnb", "dnr", "dndr",
        "dllp", "dllda", "gmax", "gminx", "swel1", "swel2", "swel3", "psiflx", "thtflx", "altcom",
        "mguid", "maut", "manual_active",
    ]
    return {n: 0 for n in names}


class _FakeSerial:
    """Doble de prueba de serial.Serial -- responde con un
    KUNTUR_SURFACE_CMD fijo si `respond` es True, o con bytes
    incompletos (simula timeout) si es False. Sin dependencias (no usa
    pyserial de verdad)."""

    def __init__(self, port: str, baudrate: int, timeout: float, respond: bool = True):
        self.port = port
        self.respond = respond
        self.last_written: bytes | None = None

    def write(self, data: bytes) -> None:
        self.last_written = data

    def read(self, size: int) -> bytes:
        from .icd_surface_cmd import pack_surface_cmd, SURFACE_CMD_SIZE
        if not self.respond:
            return b""  # timeout simulado
        return pack_surface_cmd(
            t_sim_us=0, seq_num=0, status=1,
            delax=0.1, delex=-0.2, delrx=0.05, throttle=0.5,
            gainfb1=0.0, gainfb2=0.0, gainfb3=0.0, wp_flag=0.0,
        )[:size]

    def close(self) -> None:
        pass


def _smoke_test() -> None:
    # 1) Puerto inexistente -- factory levanta OSError -> fallback, sin crash.
    def _raising_factory(port, baudrate, timeout):
        raise OSError(f"{port}: no existe")

    b = Stm32Bridge(port="/dev/ttyACM_no_existe", serial_factory=_raising_factory)
    assert not b.connected
    assert b.send_plant_state(**_dummy_plant_state_kwargs()) is None
    print("Stm32Bridge: puerto inexistente -> fallback silencioso OK")

    # 2) Conectado, respuesta a tiempo -> SurfaceCmdResult correcto.
    b2 = Stm32Bridge(port="/dev/ttyFAKE", serial_factory=lambda port, baudrate, timeout: _FakeSerial(port, baudrate, timeout, respond=True))
    assert b2.connected
    result = b2.send_plant_state(**_dummy_plant_state_kwargs())
    assert result is not None
    assert abs(result.delax_rad - 0.1) < 1e-12
    assert abs(result.throttle - 0.5) < 1e-12
    print("Stm32Bridge: conectado + respuesta OK ->", result)

    # 3) Conectado, sin respuesta a tiempo -> fallback, fallback_count++.
    b3 = Stm32Bridge(port="/dev/ttyFAKE", serial_factory=lambda port, baudrate, timeout: _FakeSerial(port, baudrate, timeout, respond=False))
    assert b3.connected  # el PUERTO abrió -- es la RESPUESTA la que no llega
    assert b3.send_plant_state(**_dummy_plant_state_kwargs()) is None
    assert b3.fallback_count == 1
    print("Stm32Bridge: timeout de respuesta -> fallback silencioso OK, fallback_count =", b3.fallback_count)


if __name__ == "__main__":
    _smoke_test()
