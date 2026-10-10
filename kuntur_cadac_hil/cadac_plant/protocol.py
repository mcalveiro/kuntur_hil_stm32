"""
protocol.py -- PlantLink: lado "planta" del protocolo Kuntur-IC v1.3.

Reemplaza a kuntur_ic.cpp (la S-Function de Simulink) como contraparte
UDP de kse_server, pero habla exactamente el mismo protocolo -- verificado
leyendo kuntur_ic.cpp línea por línea, no inferido:

  - KSE (kse_server/src/main.rs) hace bind en 0.0.0.0:<telemetry_port>
    (default 6000) esperando KUNTUR_STATES, y envía KUNTUR_COMMAND a
    (simulink_ip, <command_port>) (default 6001). O sea, desde el punto
    de vista de la planta: TX de states -> (kse_ip, telemetry_port),
    RX de comandos <- bind local en command_port.

  - Modo dual ENGINE_DRIVEN (0, default) / AUTHORITY_DRIVEN (1) (DA-14),
    replicado de kuntur_ic.cpp::mdlOutputs():
      * ENGINE_DRIVEN: intento de recv() NO bloqueante, una sola vez por
        paso. Si no llegó nada, se mantiene el último comando válido
        (posible staleness -- la planta corre a su propio ritmo, no
        espera a nadie). Esto es lo que hoy hace Simulink por defecto:
        el "engine" (el solver de la planta) manda el ritmo.
      * AUTHORITY_DRIVEN: espera (select/poll con timeout) un comando
        fresco antes de continuar -- acá la "autoridad" externa (KSE,
        vía el rendezvous con el FCS) manda el ritmo. kuntur_ic.cpp usa
        5s fijos; acá se deja configurable porque un timeout de 5s no
        tiene sentido para un lazo de validación en banco a 100 Hz.
      * El modo NO lo decide la planta: lo manda KSE en cada
        KUNTUR_COMMAND (campo exec_mode), y la planta lo adopta para su
        PRÓXIMA lectura -- exactamente como sock_data->current_exec_mode
        se actualiza desde rx_pkt.exec_mode en kuntur_ic.cpp.

Esta capa es agnóstica de cadac_py: no sabe qué es un Plane ni un flat6.
Solo sabe empaquetar/desempaquetar KUNTUR_STATES/KUNTUR_COMMAND y aplicar
la semántica de modo dual. La física entra por PlantState (dataclass) y
sale un CommandPacket ya decodificado -- el llamador (plant_driver.py)
es quien conecta esto con un adaptador de planta real.

Fork HIL: sin CtlLink -- el canal de control KUNTUR_CTL (retasking /
reemplazo de ruta) se retira del backend en esta versión (ver
plan-hil-stm32-portado-autopiloto.md §9/§14). En su lugar, ModeLink
(abajo) es un canal nuevo y deliberadamente mínimo: SOLO el flag de modo
Automático/Manual, gcs_api_worker.py (cliente) -> plant_driver.py
(servidor acá) -- ver icd_schema/kuntur_ic_mode_v1.json.
"""
from __future__ import annotations

import errno
import logging
import select
import socket
import time
from dataclasses import dataclass

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from kuntur_bridge.icd import (  # noqa: E402
    pack_states, unpack_command, CommandPacket, ICDFrameError, STATES_SIZE, COMMAND_SIZE,
)
from kuntur_bridge.icd_aux import pack_aux, AUX_SIZE  # noqa: E402
from kuntur_bridge.icd_mode import unpack_mode, ModePacket, MODE_SIZE  # noqa: E402
from kuntur_bridge.icd_mode_ack import pack_mode_ack  # noqa: E402
from kuntur_bridge.config import ExecMode, KUNTUR_AUX_PORT, KUNTUR_MODE_PORT  # noqa: E402

log = logging.getLogger("cadac_plant.protocol")

# Traduce el status numérico del ACK -- ver icd_schema/kuntur_ic_mode_v1.json.
MODE_ACK_STATUS_APPLIED = 1


@dataclass(frozen=True)
class PlantState:
    """Estado físico de un paso, en las unidades/ejes que espera el ICD.

    ax/ay/az acá son FUERZA ESPECÍFICA en ejes cuerpo (f_body), lista
    para empaquetar tal cual -- ver icd_schema/kuntur_ic_v1_3.json.

    CORRECCIÓN (encontrada leyendo el código fuente real de cadac_py,
    no supuesta): en una versión anterior de este archivo, PlantLink
    aplicaba icd_helpers.specific_force_body() -- que resta la gravedad
    -- a estos campos antes de empaquetar, replicando lo que hace
    kuntur_ic.cpp con la salida cruda del bus 'Acceleration' de
    Simulink. Eso es correcto PARA Simulink, pero cadac_py/falcon6.py ya
    calcula y expone la fuerza específica lista (flat6[230] = FSPB,
    literalmente "Specific force in body axes - m/s^2", ver newton()) --
    volver a restarle la gravedad de encima sería una doble resta y
    corrompería accel_body en silencio. Por eso esa conversión se sacó
    de PlantLink y quedó a criterio de cada adaptador: MockFalcon6Adapter
    (que no separa fuerza de gravedad en su física trivial) la sigue
    aplicando él mismo; RealFalcon6Adapter copia FSPB directo.
    """
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
    status: int = 0


@dataclass(frozen=True)
class AuxTelemetry:
    """Datos del canal ICD auxiliar de instrumentación KUNTUR_AUX (ver
    icd_schema/kuntur_ic_aux_v1.json y el plan de instrumentación) --
    separado de PlantState porque NO participa del contrato Kuntur-IC
    v1.3: no tiene status/watchdog, no pasa por KSE, y viaja decimado a
    25 Hz en vez de a la tasa nativa del solver.

    Unidades: SI salvo alppx/betax, que van en GRADOS -- igual que el
    flat6 crudo de cadac_py (la conversión a radianes es responsabilidad
    exclusiva del contrato principal/PlantState, no de este canal, que
    solo reenvía instrumentación hacia FlightGear)."""
    dvbe: float     # TAS, m/s -- cadac_py flat6[236]
    vmach: float    # Mach, ND -- cadac_py flat6[56]
    pdynmc: float   # presión dinámica, Pa -- cadac_py flat6[57]
    hbe: float      # altura AGL, m -- cadac_py flat6[239]
    altrate: float  # variómetro, m/s -- cadac_py plane[530]
    alppx: float    # AOA, deg -- cadac_py flat6[140]
    betax: float    # sideslip, deg -- cadac_py flat6[145]
    anx: float      # factor de carga normal, g -- cadac_py flat6[244]
    throttle: float  # comando de acelerador, 0-1 -- cadac_py plane[52]
    power: float    # potencia lograda, % -- cadac_py plane[59]
    thrust: float   # empuje, N -- cadac_py plane[55]


@dataclass(frozen=True)
class AutopilotInputs:
    """Derivadas aerodinámicas + waypoint activo que el autopiloto
    FALCON6 necesita y que, en modo HIL, viajan al STM32 por
    KUNTUR_PLANT_STATE (ver icd_schema/kuntur_ic_plant_state_v1.json).
    Separado de PlantState/AuxTelemetry porque son específicos del
    autopiloto (no de la física ni de la instrumentación hacia
    FlightGear) y solo se necesitan cuando hay un STM32 en el lazo --
    RealFalcon6Adapter.autopilot_inputs() los expone bajo demanda, no en
    cada step() (a diferencia de aux_telemetry(), que sí cachea cada
    paso -- acá no hace falta: stm32_bridge los pide decimado a 100 Hz).

    Índices relevados contra cadac_py/falcon6.py línea por línea (no
    asumidos del plan): dla/dlde/dma/dmq/dmde = plane[145..149] (usados
    por control_pitch_rate/control_normal_accel), dyb/dydr/dnb/dnr/dndr
    = plane[150..154] (control_lateral_accel), dllp/dllda =
    plane[155..156] (control_roll/control_roll_rate).

    grav = flat6[55] -- NO es la constante AGRAV: se recalcula cada paso
    en environment() como G*EARTH_MASS/rad², varía levemente con la
    distancia al centro de la Tierra. Usada por control_heading/
    control_altitude/guidance/guidance_line -- faltaba en el borrador
    original de este schema, encontrada recién al portar esas funciones
    (Fase 3). control_lateral_accel también lee flat6[55], pero es una
    lectura muerta que no afecta su resultado (ver
    firmware/.../control_lateral_accel.h).

    gmax/gminx = plane[163]/[167] -- límites estructurales de maniobra
    (g's), calculados en aerodynamics() a partir del aerodeck
    (alpx/weight). Usados por control() en los lazos mautp==3/5 (los
    que SÍ se ejercitan en f16c11_7.asc) -- otro campo que faltaba en
    el borrador original, encontrado al portar control() (Fase 3)."""
    grav: float
    dla: float
    dlde: float
    dma: float
    dmq: float
    dmde: float
    dyb: float
    dydr: float
    dnb: float
    dnr: float
    dndr: float
    dllp: float
    dllda: float
    gmax: float
    gminx: float
    swel1: float
    swel2: float
    swel3: float
    psiflx_deg: float
    thtflx_deg: float
    altcom: float
    mguid: int
    maut: int


class AuxTelemetryLink:
    """TX-only del canal ICD auxiliar KUNTUR_AUX -- transporte DIRECTO
    plant_driver.py -> kuntur_bridge, puerto 7000 por default, SIN pasar
    por KSE/kse_server (ver icd_schema/kuntur_ic_aux_v1.json, campo
    'transport': la señal no participa de la semántica ENGINE_DRIVEN/
    AUTHORITY_DRIVEN que arbitra KSE -- es adaptación de un simulador
    batch hacia FlightGear, no un contrato de control planta<->FCS).

    Comparte t_sim_us/seq_num con PlantLink.send_states() -- el llamador
    (plant_driver.py) es responsable de pasar los mismos valores para
    poder correlacionar ambos flujos durante logueo."""

    def __init__(self, bridge_ip: str = "127.0.0.1", aux_port: int = KUNTUR_AUX_PORT):
        self.bridge_ip = bridge_ip
        self.aux_port = aux_port
        self._tx_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def close(self) -> None:
        self._tx_sock.close()

    def send_aux(self, t_sim_us: int, seq_num: int, aux: AuxTelemetry) -> None:
        raw = pack_aux(
            t_sim_us=t_sim_us, seq_num=seq_num,
            dvbe=aux.dvbe, vmach=aux.vmach, pdynmc=aux.pdynmc, hbe=aux.hbe,
            altrate=aux.altrate, alppx=aux.alppx, betax=aux.betax, anx=aux.anx,
            throttle=aux.throttle, power=aux.power, thrust=aux.thrust,
        )
        assert len(raw) == AUX_SIZE
        self._tx_sock.sendto(raw, (self.bridge_ip, self.aux_port))


class ModeLink:
    """Servidor TCP del canal KUNTUR_MODE (flag Automático/Manual) -- ver
    icd_schema/kuntur_ic_mode_v1.json. Mismo criterio que tenía CtlLink
    (bind+listen+accept no bloqueante, poll() se llama en cada iteración
    del loop principal sin pausar la simulación, un único cliente a la
    vez), pero DELIBERADAMENTE más chico: un solo tipo de mensaje por
    dirección, sin pipelining de framing variable (KUNTUR_MODE siempre
    mide MODE_SIZE bytes) -- no hace falta leer el magic para decidir
    cuánto acumular, a diferencia de CtlLink (que sí tenía dos tamaños
    distintos en la misma conexión)."""

    def __init__(self, bind_ip: str = "0.0.0.0", mode_port: int = KUNTUR_MODE_PORT):
        self.mode_port = mode_port
        self._listen_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._listen_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._listen_sock.bind((bind_ip, mode_port))
        self._listen_sock.listen(1)
        self._listen_sock.setblocking(False)
        self._conn: socket.socket | None = None
        self._recv_buf = b""

    def close(self) -> None:
        if self._conn is not None:
            self._conn.close()
        self._listen_sock.close()

    def _accept_if_pending(self) -> None:
        if self._conn is not None:
            return
        try:
            conn, addr = self._listen_sock.accept()
            conn.setblocking(False)
            self._conn = conn
            self._recv_buf = b""
            log.info("KUNTUR_MODE: cliente conectado desde %s", addr)
        except BlockingIOError:
            pass

    def _drop_connection(self) -> None:
        if self._conn is not None:
            try:
                self._conn.close()
            except OSError:
                pass
        self._conn = None
        self._recv_buf = b""

    def poll(self) -> ModePacket | None:
        """No bloqueante. Devuelve un KUNTUR_MODE decodificado apenas se
        completa uno, o None si todavía no hay nada."""
        self._accept_if_pending()
        if self._conn is None:
            return None

        try:
            chunk = self._conn.recv(4096)
            if chunk == b"":
                log.info("KUNTUR_MODE: cliente desconectado")
                self._drop_connection()
                return None
            self._recv_buf += chunk
        except BlockingIOError:
            pass
        except OSError:
            self._drop_connection()
            return None

        if len(self._recv_buf) < MODE_SIZE:
            return None  # mensaje todavía incompleto, esperar más bytes

        raw, self._recv_buf = self._recv_buf[:MODE_SIZE], self._recv_buf[MODE_SIZE:]
        try:
            return unpack_mode(raw)
        except ICDFrameError as e:
            log.warning("KUNTUR_MODE: mensaje inválido descartado: %s", e)
            return None

    def send_mode_ack(self, seq: int, status: int = MODE_ACK_STATUS_APPLIED) -> None:
        if self._conn is None:
            return
        try:
            self._conn.sendall(pack_mode_ack(seq=seq, status=status))
        except OSError:
            log.warning("KUNTUR_MODE: no se pudo enviar MODE_ACK (conexión perdida)")
            self._drop_connection()


class PlantLink:
    def __init__(
        self,
        kse_ip: str = "127.0.0.1",
        telemetry_port: int = 6000,   # KSE escucha acá (states) -- destino de nuestro TX
        command_port: int = 6001,      # nosotros escuchamos acá (comandos) -- igual a LOCAL_PORT_RX en kuntur_ic.cpp
        authority_driven_timeout_s: float = 0.1,
    ):
        self.kse_ip = kse_ip
        self.telemetry_port = telemetry_port
        self.command_port = command_port
        self.authority_driven_timeout_s = authority_driven_timeout_s

        self._tx_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        self._rx_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._rx_sock.bind(("0.0.0.0", command_port))
        self._rx_sock.setblocking(False)  # ENGINE_DRIVEN: recv no bloqueante, igual que fcntl(O_NONBLOCK) en el C++

        self.exec_mode: int = int(ExecMode.ENGINE_DRIVEN)
        self.last_valid_command: CommandPacket | None = None
        self.last_latency_ms: float = 0.0

    def close(self) -> None:
        self._tx_sock.close()
        self._rx_sock.close()

    def send_states(self, t_sim_us: int, seq_num: int, state: PlantState) -> None:
        # state.ax/ay/az YA es fuerza específica -- ningún cálculo acá
        # (ver el docstring de PlantState sobre por qué se sacó la
        # conversión de este punto).
        raw = pack_states(
            t_sim_us=t_sim_us, seq_num=seq_num,
            north=state.north, east=state.east, down=state.down,
            phi=state.phi, theta=state.theta, psi=state.psi,
            p=state.p, q=state.q, r=state.r,
            ax=state.ax, ay=state.ay, az=state.az,
            vn=state.vn, ve=state.ve, vd=state.vd,
            ias=state.ias, alpha=state.alpha, beta=state.beta,
            status=state.status,
        )
        assert len(raw) == STATES_SIZE
        self._tx_sock.sendto(raw, (self.kse_ip, self.telemetry_port))

    def poll_command(self, current_t_sim_us: int) -> CommandPacket | None:
        """Intenta leer un KUNTUR_COMMAND fresco, respetando el modo
        vigente. Devuelve el comando recién leído, o None si no llegó
        nada nuevo en esta llamada (el llamador debe seguir usando
        self.last_valid_command). Actualiza self.exec_mode y
        self.last_latency_ms exactamente como kuntur_ic.cpp."""
        raw: bytes | None = None

        if self.exec_mode == int(ExecMode.AUTHORITY_DRIVEN):
            # AUTHORITY_DRIVEN: bloquea (con timeout) hasta que llegue un
            # comando fresco -- acá la autoridad externa marca el ritmo.
            readable, _, _ = select.select([self._rx_sock], [], [], self.authority_driven_timeout_s)
            if readable:
                try:
                    raw = self._rx_sock.recv(COMMAND_SIZE + 32)
                except OSError:
                    raw = None
            # timeout: sin novedad, se mantiene last_valid_command (igual
            # que el "pendiente" ya documentado en kuntur_ic.cpp).
        else:
            # ENGINE_DRIVEN: un solo intento no bloqueante -- la planta
            # no espera a nadie.
            try:
                raw = self._rx_sock.recv(COMMAND_SIZE + 32)
            except BlockingIOError:
                raw = None
            except OSError as e:
                if e.errno not in (errno.EAGAIN, errno.EWOULDBLOCK):
                    raise
                raw = None

        if raw is None:
            return None

        try:
            cmd = unpack_command(raw)
        except ICDFrameError:
            return None

        self.last_valid_command = cmd
        self.last_latency_ms = (current_t_sim_us - cmd.t_sim_us_grant) / 1000.0
        self.exec_mode = cmd.exec_mode
        return cmd


def _smoke_test_loopback() -> None:
    """Prueba de humo sin KSE: dos PlantLink hablando entre sí en
    localhost, para validar el framing antes de involucrar el binario
    Rust real. No reemplaza la prueba end-to-end con kse_server."""
    a = PlantLink(kse_ip="127.0.0.1", telemetry_port=15000, command_port=15001)
    b_tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    b_rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    b_rx.bind(("0.0.0.0", 15000))
    b_rx.settimeout(1.0)

    # reposo nivelado -> fuerza específica esperada [0,0,-g] (ver
    # icd_helpers._self_test()) -- se pasa ya calculada, PlantLink no
    # la deriva más.
    from .icd_helpers import specific_force_body
    fx, fy, fz = specific_force_body(0.0, 0.0, 0.0, 0.0, 0.0)
    state = PlantState(
        north=10.0, east=20.0, down=-500.0, phi=0.0, theta=0.0, psi=0.0,
        p=0.0, q=0.0, r=0.0, ax=fx, ay=fy, az=fz,
        vn=45.0, ve=0.0, vd=0.0, ias=45.0, alpha=0.02, beta=0.0,
    )
    a.send_states(t_sim_us=10_000, seq_num=1, state=state)
    data, _ = b_rx.recvfrom(200)
    from kuntur_bridge.icd import unpack_states
    sp = unpack_states(data)
    assert abs(sp.north - 10.0) < 1e-4 and abs(sp.vn - 45.0) < 1e-4
    assert abs(sp.az - (-9.80665)) < 1e-3
    print("PlantLink.send_states -> unpack_states OK:", sp)

    from kuntur_bridge.icd import pack_command
    cmd_raw = pack_command(10_000, 0.05, -0.1, 0.0, 0.6, 589.0, ExecMode.ENGINE_DRIVEN)
    b_tx.sendto(cmd_raw, ("127.0.0.1", 15001))
    time.sleep(0.05)
    cmd = a.poll_command(current_t_sim_us=10_500)
    assert cmd is not None and abs(cmd.throttle - 0.6) < 1e-4
    print("PlantLink.poll_command OK:", cmd, "latencia_ms=", a.last_latency_ms)

    a.close()
    b_tx.close()
    b_rx.close()


def _smoke_test_mode_loopback() -> None:
    """Prueba de humo del canal KUNTUR_MODE: un ModeLink (servidor) y un
    socket TCP cliente en localhost, sin gcs_api_worker.py real."""
    from kuntur_bridge.icd_mode import pack_mode
    from kuntur_bridge.icd_mode_ack import unpack_mode_ack

    server = ModeLink(bind_ip="127.0.0.1", mode_port=15200)
    client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    client.connect(("127.0.0.1", 15200))
    client.settimeout(1.0)

    client.sendall(pack_mode(seq=1, mode=1))  # 1 = manual
    time.sleep(0.05)

    msg = None
    for _ in range(20):
        msg = server.poll()
        if msg is not None:
            break
        time.sleep(0.01)
    assert isinstance(msg, ModePacket) and msg.seq == 1 and msg.mode == 1, msg
    print("ModeLink.poll() -> KUNTUR_MODE OK:", msg)

    server.send_mode_ack(seq=msg.seq, status=MODE_ACK_STATUS_APPLIED)
    ack = unpack_mode_ack(client.recv(10))
    assert ack.seq == 1 and ack.status == MODE_ACK_STATUS_APPLIED, ack
    print("ModeLink.send_mode_ack() OK:", ack)

    client.close()
    server.close()


if __name__ == "__main__":
    _smoke_test_loopback()
    _smoke_test_mode_loopback()
