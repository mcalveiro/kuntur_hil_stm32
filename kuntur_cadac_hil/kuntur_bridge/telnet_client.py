"""telnet_client.py -- Cliente del property tree de FlightGear vía Telnet."""
from __future__ import annotations

import socket
import time

from .config import FG_IP, FG_TELNET_PORT, FALLBACK_LAT, FALLBACK_LON, FALLBACK_ALT
from .geodesy import SACOLocalFrame
from . import logging_setup
from .icd_aux import AuxPacket

log = logging_setup.get_logger("telnet_client")

# Conversiones de unidades para el mapeo del canal ICD auxiliar (ver
# apply_aux_instrumentation() más abajo) -- mismos factores que ya usa
# flightgear_worker.py (0.3048 ft->m) pero invertidos, m->ft y m/s->kt.
M_TO_FT = 3.28084
MPS_TO_KT = 1.94384


class FGPropertyTree:
    def __init__(self, host: str, port: int, timeout: float = 5.0):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.settimeout(timeout)
        self.sock.connect((host, port))
        self._buf = b""
        self._connect_timeout = timeout
        self.sock.sendall(b"data\r\n")
        self._drain()

    def _drain(self, idle_timeout: float = 0.2) -> None:
        """Vacía cualquier respuesta pendiente al "data\\r\\n" del
        __init__ antes de que arranque el uso síncrono get/set.

        Antes esto era un sleep(0.5) fijo seguido de UN intento de recv()
        no bloqueante: si FlightGear tardaba más de 0.5s en contestar
        (típico justo al arrancar, con aeronave/terreno/shaders cargando),
        el drain se creía terminado sobre un socket vacío, y el ack de
        "data" llegaba recién DESPUÉS -- quedando como una línea fantasma
        que corría en uno los get() reales de calibrate_origin() (lat leía
        el ack, lon leía la respuesta real de lat, alt leía la de lon).
        Bug confirmado en producción: un origen calibrado como
        lat=0.0 lon=-31.31 (el valor real de LAT corrido al campo de LON)
        -- ver kuntur_sim_logs, sesión 20260921-211807-1f81.

        Un primer intento de arreglo (idle_timeout corto también en la
        PRIMERA lectura) resultó insuficiente: si el ack tarda más que
        idle_timeout en llegar, ese primer recv() expira igual y el drain
        se da por terminado antes de que llegue nada -- mismo bug, ventana
        más chica. Por eso acá se separan dos fases: la primera espera el
        ack usando el timeout de conexión completo (self._connect_timeout,
        el mismo con que se conectó el socket -- cuanto tarde FlightGear en
        responder, sin apostar a un número fijo); recién una vez que llegó
        ALGO, una segunda fase con idle_timeout corto drena cualquier resto
        de la misma ráfaga y corta apenas deja de llegar más.
        """
        self.sock.settimeout(self._connect_timeout)
        try:
            chunk = self.sock.recv(4096)
        except socket.timeout:
            chunk = b""
        self.sock.settimeout(idle_timeout)
        try:
            while chunk:
                chunk = self.sock.recv(4096)
        except socket.timeout:
            pass
        finally:
            self.sock.settimeout(3.0)
        self._buf = b""

    def _readline(self, timeout: float = 3.0) -> str:
        self.sock.settimeout(timeout)
        while b"\n" not in self._buf:
            try:
                chunk = self.sock.recv(4096)
                if not chunk:
                    return self._buf.decode("utf-8").strip() if self._buf else ""
                self._buf += chunk
            except socket.timeout:
                return self._buf.decode("utf-8").strip() if self._buf else ""
        line, _, self._buf = self._buf.partition(b"\n")
        return line.decode("utf-8").strip()

    def get(self, prop_path: str) -> float:
        try:
            self.sock.sendall(f"get {prop_path}\r\n".encode("utf-8"))
            resp = self._readline()
            return float(resp) if resp else 0.0
        except Exception:
            return 0.0

    def set(self, prop_path: str, value) -> bool:
        try:
            self.sock.sendall(f"set {prop_path} {value}\r\n".encode("utf-8"))
            return True
        except Exception:
            return False


def calibrate_origin(
    fg_ip: str = FG_IP,
    fg_telnet_port: int = FG_TELNET_PORT,
    fallback_lat: float = FALLBACK_LAT,
    fallback_lon: float = FALLBACK_LON,
    fallback_alt: float = FALLBACK_ALT,
    attempts: int = 5,
    delay_s: float = 2.0,
) -> tuple[SACOLocalFrame, FGPropertyTree | None]:
    log.info("conectando a FlightGear Telnet para autocalibración en SACO")
    for attempt in range(attempts):
        try:
            prop_tree = FGPropertyTree(fg_ip, fg_telnet_port, timeout=5.0)
            lat0 = prop_tree.get("/position/latitude-deg")
            lon0 = prop_tree.get("/position/longitude-deg")
            alt0_m = prop_tree.get("/position/altitude-ft") * 0.3048

            valid = (
                -90.0 <= lat0 <= 90.0 and -180.0 <= lon0 <= 180.0
                and -1000.0 <= alt0_m <= 10000.0
                and lat0 != 0.0 and lon0 != 0.0
            )
            if valid:
                log.info(
                    f"origen calibrado: lat={lat0:.6f} lon={lon0:.6f} alt={alt0_m:.2f}m MSL",
                    extra={"lat0": lat0, "lon0": lon0, "alt0_m": alt0_m, "origin_source": "telnet"},
                )
                return SACOLocalFrame(lat0, lon0, alt0_m), prop_tree
            # Lectura implausible -- no necesariamente un problema de red:
            # típico cuando el Telnet ya acepta conexiones pero
            # /position/* todavía no se asentó desde --lat/--lon/--altitude
            # (arranque en frío). ANTES, esto solo se descartaba si lat Y
            # lon daban EXACTAMENTE (0,0) -- un lat=0.0 con un lon "creíble"
            # (p.ej. el propio --lat leído en el campo de lon) pasaba el
            # chequeo igual y se aceptaba en el primer intento, sin darle
            # tiempo a asentarse. Ahora se rechaza si CUALQUIERA de los dos
            # da exactamente 0.0 -- una lat o lon real de Córdoba nunca cae
            # justo ahí ("Null Island").
            log.warning(
                f"intento {attempt + 1}: lectura implausible "
                f"(lat={lat0:.6f} lon={lon0:.6f} alt={alt0_m:.2f}m) -- probablemente "
                "FlightGear todavía no asentó /position/* tras el arranque, reintentando",
                extra={"attempt": attempt + 1, "lat0": lat0, "lon0": lon0, "alt0_m": alt0_m},
            )
            prop_tree.sock.close()
            time.sleep(delay_s)
        except Exception as e:
            log.warning(
                f"intento {attempt + 1}: Telnet falló ({e}), reintentando",
                extra={"attempt": attempt + 1, "error": str(e)},
            )
            time.sleep(delay_s)

    log.warning(
        "no se pudo calibrar vía Telnet -- usando origen de fallback",
        extra={"lat0": fallback_lat, "lon0": fallback_lon, "alt0_m": fallback_alt, "origin_source": "fallback"},
    )
    return SACOLocalFrame(fallback_lat, fallback_lon, fallback_alt), None


def connect_property_tree(
    fg_ip: str = FG_IP,
    fg_telnet_port: int = FG_TELNET_PORT,
    attempts: int = 5,
    delay_s: float = 2.0,
) -> FGPropertyTree | None:
    """Abre una sesión Telnet PROPIA e independiente al Property Tree de
    FlightGear, con reintentos -- para cualquier consumidor que necesite
    su propia conexión en vez de compartir la que ya devuelve
    calibrate_origin(). FGPropertyTree no es thread-safe (un único socket
    síncrono, sin locks): dos hilos leyendo/escribiendo por el mismo
    objeto pueden entrelazar sus respuestas (el get() de uno lee la
    respuesta del set() del otro). Usado por aux_instrument_worker.py
    para no competir con el polling de terreno de flightgear_worker.py
    sobre el prop_tree que ya usa ese hilo."""
    for attempt in range(attempts):
        try:
            return FGPropertyTree(fg_ip, fg_telnet_port, timeout=5.0)
        except Exception as e:
            log.warning(
                f"intento {attempt + 1}: conexión Telnet dedicada falló ({e}), reintentando",
                extra={"attempt": attempt + 1, "error": str(e)},
            )
            time.sleep(delay_s)

    log.warning("no se pudo abrir conexión Telnet dedicada tras %d intentos", attempts)
    return None


def apply_aux_instrumentation(prop_tree: FGPropertyTree, aux: AuxPacket) -> None:
    """Aplica al Property Tree de FlightGear el mapeo del canal ICD
    auxiliar KUNTUR_AUX (ver icd_schema/kuntur_ic_aux_v1.json y el plan
    de instrumentación -- tablas de mapeo motor/sonido e instrumentación
    básica de vuelo).

    Motor/sonido -- DOS familias de propiedad para el mismo dato físico
    (ver hallazgo del plan): 'engines/engine[0]/*' (indexada, sonidos
    internos de cabina) y 'engines/engine/*' (sin indexar, sonidos
    externos). Bajo --fdm=null no hay FDM real (JSBSim nunca corre) que
    las mantenga sincronizadas mediante un alias propio del avión, así
    que se escriben AMBAS explícitamente con el mismo valor -- si no,
    una de las dos queda muda.

    Limitación de modelo, no de mapeo: cadac_py no separa N1 de N2 (una
    sola variable de potencia integrada, 'power') ni modela post-
    combustión ni APU/JFS -- se escribe el mismo valor a n1/n2 en ambas
    familias, y los valores de afterburner/JFS quedan fijos en "apagado"
    (ver tabla del plan).

    Deliberadamente NO escrito acá (ver plan, sección "a verificar en la
    prueba de humo, no asumir" y "explícitamente pospuesto"):
      - sound-n2-vol-*/n1-out1/n1-out2/pilot-gdamped: se asume que un
        script Nasal propio del paquete FGAddon las deriva de n1/n2/
        pilot-g de forma autónoma (no depende del FDM) -- si la prueba
        de humo muestra que NO es así, el plan B es agregarlas acá.
      - velocities/groundspeed-kt: explícitamente pospuesto a la próxima
        sesión (ver plan, "Qué traer a la próxima sesión").
      - vmach/pdynmc/thrust: viajan en el paquete pero sin propiedad
        FlightGear destino decidida todavía (pdynmc queda disponible acá
        por si se necesita derivar un EAS/IAS separado de TAS).
    """
    running = 1 if aux.throttle > 0.0 else 0

    # --- Motor y sonido (N1=N2=power, ambas familias de propiedad) ---
    prop_tree.set("/engines/engine[0]/n1", aux.power)
    prop_tree.set("/engines/engine[0]/n2", aux.power)
    prop_tree.set("/engines/engine/n1", aux.power)
    prop_tree.set("/engines/engine/n2", aux.power)

    prop_tree.set("/engines/engine[0]/running", running)
    prop_tree.set("/engines/engine/running", running)

    # Afterburner/JFS: sin modelo en cadac_py -- fijos en "apagado".
    prop_tree.set("/engines/engine[0]/nozzle-pos-norm", 0)
    prop_tree.set("/engines/engine/augmentation", 0)
    prop_tree.set("/f16/engine/jfs-rpm-normalized", 0)

    # --- Instrumentación básica de vuelo ---
    prop_tree.set("/position/altitude-agl-ft", aux.hbe * M_TO_FT)
    prop_tree.set("/orientation/alpha-deg", aux.alppx)
    prop_tree.set("/velocities/airspeed-kt", aux.dvbe * MPS_TO_KT)
    prop_tree.set("/velocities/vertical-speed-fps", aux.altrate * M_TO_FT)
    prop_tree.set("/accelerations/pilot-g", aux.anx)
