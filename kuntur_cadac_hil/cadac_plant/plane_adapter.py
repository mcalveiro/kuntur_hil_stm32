"""
plane_adapter.py -- Interfaz de planta para el fork CADAC-SIL, y DOS
implementaciones: una física de reemplazo trivial (MockFalcon6Adapter,
usada para validar el cableado completo antes de tener la fuente real) y
la real (RealFalcon6Adapter), que re-conduce cadac_py/falcon6.py::Plane
paso a paso desde afuera.

Por qué RealFalcon6Adapter mantiene control()+guidance() ADENTRO de Plane
(y no los extrae a un FCS separado, como se había planeado antes de leer
la fuente):
------------------------------------------------------------------------
control()/guidance() de FALCON6 leen ~15 derivadas aerodinámicas y
variables de sintonía persistentes (dla, dlde, dma, dmq, dmde, dyb, dydr,
dnb, dnr, dndr, el bitmask maut=mauty*10+mautp, el estado de los
integradores de cada lazo) que NO viajan por el ICD y no son triviales de
recalcular fuera de Plane sin duplicar casi toda la clase. guidance() en
particular calcula wp_flag (avance de waypoint) por geometría de radio de
giro, cada paso, contra el estado interno -- no por un "radio de captura"
externo. Separarlos exigiría reimplementar casi toda la física de
FALCON6 del lado FCS, exactamente lo que este proyecto busca evitar
("reutilizar lo que ya está desarrollado... evitando rehacer componentes
que ya funcionan"). En cambio, execution.py ya expone las piezas
(parse_input_asc/build_vehicle/apply_vehicle_data/build_events/
check_events/_EXEC_DISPATCH) para re-conducir el mismo bucle que usa
run_scenario() paso a paso desde afuera -- validado bit-a-bit contra el
batch de referencia (ver regression_vs_batch.py, diferencia 0.0m).

Consecuencia directa: el ActuatorCommand que llega por KUNTUR_COMMAND
(originado en KSE/Kuntur-Bridge) NO gobierna las superficies de control
en este camino -- FALCON6 es un piloto automático cerrado sobre sí mismo,
tal como corre en cualquier *.asc de CADAC. `step()` lo recibe (para
cumplir la interfaz PlantAdapter) pero lo ignora deliberadamente; ver el
docstring de RealFalcon6Adapter.step().

Mapeo de índices flat6[]/plane[] (verificado leyendo falcon6.py línea
por línea, no asertado desde el manual):
    SBEL   = flat6[219].vec()  -> north, east, down (m, NED)
    psibl  = flat6[134].real() -> psi  (rad)
    thtbl  = flat6[135].real() -> theta (rad)
    phibl  = flat6[136].real() -> phi  (rad)
    WBEB   = flat6[164].vec()  -> p, q, r (rad/s, ejes cuerpo)
    FSPB   = flat6[230].vec()  -> ax, ay, az (YA es fuerza específica --
                                  "Specific force in body axes - m/s^2" en
                                  su propio init(); ver protocol.py, NO
                                  aplicar specific_force_body() de nuevo)
    VBEL   = flat6[233].vec()  -> vn, ve, vd (m/s, NED)
    dvba   = flat6[75].real()  -> ias (m/s, velocidad respecto del aire,
                                  NO dvbe/flat6[236] que es respecto de
                                  tierra)
    alphax = flat6[144].real() -> alpha = radians(alphax) (grados en flat6)
    betax  = flat6[145].real() -> beta  = radians(betax)
"""
from __future__ import annotations

import itertools
import logging
import math
import os
import time
from abc import ABC, abstractmethod
from copy import deepcopy
from dataclasses import dataclass
from typing import Iterator

log = logging.getLogger("cadac_plant.plane_adapter")

from .protocol import PlantState, AuxTelemetry, AutopilotInputs
from .icd_helpers import specific_force_body
from kuntur_bridge.geodesy import SACOLocalFrame

from cadac_py.execution import (
    parse_input_asc, build_vehicle, apply_vehicle_data, build_events,
    check_events, make_packet, _EXEC_DISPATCH,
)

# Punto de ruta en NED, metros -- ver RouteWaypointSource.
Coord3D = tuple[float, float, float]

# Límite fijo de diseño para KUNTUR_ROUTE_REPLACE (icd_schema/kuntur_ic_ctl_v1.json)
# -- "nada implícito", igual que _override_waypoints: más de esto se rechaza,
# no se trunca en silencio.
MAX_ROUTE_WAYPOINTS = 64


@dataclass(frozen=True)
class ActuatorCommand:
    elevator: float
    aileron: float
    rudder: float
    throttle: float


@dataclass(frozen=True)
class _Stm32SurfaceCmd:
    """Última respuesta KUNTUR_SURFACE_CMD decodificada -- ver
    RealFalcon6Adapter.apply_stm32_surface_cmd()/_apply_hil_bypass().
    received_at_s es time.monotonic() al momento de registrarla, para el
    chequeo de frescura (hold-last con expiración, ver §2/§8 del plan:
    'timeout -> fallback al autopiloto nativo Python')."""
    delax_rad: float
    delex_rad: float
    delrx_rad: float
    throttle: float
    received_at_s: float


# Ventana de frescura del bypass de HIL -- una respuesta más vieja que
# esto se considera "sin novedad" y el autopiloto nativo (ya corrido ese
# paso por 'control') queda tal cual, sin pisar sus superficies. Ver
# plan-hil-stm32-portado-autopiloto.md §8 (comentario "sc.age_ms < 30").
HIL_SURFACE_CMD_MAX_AGE_S = 0.030


def _map_unit_range(value_nd: float, limit: float) -> float:
    """Mapea un valor ND en [-1,1] a [-limit,+limit] -- ver
    RealFalcon6Adapter._apply_manual_bypass(). Clipea primero: un bypass
    de superficies no es lugar para confiar ciegamente en que
    KUNTUR_COMMAND ya vino acotado."""
    value_nd = max(-1.0, min(1.0, value_nd))
    return value_nd * limit


def _clip_to_limit(value_deg: float, limit_deg: float) -> float:
    """Clipea `value_deg` a [-limit_deg,+limit_deg] -- ver
    RealFalcon6Adapter._apply_hil_bypass(). A diferencia de
    _map_unit_range(), acá el valor de entrada YA viene en grados (no
    ND): el STM32 manda delacx/delecx/delrcx en radianes
    (icd_schema/kuntur_ic_surface_cmd_v1.json), convertidos a grados
    antes de llamar a esto -- clipear por límite real sigue siendo
    necesario (el STM32 es un proceso externo, no confiar ciegamente en
    que su salida ya viene acotada)."""
    limit_deg = abs(limit_deg)
    return max(-limit_deg, min(limit_deg, value_deg))


class RouteWaypointSource:
    """Encapsula la fuente de una ruta (ver SPEC-retasking_backend.md §2) --
    reemplaza el hardcode de "reinyectar el waypoint fijo" de la versión
    anterior de _advance_loop_waypoint() (ver RealFalcon6Adapter).

    Lookahead de 1 paso (DECISIÓN DE IMPLEMENTACIÓN, no está en el sketch
    original del spec): next() cachea internamente el punto siguiente al
    que devuelve, expuesto vía peek_next(). Hace falta porque
    _advance_route_waypoint() calcula `psiflx` como el rumbo desde el
    objetivo NUEVO hacia el que viene DESPUÉS de ése (anticipación de la
    próxima pierna, no el rumbo hacia el objetivo actual) -- el mismo
    cálculo que ya hacía el código cíclico anterior indexando
    `loop_waypoints[(idx+1) % n]`. Sin este buffer, un iterador de un solo
    paso no puede dar ese dato sin consumirlo de más.

    Para la patrulla cíclica (hoy, vía RealFalcon6Adapter(waypoints=...))
    se construye con itertools.cycle(coords) -- nunca se agota, así que
    peek_next() siempre tiene algo y el comportamiento es bit-a-bit
    idéntico al indexado modular anterior (regresión de Fase 1 del spec).
    Para una ruta finita, al agotarse: `on_exhausted="loiter_last"`
    mantiene el último punto como objetivo en cada llamada subsiguiente a
    next() (peek_next() da None -- no hay pierna siguiente, el rumbo deja
    de recalcularse); `on_exhausted="none"` hace que next() devuelva None,
    y _advance_route_waypoint() no reaplica ningún objetivo nuevo ese
    paso."""

    def __init__(self, route: Iterator[Coord3D], on_exhausted: str = "loiter_last"):
        if on_exhausted not in ("loiter_last", "none"):
            raise ValueError(f"on_exhausted debe ser 'loiter_last' o 'none', no {on_exhausted!r}")
        self._route = route
        self._on_exhausted = on_exhausted
        self._exhausted = False
        self._current: Coord3D | None = None
        self._lookahead: Coord3D | None = None

    def next(self) -> Coord3D | None:
        if self._exhausted:
            return self._current if self._on_exhausted == "loiter_last" else None

        if self._lookahead is not None:
            self._current = self._lookahead
        else:
            try:
                self._current = next(self._route)
            except StopIteration:
                self._exhausted = True
                return self._current if self._on_exhausted == "loiter_last" else None

        try:
            self._lookahead = next(self._route)
        except StopIteration:
            self._lookahead = None

        return self._current

    def peek_next(self) -> Coord3D | None:
        """Punto DESPUÉS del actual (el que devolvió el último next()), o
        None si no hay pierna siguiente (ruta finita agotándose)."""
        return self._lookahead


class PlantAdapter(ABC):
    """Interfaz que plant_driver.py necesita de cualquier planta 6-DoF."""

    @abstractmethod
    def reset(self, north0: float, east0: float, down0: float, psi0: float, ias0: float) -> None:
        """Condiciones iniciales, en el mismo marco NED que el resto del
        sistema (offset desde el origen SACOLocalFrame vigente)."""

    @abstractmethod
    def step(self, dt: float, actuator: ActuatorCommand) -> PlantState:
        """Avanza un paso de integración con el actuador dado (el mismo
        que devolvió el último KUNTUR_COMMAND válido) y devuelve el
        estado resultante, ya en las unidades/ejes del ICD."""

    @abstractmethod
    def aux_telemetry(self) -> AuxTelemetry:
        """Devuelve el AuxTelemetry (ver protocol.py) correspondiente al
        ÚLTIMO step() ya ejecutado -- NO dispara un nuevo paso de
        integración. plant_driver.py solo la llama en los pasos
        decimados que efectivamente se envían por el canal ICD auxiliar
        (ver icd_schema/kuntur_ic_aux_v1.json)."""

    def retask(
        self, seq: int, lat_deg: float, lon_deg: float,
        frame_origin_lat_deg: float, frame_origin_lon_deg: float,
    ) -> str:
        """Redirección a un punto fuera de ruta (canal KUNTUR_CTL, ver
        icd_schema/kuntur_ic_ctl_v1.json). Default NO abstracto: una
        planta que no sabe de retasking (MockFalcon6Adapter) simplemente
        lo ignora, sin tener que implementar nada de geodesia/guiado."""
        return "ignored_mock"

    def replace_route(
        self, seq: int, waypoint_count: int, on_exhausted: str,
        lat_degs: list[float], lon_degs: list[float], alt_msl_degs: list[float],
        frame_origin_lat_deg: float, frame_origin_lon_deg: float,
    ) -> str:
        """Reemplazo de la ruta completa (canal KUNTUR_CTL). Default NO
        abstracto, igual razón que retask()."""
        return "ignored_mock"

    def reset_ctl_dedup(self) -> None:
        """Reinicia la deduplicación de seq de retask()/replace_route() --
        llamado por plant_driver.py cuando CtlLink detecta una conexión
        TCP nueva (seq es monótono POR CONEXIÓN, no por vida del proceso
        planta, ver icd_schema/kuntur_ic_ctl_v1.json). Default no-op (Mock
        no trackea nada de esto)."""

    def set_manual_mode(self, active: bool) -> None:
        """Activa/desactiva el bypass de Manual (ver
        plan-hil-stm32-portado-autopiloto.md §8 y
        spec-control-manual-joystick-flysky-fsi6.md §3.3) -- llamado por
        plant_driver.py al recibir un KUNTUR_MODE fresco (ver ModeLink en
        protocol.py). Default no-op: MockFalcon6Adapter ya consume
        `actuator` directo en su física trivial, no necesita ningún
        flag."""

    def set_hil_mode(self, active: bool) -> None:
        """Activa/desactiva el bypass de HIL (STM32, ver
        plan-hil-stm32-portado-autopiloto.md §8) -- decisión de
        LANZAMIENTO (si plant_driver.py arrancó con --stm32-port), no un
        toggle en vivo de Kuntur-GCS (eso es set_manual_mode(), canal
        KUNTUR_MODE). Default no-op (MockFalcon6Adapter no participa del
        HIL)."""

    def set_autopilot_decimation(self, n: int) -> None:
        """Decima 'guidance'/'control'/'actuator' a 1/n del int_step
        nativo (Fase 2 del plan §5, SIL puro sin STM32) -- ver
        implementación real en RealFalcon6Adapter. Default no-op
        (MockFalcon6Adapter no dispatcha módulos de FALCON6, no hay nada
        que decimar)."""

    def apply_stm32_surface_cmd(
        self, delax_rad: float, delex_rad: float, delrx_rad: float, throttle: float,
    ) -> None:
        """Registra la última respuesta KUNTUR_SURFACE_CMD recibida del
        STM32 (ver kuntur_bridge/stm32_bridge.py), con timestamp de
        recepción para el chequeo de frescura del bypass (ver
        _apply_hil_bypass() en RealFalcon6Adapter). Default no-op."""

    def autopilot_inputs(self) -> AutopilotInputs:
        """Derivadas + waypoint activo para empaquetar KUNTUR_PLANT_STATE
        (ver AutopilotInputs en protocol.py) -- solo tiene sentido con
        modo HIL activo. Default: NotImplementedError (MockFalcon6Adapter
        no tiene estos conceptos; stm32_bridge.py no debería llamar esto
        si set_hil_mode() nunca se activó)."""
        raise NotImplementedError(
            f"{type(self).__name__} no implementa autopilot_inputs() -- "
            "solo tiene sentido con RealFalcon6Adapter + modo HIL activo."
        )


class MockFalcon6Adapter(PlantAdapter):
    """Física de reemplazo DELIBERADAMENTE simple -- un modelo cinemático
    de punto (vuelo recto y nivelado con primer orden hacia los comandos
    de actuador), NO una física de FALCON6. Sirve exclusivamente para
    ejercitar el protocolo ICD, el rendezvous con KSE, la geodesia y el
    feed de FlightGear de punta a punta, sin depender de tener ya el
    código fuente real. No usar esto para ninguna conclusión sobre
    fidelidad de vuelo -- eso es exactamente lo que Fase de validación
    (con la planta real) tiene que medir.
    """

    def __init__(self, ias_trim: float = 60.0):
        self.ias_trim = ias_trim
        self.north = 0.0
        self.east = 0.0
        self.down = -600.0
        self.psi = 0.0
        self.phi = 0.0
        self.theta = 0.02
        self.p = self.q = self.r = 0.0
        self.ias = ias_trim
        self._t = 0.0
        self._last_aux: AuxTelemetry | None = None

    def reset(self, north0: float, east0: float, down0: float, psi0: float, ias0: float) -> None:
        self.north, self.east, self.down = north0, east0, down0
        self.psi = psi0
        self.ias = ias0
        self.phi = self.theta = 0.0
        self.p = self.q = self.r = 0.0
        self._t = 0.0
        self._last_aux = None

    def step(self, dt: float, actuator: ActuatorCommand) -> PlantState:
        # Primer orden trivial hacia los comandos -- NO es la dinámica de
        # actuador de 2do orden real de FALCON6 (ver §3.3 del manual).
        phi_cmd = actuator.aileron * math.radians(30.0)
        theta_cmd = 0.02 + actuator.elevator * math.radians(10.0)
        self.phi += (phi_cmd - self.phi) * min(1.0, dt / 0.5)
        self.theta += (theta_cmd - self.theta) * min(1.0, dt / 0.8)
        self.r = actuator.rudder * 0.1
        self.p = (phi_cmd - self.phi) * 0.5
        self.q = (theta_cmd - self.theta) * 0.5

        self.psi += self.r * dt
        self.ias = self.ias_trim * (0.5 + actuator.throttle)

        vn = self.ias * math.cos(self.psi) * math.cos(self.theta)
        ve = self.ias * math.sin(self.psi) * math.cos(self.theta)
        vd = -self.ias * math.sin(self.theta)

        self.north += vn * dt
        self.east += ve * dt
        self.down += vd * dt
        self._t += dt

        # Este mock NO separa fuerza de gravedad en su física trivial (no
        # integra fuerzas, integra cinemática directamente) -- por eso,
        # a diferencia de RealFalcon6Adapter (que copia FSPB ya calculado
        # por cadac_py), acá SÍ hace falta aplicar specific_force_body()
        # explícitamente para producir algo ICD-válido: vuelo recto y
        # nivelado en reposo de actitud da aceleración cinemática ~0, que
        # convertida a fuerza específica debe dar [0,0,-g] -- no [0,0,0].
        ax, ay, az = specific_force_body(0.0, 0.0, 0.0, self.phi, self.theta)

        # Aproximación DELIBERADAMENTE simple para el canal auxiliar,
        # coherente con la física trivial de este mock (ver docstring de
        # clase): no modela motor real (power/thrust son proxies directos
        # del throttle comandado, sin dinámica de spool-up ni relación
        # empuje-potencia), no separa TAS de EAS (pdynmc/vmach asumen
        # ISA nivel del mar, sin viento), y no modela factor de carga
        # real (anx fijo en 1.0, vuelo "ideal"). NO usar para ninguna
        # conclusión de fidelidad -- igual que el resto de este mock.
        RHO0_ISA = 1.225   # kg/m^3, nivel del mar
        A0_ISA = 340.29    # m/s, velocidad del sonido a nivel del mar
        self._last_aux = AuxTelemetry(
            dvbe=self.ias, vmach=self.ias / A0_ISA,
            pdynmc=0.5 * RHO0_ISA * self.ias * self.ias,
            hbe=-self.down, altrate=-vd,
            alppx=math.degrees(self.theta * 0.3), betax=0.0, anx=1.0,
            throttle=actuator.throttle, power=actuator.throttle * 100.0,
            thrust=0.0,
        )

        return PlantState(
            north=self.north, east=self.east, down=self.down,
            phi=self.phi, theta=self.theta, psi=self.psi,
            p=self.p, q=self.q, r=self.r,
            ax=ax, ay=ay, az=az,
            vn=vn, ve=ve, vd=vd,
            ias=self.ias, alpha=self.theta * 0.3, beta=0.0,
            status=0,
        )

    def aux_telemetry(self) -> AuxTelemetry:
        if self._last_aux is None:
            raise RuntimeError("MockFalcon6Adapter.aux_telemetry() llamado antes de step()")
        return self._last_aux


class RealFalcon6Adapter(PlantAdapter):
    """Planta real: cadac_py/falcon6.py::Plane completo (física + control +
    guiado), re-conducido paso a paso con las mismas funciones que usa
    execution.run_scenario() -- ver el docstring de módulo para el porqué
    de mantener control/guiado adentro, y el mapeo de índices flat6[].

    Se construye a partir de un escenario *.asc COMPLETO y ya validado
    (por default el que trae cadac_py, f16c11_7.asc: 3 waypoints + IP con
    glide slope, ver CADAC/FALCON6/f16c11_7.asc) en vez de armar los ~40
    parámetros de aerodinámica/actuador/autopiloto/guiado a mano -- eso es
    exactamente "reutilizar lo que ya está desarrollado" para lo que no
    cambia (ganancias de control, límites de superficies, decks
    aerodinámicos) y tocar SOLO lo que el usuario pidió cambiar: dónde
    están los waypoints (ver `waypoints` más abajo) y la condición inicial
    (ver `reset()`).

    Sustitución de waypoints (real, no simulada):
    ----------------------------------------------
    Si se pasa `waypoints` (lista de kuntur_bridge.falcon_fcs.Waypoint, en
    NED -- ver kuntur_bridge/waypoints_file.py::load_waypoints_list()),
    se reemplazan EN EL PARSEO los valores swel1/swel2/swel3 del
    waypoint inicial (asignación incondicional) y de cada bloque
    IF wp_flag=-1 ENDIF subsiguiente, EN ORDEN, dejando psiflx/thtflx/
    altcom/mguid/maut/halt tal cual los definió el escenario original
    (`capture_radius_m` de Waypoint no se usa acá -- wp_flag lo calcula
    guidance() internamente por geometría de radio de giro, no por un
    radio de captura externo; ver docstring de módulo). La cantidad de
    waypoints en `waypoints` debe ser EXACTAMENTE 1 + (cantidad de
    bloques IF..ENDIF que asignan swel1 en el escenario) -- si no
    coincide, se rechaza en vez de truncar/completar en silencio.
    """

    def __init__(self, asc_path: str, waypoints: list | None = None):
        self._asc_path = asc_path
        self._base_dir = os.path.dirname(os.path.abspath(asc_path))
        self._parsed = parse_input_asc(asc_path)

        if len(self._parsed.vehicles) != 1 or self._parsed.vehicles[0].vtype != "PLANE6":
            raise ValueError(
                f"{asc_path}: RealFalcon6Adapter espera exactamente 1 vehículo PLANE6 "
                f"(encontrado: {[v.vtype for v in self._parsed.vehicles]})"
            )
        self._vspec_template = self._parsed.vehicles[0]

        if waypoints is not None:
            self._vspec_template = self._override_waypoints(self._vspec_template, waypoints)

        # Patrulla en bucle -- ver docstring de step()/_advance_route_waypoint().
        # Solo se activa si el usuario pasó waypoints propios: sin esto, el
        # comportamiento con el *.asc original (sin override) queda IDÉNTICO
        # a antes -- no se toca nada de lo que no cambia. Se guarda la lista
        # ORIGINAL (no el RouteWaypointSource ya armado): _build() arma un
        # RouteWaypointSource/itertools.cycle() NUEVO en cada reset(), para
        # que cada reinicio de episodio vuelva a patrullar desde el primer
        # waypoint -- un itertools.cycle() ya usado NO reinicia su posición
        # solo, hay que crear uno nuevo (ver _build()).
        self._initial_route_coords: list[Coord3D] | None = (
            [(wp.north, wp.east, wp.down) for wp in waypoints] if waypoints else None
        )
        self._route_source: RouteWaypointSource | None = None
        self._route_prev_wp_flag: int = 1

        # Canal KUNTUR_CTL (retasking / reemplazo de ruta) -- ver
        # SPEC-retasking_backend.md §3/§4.1. self._frame se inyecta recién
        # con set_frame(), no hay forma de derivarlo del *.asc.
        self._frame: SACOLocalFrame | None = None
        self._last_retask_seq: int = -1
        self._last_route_replace_seq: int = -1

        self.int_step_s: float = self._parsed.timing["int_step"]
        self._exec_modules = [m for m in self._parsed.modules if m.execution == "exec"]

        self.vehicle = None
        self._events = None
        self._packet = None
        self._sim_time = 0.0
        self._last_aux: AuxTelemetry | None = None

        # Bypass de Manual (ver set_manual_mode() / _apply_manual_bypass()
        # / step()) -- apagado por default: con esto en False, step()
        # queda IDÉNTICO al camino nativo (ver gate de bit-exactitud).
        self._manual_mode_active: bool = False

        # Bypass de HIL (ver set_hil_mode() / apply_stm32_surface_cmd() /
        # _apply_hil_bypass() / step()) -- mismo criterio: apagado por
        # default, step() queda idéntico al camino nativo.
        self._hil_mode_active: bool = False
        self._last_stm32_surface_cmd: _Stm32SurfaceCmd | None = None

        # Decimación del autopiloto en SIL puro (ver set_autopilot_decimation()
        # / step(), Fase 2 del plan-hil-stm32-portado-autopiloto.md §5) --
        # default 1 (todos los pasos son "tick" de autopiloto): step() queda
        # IDÉNTICO al camino nativo, mismo criterio que manual/HIL arriba.
        self._autopilot_decimation: int = 1
        self._autopilot_tick: int = 0
        self._autopilot_prev_cmd: tuple[float, float, float, float] | None = None
        self._autopilot_target_cmd: tuple[float, float, float, float] | None = None

    @staticmethod
    def _override_waypoints(vspec, waypoints: list):
        """Devuelve una VehicleSpec nueva (deepcopy) con swel1/swel2/swel3
        reemplazados por `waypoints`, en el orden: [asignación
        incondicional] + [assignments de cada evento IF..ENDIF, en orden
        de aparición]. Falla fuerte si la cantidad no coincide -- ver
        docstring de clase ("nada implícito")."""
        vspec = deepcopy(vspec)
        slots = [vspec.assignments] + [ev["assignments"] for ev in vspec.events]
        if len(waypoints) != len(slots):
            raise ValueError(
                f"RealFalcon6Adapter: {len(waypoints)} waypoints provistos, pero el "
                f"escenario define {len(slots)} posiciones (1 incondicional + "
                f"{len(slots) - 1} bloque(s) IF..ENDIF) -- deben coincidir exactamente."
            )

        def _set(assignments: list, name: str, value: float) -> list:
            out = [(n, v) for n, v in assignments if n != name]
            out.append((name, value))
            return out

        for slot, wp in zip(slots, waypoints):
            slot[:] = _set(slot, "swel1", wp.north)
            slot[:] = _set(slot, "swel2", wp.east)
            slot[:] = _set(slot, "swel3", wp.down)
        return vspec

    def _build(self) -> None:
        """(Re)construye Plane desde cero con la MISMA secuencia que usa
        execution.run_scenario() -- build_vehicle (def_xxx en orden) ->
        apply_vehicle_data (asignaciones + AERO_DECK/PROP_DECK) ->
        init_xxx (en orden) -> build_events. No se reimplementa nada de
        esto a mano, para no arriesgar una divergencia respecto del batch
        ya validado (regression_vs_batch.py)."""
        self.vehicle = build_vehicle("PLANE6", self._parsed.modules)
        apply_vehicle_data(self.vehicle, self._vspec_template.assignments, self._base_dir)
        for m in self._parsed.modules:
            if m.initialization == "init":
                fn = getattr(self.vehicle, f"init_{m.name}", None)
                if fn is not None:
                    fn()
        self._events = build_events(self.vehicle, self._vspec_template.events)
        self._packet = make_packet(self.vehicle, "p1")
        self._sim_time = 0.0

        # Al reconstruir (reset()), el vehículo arranca targeteando el
        # PRIMER waypoint (asignación incondicional del *.asc); los eventos
        # IF..ENDIF nativos lo van llevando por el resto EN ORDEN a medida
        # que corre. Se arma un RouteWaypointSource NUEVO (sobre un
        # itertools.cycle() nuevo) cada vez: recién cuando el bucle propio
        # dispare su primer avance (una vez que la cola nativa de eventos
        # se agotó tras el waypoint final -- ver _advance_route_waypoint()),
        # el primer next() de un cycle() recién creado da el PRIMER
        # waypoint -- el patrullaje se cierra ahí, igual que antes.
        # wp_flag se rearma en "aproximándose" (1) para no confundir un
        # arranque fresco con un cruce real de waypoint.
        if self._initial_route_coords is not None:
            self._route_source = RouteWaypointSource(itertools.cycle(self._initial_route_coords))
            self._route_prev_wp_flag = 1

        # Reinicio del contador de decimación del autopiloto -- cada episodio
        # nuevo arranca en tick de autopiloto (igual que KUNTUR_PLANT_STATE
        # con flags.init=1 en el HIL real, ver plan §4).
        self._autopilot_tick = 0
        self._autopilot_prev_cmd = None
        self._autopilot_target_cmd = None

    def reset(self, north0: float, east0: float, down0: float, psi0: float, ias0: float) -> None:
        """Sobrescribe EXACTAMENTE las 5 condiciones iniciales que la
        interfaz PlantAdapter provee (sbel1/sbel2/sbel3/psiblx/dvbe) y
        reconstruye Plane desde cero con ellas. El resto de las
        condiciones iniciales del escenario (thtblx/phiblx/alpha0x/
        beta0x -- actitud/AoA de trimado -- y todos los parámetros de
        aerodinámica/actuador/autopiloto/guiado) quedan tal cual el
        *.asc de origen, porque la interfaz de plant_driver.py no tiene
        de dónde tomar otro valor para ellos y el escenario ya trae uno
        consistente con la velocidad/actitud de crucero validada."""
        vspec = deepcopy(self._vspec_template)

        def _set(name: str, value: float) -> None:
            vspec.assignments = [(n, v) for n, v in vspec.assignments if n != name]
            vspec.assignments.append((name, value))

        _set("sbel1", north0)
        _set("sbel2", east0)
        _set("sbel3", down0)
        _set("psiblx", math.degrees(psi0))
        _set("dvbe", ias0)

        self._vspec_template = vspec
        self._build()

    def step(self, dt: float, actuator: ActuatorCommand) -> PlantState:
        """Avanza un paso re-conduciendo _EXEC_DISPATCH en el mismo orden
        que run_scenario() (environment/kinematics/aerodynamics/
        propulsion/forces/guidance/control/actuator/euler/newton, según
        f16c11_7.asc). `actuator` se recibe para cumplir la interfaz
        PlantAdapter pero se IGNORA deliberadamente: control()/guidance()
        de FALCON6 gobiernan las superficies internamente a partir de sus
        propios objetivos (altcom/swel/psiflx/...), no de un comando
        externo -- ver docstring de módulo.

        `dt` DEBE coincidir con self.int_step_s (el int_step del *.asc de
        origen, 0.001s para f16c11_7.asc): integrar a un paso distinto
        del validado en batch no tiene respaldo empírico, así que se
        rechaza en vez de integrar "a ver qué pasa".

        Modo Manual (self._manual_mode_active, ver set_manual_mode()):
        `actuator` deja de ignorarse -- se aplica un bypass entre el
        despacho de 'control' y el de 'actuator' (ver
        _apply_manual_bypass()).

        Modo HIL (self._hil_mode_active, ver set_hil_mode()): si hay una
        respuesta KUNTUR_SURFACE_CMD reciente (< HIL_SURFACE_CMD_MAX_AGE_S,
        ver apply_stm32_surface_cmd()), el despacho nativo de 'actuator'
        se SALTA entero ese paso y se escribe directo lo que calculó el
        STM32 -- que ya incluye su propia dinámica de actuador (ver
        _apply_hil_bypass()). Si no hay respuesta fresca, 'actuator'
        corre nativo como siempre, con lo que 'control' (nativo, ya
        corrido) escribió: fallback silencioso, sin estado especial que
        mantener.

        Decimación de autopiloto en SIL puro (self._autopilot_decimation,
        ver set_autopilot_decimation()): fuera de los ticks de autopiloto,
        'guidance'/'control'/'actuator' NO se despachan -- sus salidas
        quedan en plane[] tal cual el último tick (hold implícito, sin
        estado extra). Con n=1 (default) nunca se activa.

        Con todos los flags en default (manual/HIL off, decimación=1),
        este método es bit-a-bit idéntico al camino nativo descripto
        arriba.
        """
        if self.vehicle is None:
            raise RuntimeError("RealFalcon6Adapter.step() llamado antes de reset()")
        if abs(dt - self.int_step_s) > 1e-9:
            raise ValueError(
                f"RealFalcon6Adapter: dt={dt}s no coincide con el int_step del "
                f"escenario ({self.int_step_s}s) -- ver docstring de step()."
            )

        is_autopilot_tick = (self._autopilot_tick % self._autopilot_decimation) == 0
        self._autopilot_tick += 1

        check_events(self._events)
        if self._packet.get_status() == 1:
            plane = self.vehicle.plane
            for m in self._exec_modules:
                # HIL: el STM32 porta actuator()/actuator_scnd() además del
                # autopiloto (ver plan §2/§3) -- su respuesta ya es la
                # deflexión REAL post-dinámica de actuador, así que acá se
                # SALTA el despacho nativo de 'actuator' entero (no solo se
                # pisan sus entradas, como con Manual). Si no hay respuesta
                # fresca, no se salta nada: el 'actuator' nativo corre con
                # lo que 'control' (nativo, ya corrido) escribió -- fallback
                # silencioso sin rama especial.
                if (
                    m.name == "actuator"
                    and self._hil_mode_active
                    and not self._manual_mode_active
                    and self._hil_cmd_is_fresh()
                ):
                    self._apply_hil_bypass(plane)
                    continue
                # Fase 2 (SIL puro, ver set_autopilot_decimation()): fuera de
                # un tick de autopiloto, no despachar estos tres módulos deja
                # sus salidas en plane[] tal cual quedaron el último tick.
                # Para mitigar la inestabilidad numérica del ZOH puro, se
                # reemplaza el "hold" por una interpolación lineal del comando
                # entre el último valor actualizado y el siguiente objetivo.
                if (
                    m.name in ("guidance", "control", "actuator")
                    and not is_autopilot_tick
                ):
                    continue
                fn = _EXEC_DISPATCH.get(m.name)
                if fn is not None:
                    fn(self.vehicle, self._sim_time, dt, [self._packet], 0, self._parsed.title)
                if m.name == "control" and self._manual_mode_active:
                    self._apply_manual_bypass(plane, actuator)
            if (
                self._autopilot_decimation > 1
                and not self._manual_mode_active
                and not self._hil_mode_active
            ):
                if is_autopilot_tick:
                    prev = self._autopilot_target_cmd
                    target = self._read_autopilot_cmd(plane)
                    self._autopilot_prev_cmd = prev if prev is not None else target
                    self._autopilot_target_cmd = target
                    self._apply_autopilot_decimation(plane, 0)
                elif self._autopilot_prev_cmd is not None and self._autopilot_target_cmd is not None:
                    # Fix: self._autopilot_tick ya fue incrementado en la
                    # línea de arriba (step()), así que hay que restar 1
                    # para obtener la fase real dentro del intervalo de
                    # decimación -- sin el -1, la fase quedaba corrida un
                    # paso (saltaba alpha=0.1 y reemplazaba alpha=0.9 por
                    # alpha=0.0 en el último paso interior de cada ciclo,
                    # ver validación Fase 2).
                    self._apply_autopilot_decimation(plane, (self._autopilot_tick - 1) % self._autopilot_decimation)
            status_snapshot = self._packet.get_status()
            self._packet = make_packet(self.vehicle, self._packet.get_id())
            self._packet.set_status(status_snapshot)
        self._sim_time += dt

        if self._route_source is not None:
            self._advance_route_waypoint()

        f = self.vehicle.flat6
        p = self.vehicle.plane
        SBEL = f[219].vec()
        WBEB = f[164].vec()
        FSPB = f[230].vec()
        VBEL = f[233].vec()

        # Canal ICD auxiliar de instrumentación (ver protocol.AuxTelemetry
        # y icd_schema/kuntur_ic_aux_v1.json) -- índices verificados
        # leyendo cadac_py/falcon6.py línea por línea (ver docstring de
        # módulo y el plan de instrumentación), NO asertados desde el
        # manual. alppx/betax quedan en GRADOS (crudo de flat6), igual
        # que el schema del canal auxiliar -- a diferencia de alpha/beta
        # de PlantState arriba, que SÍ se convierten a radianes para el
        # contrato principal.
        self._last_aux = AuxTelemetry(
            dvbe=f[236].real(), vmach=f[56].real(), pdynmc=f[57].real(),
            hbe=f[239].real(), altrate=p[530].real(),
            alppx=f[140].real(), betax=f[145].real(), anx=f[244].real(),
            throttle=p[52].real(), power=p[59].real(), thrust=p[55].real(),
        )

        return PlantState(
            north=SBEL[0], east=SBEL[1], down=SBEL[2],
            phi=f[136].real(), theta=f[135].real(), psi=f[134].real(),
            p=WBEB[0], q=WBEB[1], r=WBEB[2],
            ax=FSPB[0], ay=FSPB[1], az=FSPB[2],
            vn=VBEL[0], ve=VBEL[1], vd=VBEL[2],
            ias=f[75].real(),
            alpha=math.radians(f[144].real()),
            beta=math.radians(f[145].real()),
            status=self._packet.get_status(),
        )

    def aux_telemetry(self) -> AuxTelemetry:
        if self._last_aux is None:
            raise RuntimeError("RealFalcon6Adapter.aux_telemetry() llamado antes de step()")
        return self._last_aux

    def set_manual_mode(self, active: bool) -> None:
        if active != self._manual_mode_active:
            log.info(f"RealFalcon6Adapter: modo manual {'activado' if active else 'desactivado'}")
        self._manual_mode_active = active

    def _apply_manual_bypass(self, plane, actuator: ActuatorCommand) -> None:
        """Bypass de Manual -- ver spec-control-manual-joystick-flysky-fsi6.md
        §3.3 y plan-hil-stm32-portado-autopiloto.md §8. Se llama desde
        step() justo DESPUÉS de que 'control' corrió (que ya escribió
        delacx/delecx/delrcx = plane[519/520/521] con el autopiloto
        nativo) y ANTES de que 'actuator' los lea (actuator() los toma al
        principio de su propio cuerpo, ver falcon6.py::actuator()) --
        acá se pisan con lo que llegó por KUNTUR_COMMAND. Sin ninguna
        asistencia de estabilidad: control directo de superficies, igual
        que especifica el spec de FS-i6.

        actuator.aileron/elevator/rudder llegan ND en [-1,1]
        (icd_schema/kuntur_ic_v1_3.json) -- se mapean linealmente a
        [-límite,+límite] usando los límites REALES del vehículo
        (dalimx/delimx/drlimx = plane[508/509/510], en grados, no
        valores arbitrarios)."""
        dalimx = plane[508].real()
        delimx = plane[509].real()
        drlimx = plane[510].real()
        plane[519].gets(_map_unit_range(actuator.aileron, dalimx))
        plane[520].gets(_map_unit_range(actuator.elevator, delimx))
        plane[521].gets(_map_unit_range(actuator.rudder, drlimx))
        # throttle (plane[52]) solo tiene efecto si mprop (plane[50]) == 1
        # -- ver falcon6.py::propulsion(). No se fuerza mprop acá: si el
        # escenario no está en modo throttle manual, el comando de
        # throttle del joystick queda sin efecto -- diagnosticable (el
        # throttle no responde), no un fallo silencioso.
        plane[52].gets(max(0.0, min(1.0, actuator.throttle)))

    def _read_autopilot_cmd(self, plane) -> tuple[float, float, float, float]:
        """Comando de control del autopiloto en la misma frontera que usa
        cadac_py::actuator() (plane[519/520/521] + throttle). Se usa para
        interpolar linealmente en lugar de mantener un ZOH puro entre ticks
        de autopiloto.

        Es importante leer las salidas de 'control' (los comandos de
        superficie) y no solo las salidas ya filtradas de 'actuator':
        durante los pasos desactivados el adaptador salta 'actuator' y la
        siguiente vez que 'actuator' vuelva a correr, volverá a leer los
        valores de plane[519/520/521]. Mantener solo delax/delex/delrx
        suavizados dejaba la cadena interna inconsistente y seguía
        produciendo saltos discretos en el lazo de control.
        """
        return (plane[519].real(), plane[520].real(), plane[521].real(), plane[52].real())

    def _write_autopilot_cmd(self, plane, cmd: tuple[float, float, float, float]) -> None:
        # Reescribimos la semilla que consume el siguiente actuator() (raw
        # control command) y, además, la salida ya filtrada del actuator
        # para mantener la cadena completa de estados coherente con la
        # rampa interpolada.
        plane[519].gets(cmd[0])
        plane[520].gets(cmd[1])
        plane[521].gets(cmd[2])
        plane[619].gets(cmd[0])
        plane[620].gets(cmd[1])
        plane[621].gets(cmd[2])
        plane[52].gets(cmd[3])

    def _apply_autopilot_decimation(self, plane, phase: int) -> None:
        """Interpolación lineal del comando del autopiloto durante el
        intervalo de decimación: u(k) = u_prev + (k/n) * (u_target - u_prev).
        El adaptador aplica la rampa sobre la salida de 'control' y la de
        'actuator' para que las dos cadenas internas (comando y deflexión
        real) no queden desacomodadas durante todo el intervalo decimado."""
        if self._autopilot_prev_cmd is None or self._autopilot_target_cmd is None:
            return
        prev = self._autopilot_prev_cmd
        target = self._autopilot_target_cmd
        alpha = phase / self._autopilot_decimation
        cmd = tuple(
            prev[i] + alpha * (target[i] - prev[i])
            for i in range(4)
        )
        self._write_autopilot_cmd(plane, cmd)

    def set_autopilot_decimation(self, n: int) -> None:
        """Fase 2 del plan (plan-hil-stm32-portado-autopiloto.md §5):
        decima 'guidance'/'control'/'actuator' a 1/n del int_step nativo,
        reemplazando el ZOH puro por interpolación lineal del comando del
        autopiloto para mitigar la inestabilidad numérica observada.

        n=1 (default) dispatcha los tres módulos TODOS los pasos --
        step() queda bit-a-bit idéntico al camino nativo, mismo criterio
        que manual/HIL. n>1 es exclusivamente para comparar contra el SIL
        nativo (Fase 2) -- no se combina con HIL real
        (self._hil_mode_active): ese modo decima por su cuenta vía
        plant_driver.py/stm32_bridge.py, con su propio mecanismo de salida."""
        if n < 1:
            raise ValueError(f"set_autopilot_decimation: n={n} debe ser >= 1")
        if n != self._autopilot_decimation:
            rate_desc = "nativo, 1kHz" if n == 1 else f"{1.0 / (n * self.int_step_s):.1f}Hz"
            log.info(f"RealFalcon6Adapter: decimación de autopiloto n={n} ({rate_desc})")
        self._autopilot_decimation = n
        self._autopilot_prev_cmd = None
        self._autopilot_target_cmd = None

    def set_hil_mode(self, active: bool) -> None:
        if active != self._hil_mode_active:
            log.info(f"RealFalcon6Adapter: modo HIL {'activado' if active else 'desactivado'}")
        self._hil_mode_active = active
        if not active:
            self._last_stm32_surface_cmd = None

    def apply_stm32_surface_cmd(
        self, delax_rad: float, delex_rad: float, delrx_rad: float, throttle: float,
    ) -> None:
        """Registra la última respuesta KUNTUR_SURFACE_CMD -- delax/delex/
        delrx son la deflexión YA APLICADA (post actuator()/actuator_scnd()
        del lado STM32, ver icd_schema/kuntur_ic_surface_cmd_v1.json), NO
        un comando crudo de 'control' todavía sin dinámica de actuador."""
        self._last_stm32_surface_cmd = _Stm32SurfaceCmd(
            delax_rad=delax_rad, delex_rad=delex_rad, delrx_rad=delrx_rad,
            throttle=throttle, received_at_s=time.monotonic(),
        )

    def _hil_cmd_is_fresh(self) -> bool:
        cmd = self._last_stm32_surface_cmd
        return cmd is not None and (time.monotonic() - cmd.received_at_s) < HIL_SURFACE_CMD_MAX_AGE_S

    def _apply_hil_bypass(self, plane) -> None:
        """Bypass de HIL -- ver plan-hil-stm32-portado-autopiloto.md §8.
        Llamado en vez de despachar 'actuator' (no entre 'control' y
        'actuator' como el bypass de Manual): el STM32 ya corrió su propia
        copia de actuator()/actuator_scnd(), así que esto escribe
        directo en plane[619/620/621] (delax/delex/delrx -- las SALIDAS de
        actuator(), no sus entradas) y el dispatch nativo de 'actuator' se
        saltea entero para ese paso (ver step()) -- aplicarlo también
        sería una segunda dinámica de actuador encima de la que ya vino
        del STM32. Solo se llama cuando el caller (step()) ya confirmó
        frescura vía _hil_cmd_is_fresh()."""
        cmd = self._last_stm32_surface_cmd
        dlimx = plane[602].real()  # "Control fin limiter" -- mismo límite único para los 3 ejes en actuator()
        plane[619].gets(_clip_to_limit(math.degrees(cmd.delax_rad), dlimx))
        plane[620].gets(_clip_to_limit(math.degrees(cmd.delex_rad), dlimx))
        plane[621].gets(_clip_to_limit(math.degrees(cmd.delrx_rad), dlimx))
        plane[52].gets(max(0.0, min(1.0, cmd.throttle)))

    def autopilot_inputs(self) -> AutopilotInputs:
        """Ver AutopilotInputs (protocol.py) -- índices relevados contra
        cadac_py/falcon6.py, no asumidos del plan (ver docstring de la
        dataclass)."""
        p = self.vehicle.plane
        f = self.vehicle.flat6
        return AutopilotInputs(
            grav=f[55].real(),
            dla=p[145].real(), dlde=p[146].real(), dma=p[147].real(),
            dmq=p[148].real(), dmde=p[149].real(),
            dyb=p[150].real(), dydr=p[151].real(), dnb=p[152].real(),
            dnr=p[153].real(), dndr=p[154].real(),
            dllp=p[155].real(), dllda=p[156].real(),
            gmax=p[163].real(), gminx=p[167].real(),
            swel1=p[405].real(), swel2=p[406].real(), swel3=p[407].real(),
            psiflx_deg=p[408].real(), thtflx_deg=p[409].real(), altcom=p[527].real(),
            mguid=p[400].integer(), maut=p[500].integer(),
        )

    def _bearing_to(self, from_ne: tuple[float, float], to_ne: tuple[float, float]) -> float:
        """Rumbo geométrico (grados) de from_ne hacia to_ne, en el plano
        NED -- extraído del atan2 que antes vivía inline en
        _advance_loop_waypoint(), para reusarlo también desde retask()."""
        d_north = to_ne[0] - from_ne[0]
        d_east = to_ne[1] - from_ne[1]
        return math.degrees(math.atan2(d_east, d_north))

    def _apply_target(
        self, north: float, east: float, down: float, psiflx_deg: float, altcom: float,
        mguid: int = 30, maut: int = 35,
    ) -> None:
        """Fija el objetivo de guiado (swel1/2/3 + psiflx + altcom) y el
        modo "en ruta" (mguid=30/maut=35 por default) -- extraído de
        _advance_loop_waypoint() para que tanto el seguimiento normal de
        ruta como retask() usen EXACTAMENTE el mismo código, sin duplicar
        los índices p[...] ni arriesgar que diverjan con el tiempo.

        Por qué mguid/maut importan (no es cosmético, bug encontrado en
        vuelo real en la versión original de este método): el último
        bloque IF..ENDIF de f16c11_7.asc (el que arma el IP) no solo mueve
        el waypoint -- también cambia a mguid=33/maut=33 (glide-slope 3D +
        picada dirigida hacia el ancomx de guidance_line(), apropiado SOLO
        para el impacto final que el escenario original preveía). Sin
        restaurar mguid=30/maut=35 ("en ruta", con altitude-hold real vía
        altcom) en cada reenganche, cada vuelta del bucle terminaba en modo
        picada contra el waypoint nuevo -- "cada vuelta se entierra más"."""
        p = self.vehicle.plane
        p[405].gets(north)
        p[406].gets(east)
        p[407].gets(down)
        p[408].gets(psiflx_deg)
        p[400].gets(mguid)
        p[500].gets(maut)
        p[527].gets(altcom)

    def _advance_route_waypoint(self) -> None:
        """Patrulla en ruta -- reinyecta el siguiente punto de
        self._route_source como nuevo objetivo apenas se agota la cola
        nativa de eventos IF..ENDIF del *.asc (es decir, apenas se cruza
        el último waypoint real, el "IP"). Reemplaza al mecanismo cíclico
        anterior (_advance_loop_waypoint(), indexado manual sobre una
        lista fija) por uno basado en RouteWaypointSource, que admite
        rutas de longitud arbitraria y reemplazo en caliente
        (set_route_source(), ver replace_route()) -- SPEC-retasking_backend.md §2.

        Por qué hace falta esto (no es cosmético): guidance_line() en
        falcon6.py corrige la trayectoria hacia swel1/2/3 en TODO paso, sin
        condición de "misión cumplida" -- el *.asc original de CADAC nunca
        previó qué hacer después del IP, porque en el escenario original el
        vehículo estaba pensado para terminar (halt=1) ahí. En este fork
        `execution.py` no actúa sobre halt (comportamiento nativo, fuera de
        alcance tocar cadac_py -- ver FORK_CADAC_SIL.md), así que sin esto
        guidance_line() sigue corrigiendo indefinidamente hacia un punto
        que queda atrás de la aeronave -- divergencia confirmada en vuelo
        real (theta cada vez más picado, terminando en maniobras erráticas
        y entrada en tierra). Reinyectar un waypoint válido le da a
        guidance_line() un objetivo POR DELANTE otra vez.

        Detección "cola de eventos agotada": los N-1 eventos nativos ya
        cubren las transiciones waypoint 1->2->...->N (ver
        _override_waypoints -- N waypoints requieren exactamente N-1
        bloques IF..ENDIF). Una vez que TODOS dispararon ("fired"), no
        queda nada nativo que actualice swel -- ahí es donde este método
        toma el control, disparando en el mismo evento (wp_flag=-1) que
        usaría el siguiente bloque IF..ENDIF si existiera, con el mismo
        flanco (no en cada paso mientras wp_flag siga en -1, solo en la
        transición) para no reescribir swel decenas de miles de veces por
        segundo mientras se aleja del punto ya alcanzado.

        psiflx (heading de línea de ataque) se recalcula como el rumbo
        geométrico hacia el waypoint SIGUIENTE (peek_next()) -- igual
        anticipación de pierna que ya hacía el mecanismo cíclico anterior.
        Si no hay pierna siguiente (ruta finita agotándose, ver
        RouteWaypointSource), se deja el último psiflx vigente -- no hay
        geometría nueva que anticipar."""
        p = self.vehicle.plane
        if not all(ev["fired"] for ev in self._events):
            return  # todavía queda algún tramo nativo por recorrer

        wp_flag = p[419].integer()
        if wp_flag == -1 and self._route_prev_wp_flag != -1:
            target = self._route_source.next()
            if target is not None:
                north, east, down = target
                next_leg = self._route_source.peek_next()
                if next_leg is not None:
                    psiflx_deg = self._bearing_to((north, east), (next_leg[0], next_leg[1]))
                else:
                    psiflx_deg = p[408].real()  # sin pierna siguiente: no hay rumbo nuevo que anticipar
                altcom = -down

                self._apply_target(north, east, down, psiflx_deg, altcom)

                log.info(
                    "patrulla en ruta: waypoint alcanzado, reapuntando "
                    "(north=%.1f east=%.1f down=%.1f altcom=%.1f psiflx=%.1fdeg) en t_sim=%.3fs",
                    north, east, down, altcom, psiflx_deg, self._sim_time,
                )
        self._route_prev_wp_flag = wp_flag

    def set_route_source(self, source: RouteWaypointSource) -> None:
        """Reemplaza la ruta activa por reasignación de referencia
        (backlog #11) -- usado por replace_route(). NO toca
        _route_prev_wp_flag: el próximo cruce de wp_flag=-1 (real, por
        geometría) dispara _advance_route_waypoint() normalmente contra
        la ruta nueva."""
        self._route_source = source

    def set_frame(self, frame: SACOLocalFrame) -> None:
        """Inyecta el SACOLocalFrame vigente -- requerido por
        retask()/replace_route() para validar el origen del cliente y
        convertir lat/lon a NED. plant_driver.py lo arma a partir de
        --origin-lat/--origin-lon/--origin-alt y lo pasa acá explícitamente
        (no se deriva del *.asc, que no tiene noción de WGS-84)."""
        self._frame = frame

    def retask(
        self, seq: int, lat_deg: float, lon_deg: float,
        frame_origin_lat_deg: float, frame_origin_lon_deg: float,
    ) -> str:
        """Redirección de emergencia a un punto fuera de ruta -- SPEC-retasking_backend.md
        §3.2/§4.1. NO consume el iterador de la ruta ni muta
        self._route_source: es un desvío aislado, _advance_route_waypoint()
        retoma la ruta normalmente en cuanto la geometría vuelva a cruzar
        wp_flag=-1 contra el punto de retask.

        El chequeo de origen usa frame_origin_lat_deg/frame_origin_lon_deg
        (lo que el CLIENTE cree que es el origen del plano local) -- NO
        lat_deg/lon_deg (el punto objetivo, que en general está lejos del
        origen y nunca pasaría matches()). Confundir ambos pares fue un bug
        real en el primer borrador de este método (ver nota de la sesión
        de corrección del spec) -- corregido acá."""
        if seq <= self._last_retask_seq:
            return "rejected_stale_seq"

        if self._frame is None:
            raise RuntimeError("retask() requiere frame -- ver set_frame()")

        if not self._frame.matches(frame_origin_lat_deg, frame_origin_lon_deg, tol_deg=1e-5):
            return "rejected_origin_mismatch"

        north, east, _ = self._frame.geodetic_to_ned(lat_deg, lon_deg, 0.0)

        if math.hypot(north, east) > 250_000.0:  # límite: 250 km
            return "rejected_out_of_range"

        hbe = self.vehicle.flat6[239].real()  # altura AGL actual -- se mantiene (hold AGL)
        psiflx = self._bearing_to(
            (self.vehicle.flat6[219].vec()[0], self.vehicle.flat6[219].vec()[1]), (north, east)
        )

        self._apply_target(north, east, -hbe, psiflx, altcom=hbe)
        self._route_prev_wp_flag = -1  # rearme de flanco -- soporta puntos a espaldas del vehículo
        self._last_retask_seq = seq

        log.info(
            "retask seq=%d aplicado: north=%.1f east=%.1f altcom_agl=%.1f psiflx=%.1f t_sim=%.3fs",
            seq, north, east, hbe, psiflx, self._sim_time,
        )
        return "applied"

    def replace_route(
        self, seq: int, waypoint_count: int, on_exhausted: str,
        lat_degs: list[float], lon_degs: list[float], alt_msl_degs: list[float],
        frame_origin_lat_deg: float, frame_origin_lon_deg: float,
    ) -> str:
        """Reemplazo de la ruta completa -- SPEC-retasking_backend.md §2/§4.2.
        A diferencia de retask() (mantiene AGL actual, redirección de
        emergencia), una ruta completa SÍ lleva altitud propia por
        waypoint (alt_msl_degs) -- mismo convenio que ya usa
        load_waypoints_list() para la ruta inicial."""
        if seq <= self._last_route_replace_seq:
            return "rejected_stale_seq"

        if self._frame is None:
            raise RuntimeError("replace_route() requiere frame -- ver set_frame()")

        if not self._frame.matches(frame_origin_lat_deg, frame_origin_lon_deg, tol_deg=1e-5):
            return "rejected_origin_mismatch"

        if waypoint_count < 1:
            return "rejected_empty"
        if waypoint_count > MAX_ROUTE_WAYPOINTS:
            return "rejected_too_many"

        coords: list[Coord3D] = [
            self._frame.geodetic_to_ned(lat_degs[i], lon_degs[i], alt_msl_degs[i])
            for i in range(waypoint_count)
        ]
        self.set_route_source(RouteWaypointSource(iter(coords), on_exhausted=on_exhausted))
        self._last_route_replace_seq = seq

        log.info(
            "replace_route seq=%d aplicado: %d waypoint(s), on_exhausted=%s, t_sim=%.3fs",
            seq, waypoint_count, on_exhausted, self._sim_time,
        )
        return "applied"

    def reset_ctl_dedup(self) -> None:
        self._last_retask_seq = -1
        self._last_route_replace_seq = -1
