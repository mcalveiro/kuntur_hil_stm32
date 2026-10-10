# Arquitectura de la fusión Kuntur-SIM + CADAC Py (fork CADAC-SIL)

## Resumen

CADAC-SIL es un fork de Kuntur-SIM que reemplaza la planta física
(originalmente Simulink) por `cadac_py::falcon6.py` (puerto Python de
CADAC++ FALCON6), corriendo en modo **SIL puro**: sin Pixhawk/PX4 en el
lazo de actuación. La fusión se hizo **por sustitución, no por
reescritura**: `kse_server` (Rust) y `kuntur_bridge` (Python) quedaron
prácticamente intactos porque el nuevo proceso de planta (`cadac_plant/`)
habla exactamente el mismo protocolo binario que hablaba la S-Function de
Simulink (`kuntur_ic.cpp`). Este documento explica qué aporta cada
sistema, dónde ocurre exactamente el punto de inyección, y qué cambió
realmente.

## 1. Qué aporta Kuntur-SIM (el sistema preexistente)

Kuntur-SIM es la **plataforma de orquestación e integración** — es
agnóstica de qué física corre del otro lado. Sus piezas:

| Pieza | Rol | Lenguaje |
|---|---|---|
| `kuntur_ic.cpp` | S-Function de Simulink, contraparte UDP original del protocolo (la planta física vivía en Simulink) | C++ |
| **KSE** (`kse_server/`) | Broker de rendezvous entre planta y FCS/autopiloto — no computa física ni guiado, solo arbitra el intercambio de paquetes ICD y decide el modo temporal (`ENGINE_DRIVEN`/`AUTHORITY_DRIVEN`) | Rust |
| **kuntur_bridge** (Python) | Orquestador (`main.py`), cliente de KSE (`kse_client.py`), interfaz con FlightGear (`telnet_client.py`, `flightgear_worker.py`), geodesia de origen único (`geodesy.py::SACOLocalFrame`), configuración (`config_file.py`), componentes intercambiables: `fcs` (autopiloto propio), `px4` (HITL vía MAVLink, Pixhawk real), `flightgear` (visual), `mission` (ROS2, no usado en este fork) | Python |
| **Kuntur-IC v1.3** | El contrato — único origen de verdad, generado desde `icd_schema/kuntur_ic_v1_3.json` vía `tools/gen_icd.py` hacia `icd.py`/`icd.rs`/`kuntur_ic_generated.h`. `KUNTUR_STATES` (92 bytes, magic `KST3`) y `KUNTUR_COMMAND` (34 bytes, magic `CMD2`) | JSON + generado |
| FlightGear | Visualización 3D (`--fdm=null`: nunca calcula física propia) | — |
| PX4 (opcional) | Piloto automático real por HITL vía MAVLink — **no usado en este fork** | — |

Punto clave: **Kuntur-SIM nunca trajo un modelo de vuelo propio** — la
física dependía enteramente de lo que hubiera cargado en Simulink. Es
exactamente ese hueco el que CADAC Py viene a llenar en este fork.

## 2. Qué aporta CADAC Py

`cadac_py/` es un **puerto Python línea por línea de CADAC++ FALCON6**
(`falcon6.py`, ~1500 líneas): un F16-like, 6 grados de libertad, Tierra
plana, con:

- Física completa (`environment`/`kinematics`/`aerodynamics`/
  `propulsion`/`forces`/`euler`/`newton`), aerodeck y propdeck reales
  (`f16_aero_deck.asc`/`f16_prop_deck.asc`).
- **Autopiloto y guiado YA integrados dentro de la clase `Plane`**
  (`control()`/`guidance()`) — no es un modelo de "solo física" que
  espera comandos externos; es un vehículo autónomo que sigue waypoints
  por sí mismo.
- Un motor de ejecución propio (`execution.py`): parsea escenarios
  `*.asc` (`parse_input_asc`), arma el vehículo (`build_vehicle`,
  `apply_vehicle_data`), y corre una **cola de eventos** tipo
  `IF <condición> ENDIF` (`build_events`/`check_events`) que reasigna
  parámetros de guiado en caliente durante la corrida (así es como el
  escenario original avanza de un waypoint al siguiente).

CADAC Py es **completamente autónomo**: no sabe qué es un socket, un ICD
o FlightGear. Es una librería de simulación de vuelo standalone, validada
contra su propio batch de referencia (`regression_vs_batch.py`,
diferencia 0.0 m, bit-exacto).

## 3. Cómo se hizo la fusión — el punto de inyección exacto

### 3.1 Decisión de diseño: no extraer, envolver

Se evaluó extraer `control()`/`guidance()` a un FCS externo (para que
viajaran por el ICD como cualquier otro autopiloto) — se descartó
**empíricamente**: esas funciones leen ~15 derivadas aerodinámicas y
variables de sintonía persistentes (ganancias, estado de integradores,
`maut`) que no viajan por el ICD y no son triviales de recalcular fuera
de `Plane` sin duplicar casi toda la clase. La alternativa elegida:
**FALCON6 corre completo** (física + autopiloto + guiado, una caja
cerrada), reutilizando el motor de re-conducción paso a paso que
`execution.py` ya expone.

Consecuencia directa: el actuador que llega por `KUNTUR_COMMAND` se
recibe pero **se ignora deliberadamente** — FALCON6 gobierna sus propias
superficies de control. Esto es consciente: este camino es un SIL de
física+guiado, no un banco de validación de un FCS externo (para eso
existe el otro camino, `kuntur_bridge/falcon_fcs.py`, no usado acá).

### 3.2 El adaptador — `cadac_plant/plane_adapter.py::RealFalcon6Adapter`

Este es el punto de inyección real. Reconduce, **un paso de integración
a la vez**, exactamente la misma secuencia que usaría
`execution.py::run_scenario()` en batch:

```
parse_input_asc(asc) -> build_vehicle() -> apply_vehicle_data()
  -> init_xxx() (por módulo) -> build_events()
  -> [paso a paso] check_events() + _EXEC_DISPATCH (por módulo, en orden)
```

Validado bit a bit contra el batch original (0.0 m de diferencia). Lo
único que se inyecta desde afuera:

- **Condición inicial** (`reset()`): posición NED, rumbo, velocidad —
  las 5 que la interfaz `PlantAdapter` define. El resto (actitud de
  trimado, aerodinámica, ganancias) queda tal cual el `*.asc` de origen.
- **Waypoints reales** (opcional, `waypoints=`): `_override_waypoints()`
  sustituye `swel1/swel2/swel3` en la asignación incondicional inicial y
  en cada bloque `IF wp_flag=-1 ENDIF` del escenario, **en orden y sin
  tocar nada más** (`psiflx`/`thtflx`/`altcom`/`mguid`/`maut` quedan
  como el `*.asc` original los definió). Requiere exactamente
  1 + (cantidad de eventos) waypoints, o falla en vez de adivinar.
- **Patrulla en bucle** (agregado en esta sesión, `_advance_loop_waypoint()`):
  una vez que la cola nativa de eventos se agota (se pasó el último
  waypoint, el "IP" del escenario original), reinyecta el primer
  waypoint como nuevo objetivo en cada cruce de `wp_flag=-1`,
  restaurando explícitamente `mguid=30`/`maut=35`/`altcom` (el modo "en
  ruta" que ya usaban los tramos intermedios) — sin esto, el vehículo
  queda para siempre en el modo de picada terminal que el `*.asc`
  original preveía solo para el impacto final. Detalle completo en
  `FORK_CADAC_SIL.md`.

### 3.3 El acople con Kuntur-IC — `cadac_plant/protocol.py::PlantLink`

Esta capa reimplementa, verificado línea por línea contra `kuntur_ic.cpp`
(no inferido), **exactamente** el protocolo UDP que la S-Function de
Simulink hablaba con KSE:

- Mismos puertos (aún llamados `simulink_telemetry_port`/
  `simulink_command_port` en la config — 6000/6001 por default): TX de
  `KUNTUR_STATES` hacia `(kse_ip, telemetry_port)`, RX de
  `KUNTUR_COMMAND` en bind local sobre `command_port`.
- Mismo framing binario ICD v1.3 (los mismos `icd.py` generados).
- Mismo modo dual `ENGINE_DRIVEN`/`AUTHORITY_DRIVEN`, con la misma
  semántica que `kuntur_ic.cpp::mdlOutputs()` (el modo lo decide KSE en
  cada `KUNTUR_COMMAND`, no la planta).

**Esta capa es agnóstica de `cadac_py`** — no sabe qué es un `Plane` ni
un `flat6`; solo empaqueta/desempaqueta ICD. El que conecta ambos mundos
es `cadac_plant/plant_driver.py`, el proceso "planta" standalone: en cada
paso llama `adapter.step()` (física+guiado+control de FALCON6),
`link.send_states()` (ICD hacia KSE) y `link.poll_command()` (ICD desde
KSE, recibido pero ignorado por el adapter).

**Desde el punto de vista de KSE y de `kuntur_bridge`, `plant_driver.py`
ES Simulink.** No hubo que tocar una sola línea de `kse_server` para el
cableado de datos.

### 3.4 Gobierno del tiempo (reemplazo del Pacing de Simulink)

En Simulink, el bloque *Pacing* sincronizaba el avance del solver a
tiempo real. Acá ese rol lo cumple `plant_driver.py::run()`, con
`time.monotonic()` + `time.sleep()` y paceo por **tiempo absoluto**
(`target_wall = t_wall_start + seq_num * int_step_s`, autocorrector, sin
deriva acumulada). Medido con física real a 1000 Hz: 13.4% de pasos
individuales exceden el presupuesto de su período, con un atraso máximo
de ~4.05 ms y p99 ≈ 2.53 ms — jitter no eliminado pero acotado. KSE
nunca gobierna el tiempo; es un broker reactivo, no un reloj.

### 3.5 Cambios reales del lado `kuntur_bridge`/`kse_server`

Con todo lo anterior, el único cambio real necesario (no cosmético) fue:
el default de `[components] enabled` traía `px4` habilitado (HITL por
defecto, heredado del diseño original de Kuntur-SIM), lo cual
contradecía el "SIL puro" ya acordado — corregido a `executive,
flightgear`. `kse_server` (Rust): **0 cambios**. `cadac_py` vendorizado:
**0 cambios** (todo el trabajo de adaptación quedó en `cadac_plant/`,
código propio del fork).

## 4. Arquitectura resultante

```
┌─────────────────────────────┐        ICD v1.3 / UDP        ┌──────────────────────┐
│  cadac_plant/plant_driver.py │  states (6000) ──────────►  │                      │
│  (proceso standalone,        │                              │     kse_server        │
│   Terminal 2)                │  ◄────────── command (6001)  │     (Rust, KSE)       │
│                               │                              │  broker de rendezvous │
│  RealFalcon6Adapter           │                              │  ENGINE_DRIVEN /      │
│  = cadac_py::falcon6.Plane    │                              │  AUTHORITY_DRIVEN     │
│  (física+guiado+control       │                              └──────────┬───────────┘
│   COMPLETO, actuador          │                                         │ socket local
│   ICD ignorado)                │                                         │ (KSEClient)
└─────────────────────────────┘                              ┌──────────▼───────────┐
                                                                │   kuntur_bridge       │
                                                                │   (Python, main.py,   │
                                                                │   Terminal 1, junto    │
                                                                │   con FlightGear)      │
                                                                │  - telnet_client.py:   │
                                                                │    calibra origen +    │
                                                                │    despausa FlightGear │
                                                                │  - geodesy.py:         │
                                                                │    SACOLocalFrame      │
                                                                │    (autoridad única    │
                                                                │    de origen NED)      │
                                                                └──────────┬───────────┘
                                                                           │ --native-fdm (5501)
                                                                           │ --telnet (5000)
                                                                ┌──────────▼───────────┐
                                                                │      FlightGear        │
                                                                │  (--fdm=null,          │
                                                                │   puramente visual,    │
                                                                │   F-16 Zaretto/f16)    │
                                                                └───────────────────────┘
```

`run_sim.py` orquesta Terminal 1 (FlightGear + `kuntur_bridge` +
`kse_server`, todo con `start --aircraft f16`); `plant_driver.py` corre
aparte, en Terminal 2 — así como hoy Simulink corría aparte y KSE
simplemente escuchaba su UDP.

## 5. Glosario corto

- **ICD**: Interface Control Document — el contrato binario Kuntur-IC.
- **KSE**: Kuntur Simulation Executive — el broker Rust.
- **SIL**: Software-in-the-Loop (sin hardware real en el lazo).
- **HITL**: Hardware-in-the-Loop (con Pixhawk real, no usado en este fork).
- **NED**: North-East-Down, el marco local de posición.
- **FDM**: Flight Dynamics Model — el motor de física de FlightGear (acá, `null`: deshabilitado).
- **IP** (en el contexto de CADAC/FALCON6): *Impact Point* — el waypoint terminal del escenario original, NO una dirección de red.

## 6. Estado de verificación

Ver `FORK_CADAC_SIL.md` para el detalle completo (validaciones,
mediciones de jitter, bugs encontrados y corregidos: desincronización
Telnet en `calibrate_origin()`, divergencia post-IP y su fix de
patrulla en bucle, v1→v2).

**Soak test de larga duración (confirmación definitiva, sesión actual):**
corrida real de ~3762 s simulados (~1 hora) con patrullaje en bucle de 4
waypoints sobre Córdoba, registrando **19 reenganches de waypoint
(índice 0) → 18 ciclos completos de bucle**. Análisis cuantitativo de
dispersión de `dwbh` (rango horizontal al waypoint) y `psivlx` (rumbo de
velocidad) en cada reenganche, y del período de ciclo:

| Índice de waypoint | n (ciclos) | Dispersión dwbh | Dispersión psivlx |
|---|---|---|---|
| 0 (excluyendo transitorio inicial) | 17 | 0.000202 m | 0.006668° |
| 1 | 18 | 0.028089 m | 0.010448° |
| 2 | 18 | 0.001080 m | 0.006421° |
| 3 | 18 | 0.000355 m | 0.006353° |

Período de ciclo constante en régimen estacionario: **199.467–199.468 s**
en las 18 vueltas.

Esta dispersión (submilimétrica a pocos centímetros, fracciones de
centésima de grado) sobre 18 ciclos consecutivos —muy por encima de las
3 vueltas usadas en la primera verificación corta— confirma que el fix
v2 (restauración de `mguid`/`maut`/`altcom` al reenganchar) produce una
**órbita periódica genuinamente estable**, sin deriva ni degradación de
altitud a lo largo de una corrida de una hora.

Resumen general: física real validada bit-a-bit contra batch,
patrullaje de 4 waypoints sobre Córdoba corriendo en bucle estable
(limit cycle confirmado sobre 18 ciclos consecutivos en una corrida de
~1 hora), pipeline completo (FlightGear + kse_server + kuntur_bridge +
plant_driver) verificado en vuelo real.