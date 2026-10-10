# Guía de Producto — Kuntur-Bridge y KSE (fork CADAC-SIL)

**Versión del documento:** 3.0 (reemplaza a 2.0 -- no coexisten: 2.0
describía una etapa anterior del proyecto real, con backends KSE
in-process en Rust/Python que ya no existen, y Simulink como única
fuente de física)
**Contrato ICD cubierto:** Kuntur-IC v1.3
**Fork cubierto:** CADAC-SIL -- reemplaza a MATLAB/Simulink como fuente
de física por la planta 6-DoF FALCON6 de `cadac_py`, SIL puro (Pixhawk
fuera del lazo de actuación), waypoints reales sobre Córdoba por archivo
estático (sin `kuntur-mission`)
**Componentes cubiertos:** `cadac_py` (vendorizado, física+control+guiado
de FALCON6), `cadac_plant` (`plane_adapter.py::RealFalcon6Adapter`,
`plant_driver.py` -- reemplazo de Simulink+`kuntur_ic.cpp`), `kse_server`
(único camino de ejecución de KSE), `kuntur_bridge` (paquete Python,
configuración `.ini`), `runfg.sh` (FlightGear, perfil `f16` nuevo),
`run_sim.py` (orquestador), `icd_schema/` + `tools/gen_icd.py` (ICD
única fuente de verdad)

---

## 0. Respuesta directa: ¿hay que compilar KSE antes de lanzar el simulador?

**Sí, es obligatorio, no opcional.** `kuntur_bridge/kse_client.py`
verifica que el binario exista ANTES de intentar lanzarlo
(`KSEClient.__init__` con `spawn=True`, el default) y si no lo
encuentra levanta:

```
FileNotFoundError: kse_server no encontrado en .../kse_server/target/release/kse_server.
Compilar con 'cargo build --release' dentro de kse_server/.
```

Esto es así porque el binario compilado (`target/`) **no viaja en el
zip del fork** -- son artefactos de build de Rust (~100 MB), se excluyen
a propósito. Se compila una sola vez (o cada vez que cambie
`kse_server/src/*.rs` o se regenere el ICD -- ver §5.4):

```bash
cd kse_server
cargo build --release
```

Sin este paso, tanto `python3 -m kuntur_bridge.main` como
`python3 run_sim.py start` fallan de entrada.

---

## 1. Qué es este fork (CADAC-SIL)

Kuntur-SIM original integra MATLAB/Simulink (planta) con KSE
(orquestador de tiempo lógico) y FlightGear (visualización), con PX4/
Pixhawk como HITL opcional y `kuntur-mission` (ROS 2) para guiado. Este
fork reemplaza **solo la fuente de física**, dejando el resto de la
arquitectura (KSE, geodesia, feed a FlightGear) intacta:

- **`cadac_py`** (Python, vendorizado en el fork) -- puerto de
  Zipfel's CADAC++, con `falcon6.py::Plane` modelando un F16-like 6-DoF
  completo: física + los 7 controladores del autopiloto + guiado de
  waypoints, todo dentro de la misma clase (no hay un `kcs.py` separado
  en el código fuente real).
- **`cadac_plant/plane_adapter.py::RealFalcon6Adapter`** -- re-conduce
  `Plane` paso a paso (las mismas funciones que usa
  `cadac_py.execution.run_scenario()` internamente, validado bit-exacto
  contra el batch) y lo expone del lado UDP del protocolo Kuntur-IC,
  **reemplazando directamente a Simulink + `kuntur_ic.cpp`** como
  contraparte de `kse_server`.
- **`cadac_plant/plant_driver.py`** -- el proceso que corre eso en
  tiempo real, con paceo a reloj de pared (reemplaza al modo Pacing de
  Simulink -- ver §9 para el detalle y el jitter medido).
- **SIL puro**: PX4/Pixhawk sigue existiendo en el código
  (`px4_bridge.py`) pero está **fuera del lazo de actuación en este
  fork** -- de hecho, con `RealFalcon6Adapter` corriendo, ningún
  actuador externo tiene efecto: FALCON6 gobierna sus propias
  superficies internamente (ver §3).
- **Sin `kuntur-mission`**: los waypoints son reales (sobre Córdoba
  capital) pero se cargan de un archivo estático
  (`scenarios/waypoints_cordoba.txt`) y se hornean en el escenario
  `*.asc` de FALCON6 al construir `RealFalcon6Adapter` -- no hay
  inyección dinámica por ROS 2 en este camino.
- **FlightGear** sigue siendo puramente visual (`--fdm=null`), ahora con
  un perfil de aeronave F-16 nuevo en `runfg.sh` (§6.3).

Documento de referencia técnica más detallado (arquitectura, hallazgos,
validaciones, riesgos): `FORK_CADAC_SIL.md` en la raíz del repo -- esta
guía es el paso a paso operativo, esa es la memoria técnica de las
decisiones y su validación empírica.

---

## 2. Estado de verificación

| Componente | Estado |
|---|---|
| `cadac_py` (vendorizado) | **Verificado.** Re-conducción externa de `execution.py` paso a paso reproduce **bit-exacto** (diff 0.0 m) el batch de `run_scenario()` sobre el mismo escenario. |
| `RealFalcon6Adapter` | **Verificado con física real, no mock.** Patrulla completa de 4 waypoints reales sobre Córdoba (250s simulados @ 1kHz): captura cada waypoint, rumbos geográficamente correctos, sin NaN ni divergencia. |
| Rendimiento a 1000 Hz (int_step real de FALCON6) | **Medido**: ~2.7x tiempo real en cómputo puro (promedio). Jitter por paso individual también medido -- ver §9, no es cero. |
| `plant_driver.py` contra `kse_server` real | **Verificado**: 10s continuos @ 1kHz a través del binario sin modificar, sin caídas. |
| `kuntur_bridge` (paquete Python) | **Verificado** (heredado, sin cambios de comportamiento fuera de lo listado en §1) -- corre `main.run()` de punta a punta. |
| `icd_schema/` + `tools/gen_icd.py` | **Verificado.** Genera `icd.py`, `kse_server/src/icd.rs` y `kuntur_ic_generated.h` desde un solo schema. |
| `kse_server` | **Verificado**, pero **no viene compilado en el fork** -- ver §0. |
| `MockFalcon6Adapter` / `FalconAutopilotCoreMock` | Siguen funcionando (cableado, no fidelidad de vuelo) -- útiles para probar sin depender de `cadac_py`. |
| `px4_bridge.py`, `kuntur_fcs.py`, `mission_bridge.py` | **Código heredado sin tocar, no ejercitado en este fork.** Siguen siendo válidos como componentes de `kuntur_bridge` (`[components] enabled`), pero no son parte del camino de fidelidad real de este fork -- ver §1 y §3. |
| FlightGear + modelo F-16 (`runfg.sh`, perfil `f16`) | **No verificado visualmente** -- FlightGear corre en tu equipo, no en el entorno donde se construyó este fork. Los puertos/protocolo sí están verificados (ver §6.3). |
| ROS 2 / `kuntur_msgs` / `kuntur_mission` | Fuera de alcance de este fork (ver §1) -- código original sin tocar, no ejercitado. |

---

## 3. Arquitectura de ejecución

```
              icd_schema/kuntur_ic_v1_3.json
             (fuente única del contrato binario)
                            |
            tools/gen_icd.py (regenera los 3 destinos)
              /             |              \
      icd.py (Py)   kse_server/src/icd.rs   kuntur_ic_generated.h
                                              (legado -- sin uso en
                                               este fork, ver §1)

cadac_plant/plant_driver.py  (REEMPLAZA a Simulink+kuntur_ic.cpp)
  = RealFalcon6Adapter (física+control+guiado FALCON6 completos,
    cadac_py, monolítico -- ver FORK_CADAC_SIL.md para el porqué)
        |  UDP :6000 (TX KUNTUR_STATES, 92 B)
        v  UDP :6001 (RX KUNTUR_COMMAND, 34 B -- RECIBIDO PERO IGNORADO:
        |             FALCON6 no necesita actuador externo, ver más abajo)
     kse_server (único backend de KSE -- Rust, proceso separado,
     |            lanzado automáticamente por KSEClient)
     |  new_state_event / actuators_ready_event (rendezvous acotado)
     |  reset() -- reinicializa estado lógico sin matar el proceso
     v
kuntur_bridge (main.py, componentes 'executive'+'flightgear' -- suficiente
  para este fork; 'fcs'/'px4'/'mission' siguen disponibles pero inertes
  para la fidelidad de vuelo, ver abajo)
        |
        +-- flightgear_worker.py --> FlightGear (UDP :5501, Telnet :5000)
        |     (--fdm=null, modelo F-16 -- puramente visual)
        |
        +-- (opcional, no necesario acá) fcs_worker/px4_bridge_worker
              --> KUNTUR_COMMAND -- SIN EFECTO en este fork mientras
                  RealFalcon6Adapter esté del otro lado del wire
```

**Por qué el actuador se ignora:** `control()`/`guidance()` de FALCON6
viven dentro de `Plane`, no en un FCS separado -- leen ~15 derivadas
aerodinámicas y estado de integradores que no viajan por el ICD, así que
extraerlos exigiría duplicar casi toda la física (ver
`FORK_CADAC_SIL.md`, sección de pivote arquitectónico). Consecuencia
directa: no hace falta ni `fcs` ni `px4` habilitados en `kuntur_bridge`
para que el avión vuele -- FALCON6 es un piloto automático cerrado sobre
sí mismo. Se dejan disponibles solo por si en algún momento se quiere
ejercitar ese camino contra `MockFalcon6Adapter`.

---

## 4. Requisitos de sistema

### 4.1 Comunes

- Linux (Ubuntu 22.04/24.04 recomendado).
- **Python 3.9+** -- sin dependencias de terceros para `kuntur_bridge`
  ni para `cadac_py`/`cadac_plant` (ambos son librería estándar pura:
  `math`, `socket`, `struct`, `configparser`, etc.).
- **FlightGear** 2020.3.x o superior, con algún paquete de aeronave F-16
  instalable (ver §6.3) -- **NO hace falta MATLAB/Simulink** en este
  fork (sí en el Kuntur-SIM original, para el otro camino de física).
- `rustc`/`cargo` -- para compilar `kse_server` (§0/§5.2). Verificado
  con la toolchain de Ubuntu, sin `rustup`.

### 4.2 Ya NO hacen falta en este fork

- MATLAB/Simulink + compilador `mex` (era 4.1/5.2 en la guía anterior).
- `maturin`/`pyo3` (backend `kse_native`, in-process -- ya no existe;
  `kse_server` es el único camino, sin dependencias de terceros).
- ROS 2/`colcon` -- solo si además querés ejercitar `kuntur_mission` por
  fuera de este fork (no es parte del camino de fidelidad real acá).
- `pymavlink` -- solo hace falta si además vas a habilitar `[components]
  px4` a mano (no es el default de este fork, ver §7).

---

## 5. Instalación paso a paso

### 5.1 Sistema base

```bash
sudo apt-get update
sudo apt-get install -y build-essential python3 python3-pip cargo rustc
```

### 5.2 Compilar `kse_server` (obligatorio -- ver §0)

```bash
cd kse_server
cargo build --release
cargo test           # opcional, 4/4 esperado
cd ..
```

Sin `pip install` -- `kuntur_bridge/kse_client.py` lanza
`kse_server/target/release/kse_server` como subproceso automáticamente
en cuanto detecta que existe.

### 5.3 Instalar FlightGear

```bash
sudo apt-get install -y flightgear
```

Instalá además el paquete de aeronave F-16 que quieras usar (§6.3) desde
el Launcher de FlightGear -- este fork no lo trae vendorizado (es un
recurso binario grande y de terceros, fuera del alcance del zip).

### 5.4 Verificar `cadac_py`/`cadac_plant`

Sin instalación -- vienen vendorizados en el fork. Verificación rápida:

```bash
python3 -c "from cadac_py.execution import run_scenario; print('cadac_py OK')"
python3 -m cadac_plant.icd_helpers      # debe imprimir el caso de reposo [0,0,-9.80665]
python3 -m cadac_plant.protocol         # smoke test de loopback del protocolo ICD
```

### 5.5 Instalar `kuntur_bridge`

Sin dependencias de terceros:

```bash
python3 -c "import kuntur_bridge; print('kuntur_bridge OK')"
```

### 5.6 Regenerar el ICD (solo si el contrato cambia -- no es parte del arranque normal)

```bash
# 1. Editar icd_schema/kuntur_ic_v1_3.json
python3 tools/gen_icd.py
# 2. Recompilar lo que depende del ICD:
cd kse_server && cargo build --release && cd ..
```

`kuntur_bridge/icd.py`, `kse_server/src/icd.rs` y
`kuntur_ic_generated.h` (legado, ver §1) llevan el encabezado
`*** ARCHIVO GENERADO — NO EDITAR A MANO ***` -- nunca tocarlos
directamente.

---

## 6. Configuración

### 6.1 Generar un `.ini` de ejemplo

```python
from kuntur_bridge.config_file import write_default_ini
write_default_ini("kuntur_sim.ini")
```

El default de `[components]` en este fork es **`executive, flightgear`**
(corregido respecto de una versión anterior que traía `px4` habilitado
por default -- contradecía el "SIL puro" ya acordado). Ver §3 para por
qué ni `fcs` ni `px4` hacen falta acá.

### 6.2 Puertos -- deben coincidir entre `kuntur_bridge` y `plant_driver.py`

| Clave `.ini` | Default | Lado `cadac_plant` equivalente |
|---|---|---|
| `[network] simulink_telemetry_port` | 6000 | `plant_driver.py --telemetry-port` (default 6000) |
| `[network] simulink_command_port` | 6001 | `plant_driver.py --command-port` (default 6001) |
| `[network] fg_udp_port` | 5501 | -- (FlightGear, no CADAC) |
| `[network] fg_telnet_port` | 5000 | -- (FlightGear, no CADAC) |

Los nombres de clave siguen diciendo "simulink_*" (heredado del ICD
original) pero en este fork ese puerto lo escucha/usa `plant_driver.py`,
no Simulink. Si cambiás uno de los dos lados, cambiá el otro igual --
`load_settings()` no valida esta coincidencia automáticamente.

### 6.3 FlightGear -- aeronave F-16 (`runfg.sh`)

```bash
KUNTUR_AIRCRAFT=f16 ./runfg.sh
```

`runfg.sh` ya trae los puertos/tasa alineados con la tabla de arriba
(`FDM_PORT=5501 @ 30Hz`, `TELNET_PORT=5000`) y el origen por default
(`LAT=-31.31 LON=-64.21`, el mismo fallback de `SACOLocalFrame`). El
perfil `f16` usa por default `f16-simplified` (paquete Zaretto/f16) --
ver los comentarios del caso `f16)` en el script para la tabla completa
de variantes/IDs y el porqué de esa elección (no hay match aerodinámico
posible: FALCON6 no está atado a ningún bloque específico). Antes de la
primera corrida, instalá la variante elegida desde el Launcher de
FlightGear y ubicá dónde quedó (`find "$HOME/.fgfs/Aircraft" -iname
"*-set.xml"`) para setear `FG_AIRCRAFT_DIR` si no cae en el path default
del script.

### 6.4 `[fcs]` -- no relevante para la fidelidad real de este fork

Sigue existiendo (`backend = kuntur | falcon`) para quien quiera
ejercitar `MockFalcon6Adapter`/`FalconFCS` sin `cadac_py` real -- no usar
esto para nada relacionado con fidelidad de vuelo (ver §1/§3 y
`FORK_CADAC_SIL.md`).

---

## 7. Cómo correr -- paso a paso

### 7.1 Orden de arranque (reemplaza a "1. FlightGear, 2. Simulink, 3. kuntur_bridge")

1. **Compilar `kse_server`** (una sola vez, o tras cambios -- §0/§5.2).
2. **FlightGear** (queda en pausa) -- vía `runfg.sh` o vía `run_sim.py start` (paso 4 hace esto por vos).
3. **`kuntur_bridge`** -- lanza `kse_server` automáticamente como subproceso.
4. **La planta CADAC real** (`plant_driver.py --adapter real`) -- reemplaza a "arrancar la simulación en Simulink". Es un proceso aparte, `run_sim.py`/`kuntur_bridge` NO lo lanzan por vos.

### 7.2 Forma recomendada (dos terminales)

**Terminal 1** -- FlightGear + kuntur_bridge juntos:

```bash
python3 run_sim.py start --aircraft f16
```

**Terminal 2** -- la planta CADAC real, con los waypoints reales de Córdoba:

```bash
python3 -m cadac_plant.plant_driver --adapter real \
    --waypoints scenarios/waypoints_cordoba.txt \
    --north0 1891.1 --east0 952.0 --down0 -1000 \
    --psi0-deg 180 --ias0 180
```

(La condición inicial de ejemplo arriba es la validada en
`FORK_CADAC_SIL.md` -- 3 km al norte del primer waypoint, rumbo sur,
altitud/velocidad de crucero de `f16c11_7.asc`.)

Sin `--waypoints`, usa el escenario `f16c11_7.asc` con SUS waypoints
originales (no los de Córdoba) y su propia condición inicial por default
(`--north0 0 --east0 0 --down0 -1000 --psi0-deg 0 --ias0 180`).

### 7.3 Headless (sin FlightGear, para pruebas)

```ini
[components]
enabled = executive
```

```bash
python3 -m kuntur_bridge.main --config kuntur_sim.ini --no-flightgear
# en otra terminal:
python3 -m cadac_plant.plant_driver --adapter real
```

### 7.4 Solo la planta, sin `kse_server`/`kuntur_bridge` (para iterar rápido sobre la física)

```bash
python3 -m cadac_plant.plant_driver --adapter mock          # física mock
python3 -m cadac_plant.plant_driver --adapter real           # física real, escenario default
```

Sin un receptor UDP del otro lado, `plant_driver.py` igual corre (los
paquetes `KUNTUR_STATES` se pierden en el aire) -- útil para ver que la
física/paceo andan antes de meter a KSE en la ecuación.

### 7.5 Ver logs

```bash
python3 run_sim.py logs -f                    # seguir en vivo
python3 run_sim.py logs --level WARN          # solo advertencias/errores
python3 run_sim.py logs --component flightgear
```

`plant_driver.py` no está integrado a este sistema de logs (usa
`logging` estándar a stdout) -- sus mensajes NO aparecen en
`run_sim.py logs`, hay que mirar la Terminal 2 directamente.

### 7.6 Terminación

```bash
python3 run_sim.py stop    # detiene FlightGear + kuntur_bridge (y kse_server, que kuntur_bridge lanzó)
```

`plant_driver.py` (Terminal 2) se detiene aparte, con `Ctrl+C` en su
propia terminal -- `run_sim.py stop` no lo conoce ni lo gestiona.

### 7.7 Limpieza de emergencia

```bash
python3 run_sim.py clean    # mata fgfs/kuntur_bridge residuales, borra sockets huérfanos
```

---

## 8. Referencia de la interfaz `IKunturRuntime` (sin cambios de contrato en este fork)

| Método | Qué hace |
|---|---|
| `start()` | Arranca el lazo Grant/STEP. No bloquea. |
| `stop(timeout_s=2.0)` | Terminación ordenada. |
| `reset()` | Reinicializa snapshot/actuadores/staleness/ready sin detener el proceso. |
| `snapshot()` | Último `KUNTUR_STATES` válido, o `None`. |
| `set_actuators(e, a, r, t)` | Hook de FCS/PX4 -- sin efecto sobre el vuelo si del otro lado hay un `RealFalcon6Adapter` (ver §3). |
| `set_ground_elevation(elev_m)` | Poblado por `flightgear_worker.py`; `RealFalcon6Adapter` no lo consume (CADAC no tiene modelo de terreno propio). |
| `set_exec_mode(mode)` | Cambia `ENGINE_DRIVEN`/`AUTHORITY_DRIVEN` en runtime -- este fork usa `ENGINE_DRIVEN` (default); `AUTHORITY_DRIVEN` no aporta nada con `RealFalcon6Adapter` (ignora el actuador de todos modos). |
| `ready_event()` | Seteado tras el primer `KUNTUR_STATES` válido. |
| `control_staleness_cycles()` | Ciclos consecutivos sin confirmación de actuador. Con este fork y sin `fcs`/`px4` habilitados, **crece sin límite indefinidamente** -- es esperado y benigno (ver §9), no un síntoma de falla. |

`kse_server` es el único backend -- no hay selección (`create_runtime(backend=...)` de la guía anterior ya no aplica; `KSEClient` habla directo con el binario).

---

## 9. Quién gobierna el tiempo ahora (Pacing de Simulink -> `plant_driver.py`)

Con Simulink, el solver + el bloque de Pacing eran la fuente de verdad
del tiempo. `kse_server` nunca gobernó el tiempo -- es reactivo. En este
fork, **`cadac_plant/plant_driver.py::run()`** asumió ese rol: paso fijo
= `int_step` del `*.asc` de origen (0.001s para FALCON6), sincronización
a reloj de pared con `time.monotonic()`+`time.sleep()` de tiempo
absoluto (no acumulativo -- un atraso puntual no deriva).

Medido (no asumido): a 1kHz, sin red, **13.4% de los pasos individuales
excedieron su presupuesto de 1ms de cómputo**, con un atraso máximo
puntual de ~4.05ms (p99 ≈2.53ms). No se traduce en deriva acumulada, pero
sí en jitter real paso a paso -- peor que lo que garantizaba el Pacing
de Simulink. Detalle completo en `FORK_CADAC_SIL.md`.

---

## 10. Pendientes conocidos de este fork

- **Verificación visual de FlightGear** -- no se pudo correr FlightGear
  en el entorno donde se construyó este fork (no tiene GPU/GUI). Los
  puertos/protocolo/paceo sí están verificados de punta a punta contra
  `kse_server` real; falta la corrida con FlightGear real de tu lado.
- **Variante de aeronave F-16** (`f16-simplified` vs. `f16b32` u otra) --
  elección razonada, no verificada visualmente (ver §6.3).
- **Comportamiento de `halt` al final del escenario** -- el puerto
  actual de `cadac_py/execution.py` no usa el flag `halt` para detener
  el paquete `combus`; la aeronave sigue volando (y `guidance()` sigue
  logueando "closest approach") más allá del punto final de un
  escenario tipo `f16c11_7.asc`. No es un bug de la integración, es
  comportamiento del código fuente provisto -- no se tocó `cadac_py`
  para "arreglarlo" (fuera de alcance).
- **`ground_elev_m`** sigue viajando por `KUNTUR_COMMAND` sin cambios,
  pero `RealFalcon6Adapter` no lo consume -- sin condición de colisión
  con el terreno de este lado. Los waypoints de Córdoba se eligieron
  deliberadamente sobre la llanura para que esto no importe en la
  práctica.
- **`px4_bridge.py`/`kuntur_fcs.py`/`mission_bridge.py`** -- código
  heredado sin tocar, sin ejercitar en el contexto de este fork (sí
  siguen siendo funcionales por sí mismos, para quien quiera volver al
  camino original de Kuntur-SIM con Simulink).
- **Jitter del paceo de `plant_driver.py`** -- medido y documentado
  (§9), no eliminado. Si en algún momento hiciera falta un paceo más
  estricto, sería el próximo punto a mejorar (p.ej. un loop en un hilo
  con prioridad de tiempo real, o mover el paceo fuera del GIL).
