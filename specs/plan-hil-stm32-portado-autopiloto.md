# Plan HIL: migración del autopiloto FALCON6 completo a STM32F767

**Fecha:** 2026-10-02 (reemplaza al plan anterior "lazos de aceleración")
**Estado:** especificación técnica, lista para iniciar implementación.
**Supersede** la Opción F de `plan-hil-tesina.md` y la versión previa de
este mismo documento que proponía portar solo los lazos de aceleración.

## 1. Decisiones de arquitectura cerradas

| Decisión | Elección |
|---|---|
| Qué va al hardware | **Autopiloto FALCON6 completo** (outer loops + aceleración + rate loops + mezcla a superficies). La planta se queda en Python. |
| Validación | Comparación contra el **SIL actualmente validado** (soak test de 18 ciclos, ya auditado en `arquitectura-fusion-kuntur-cadac.md` §6). El SIL es el oráculo. |
| Hardware | STM32F767ZI en Nucleo-144, usando **PlatformIO + STM32Cube**. |
| Transporte Host↔STM32 | **USB CDC** (virtual COM) sobre USB FS. Ethernet/LwIP queda **fuera de alcance**. |
| Cadencia del autopiloto en HIL | **100 Hz** (decimación ×10 respecto al paso de 1 kHz de la física). Decisión de diseño documentada, no una limitación oculta. Ver §5. |
| FS-i6 | Vía **USB HID por adaptador Flysky SM100 2.4G**, lectura en Host como joystick, pass-through a `KUNTUR_COMMAND` cuando Kuntur-GCS conmuta a Manual. **No pasa por el lazo HIL** — es canal secundario independiente. |
| FlightGear | Rol actual sin cambios. No se tocan `flightgear_worker.py`, `telnet_client.py`, `aux_instrument_worker.py`, `fgnetfdm.py`. |
| **Kuntur-GCS** | **Componente de primera clase del proyecto** — es el GCS construido explícitamente para la tesina, ya mayormente concluido. Roles en el HIL: visualización de telemetría, indicador de estado del STM32, conmutador de modo (Automático/Manual). |
| Retasking (`KUNTUR_CTL`) | **Fuera del alcance** de la versión inicial. Si eventualmente se trae, es trabajo post-defensa. |
| Base del proyecto | **Fork minimalista** del CADAC-SIL actual, con limpieza selectiva (no refactor). Ver §9. |

## 2. Arquitectura general

```
┌────────────────────────────────────────┐
│  cadac_plant/plant_driver.py           │
│                                         │
│  Planta (Python, 1 kHz, UNCHANGED):     │
│   environment / kinematics /            │
│   aerodynamics / propulsion /           │
│   forces / newton / euler              │
│                                         │
│  Autopiloto nativo FALCON6:             │           ┌───────────────────────────┐
│   DESHABILITADO cuando HIL activo       │           │  STM32F767 Nucleo-144     │
│   (bypass con fallback automático)      │           │  PlatformIO + STM32Cube   │
│                                         │           │                           │
│  stm32_bridge (worker nuevo):           │  USB CDC  │  Falcon6 Autopilot FULL:  │
│   - cada 10 pasos (100 Hz):             │ ────────▶ │   control_altitude        │
│     empaqueta KUNTUR_PLANT_STATE        │   @100 Hz │   control_heading         │
│     envía por USB CDC                   │           │   control_gamma           │
│   - lee KUNTUR_SURFACE_CMD por USB CDC  │ ◀──────── │   control_normal_accel    │
│   - hold-last entre llamadas            │           │     (pole placement       │
│   - timeout → fallback al autopiloto    │           │      adaptativo)          │
│     nativo Python                       │           │   control_lateral_accel   │
│                                         │           │   control_roll            │
│  Modo Manual (desde Kuntur-GCS):        │           │   control_pitch_rate      │
│   KUNTUR_SURFACE_CMD del STM32 ignorado;│           │   control_roll_rate       │
│   se usan los valores del joystick FS-i6│           │   control_yaw_rate        │
│                                         │           │   control_mix             │
└──────────┬────────────────┬─────────────┘           │   guidance_line           │
           │                │                          │   actuator                │
           │                │                          │                           │
           │ ICD v1.3       │ SSE/HTTP                 │   @ 100 Hz, double FPU    │
           │ (KSE)          │ (telemetría +            └───────────────────────────┘
           ▼                │  mode switch)
    FlightGear              │
    (visualización —        ▼
     telnet, aux_instr)   ┌──────────────────────────────────────┐
                           │  Kuntur-GCS (web app, repo propio)   │
                           │                                       │
                           │  - Mapa + ruta + marcador aeronave   │
                           │  - Six-pack telemetría               │
                           │  - Panel modo: Automático / Manual   │
                           │  - Indicador STM32: conectado /      │
                           │    autopilot activo / fallback count │
                           │                                       │
                           │  (retasking: UI oculta en v1,         │
                           │   backend quitado)                   │
                           └──────────────────┬────────────────────┘
                                              │ activa modo Manual
                                              ▼
┌──────────────────┐  2.4GHz  ┌──────────────┐  USB HID  ┌──────────────────────┐
│  FS-i6 radio     │ ──────▶  │  SM100 USB   │ ────────▶ │  manual_joystick_    │
└──────────────────┘          │  dongle      │           │  worker.py           │
                              └──────────────┘           │                      │
                                                          │  Si modo == Manual: │
                                                          │  empaqueta joystick │
                                                          │  → KUNTUR_COMMAND   │
                                                          │  override           │
                                                          └──────────────────────┘
```

Tres planos independientes:
1. **HIL** — STM32 corre autopiloto; validación contra SIL.
2. **Visualización + control de misión** — Kuntur-GCS consume telemetría
   SSE y emite comandos de modo.
3. **Operador manual** — FS-i6 por USB HID; activo cuando Kuntur-GCS
   decide Manual.

El modo lo decide el operador vía Kuntur-GCS; la planta elige qué
fuente de comandos aplicar.

## 3. La frontera del autopiloto — qué funciones migran

De `cadac_py/falcon6.py::Plane`, las funciones del dominio **autopiloto
+ guiado + actuador** que se portan:

**Lazos exteriores:**
- `init_control_altitude`, `control_altitude`
- `init_control_heading`, `control_heading`
- `init_control_gamma`, `control_gamma`

**Lazos de aceleración (contienen el pole placement adaptativo):**
- `init_control_normal_accel`, `control_normal_accel`
- `init_control_lateral_accel`, `control_lateral_accel`

**Lazos de tasa:**
- `init_control_roll`, `control_roll`
- `init_control_pitch_rate`, `control_pitch_rate`
- `init_control_roll_rate`, `control_roll_rate`
- `init_control_yaw_rate`, `control_yaw_rate`

> **Auditoría cerrada (2026-10-10):** `control_roll` (posición,
> `mroll==0`) y `control_roll_rate` (tasa, `mroll==1`) son dos caminos
> **alternativos** seleccionados por `mroll` dentro de `control_mix()`
> — no una cascada donde uno alimente al otro. `control_roll(phicomx)`
> calcula `delacx` directo, combinando realimentación de posición y de
> tasa en una sola función (ganancias `gkp`/`gkphi`, derivadas de
> `wrcl`/`zrcl`, `plane[512/513]`). Verificado línea por línea contra
> `cadac_py/falcon6.py` (líneas 1162-1296) y contra el port en
> `firmware/kuntur_autopilot/src/autopilot/control_rates.c` y
> `control_mix.c`: ambos lazos están portados explícitos y completos,
> bit-exactos. `mroll` se lee explícito en `control_mix()` y está fijo
> en `0` en `scenario_params.h`, justificado contra `f16c11_7.asc`
> (nunca fija `mroll`, queda en el default `0`). Detalle completo de la
> auditoría en `spec-auditoria-control-roll-stm32.md`. No reabrir esta
> pregunta.

**Mezcla a superficies:**
- `control_mix` (si existe explícito; si no, está dentro de los lazos de tasa)

**Guiado:**
- `init_guidance_line`, `guidance_line` (geometría de waypoints, detección de `wp_flag`)

**Actuador:**
- `init_actuator`, `actuator` (dinámica de superficies, `plane[519..521]` → actuales deflexiones aplicadas)

**Matemática de soporte:**
- Limitadores, filtros, utilidades trigonométricas específicas que usen las funciones anteriores
- Integradores (manteniendo su estado persistente — ver §4 sobre encapsulación)

### Lo que **no** se porta — se queda en Python

- Toda la **física**: `environment`, `kinematics`, `aerodynamics`,
  `propulsion`, `forces`, `newton`, `euler`.
- La **interpolación del aerodeck y propdeck** (tablas
  `f16_aero_deck.asc`/`f16_prop_deck.asc`). La física los interpola; las
  derivadas resultantes viajan al STM32 por ICD.
- El motor de **ejecución y eventos** de `execution.py` (parseo `.asc`,
  cola de eventos `IF ... ENDIF`, dispatch). Lo que el STM32 necesita de
  esos eventos (p. ej. cambio de waypoint, de modo) viaja por el canal
  de estado.

Esta frontera es la del plan de desacople
(`plan-desacople-gnc-autopiloto-guiado.md`) llevada hasta el final: el
STM32 implementa las Fases 2+4 de ese plan en un solo paso, pero con el
corte ejecutándose en hardware.

### Volumen de código estimado

~800-1100 líneas de C (ANSI, `double` nativo, sin dependencias externas
más allá de `<math.h>`). Equivale a ~2500 líneas de Python originales
en FALCON6. Portado línea por línea, sin reescritura algorítmica.

## 4. Estado persistente del autopiloto — cómo se encapsula

Un problema real del portado: el autopiloto FALCON6 guarda estado
disperso en `plane[]` (integradores `plane[516/517]`, polos calculados
`plane[504..506]`, vector de ganancias `plane[524]`, modos `maut`, swel
activos `swel1-3`, etc.). Al mover el autopiloto a STM32:

- Ese estado **vive en el STM32** entre iteraciones. No viaja por ICD.
- Al iniciar una corrida, el STM32 recibe un `KUNTUR_PLANT_STATE` con
  flag `init=1` que lo pone en condición de arranque consistente con el
  `.asc` del escenario.
- Encapsularlo en una `struct Autopilot_State_t` dentro del firmware,
  mapeando 1:1 contra los slots `plane[...]` originales (con nombres
  significativos vía `#define` o typedef). Esto preserva trazabilidad
  contra el código Python original y facilita la validación bit a bit.

## 5. Cadencia del autopiloto en HIL — 100 Hz, con justificación

En el SIL actual, el autopiloto corre a 1 kHz (misma tasa que la
física). En HIL vía USB CDC esto no se sostiene — el round-trip USB FS
está en 1-3 ms y el autopiloto quedaría 1-3 iteraciones atrás,
introduciendo una fase variable no reproducible.

**Elección:** decimar el autopiloto a **100 Hz** (cada 10 pasos físicos
se consulta al STM32; entre consultas, se mantienen las últimas
superficies recibidas). Esto se defiende por tres razones:

1. **Es la cadencia típica real.** Los FMS/autopilotos de aviones de
   combate reales corren outer loops a 20-100 Hz, inner loops a 100-500
   Hz. Un autopiloto completo a 100 Hz es más fiel a la realidad
   industrial que uno a 1 kHz.
2. **La dinámica lo tolera.** El F-16 tiene modo de corto período
   ~2-4 Hz y phugoid ~0.1 Hz. Nyquist pide ×10 sobre la frecuencia más
   rápida (los rate loops), y 100 Hz lo cubre con margen.
3. **Permite USB CDC sin Ethernet** → mantiene la infraestructura
   minimalista.

**Riesgo residual mitigado por protocolo:** antes de confiar en el
HIL, correr la validación "SIL-nativo a 1 kHz vs. SIL-con-autopiloto-a-100Hz"
en el mismo Python (decimando internamente, sin tocar hardware). Si el
patrullaje a 100 Hz converge al mismo ciclo límite (dispersión del mismo
orden que la del soak test original), la decimación es benigna. Si no,
hay que revisar la cadencia antes de meter STM32.

**Resultado documentable:** la caracterización "autopiloto a 100 Hz vs
1 kHz" es contenido de tesis en sí misma — justifica una elección de
diseño con evidencia.

## 6. Extensiones de ICD

Dos canales nuevos. Mismo patrón que `KUNTUR_AUX`/`KUNTUR_CTL`: schemas
propios, codegen separado, no tocan Kuntur-IC v1.3.

### 6.1 `KUNTUR_PLANT_STATE` — Host → STM32 a 100 Hz

Transporta todo lo que el autopiloto necesita del punto de vuelo
instantáneo:

| Grupo | Campos | Fuente |
|---|---|---|
| Framing | `magic_header`, `t_sim_us`, `seq_num`, `flags` (bit 0 = init, bit 1 = active) | `plant_driver.py` |
| Kinematic | pos NED (`sn`, `se`, `sd`), vel NED (`vn`, `ve`, `vd`), actitud (`phi`, `theta`, `psi`), tasas (`p`, `q`, `r`) | `flat6[]` |
| Aceleraciones | `ax`, `ay`, `az` (body-frame, para los lazos de aceleración) | `flat6[]` |
| Flight condition | `pdynmc`, `vmach`, `dvbe`, `alpha`, `beta` | `flat6[]` |
| Derivadas longitudinales | `dla`, `dma`, `dmq`, `dmde` | `plane[145..148]` |
| Derivadas laterales | `dyb`, `dnb`, `dyr`, `dyp`, `dnr`, `dnp`, `dyda`, `dnda`, `dydr`, `dndr` | `plane[...]` (relevar índices en Fase 1) |
| Waypoint activo | `swel1`, `swel2`, `swel3` (NED del target), `psiflx`, `thtflx`, `altcom`, `mguid`, `maut` | `plane[...]` |
| Modo | `manual_active` (si 1 → STM32 puede opcionalmente saltar cómputo) | Kuntur-GCS mode |

Payload estimado: ~200 bytes (25 `double` + headers). A 100 Hz → 20
KB/s. USB CDC lo cubre con 20× de margen.

### 6.2 `KUNTUR_SURFACE_CMD` — STM32 → Host a 100 Hz

El retorno:

| Campo | Semántica |
|---|---|
| Framing | `magic_header`, `t_sim_us` (eco), `seq_num` (eco), `status` (OK/degraded) |
| Superficies comandadas | `delacx` (aileron), `delecx` (elevator), `delrcx` (rudder) — radianes |
| Throttle | `throttle` ∈ [0, 1] |
| Debug/telemetría opcional | `gainfb1-3` (ganancias actuales del pole placement), `wp_flag`, autopilot internal state de interés |

Payload: ~80 bytes. A 100 Hz → 8 KB/s.

### 6.3 Codegen C para el STM32

El `tools/gen_icd.py` hoy emite Python y Rust. Para STM32 hace falta C.
Dos caminos:

- **(a)** Extender `gen_icd.py` con emisor C (ruta correcta a largo
  plazo).
- **(b)** Escribir a mano `kuntur_ic_plant_state.h/c` y
  `kuntur_ic_surface_cmd.h/c`, con test de regresión que empaquete el
  mismo payload desde Python y desde C y verifique diff byte a byte.

**Recomendación: (b) para arrancar.** Son dos estructuras, el patrón de
`struct` + `pack`/`unpack` por campo es mecánico, y el test de regresión
previene drift. Migrar a (a) solo si el patrón se extiende a más de
3-4 mensajes.

## 7. Firmware STM32F767 — arquitectura mínima

PlatformIO + framework STM32Cube. Estructura esperada:

```
firmware/kuntur_autopilot/
├── platformio.ini                 # board = nucleo_f767zi; framework = stm32cube
├── src/
│   ├── main.c                     # init + loop principal
│   ├── usb_cdc_io.c/h             # pack/unpack buffers USB CDC + handshake
│   ├── icd/
│   │   ├── kuntur_ic_plant_state.c/h
│   │   └── kuntur_ic_surface_cmd.c/h
│   ├── autopilot/
│   │   ├── autopilot_state.h      # struct Autopilot_State_t (encapsula plane[] relevante)
│   │   ├── control_altitude.c
│   │   ├── control_heading.c
│   │   ├── control_gamma.c
│   │   ├── control_normal_accel.c # el pole placement vive acá
│   │   ├── control_lateral_accel.c
│   │   ├── control_rates.c        # control_roll (posición, mroll==0, gkp/gkphi vía wrcl/zrcl) + control_pitch_rate/control_roll_rate/control_yaw_rate (tasa, mroll==1 para roll) — caminos alternativos seleccionados en control_mix.c, no en cascada. Auditado 2026-10-10, ver spec-auditoria-control-roll-stm32.md
│   │   ├── control_mix.c
│   │   ├── guidance_line.c
│   │   ├── actuator.c
│   │   └── math_utils.c           # limitadores, filtros, trig
│   └── config/
│       └── scenario_params.h      # parámetros del .asc hardcodeados (gainp, gainl, pgam, wgam, zgam, waclp, zaclp, paclp, ...)
└── test/
    └── host/                      # tests que corren en host, no en MCU
        ├── test_bitexact.c
        └── fixtures/
            └── traces_falcon6_10pts.bin
```

**Decisiones técnicas específicas de PlatformIO + STM32Cube:**

- `framework = stm32cube` da acceso directo a HAL + middleware USB CDC
  ya portado. Setup USB CDC = incluir el middleware y configurar el
  descriptor; no es trabajo pesado.
- Habilitar FPU hardware (ya viene por default para el F767).
- Habilitar L1 I/D-cache. **Importante:** la memoria usada para
  descriptors USB debe estar marcada como no-cacheable o invalidada
  manualmente — mismo cuidado que para DMA.
- Reloj a 216 MHz (full speed).
- Sin RTOS. Loop principal gatillado por recepción USB CDC. Determinismo
  = interrupt latency (<5 μs) + tiempo de cómputo del autopiloto
  (~100-500 μs estimado) muy por debajo del presupuesto de 10 ms.
- `double` nativo en todo el autopiloto (FPU FPv5-D16 lo soporta).
  Esto es lo que habilita la validación bit-exacta contra Python.

**Lo que explícitamente no va en el firmware:**

- Pantalla, display, UI.
- Sistemas de archivos, EEPROM, flash para config.
- Nada relacionado con el FS-i6 (el adaptador SM100 habla con el host,
  no con el STM32).
- Ethernet. Nada de LwIP.
- RTOS.

## 8. Integración FS-i6 — el canal secundario

Independiente del lazo HIL. Camino único:

```
FS-i6 transmisor →(2.4G)→ SM100 USB dongle →(USB HID)→ Host PC
                                                              │
                                                              ▼
                                                   manual_joystick_worker.py
                                                   (nuevo, basado en spec
                                                    FS-i6 ya existente)
                                                              │
                                                              ▼
                                                   Si Kuntur-GCS modo == Manual:
                                                     empaqueta joystick → KUNTUR_COMMAND
                                                     plant_driver.py aplica
                                                     (bypass ítem #16)
                                                   
                                                   Si modo != Manual:
                                                     joystick ignorado, se usa
                                                     el output del STM32
```

**Lo que esto implica para el worker:**

- `manual_joystick_worker.py` lee el SM100 como joystick HID estándar
  (`pygame.joystick` o `evdev`), según `spec-control-manual-joystick-flysky-fsi6.md` §3.2.
- Kuntur-GCS mantiene un endpoint HTTP (o WS) para alternar entre Manual
  (joystick) y Automático (STM32).
- Mapeo de canales ya documentado en la spec existente — se puede reusar
  directo.

**Lo que NO hay que hacer:**

- No conectar el FS-i6 ni el SM100 al STM32.
- No pasar la señal del joystick por USB CDC hacia el STM32.
- No implementar lógica de modo en el STM32 (el STM32 siempre computa
  el output automático; es la planta quien decide qué fuente usar).

**Spec de bypass a implementar en la planta** (ítem #16 del backlog,
ya diseñado en `spec-control-manual-joystick-flysky-fsi6.md` §3.3):

```python
# En RealFalcon6Adapter.step(), entre control y actuator:
if self._manual_mode_active:   # flag de Kuntur-GCS
    cmd = self._last_joystick_cmd
    plane[519] = _map(cmd.aileron,  -plane[508], plane[508])
    plane[520] = _map(cmd.elevator, -plane[509], plane[509])
    plane[521] = _map(cmd.rudder,   -plane[510], plane[510])
    plane[52]  = _clip(cmd.throttle, 0.0, 1.0)
elif self._hil_mode_active:    # flag del stm32_bridge
    sc = self._last_stm32_surface_cmd
    if sc is not None and sc.age_ms < 30:
        plane[519], plane[520], plane[521] = sc.delacx, sc.delecx, sc.delrcx
    # else: fallback silencioso al cálculo nativo (control() nativo ya corrió)
# else: cálculo nativo (SIL puro, sin HIL ni manual)
```

## 9. Fork minimalista — qué queda, qué sale

Partiendo del `kuntur_cadac` actual, el nuevo repo (`kuntur_cadac_hil`
o nombre a elegir) conserva solo lo necesario para el flujo HIL +
Kuntur-GCS.

### Backend (`kuntur_cadac_hil`)

**Conservar (núcleo):**

```
kuntur_cadac_hil/
├── cadac_plant/          # planta Python, UNCHANGED salvo bypass de §8
├── cadac_py/             # motor CADAC completo, UNCHANGED
├── kse_server/           # broker Rust, UNCHANGED
├── icd_schema/
│   ├── kuntur_ic_v1_3.json           # contrato principal (planta ↔ KSE)
│   ├── kuntur_ic_aux_v1.json         # instrumentación FG (ya validada)
│   ├── kuntur_ic_plant_state_v1.json # NUEVO (host → STM32)
│   └── kuntur_ic_surface_cmd_v1.json # NUEVO (STM32 → host)
├── tools/
│   └── gen_icd.py        # mismo generador, se verifica no regresión
├── kuntur_bridge/
│   ├── main.py           # orquestador, con componentes depurados
│   ├── config.py, config_file.py
│   ├── logging_setup.py
│   ├── geodesy.py
│   ├── kse_client.py
│   ├── icd.py            # generado del contrato principal
│   ├── icd_aux.py        # generado del canal aux
│   ├── flightgear_worker.py          # visualización (UNCHANGED)
│   ├── telnet_client.py              # FlightGear property tree (UNCHANGED)
│   ├── fgnetfdm.py                   # protocolo FG (UNCHANGED)
│   ├── aux_instrument_worker.py      # instrumentación FG (UNCHANGED)
│   ├── telemetry_stream_worker.py    # SSE para Kuntur-GCS (UNCHANGED — Kuntur-GCS lo consume)
│   ├── stm32_bridge.py               # NUEVO — worker USB CDC ↔ STM32
│   ├── manual_joystick_worker.py     # NUEVO — SM100 USB HID
│   └── gcs_api_worker.py             # CONSERVADO Y SIMPLIFICADO — solo endpoint de mode switch; endpoints de retasking retirados
├── scenarios/
│   └── waypoints_cordoba.txt          # escenario validado
├── firmware/
│   └── kuntur_autopilot/              # NUEVO — proyecto PlatformIO
├── docs/                              # documentación de defensa
├── run_sim.py                         # launcher principal
└── runfg.sh                           # launcher FlightGear
```

**Eliminar (todo esto se queda fuera):**

- `kuntur_bridge/px4_bridge.py` — ruta PX4 abandonada
- `kuntur_bridge/falcon_fcs.py` — FCS Python alternativo no usado
- `kuntur_bridge/kuntur_fcs.py` — ídem
- `kuntur_bridge/icd_retask*.py`, `icd_route*.py` — generados del canal CTL (retasking)
- `kuntur_bridge/icd_aux_channel_delivery/` — copia de entrega vieja
- `kuntur_bridge/mission_bridge.py` — no existe (import roto)
- Toda la rama `elif component == 'mission'` en `main.py`
- Toda la rama `elif component == 'px4'` en `main.py`
- `icd_schema/kuntur_ic_ctl_v1.json` — canal CTL (retasking)
- `cadac_plant/protocol.py::CtlLink` — remover
- `ros2_ws/` — nunca se usó
- Dentro de `gcs_api_worker.py`: los endpoints `POST /retask`,
  `POST /route_replace`, los imports de `icd_retask*`/`icd_route*`, y el
  `CtlClient` TCP. Queda solo lo necesario para publicar/recibir el
  modo (Manual/Automático) y, opcionalmente, el estado del STM32
  (conectado, fallback count).
- Escenarios no usados

### Frontend (`kuntur-gcs`) — componente de primera clase

Kuntur-GCS **se mantiene** como el GCS construido para la tesina, ya
mayormente concluido. En la versión inicial del HIL:

**Conservar:**
- Mapa, ruta, marcador de aeronave, six-pack de telemetría — todo lo
  que ya consume `useTelemetry.ts` del SSE de
  `telemetry_stream_worker.py`.
- Panel/badge de modo autoritativo (`Patrullaje`, `Fuera de Ruta`, etc.).
- **Nuevo o reusado**: panel de modo del operador — toggle entre
  Automático (STM32) y Manual (joystick), que llama al endpoint
  simplificado de `gcs_api_worker.py`.
- **Nuevo**: indicador de estado del STM32 (conectado / activo /
  cantidad de fallbacks al cálculo nativo) — vive dentro del SSE
  existente, con un campo nuevo que el `stm32_bridge.py` publica.

**Esconder/retirar en v1 (sin borrar del código):**
- UI de retasking (clic en mapa → "Próximo Waypoint" → amarillo
  persistente → "Fuera de Ruta"): se desactiva en v1 porque el backend
  (`KUNTUR_CTL`) no está. El código queda en el repo pero oculto detrás
  de un flag; cuando se decida retomar retasking post-tesis, se enciende.
- UI de reemplazo de ruta / editor de misión (nunca se implementó;
  seguirá así).

Esto honra que Kuntur-GCS **ya está mayormente concluido**: no se
reescribe, solo se oculta la parte que depende de un canal que la v1 no
expone, y se le agregan dos piezas chicas (toggle de modo + indicador
STM32).

### Decisión sobre `gcs_api_worker.py`

Dado que Kuntur-GCS necesita un endpoint HTTP para el mode switch, la
decisión práctica es **conservar `gcs_api_worker.py` y simplificarlo**
en vez de crear uno nuevo:

- Mantener la estructura HTTP y CORS (ya resueltas, con preflight OK).
- **Retirar**: endpoints `POST /retask`, `POST /route_replace`, el
  cliente TCP a `plant_driver.py` (`CtlClient`), los imports de
  `icd_retask*`/`icd_route*`.
- **Agregar**: `POST /mode` (body: `{ "mode": "automatic" | "manual" }`),
  `GET /status` (devuelve estado del STM32 + modo actual).

Es el mismo trabajo que un worker nuevo, con el beneficio de reusar la
infraestructura CORS/HTTP ya probada.

## 10. Validación — estrategia por capas

El SIL validado es el oráculo. HIL se valida contra él en tres niveles
de rigurosidad:

### Nivel 0 — Equivalencia Python↔C en host (ANTES de tocar hardware)

- Portar las ~1000 líneas de autopiloto a C puro en el host.
- Capturar 10 traces completos de FALCON6 Python (`plane[]`/`flat6[]`
  cada paso durante 60 s de patrullaje) en 10 puntos distintos del
  escenario.
- Correr el C sobre las mismas traces, comparar salidas (`delacx`,
  `delecx`, `delrcx`, estado interno del autopiloto).
- **Gate:** diff numérico < 1e-10 en todas las muestras. Si no cumple,
  hay una divergencia que entender (y es, en sí misma, un hallazgo
  valioso para la tesis).

### Nivel 1 — SIL-decimado-a-100-Hz vs SIL-nativo-a-1-kHz

- Correr dos versiones del SIL en Python:
  - **A:** nativo, autopiloto llamado cada paso (1 kHz).
  - **B:** autopiloto llamado cada 10 pasos (100 Hz), superficies
    mantenidas entre llamadas.
- Comparar el soak test de patrullaje de 18 ciclos.
- **Gate:** dispersión de `dwbh` y `psivlx` por reenganche del mismo
  orden que el soak original (submilimétrica a pocos centímetros,
  centésimas de grado). Si la decimación degrada mucho, revisar la
  cadencia antes de pasar al Nivel 2.
- Este nivel es la **justificación cuantitativa** de la decisión de
  §5 (correr autopiloto a 100 Hz).

### Nivel 2 — Firmware STM32 en loopback (sin red)

- Compilar el C del Nivel 0 cross para Cortex-M7 con el toolchain de
  PlatformIO.
- Correr sobre las mismas 10 traces, embebidas en flash como testbench,
  salida sobre UART.
- Comparar contra la referencia Python.
- **Gate:** diff numérico < 1e-6 (tolerancia IEEE 754 double consistente).
  Si falla, hay un tema de precisión FPU o de orden de operaciones a
  investigar.

### Nivel 3 — HIL vivo

- `plant_driver.py` con `stm32_bridge` activo y STM32 conectado por USB CDC.
- Correr el soak test de patrullaje.
- Comparar la trayectoria resultante contra la referencia SIL
  (idealmente el SIL-a-100Hz del Nivel 1, que es la referencia justa).
- **Gate:** ciclo límite estable sobre ≥10 reenganches, dispersión del
  mismo orden que la referencia.
- Caracterización adicional (contenido documentable): latencia
  round-trip USB CDC, jitter, tasa de fallback a cálculo nativo (ideal
  = 0; cualquier valor > 0 es diagnóstico).

## 11. Plan de trabajo por fases

Cada fase es un corte limpio con entregable defendible por sí mismo.

### Fase 1 — Fork + limpieza + bypass + ICD (2 semanas)

- Crear el fork minimalista según §9.
- Implementar el bypass en `plane_adapter.py` (ítem #16): acepta
  superficies externas (del STM32 o del joystick) o cae al cálculo
  nativo.
- Simplificar `gcs_api_worker.py` según §9 (retirar endpoints de
  retasking, agregar endpoints de modo/estado).
- Esconder la UI de retasking en Kuntur-GCS tras un flag — el resto de
  Kuntur-GCS sigue funcionando sin cambios.
- Definir los dos schemas nuevos (`kuntur_ic_plant_state_v1.json`,
  `kuntur_ic_surface_cmd_v1.json`) y generar `icd_plant_state.py` /
  `icd_surface_cmd.py` desde `gen_icd.py`.
- Escribir manualmente los equivalentes C (`.h`/`.c`) y el test de
  regresión Python↔C byte a byte.
- Implementar `stm32_bridge.py` (worker USB CDC) con dos modos: STM32
  conectado o fallback silencioso.
- **Gate:** con `stm32_bridge` desactivado, el soak test de 18 ciclos
  es bit-exacto respecto al SIL original. Con `stm32_bridge` activo
  pero sin STM32 escuchando (todos los paquetes a timeout), el soak
  sigue bit-exacto (fallback funciona). Kuntur-GCS sigue mostrando
  telemetría normalmente.
- **Entregable:** fork minimalista corriendo, con canales y bypass
  listos; Kuntur-GCS ajustado al nuevo alcance; cero cambio funcional
  en lo visible todavía.

### Fase 2 — SIL-a-100-Hz (1 semana)

- Agregar flag `--autopilot-rate-hz` al `plant_driver.py`.
- Correr el soak test a 100 Hz, caracterizar dispersión.
- Comparar con el soak nativo a 1 kHz.
- **Gate:** el patrullaje converge al mismo ciclo límite dentro del
  mismo orden de dispersión.
- **Entregable:** validación cuantitativa de la cadencia elegida —
  contenido de tesis.

### Fase 3 — Portado del autopiloto a C en host (3 semanas)

- Portar línea por línea las ~15 funciones listadas en §3 a C.
- Escribir el testbench Nivel 0 con 10 traces de FALCON6.
- Validar diff < 1e-10 contra Python.
- **Gate:** Nivel 0 pasa.
- **Entregable:** `libautopilot.a` (static library) + tests que
  demuestran equivalencia bit-exacta contra FALCON6 Python.

### Fase 4 — Firmware STM32 offline (2 semanas)

- Setup del proyecto PlatformIO.
- Compilar el C del Nivel 0 cross para Cortex-M7.
- Testbench embebido en flash + reporte sobre UART.
- Habilitar FPU + cache + reloj a 216 MHz.
- Validar diff < 1e-6 contra referencia Python.
- Medir ciclos por iteración (performance budget).
- **Gate:** Nivel 2 pasa.
- **Entregable:** firmware STM32 que ejecuta el autopiloto FALCON6
  offline, con reporte de performance (ciclos, μs, headroom).

### Fase 5 — USB CDC integration (1.5 semanas)

- Habilitar middleware USB CDC en el firmware.
- Implementar el loop `receive KUNTUR_PLANT_STATE → compute → send
  KUNTUR_SURFACE_CMD`.
- Verificar handshake con `stm32_bridge.py` del host.
- Medir latencia round-trip, jitter.
- **Gate:** `stm32_bridge` recibe respuestas del STM32 dentro de
  presupuesto (<30 ms de margen de fallback con tolerancia).
- **Entregable:** HIL funcional sobre USB CDC, con metrics.

### Fase 6 — HIL completo y validación (1.5 semanas)

- Correr el soak test completo con `stm32_bridge` activo.
- Comparar trayectoria contra SIL-a-100-Hz (Nivel 1 referencia).
- Caracterizar: estabilidad del ciclo límite, dispersión por reenganche,
  tasa de fallback, latencia, jitter.
- Agregar a Kuntur-GCS el indicador de estado del STM32 (conectado /
  activo / fallbacks) — pieza chica que consume un campo nuevo del SSE.
- **Gate:** Nivel 3 pasa.
- **Entregable:** HIL validado contra el soak test de 18 ciclos,
  observable desde Kuntur-GCS.

### Fase 7 — FS-i6 Manual + mode switch desde Kuntur-GCS (1 semana)

- Implementar `manual_joystick_worker.py` según spec FS-i6 existente,
  usando el SM100 como joystick HID.
- Agregar a Kuntur-GCS el toggle Automático/Manual (nueva pieza en el
  panel de control, usa el endpoint `POST /mode` de
  `gcs_api_worker.py` simplificado).
- Probar: vuelo recto manual, viraje, cambio de altitud; alternar entre
  HIL (STM32) y Manual (joystick) en vuelo desde Kuntur-GCS.
- **Entregable:** sistema completo con dos fuentes de control
  conmutables desde Kuntur-GCS.

### Fase 8 — Documentación y defensa (1-2 semanas)

- Documento de tesis: arquitectura (los tres planos — HIL, GCS,
  Manual), decisiones (cadencia, encapsulación de estado, elección USB
  CDC vs Ethernet, qué sale del canal CTL en v1), validación
  cuantitativa de los 4 niveles, caracterización de performance.
- Diagramas actualizados (bloques, secuencia, estados).
- Preparación de demos para la defensa (que Kuntur-GCS cubre
  visualmente).

**Tiempo total estimado:** ~13 semanas de trabajo neto a medio tiempo
(~26 semanas calendario con otras ocupaciones). Es un plan ambicioso.

## 12. Riesgos y mitigaciones

| Riesgo | Prob. | Impacto | Mitigación |
|---|---|---|---|
| Divergencia numérica Python↔C en el portado del pole placement | Media | Medio | Gate Nivel 0 obligatorio; traces de 10 puntos distintos; si diverge, documentar como hallazgo (orden de operaciones, truncamiento, etc.) |
| Un lazo del autopiloto resulta más acoplado al `plane[]` global de lo esperado | Media | Medio | Portar línea por línea, no refactorizando; mapear cada acceso `plane[i]` a un campo nombrado en `Autopilot_State_t`. Si aparece un acople no documentado, es contenido de tesis sobre "qué significa realmente desacoplar un autopiloto embebido en una planta" |
| 100 Hz resulta insuficiente para los rate loops | Baja | Alto | Gate Nivel 1 detecta esto antes de tocar hardware. Mitigación: evaluar multi-rate (outer a 50 Hz, rate loops a 200 Hz) con dos tipos de paquete; o migrar a Ethernet |
| USB CDC introduce más latencia que lo medido | Baja | Medio | Fallback silencioso a cálculo nativo preserva funcionamiento; latencia caracterizada es contenido de tesis |
| El aerodeck en Python degrada numéricamente al pasar las derivadas al STM32 por doble conversión | Baja | Bajo | Transportar `double` nativo (sin convertir a `float`) sobre USB CDC — 8 bytes por valor |
| Alcance se expande (p. ej. "ya que estoy, porto también la física al STM32") | Media | Alto | Decisión documentada en §3: la física se queda en Python. Si aparece interés, es post-tesis |
| El toggle de Kuntur-GCS introduce transientes de control al cambiar de modo | Media | Bajo | Suavizar transición (rampa de 0.5 s entre fuentes); caracterizar y documentar |
| Esconder la UI de retasking rompe algo en Kuntur-GCS (dependencias no obvias) | Baja | Bajo | Flag de build/env (`VITE_ENABLE_RETASKING=false`); el código queda intacto para encender post-tesis |

## 13. Fallbacks explícitos

Si el tiempo aprieta, corte limpio por fase:

- **Hasta Fase 2:** entregable "arquitectura HIL diseñada + bypass +
  canales + validación cuantitativa de cadencia a 100 Hz" — defendible
  como diseño sin implementación hardware.
- **Hasta Fase 3:** entregable "portado offline validado bit-exacto
  contra FALCON6 Python" — equivalencia numérica demostrada,
  preparación completa para HIL.
- **Hasta Fase 4:** entregable "firmware STM32 ejecutando autopiloto
  FALCON6 completo auto-validado" — HIL en sentido estricto
  (controlador en hardware) sin integración viva.
- **Hasta Fase 6:** HIL completo validado. Meta primaria de la tesis.
- **Fase 7:** FS-i6 es opcional estricto. No bloquea defensa.

## 14. Qué queda explícitamente fuera

- La física FALCON6 **no se porta** — se queda en Python a 1 kHz.
- `kse_server` **no se toca**.
- El contrato Kuntur-IC v1.3 principal **no se toca** — canales nuevos
  siguen el patrón `KUNTUR_AUX`.
- La Pixhawk **no participa** del proyecto en ninguna forma.
- Ethernet / LwIP **no se usa** — USB CDC es suficiente.
- El canal `KUNTUR_CTL` (retasking) **se retira del backend** en la
  versión inicial. La UI que lo consumía en Kuntur-GCS **se oculta
  tras un flag** (no se borra). Si post-defensa se decide retomar
  retasking, se enciende el flag y se vuelve a compilar el canal CTL.
- ROS 2 **no se usa**.
- FlightGear **no se modifica** — rol actual intacto.
- No se desarrolla un autopiloto nuevo: el firmware STM32 es un portado
  línea-por-línea del autopiloto que ya está en FALCON6, validado por
  equivalencia bit-exacta.

## 15. Qué traer a la próxima sesión

1. Confirmación del encuadre con el director (gate aún abierto): "HIL
   clásico — controlador completo en hardware real, planta en software,
   validado contra el SIL ya auditado". Más fuerte que las versiones
   anteriores del plan.
2. Nombre definitivo del fork (`kuntur_cadac_hil` es tentativo).
3. Setup Nucleo-144 + PlatformIO verificado — compilar un blink antes
   de empezar.
4. Decisión sobre el emisor C del codegen: (a) extender `gen_icd.py` o
   (b) escribir a mano con test de regresión byte a byte.
5. Cómo esconder la UI de retasking en Kuntur-GCS — flag de build
   (`VITE_ENABLE_RETASKING`) vs ruteo (ocultar botón de "Próximo
   Waypoint" en el MapPanel) vs otra alternativa; decisión que afecta
   al frontend más que al backend.
