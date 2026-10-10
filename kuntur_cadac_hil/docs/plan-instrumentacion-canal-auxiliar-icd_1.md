# Plan: canal ICD auxiliar de instrumentación (cockpit completo, kuntur-cadac fork)

**Estado: implementado y validado en prueba de humo con FlightGear real
(punto 6 completo). Canal `KUNTUR_AUX` confirmado funcionando de punta a
punta — potencia → sonido de motor audible y reactivo, sobre perfil
`757`. La aeronave `f16-simplified`/`f16-block-*` (paquete FGAddon)
quedó descartada para `--fdm=null` por una incompatibilidad estructural
de su Nasal con la ausencia de JSBSim — ver "Prueba de humo (equipo
real)" más abajo para el detalle completo, incluyendo dos bugs de
despliegue encontrados y corregidos en `run_sim.py`/`telnet_client.py`
que no tienen que ver con este canal pero bloqueaban poder probarlo.**

**Este documento reemplaza a `claude/plan-sonido-motor-flightgear.md`**
(marcado como superado — ver ese archivo). El sonido de motor queda
absorbido acá como un caso particular de un contrato más general de
instrumentación.

## Encuadre general de esta etapa

Se unifican dos frentes que estaban en documentos separados:

1. **Instrumentación básica de vuelo** (actitud, altitud, velocidad, AOA,
   sideslip, factor de carga) — no cubierta antes por ningún plan.
2. **Sonido e indicadores de motor** — cubierto antes por
   `plan-sonido-motor-flightgear.md`, ahora absorbido acá con datos reales
   en vez de supuestos.

Ambos se resuelven con el mismo mecanismo: un **canal ICD auxiliar
nuevo**, separado del contrato Kuntur-IC v1.3 (`KUNTUR_STATES`/
`KUNTUR_COMMAND`, definido en `icd_schema/kuntur_ic_v1_3.json`).

**Explícitamente pospuesto, fuera de esta etapa:** Fases 1-2 de
`plan-desacople-gnc-autopiloto-guiado.md` (extracción de sintonía y
separación interna Planta/Autopiloto/Guiado). El objetivo inmediato es un
cockpit funcional completo que permita luego el análisis de arquitectura
y sintonía sobre la implementación actual en CADAC, con la mira puesta en
eventualmente replicar esa arquitectura en Simulink. Ese plan de
desacople probablemente necesite revisarse con este objetivo en mente
("caracterizar para replicar" en vez de "desacoplar en tres procesos")
antes de retomarlo — son objetivos relacionados pero no idénticos.

## Diagnóstico confirmado (no requiere más investigación)

- `--fdm=null` hace que FlightGear nunca instancie un FDM interno: los
  árboles `/engines/engine[0]` y `/fdm/jsbsim/propulsion` existen (los
  nodos padre aparecen en `ls`) pero quedan completamente vacíos —
  confirmado por `dump` vía Telnet. No hay ninguna consulta en tiempo de
  ejecución que revele más que lo que ya se obtuvo leyendo los `.xml` del
  paquete directamente.
- **Corrección post-prueba de humo real:** lo anterior es cierto para
  `engines/engine[0]/*` (que sí aparece, vacío) pero **no generaliza** a
  todo alias derivado de JSBSim. Confirmado en el equipo real: aliases
  como `gear/gear/position-norm` (sin índice — no es lo mismo que
  `gear/gear[0]/position-norm`) o `/fdm/jsbsim/propulsion/tank[i]/priority`
  **ni siquiera existen como nodo** bajo `--fdm=null` — `getprop()` sobre
  ellos devuelve `nil`, no `0`. El Nasal de sistemas del paquete F-16 de
  FGAddon (`f16.nas`/`fuel.nas`, compartido por **todas** las variantes
  de bloque y por `f16-simplified` por igual) no verifica `nil` antes de
  usarlos en contexto numérico, y crashea en la inicialización — ver
  "Prueba de humo (equipo real)" para el detalle exacto. Esto invalida
  la premisa original de que `f16-simplified` "arrastra menos efectos
  Nasal": comparte el Nasal completo de sistemas con los blocks
  completos, solo cambia el dataset aerodinámico del `-set.xml`.
- Los archivos `Systems/jsb-propulsion-block*.xml` (block1/20/30/32/42/50/
  52/60) son configuraciones de motor **de JSBSim**, usadas por las
  variantes `f16-block-XX-set.xml` que sí corren con FDM completo. No
  aplican a `f16-simplified` bajo `--fdm=null` — se cierra esta vía de
  investigación como irrelevante para este canal, no como pendiente.
- Aeronave real: `f16-simplified`, `-set.xml` ubicado en
  `.../org.flightgear.fgaddon.stable_2020/Aircraft/f16/f16-simplified-set.xml`
  (nota: hay un nivel `Aircraft/` interno dentro del catálogo FGAddon que
  no era evidente desde `--fg-aircraft=`). Comparte `Systems/f16-sound.xml`
  con las demás variantes de bloque de la carpeta padre `f16/`.
- `cadac_py::falcon6.Plane` no modela N1/N2 como ejes independientes, ni
  temperatura de motor (EGT/ITT, confirmado ausente en el grep de
  propiedades del XML de sonido), ni postcombustión, ni APU/JFS. Son
  limitaciones del modelo, documentadas como tales — no se compensan con
  datos sintéticos salvo donde se indica explícitamente.
- `AUTHORITY_DRIVEN` (uno de los dos modos de `exec_mode` en
  `KUNTUR_COMMAND`) está previsto pero no implementado — hoy todo corre en
  `ENGINE_DRIVEN`, con paceo propio en `plant_driver.py::run()`. Esto
  corrige una lectura anterior de `arquitectura-fusion-kuntur-cadac.md`,
  que describe el modo dual como si ambos estuvieran igualmente
  operativos — vale la pena anotar esta discrepancia en ese documento
  cuando se lo retome.

## Decisiones de arquitectura (cerradas)

| Punto | Decisión | Razón |
|---|---|---|
| Transporte | Directo `plant_driver.py → kuntur_bridge`, **sin pasar por `kse_server`** | La señal no participa de la semántica que KSE arbitra (`ENGINE_DRIVEN`/`AUTHORITY_DRIVEN`); es adaptación de un simulador batch a un esquema en tiempo real, no un contrato de control planta↔FCS |
| Puerto | `7000` (a confirmar disponibilidad en el equipo antes de correr, ej. `ss -tuln \| grep 7000`) | Convención +1000 sobre 6000/6001 (canal principal) |
| Tasa de transporte | 1 kHz en origen (mismo `int_step`), **decimado a 25 Hz** antes de enviar | 1000/25 = 40, factor entero exacto — evita jitter de fase (a diferencia de 30 Hz, que da 33.33 no entero) |
| Dónde decima | `plant_driver.py`, contador módulo 40, mismo lugar donde ya se llama `link.send_states()` | Mantiene `kuntur_bridge` simple (reenvío 1:1) y no agrega carga al lazo caliente de 1 kHz |
| Tipo de dato | `f32`, empaquetado explícito sin padding de alineación de compilador | Misma convención que `KUNTUR_STATES`/`KUNTUR_COMMAND` (confirmado: ningún campo en `kuntur_ic_v1_3.json` es `f64`, y el `expected_size` de 92 bytes solo cierra sin padding) |
| Schema | Archivo separado `icd_schema/kuntur_ic_aux_v1.json`, **sin generar binding Rust** | Preserva `kse_server` acotado a su rol de broker; evita código muerto en un componente ya enfocado |
| Correlación temporal | Comparte `t_sim_us` + `seq_num` con `KUNTUR_STATES`, no usa contador propio | Permite correlacionar el paquete auxiliar decimado contra el estado completo durante logueo — insumo para la caracterización futura |
| `tpower` (constante de spool-up) | Excluido del contrato | `power` ya es salida de un integrador con esa dinámica incorporada |

## Schema — `icd_schema/kuntur_ic_aux_v1.json`

```json
{
  "_readme": "Canal auxiliar de instrumentacion Kuntur-AUX v1. NO es parte del contrato Kuntur-IC v1.3 (kuntur_ic_v1_3.json) y NO pasa por kse_server -- transporte directo plant_driver.py -> kuntur_bridge, puerto 7000, decimado a 25 Hz desde el paso base de 1 kHz (submuestreo cada 40 pasos). Proposito: adaptar senales de instrumentacion (vuelo + motor) del simulador batch cadac_py hacia FlightGear via Property Tree/Telnet -- no es un canal de control planta<->FCS. gen_icd.py debe generar unicamente icd_aux.py (Python) -- SIN binding Rust -- para preservar la separacion de responsabilidad de kse_server como broker acotado a Kuntur-IC v1.3.",
  "version": "1.0",
  "transport": {
    "path": "plant_driver.py -> kuntur_bridge",
    "via_kse_server": false,
    "port": 7000,
    "rate_hz": 25,
    "decimation": "cada 40 pasos de int_step=0.001s, aplicada en plant_driver.py"
  },
  "codegen": {
    "python": true,
    "rust": false
  },
  "packets": {
    "KUNTUR_AUX": {
      "magic_hex": "0x4B415831",
      "magic_str": "KAX1",
      "direction": "plant_driver.py -> kuntur_bridge",
      "expected_size": 60,
      "fields": [
        {"name": "magic_header", "kind": "scalar", "type": "u32", "unit": null, "note": "framing"},
        {"name": "t_sim_us", "kind": "scalar", "type": "u64", "unit": "us", "note": "mismo reloj que KUNTUR_STATES.t_sim_us"},
        {"name": "seq_num", "kind": "scalar", "type": "u32", "unit": null, "note": "mismo contador que KUNTUR_STATES.seq_num -- NO es un contador propio del canal auxiliar"},
        {"name": "dvbe", "kind": "scalar", "type": "f32", "unit": "m/s", "note": "TAS -- cadac_py flat6[236]"},
        {"name": "vmach", "kind": "scalar", "type": "f32", "unit": null, "note": "Mach -- cadac_py flat6[56]"},
        {"name": "pdynmc", "kind": "scalar", "type": "f32", "unit": "Pa", "note": "presion dinamica -- cadac_py flat6[57] -- usada para derivar IAS/EAS del lado kuntur_bridge"},
        {"name": "hbe", "kind": "scalar", "type": "f32", "unit": "m", "note": "altura AGL -- cadac_py flat6[239]"},
        {"name": "altrate", "kind": "scalar", "type": "f32", "unit": "m/s", "note": "variometro -- cadac_py plane[530]"},
        {"name": "alppx", "kind": "scalar", "type": "f32", "unit": "deg", "note": "AOA -- cadac_py flat6[140]"},
        {"name": "betax", "kind": "scalar", "type": "f32", "unit": "deg", "note": "sideslip -- cadac_py flat6[145]"},
        {"name": "anx", "kind": "scalar", "type": "f32", "unit": "g", "note": "factor de carga normal -- cadac_py flat6[244]"},
        {"name": "throttle", "kind": "scalar", "type": "f32", "unit": null, "range": [0.0, 1.0], "note": "cadac_py plane[52] -- fuente de 'running' derivado en kuntur_bridge (running = throttle > 0)"},
        {"name": "power", "kind": "scalar", "type": "f32", "unit": "%", "note": "potencia lograda (estado integrado, incluye dinamica de spool-up) -- cadac_py plane[59] -- escrita a n1 y n2 de FlightGear (indexado y sin indexar, ver mapeo)"},
        {"name": "thrust", "kind": "scalar", "type": "f32", "unit": "N", "note": "empuje -- cadac_py plane[55]"}
      ]
    }
  }
}
```

Layout resultante (60 bytes, sin padding): igual que en la sesión de
diseño — cabecera `magic_header`(u32) + `t_sim_us`(u64) + `seq_num`(u32)
en offset 0-15, luego los 11 campos `f32` en offset 16-59.

## Cambio necesario en `gen_icd.py`

Hoy la herramienta genera siempre `icd.py` + `icd.rs` + el header C a
partir de un único JSON. Para este archivo separado necesita una rama
condicional (detectar el bloque `codegen`, o asumir "solo Python" para
cualquier schema que no sea `kuntur_ic_v1_3.json`) que emita únicamente
`icd_aux.py`, sin tocar `icd.rs` ni el header C.

## Mapeo verificado a Property Tree de FlightGear

Obtenido leyendo el contenido completo de
`Systems/f16-sound.xml` y el listado ordenado de todas las propiedades
que referencia (no una muestra parcial) — reemplaza cualquier mapeo
especulativo de sesiones anteriores.

### Hallazgo central: dos familias de propiedades, no una

El XML usa **dos rutas de propiedad distintas para el mismo dato físico**:

| Familia | Ejemplos | Usada por |
|---|---|---|
| Indexada | `engines/engine[0]/n1`, `.../n2`, `.../running`, `.../nozzle-pos-norm` | Sonidos internos de cabina (`engine0-*-low`, `engine0-*-high`, `engine0n2-*`, `engine0n100-*`, `burner0-*`) |
| Sin indexar | `engines/engine/n1`, `engines/engine/n2`, `engines/engine/running`, `engines/engine/augmentation` | Sonidos externos (roar, fan, rumble, exhaust, afterburner) |

Bajo un FDM real (JSBSim en marcha), estas suelen quedar sincronizadas
por el propio motor de FDM o por un alias declarado en la configuración
del avión. Bajo `--fdm=null` **no hay ninguna garantía de que exista ese
alias** — JSBSim nunca corre, así que nada sincroniza una familia con la
otra. **Conclusión de diseño: `kuntur_bridge` debe escribir ambas
familias explícitamente, con el mismo valor**, o el sonido interno
funcionará pero el externo (y viceversa) quedará mudo.

**Confirmado en la prueba de humo real (perfil `757`): esta decisión de
diseño es correcta — el motor sonó y reaccionó al throttle escribiendo
ambas familias.**

### Tabla de mapeo — motor y sonido

| Campo del canal auxiliar | Property(s) FG destino | Nota |
|---|---|---|
| `power` | `engines/engine[0]/n1`, `engines/engine[0]/n2`, `engines/engine/n1`, `engines/engine/n2` (mismo valor en las cuatro) | Ver limitación de modelo abajo (N1=N2) |
| *(derivado)* `throttle > 0` | `engines/engine[0]/running`, `engines/engine/running` | Booleano puro en el XML, sin umbral |
| — (fijo) | `engines/engine[0]/nozzle-pos-norm = 0`, `engines/engine/augmentation = false` | Afterburner: sin modelo en `cadac_py` (gap de modelo, no de mapeo) |
| — (fijo) | `f16/engine/jfs-rpm-normalized = 0` | Arrancador/APU (JFS): sin modelo en `cadac_py`, no es continuo durante vuelo — se deja en 0 |
| `thrust` | Sin uso en `f16-sound.xml` — reservado para instrumento de empuje en panel | — |

**Confirmación de escala (ya verificada, ahora con el XML completo a la
vista):** la curva de pitch en `engine0-front-left-low`
(`offset=0.86349949816, factor=0.00455001673` sobre `n1`) y la condición
de afterburner (`n2 >= 100`) solo tienen sentido si `n1`/`n2` están en
escala 0–100 — coincide exactamente con `power` (P[59], "% Achieved power
setting"). No hace falta conversión de unidades.

**Limitación de modelo, documentada como tal:** un turbofan real tiene
N1/N2 con dinámica independiente; `cadac_py` solo modela una variable de
potencia integrada. Escribir el mismo valor a las cuatro propiedades
(`n1`/`n2` × indexado/sin indexar) es la única opción defendible sin
inventar una relación que el modelo no sustenta.

### Propiedades derivadas que puede computar el propio XML — específico del paquete F-16, descartado junto con él

El XML de `f16-sound.xml` referencia `engines/engine[0]/sound-n2-vol-a/b/c/d`
(crossfade entre capas de sonido) y `engines/engine[0]/n1-out1`, `n1-out2`
(volumen de sonido externo), calculadas por un script Nasal propio del
paquete. **No llegó a verificarse si se actualizan solas** porque el
paquete F-16 completo quedó descartado por la incompatibilidad con
`--fdm=null` documentada en "Prueba de humo (equipo real)" — este punto
queda sin resolver para F-16 específicamente, y sin aplicar al perfil
`757` en uso (que tiene su propio `sound.xml`, no relevado).

### Tabla de mapeo — instrumentación básica de vuelo

Todas confirmadas presentes en el grep de propiedades del XML de sonido
(reutilizadas ahí para efectos de viento/G, lo cual confirma que son
propiedades genéricas de FlightGear, no específicas del paquete):

| Campo | Property FG destino | Conversión |
|---|---|---|
| `hbe` | `position/altitude-agl-ft` | metros → pies (`× 3.28084`) |
| `alppx` | `orientation/alpha-deg` | directo (ya en grados) |
| `dvbe` | `velocities/airspeed-kt` (o `fdm/jsbsim/velocities/vtrue-kts` si algún efecto lo requiere puntualmente) | m/s → nudos (`× 1.94384`) |
| `altrate` | `velocities/vertical-speed-fps` | m/s → ft/s (`× 3.28084`) |
| `anx` | `accelerations/pilot-g` | directo (ya en g) |
| `pdynmc` | — (no hay property directa de presión dinámica en el árbol genérico) | usar para derivar velocidad indicada si se necesita un EAS/IAS separado de TAS: `V_eas = sqrt(2·pdynmc / 1.225)` en m/s, luego convertir a nudos |

**Nota sobre `accelerations/pilot-gdamped`:** el XML usa esta variante
amortiguada para el efecto de sonido de G, además de `pilot-g` puro (que
aparece en el grep aunque no se vio en el fragmento de XML leído). Es
razonable asumir que `pilot-gdamped` es un filtro calculado del lado
FlightGear a partir de `pilot-g` — específico del paquete F-16
descartado, no verificado.

**No incluida en esta iteración, candidata a agregar sin tocar el
schema:** `velocities/groundspeed-kt`, referenciada en el XML pero sin
señal directa equivalente en `cadac_py` (que no modela viento activo por
defecto en el escenario de referencia). Aproximación razonable:
`groundspeed ≈ dvbe` (TAS) cuando `mwind=0` en el `.asc` — no requiere
campo nuevo en el canal, se deriva del mismo `dvbe` ya transportado. **NO
escrita en esta sesión** — sigue pendiente de decisión (ver "Qué traer a
la próxima sesión").

### Sinergia no buscada — específica del paquete F-16 descartado

`Models/Cockpit/Main/engine.xml` existe en el paquete F-16 — probablemente
las agujas de los instrumentos de motor en cabina leerían las mismas
propiedades `engines/engine[0]/n1`/`n2`. Sin relevancia práctica ahora
que ese paquete quedó descartado (ver "Prueba de humo (equipo real)");
si se retoma instrumentación de panel, habría que rehacer este
relevamiento sobre el paquete visual que se termine usando.

## Orden de implementación

1. ✅ Crear `icd_schema/kuntur_ic_aux_v1.json` (contenido arriba, sin cambios respecto del diseño).
2. ✅ Extender `tools/gen_icd.py`: nueva función genérica `gen_python_generic()`
   (independiente de `gen_python()`, que sigue hardcodeada a
   STATES/COMMAND sin tocar) + rama en `main()` que detecta si el schema
   es el contrato principal (`kuntur_ic_v1_3.json`) o no, y en ese caso
   emite únicamente `kuntur_bridge/icd_aux.py`. Verificado que
   `icd.py`/`icd.rs`/`kuntur_ic_generated.h` quedan byte-idénticos al
   regenerar el contrato principal (sin regresión).
3. ✅ `cadac_plant/plant_driver.py`: contador módulo 40 (`AUX_DECIMATION`)
   sobre el loop; en cada disparo empaqueta y envía por UDP, además de
   (no en reemplazo de) `link.send_states()`. Nuevo:
   `protocol.AuxTelemetry` (dataclass) + `protocol.AuxTelemetryLink`
   (TX), y `PlantAdapter.aux_telemetry()` implementado tanto en
   `MockFalcon6Adapter` (aproximación simple, documentada como tal) como
   en `RealFalcon6Adapter` (índices `flat6`/`plane` reales, verificados
   contra `cadac_py/falcon6.py`). CLI: `--aux-ip`/`--aux-port` nuevos,
   default `127.0.0.1:7000`. Si `int_step` ≠ 0.001s, loguea un warning
   con la tasa efectiva real del canal (la decimación x40 es fija).
4. ✅ `kuntur_bridge/aux_instrument_worker.py` (nuevo): listener UDP del
   puerto 7000, desempaqueta con `icd_aux.unpack_aux`. Corre en un hilo
   propio, con **su propia conexión Telnet dedicada**
   (`telnet_client.connect_property_tree()`) — no comparte el
   `FGPropertyTree` de `flightgear_worker.py`, que no es thread-safe
   (un único socket síncrono sin locks; compartirlo entre dos hilos
   podía entrelazar respuestas de `get()`/`set()`). Se lanza desde
   `main.py` junto con el componente `flightgear` (mismo `enabled`,
   nuevo hilo `AuxInstrumentWorker`). Puerto configurable vía
   `[network] kuntur_aux_port` en el `.ini` (`config_file.py`).
5. ✅ `kuntur_bridge/telnet_client.py::apply_aux_instrumentation()`:
   mapeo completo — ambas familias de propiedad de motor
   (indexada/sin indexar), `running` derivado de `throttle > 0`,
   afterburner/JFS fijos en 0. **Deliberadamente sin escribir** (ver
   plan B / pendientes arriba): `sound-n2-vol-*`/`n1-out1`/`n1-out2`/
   `pilot-gdamped` (específico del paquete F-16, descartado) y
   `velocities/groundspeed-kt` (pospuesto a próxima sesión). `vmach`/
   `pdynmc`/`thrust` viajan en el paquete pero sin property destino
   todavía (no había mapeo firme decidido para ellos en este plan).
6. ✅ **Completo — prueba de humo en la máquina real del usuario.**
   Resultado, en el mismo orden del plan original:
   - `run_sim.py` + `plant_driver.py --adapter real`: corrido con éxito,
     tras corregir dos bugs de despliegue (ver "Prueba de humo (equipo
     real)").
   - Confirmado por Telnet y de oído que `engines/engine[0]/n1` y
     `engines/engine/n1` dejan de estar en 0 y siguen a `power`, con
     perfil `757`.
   - Confirmado que el sonido de motor reacciona al throttle del
     escenario (perfil `757`).
   - `sound-n2-vol-*`/`n1-out1`/`n1-out2`/`pilot-gdamped` y el panel de
     `engine.xml`: **no verificables** — eran específicos del paquete
     F-16, descartado por incompatibilidad con `--fdm=null` (ver más
     abajo). Quedan como trabajo futuro si se define un nuevo perfil
     visual.
   - Instrumentación básica de vuelo: validada indirectamente por el
     funcionamiento correcto del patrullaje (altímetro/velocidad
     consistentes con los logs de `plant_driver.py`); no se hizo una
     inspección visual dedicada de cada instrumento de cabina.

## Implementación (sesión de código)

Todo el código (puntos 1-5) se implementó y se verificó **sin FlightGear
real**, con pruebas dirigidas que no reemplazan el punto 6 pero sí
validan que el cableado es correcto antes de necesitar el equipo del
usuario:

- Round-trip `pack_aux`/`unpack_aux` (framing, tamaño, magic header, caso
  de error) sobre `kuntur_bridge/icd_aux.py` generado.
- `plant_driver.run()` de punta a punta con `MockFalcon6Adapter` y con
  `RealFalcon6Adapter` (física real de `cadac_py`, escenario
  `f16c11_7.asc`) enviando a un socket UDP propio: decimación x40
  correcta, `seq_num` compartido con `KUNTUR_STATES`, valores físicos en
  rango razonable (dvbe, vmach, power durante el spool-up del motor).
- `kuntur_bridge/aux_instrument_worker.py` contra un servidor Telnet
  falso (emulando el Property Tree de FlightGear): se verificaron los
  14 comandos `set` exactos que produce `apply_aux_instrumentation()`
  para un paquete sintético — ambas familias de motor, `running`
  derivado, fijos de afterburner/JFS, y las 5 conversiones de unidades
  de instrumentación básica (todas dentro de tolerancia de precisión
  `f32`).
- Regeneración del contrato principal (`kuntur_ic_v1_3.json`) después de
  extender `gen_icd.py`: `icd.py`/`icd.rs`/`kuntur_ic_generated.h`
  quedan byte-idénticos — sin regresión en Kuntur-IC v1.3.

**Bug encontrado y corregido en el propio generador, durante esta
sesión:** la primera versión de `gen_python_generic()` producía un
mensaje de error de tamaño con el nombre de la constante *sin evaluar*
(`f"tamaño {len(data)} != AUX_SIZE bytes"`, literal, en vez de
interpolar el valor en runtime) — encontrado por el round-trip test, no
por inspección visual. Corregido antes de generar el archivo final.

Quedan como no implementados a propósito (ver tablas de mapeo arriba):
`sound-n2-vol-*`/`n1-out1`/`n1-out2`/`pilot-gdamped` (específico del
paquete F-16, descartado tras la prueba de humo) y
`velocities/groundspeed-kt` (pospuesto). No se tocó
`kuntur_bridge/README.md` ni `docs/Guia_Producto_Kuntur_Bridge_KSE_CADAC.md`
— quedan desactualizados respecto del nuevo componente
`aux_instrument_worker`/canal `KUNTUR_AUX` si se los retoma.

## Prueba de humo (equipo real)

Ejecutada en la máquina real del usuario (Ubuntu, FlightGear 2020.3.6),
con el flujo normal de dos terminales
(`run_sim.py start --aircraft <perfil>` + `plant_driver.py --adapter real`).

### Bugs de despliegue encontrados y corregidos (no son del canal `KUNTUR_AUX`, pero lo bloqueaban)

- **Condición de carrera en `run_sim.py`**: el `STARTUP_GUARD_S=1.0`
  (sleep fijo) antes de lanzar `kuntur_bridge` no alcanzaba con carga de
  escena real (no cacheada) — `calibrate_origin()` agotaba sus
  reintentos y FlightGear quedaba pausado para siempre
  (`--enable-freeze` nunca se levantaba). Corregido reemplazando el
  sleep fijo por polling activo del puerto Telnet
  (`_wait_for_flightgear_telnet()`, hasta 90s).
- **Bug de calibración de origen ("Null Island" parcial)** en
  `telnet_client.py::calibrate_origin()`: el chequeo de validez solo
  descartaba la lectura si lat Y lon daban *exactamente* `(0,0)` — un
  `lat=0.0` con un `lon` "creíble" (de hecho, el propio valor de `--lat`
  corrido al campo de `lon` por un desfasaje de lectura del ack de
  Telnet) pasaba el chequeo igual, calibrando el origen en un lugar del
  mundo sin relación con SACO/Córdoba. Reproducido en producción
  (`lat=0.000000 lon=-31.310000 alt=-19.57m`) y corregido exigiendo
  `lat0 != 0.0 and lon0 != 0.0` — deja que el retry loop existente
  (5×2s) haga su trabajo.
- Ambos confirmados corregidos por el usuario en el equipo real: origen
  calibrado correctamente, patrullaje en bucle funcionando sobre los 4
  waypoints reales de Córdoba (`waypoints_cordoba.txt`: SACO, Templo
  SUD, EAM/FAdeA, UTN FRC).

### Canal `KUNTUR_AUX` — confirmado funcional de punta a punta

Con perfil `757` (`KUNTUR_AIRCRAFT=757`): `aux_instrument_worker`
escuchando UDP:7000, conexión Telnet dedicada establecida, `power` →
`n1`/`n2` (ambas familias de propiedad) escribiéndose correctamente —
**motor audible en ralentí y reactivo a los cambios de potencia del
patrullaje**. Confirma que el mapeo de `apply_aux_instrumentation()` es
correcto en su diseño (ambas familias indexada/sin-indexada, `running`
derivado de `throttle>0`) para un perfil FGAddon sin problemas de Nasal.

**Observación, no un bug del canal:** durante los virajes cerrados del
patrullaje (radios de giro pensados para el perfil F16-like de
FALCON6), el 757 — geometría de ala y envolvente estructural muy
distintas — produce ruidos intermitentes de sobreesfuerzo/"rotura" y el
sonido de motor queda apagado la mayor parte del tiempo. Es un
desajuste esperable entre el perfil visual (757, elegido por ser el
único "confirmado funcional" a mano) y el perfil de maniobra que
efectivamente vuela la planta FALCON6 — no indica un problema en
`KUNTUR_AUX` ni en `apply_aux_instrumentation()`.

### Incompatibilidad del paquete F-16 de FGAddon con `--fdm=null` (hallazgo nuevo, cierra la vía F-16 para este canal)

Probado con `f16-simplified` y luego, para descartar que fuera una
variante puntual, también con `f16-block-52` — **mismo crash, mismas
líneas exactas**, confirmando que el problema es del Nasal de sistemas
compartido por todo el paquete (`f16/Nasal/f16.nas`, `fuel.nas`), no de
una variante:

```
Nasal runtime error: nil used in numeric context
  at Nasal/f16.nas, line 966          -- if (getprop("gear/gear/position-norm") > 0) { ... }
  called from Nasal/f16.nas, line 513 -- CARA();
  called from Nasal/f16.nas, line 436 -- me.loop(); (dentro de medium.init())
  called from Nasal/f16.nas, line 1843 -- medium.init();
```

Más un error hermano en `fuel.nas` (~líneas 292-298/319-320,
`setprop() value is not string or number`) sobre
`/fdm/jsbsim/propulsion/tank[i]/priority`.

**Causa raíz:** bajo `--fdm=null` FlightGear no instancia JSBSim en
absoluto — no es que sus propiedades queden en `0` o desactualizadas,
directamente el árbol `/fdm/jsbsim/*` y los alias que de él derivan
(como `gear/gear/position-norm`, sin índice) **no existen como nodo**,
y `getprop()` devuelve `nil`. El Nasal de sistemas de este paquete está
cableado a esas propiedades internas de JSBSim de forma profunda (no es
un caso aislado corregible con una sola propiedad precargada vía
`--prop:` — el mismo patrón se repite en `fuel.nas` y probablemente en
más lugares del archivo, no relevados exhaustivamente). Es una
incompatibilidad estructural del paquete con FDM externo, no un bug
puntual.

**Decisión:** descartar este paquete F-16 para este fork mientras se use
`--fdm=null`. Usar `757` (u otro perfil sin esta dependencia) como
aeronave decorativa — ninguno de los perfiles disponibles coincide con
la física real de FALCON6 de todos modos (ver `runfg.sh`), así que no es
una pérdida funcional. Si más adelante se quiere recuperar la estética
F-16, el camino sería un paquete distinto no acoplado a JSBSim interno
(no relevado en esta sesión), o parchear expresamente cada referencia a
`/fdm/jsbsim/*` en el Nasal del paquete actual (esfuerzo no acotado).

Esto también deja sin verificar los puntos del plan original que
asumían este paquete específico (`sound-n2-vol-*`/`n1-out1`/`n1-out2`/
`pilot-gdamped`, y la sinergia con `Models/Cockpit/Main/engine.xml`) —
ya no aplican al perfil `757` en uso; si se retoma instrumentación fina
de motor, habría que rehacer el mismo relevamiento de propiedades sobre
el `sound.xml` propio del 757.

## Qué traer a la próxima sesión

- Decidir el destino de la estética F-16: parchear el paquete FGAddon
  actual (esfuerzo no acotado, ver hallazgo de incompatibilidad con
  `--fdm=null`), buscar un paquete F-16 alternativo no acoplado a
  JSBSim interno, o quedarse con `757`/`c172p` como perfil decorativo
  definitivo.
- Si se sigue con `757`: relevar su propio `sound.xml` para decidir si
  vale la pena escribir las propiedades derivadas de crossfade
  (equivalentes a `sound-n2-vol-*`/`n1-out1`/`n1-out2` del paquete F-16)
  o si ya alcanza con `n1`/`n2` (el motor ya suena y reacciona, así que
  esto es una mejora de fidelidad, no un bloqueante).
- Si conviene agregar `velocities/groundspeed-kt` con la aproximación
  descripta, o dejarlo para una iteración posterior (sigue sin tocar).
- Confirmar el patrullaje/sonido con un perfil de maniobra menos
  agresivo (radios de giro más amplios) si se quiere evitar los
  sonidos de sobreesfuerzo del 757 observados en la prueba de humo —
  o aceptarlo como limitación conocida de usar un perfil visual no
  aerodinámicamente afín a FALCON6.

## Etapa 2 (explícitamente fuera de este plan)

Una vez el cockpit esté funcional, la etapa siguiente es la
caracterización de arquitectura y sintonía de Planta/Autopiloto/Guiado
sobre la implementación actual en CADAC — no el desacople en tres
procesos separados que planteaba originalmente
`plan-desacople-gnc-autopiloto-guiado.md`, sino un objetivo más cercano a
entender y documentar el comportamiento existente para replicarlo en
Simulink. Ese plan probablemente necesite revisarse con este objetivo en
mente antes de retomarlo.
