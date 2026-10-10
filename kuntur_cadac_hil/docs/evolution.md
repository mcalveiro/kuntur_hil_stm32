# Plan: desacople de Planta, Autopiloto y Guiado (evolución GNC del fork CADAC-SIL)

**Estado: planificación, pendiente de implementar en otra sesión.**

## Motivación

Hoy `RealFalcon6Adapter` reconduce `cadac_py::falcon6.Plane` como una
caja cerrada: física, autopiloto y guiado corren fusionados dentro de la
misma clase, leyendo y escribiendo un array numerado compartido
(`plane[]`/`flat6[]`). Es la decisión correcta para el objetivo ya
cumplido (SIL de física+guiado con fidelidad bit-exacta contra el batch),
pero no es una arquitectura GNC de tres bloques independientes.

El objetivo de este plan es extraer el diseño y la sintonía ya validados
en la simulación batch de CADAC y reorganizarlos en tres componentes con
interfaces explícitas: **Planta**, **Autopiloto**, **Guiado** — cada uno
implementado, sintonizado y testeable por separado.

## Diagnóstico ya establecido (no requiere más investigación de código)

Se verificó contra el código fuente real de `cadac_py/falcon6.py` y el
escenario `f16c11_7.asc` que la "sintonía de CADAC" no es una tabla de
ganancias fija — es un **piloto automático autoadaptativo por ubicación
de polos**, con el diseño repartido en dos capas hoy invisibles como
parámetros:

1. **Ganancias de lazo externo declaradas en el `.asc`** — confirmado en
   `f16c11_7.asc` líneas 82/87: `gainp` (ganancia proporcional del lazo
   de aceleración en cabeceo, `s²/m`) y `gainl` (ganancia del lazo de
   aceleración lateral, `rad/g's`), con unidades físicas documentadas en
   el propio comentario del escenario. Hay análogas en los lazos de
   `control_gamma`/`control_heading`/`control_altitude` (`pgam`/`wgam`/
   `zgam`, etc.) que quedan por relevar completas.
2. **Un esquema de ubicación de polos, hardcodeado dentro de
   `falcon6.py`** — `control_normal_accel()` (línea 1323) recalcula la
   ganancia de realimentación en **cada paso**, no una vez: toma las
   derivadas aerodinámicas locales del punto de vuelo (`dla`, `dma`,
   `dmq`, `dmde` — sustentación-α, momento de cabeceo-α, amortiguamiento
   en cabeceo, momento de cabeceo-elevador; `plane[145..149]`) y la
   presión dinámica instantánea `pdynmc` (`flat6[57]`), y a partir de un
   polo deseado programado como tres ecuaciones lineales en función de
   `pdynmc` (`waclp`, `zaclp`, `paclp`) invierte el modelo local
   linealizado para obtener `gainfb1/2/3`. Eso **es** el diseño de
   control clásico (root-locus/pole-placement) contra la dinámica
   linealizada del F-16 en ese punto de vuelo — pero vive como literales
   numéricos dentro de un método Python, no como un artefacto de diseño
   propio, versionable e independiente de la física.

Consecuencia directa para el desacople: el Autopiloto no puede
reducirse a "estado cinemático → ganancias fijas → deflexiones". Necesita,
en cada paso, un **paquete aerodinámico del punto de vuelo instantáneo**
(`pdynmc`, `dvbe`, y las derivadas de estabilidad relevantes) que hoy
solo existe dentro de la Planta.

El Guiado, en cambio, **sí** es extraíble limpiamente ya: `guidance_line()`
es geometría pura (rango al waypoint, `wp_flag` por signo del producto
punto de la velocidad de cierre) sobre estado cinemático y lista de
waypoints — no toca ninguna derivada aerodinámica.

## Los tres bloques y su interfaz objetivo

- **Planta** — física pura (`environment`/`kinematics`/`aerodynamics`/
  `propulsion`/`euler`/`newton`). Salida: estado cinemático (posición
  NED, velocidad, actitud, tasas) **más** el paquete aerodinámico del
  punto de vuelo instantáneo que hoy el autopiloto lee directo del array
  compartido.
- **Guiado** — entrada: estado cinemático + waypoint activo. Salida:
  comando de guiado (`ancomx`/`alcomx`, o `thtvlcomx`/`psivlcomx` según
  el modo). Al extraerlo hay que portar la semántica real de disparo por
  geometría de cierre (`wp_flag`), no la aproximación de "radio de
  captura" que ya existe como sustituto simple en
  `kuntur_bridge/falcon_fcs.py::WaypointList` (esa clase quedó
  documentada en el propio código como un control de reemplazo para
  probar el cableado KSE↔FCS, no como ruta hacia fidelidad real — ver
  docstring de `FalconAutopilotCoreReal`, que además deja explícito por
  qué el intento anterior de extracción se abandonó).
- **Autopiloto** — entrada: estado + paquete aerodinámico + comando de
  guiado. Salida: deflexiones de superficie + empuje. Debe ser dueño
  explícito de su propio estado persistente — hoy disperso en slots
  numerados (`p[516]/p[517]` los integradores, `p[504..506]` los polos
  calculados, `p[524]` el vector de ganancias) — encapsularlo en un
  objeto propio es tan importante como separar el cálculo en sí.

## Fases de evolución (sin romper lo que ya funciona)

### Fase 1 — Extraer la sintonía como artefacto propio

Script que recorra el `.asc` y capture los parámetros declarados con su
tag de módulo (`module control`, `module guidance`) hacia un archivo
estructurado y versionado (YAML/JSON), separado de las condiciones
iniciales y waypoints operativos del escenario. En paralelo, documentar
el esquema de ubicación de polos (`waclp/zaclp/paclp` vs `pdynmc`, y sus
análogos en los demás lazos `control_*`) como un segundo artefacto de
diseño explícito, hoy enterrado en código Python.

- Riesgo: nulo (no toca ejecución, solo documenta/extrae).
- Es la fase que efectivamente "captura el aprendizaje" — valiosa aunque
  las fases siguientes no se lleguen a hacer.

### Fase 2 — Desacople interno (mismo proceso, sin tocar el ICD)

Partir `control()`/`guidance()` de `Plane` en dos clases nuevas
(`Falcon6Autopilot`, `Falcon6Guidance`) con entradas/salidas tipadas
explícitas (dataclasses) en vez de lectura/escritura directa sobre
`plane[]`/`flat6[]`. `RealFalcon6Adapter` los compone en el mismo orden
causal que hoy.

- Gate de aceptación: seguir bit-exacto contra el batch de referencia
  (0.0 m de diferencia) — si esto se rompe, el refactor no es válido.
- Riesgo: bajo-medio (refactor de software, cero cambio de protocolo).
- Resultado: los "tres elementos bien definidos" ya existen a nivel de
  software, sin riesgo de protocolo ni de fidelidad.

### Fase 3 — Guiado realmente independiente (primer proceso separado)

Al no necesitar derivadas aerodinámicas, el Guiado es el primer
candidato viable para correr como proceso separado sobre el estado que
ya viaja por el ICD, sin extender `KUNTUR_STATES`/`KUNTUR_COMMAND`.

- Riesgo: bajo.
- Entrega el desacople "de verdad" (proceso independiente) para uno de
  los tres bloques, más rápido y con el menor riesgo de los tres.

### Fase 4 — Autopiloto independiente

Acá sí hay una decisión de fondo, porque el ICD actual (`KUNTUR_STATES`
92 bytes / `KUNTUR_COMMAND` 34 bytes) no transporta el paquete
aerodinámico que el autopiloto necesita. Dos caminos:

- **(a) Canal auxiliar nuevo**: un mensaje ICD adicional (magic propio,
  sin tocar `KUNTUR_STATES`/`KUNTUR_COMMAND` v1.3, así no afecta el
  camino PX4/HITL existente) que transporte `pdynmc`, `dvbe` y las
  derivadas de estabilidad. Mantiene bit-exactitud.
- **(b) Modelo aerodinámico propio en el Autopiloto**: una copia
  versionada del aerodeck (`f16_aero_deck.asc`), calculando las
  derivadas localmente a partir de Mach/α recibidos por el ICD existente
  — más fiel a cómo funciona un computador de vuelo real (sin acceso
  omnisciente a la planta), a costa de resignar la garantía de
  bit-exactitud y de tener que validar la tolerancia numérica contra el
  batch.

- Riesgo: medio-alto (toca el contrato ICD y/o resigna fidelidad exacta).
- Solo abordar esta fase si hay un driver real (probar otros diseños de
  autopiloto, correrlo en un nodo de cómputo separado, o emular un
  computador de vuelo real).

## Recomendación

Empezar por la **Fase 1** en la próxima sesión: bajo ningún riesgo,
captura el conocimiento de diseño que hoy es frágil (repartido entre
comentarios de un `.asc` y literales de código), y no compromete nada de
lo ya validado y corriendo estable (patrullaje en bucle, soak test de 18
ciclos — ver `claude/arquitectura-fusion-kuntur-cadac.md`, sección 6).
Evaluar avanzar a la Fase 2 una vez que la Fase 1 esté completa y
documentada; las Fases 3 y 4 son, en sí mismas, un proyecto de refactor
con su propio ciclo de validación y solo se justifican si aparece una
necesidad concreta de correr Guiado o Autopiloto como procesos
verdaderamente independientes.

## Qué traer a la próxima sesión

- Relevamiento completo de constantes de sintonía por lazo (`control_roll`,
  `control_roll_rate`, `control_pitch_rate`, `control_yaw_rate`,
  `control_normal_accel`, `control_lateral_accel`, `control_gamma`,
  `control_heading`, `control_altitude`) — cuáles vienen del `.asc` y
  cuáles están hardcodeadas en `falcon6.py`, para que la Fase 1 sea
  exhaustiva y no solo cubra el lazo de aceleración normal ya analizado.
- Decisión sobre si además de `f16c11_7.asc` conviene relevar otros
  escenarios `.asc` de `cadac_py/CADAC/FALCON6/`, en caso de que se use
  más de uno como fuente de sintonía.