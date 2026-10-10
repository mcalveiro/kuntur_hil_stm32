# Estado de desarrollo — Retasking, canal KUNTUR_CTL y "Fuera de Ruta"

**Fecha:** 2026-10-02
**Alcance de este documento:** toda la cadena de trabajo de la sesión que implementó retasking de punta a punta — desde la revisión de los specs originales hasta el estado `Fuera de Ruta` en Kuntur-GCS. No es un resumen del proyecto completo; para eso ver `specs/planing/backlog-cadac-sil.md` (actualizado junto con este documento, ver §5).

**Propósito:** referencia para construir planes de trabajo en sesiones futuras. La sección 4 (Pendientes) está escrita para eso — cada ítem indica qué falta, por qué, y qué archivos probablemente toque.

---

## 1. Resumen ejecutivo

Hoy existe un camino end-to-end real y verificado: un operador hace clic en el mapa de Kuntur-GCS, confirma "Próximo Waypoint", el comando viaja por HTTP → TCP → la física real de FALCON6 lo aplica, la aeronave se desvía, y el frontend lo refleja (marcador amarillo persistente + estado "Fuera de Ruta") hasta que el vehículo llega y retoma la ruta preestablecida solo. Esto no existía al empezar la sesión — las dos puntas (frontend simulado, backend con el canal sin construir) estaban desconectadas.

Lo que **no** se tocó, a propósito: la extensión de telemetría ("v2", persistencia pasiva de `last_retask_seq` etc.), cualquier UI de reemplazo de ruta/editor de misión, y cualquier archivo de `kse_server` (Rust) o del contrato principal Kuntur-IC v1.3.

## 2. Qué se implementó

### 2.1 Specs — revisión y corrección contra el código real

* `kuntur-gcs/specs/SPEC-retasking_frontend.md` — evaluado. Asumía un canal TCP real desde el navegador, que no es viable (un browser no abre sockets TCP crudos). Se aplicó solo lo realista: `seq` monótono y timeout de seguridad de 5s en `useFsm.ts`, dejando el resto explícitamente bloqueado por la falta del canal backend (en ese momento).
* `kuntur_cadac/specs/planing/SPEC-retasking_backend.md` — revisado línea por línea contra el código real (`plane_adapter.py`, `plant_driver.py`, `protocol.py`, `geodesy.py`, `gen_icd.py`, schemas ICD existentes) y **corregido in situ**: el documento original afirmaba que `KUNTUR_CTL`/`ROUTE_REPLACE`/`ROUTE_ACK` ya existían ("sin cambios") cuando no existía nada; el JSON de ICD propuesto era incompatible con el generador real (`"messages"` en vez de `"packets"`, tipos `f64`/`u8`/`struct` no soportados); y el propio `retask()` del spec tenía un bug real (validaba el origen contra las coordenadas del punto objetivo, no contra el origen del cliente).

### 2.2 Backend (`kuntur_cadac`) — canal de control real

* **`cadac_plant/plane_adapter.py`**: `RouteWaypointSource` (reemplaza el indexado cíclico manual anterior; soporta rutas de longitud arbitraria y política `loiter_last`/`none` al agotarse — con un buffer de lookahead de 1 paso, necesario para preservar el cálculo de bearing de anticipación de pierna que ya usaba el mecanismo viejo, verificado bit a bit). `RealFalcon6Adapter.retask()` y `.replace_route()`, `_apply_target()`/`_bearing_to()` extraídos como helpers reusables, `set_frame()`, `reset_ctl_dedup()`.
* **`icd_schema/kuntur_ic_ctl_v1.json`** (nuevo): 4 paquetes — `KUNTUR_RETASK`/`KUNTUR_RETASK_ACK` (redirección a un punto) y `KUNTUR_ROUTE_REPLACE`/`KUNTUR_ROUTE_ACK` (reemplazo de ruta completa, hasta 64 waypoints).
* **`tools/gen_icd.py`**: extendido para soportar campos "arreglo a granel" (`list[float]` en vez de N campos nombrados) — necesario para los 64 waypoints de `ROUTE_REPLACE`. Aditivo, sin regresión verificada contra el schema auxiliar existente (diff byte a byte antes/después).
* **`cadac_plant/protocol.py`**: `CtlLink` — servidor TCP (puerto 6100 default) con framing explícito por `magic_header` (dos tipos de mensaje de tamaño distinto en la misma conexión), `connection_generation` para detectar reconexiones.
* **`cadac_plant/plant_driver.py`**: `CtlLink` integrado al loop principal (mismo hilo que `step()`, sin locks — se evitó deliberadamente introducir un hilo receptor separado), reseteo de deduplicación de `seq` en cada conexión nueva, `--ctl-port` nuevo en el CLI, `SACOLocalFrame` ahora se construye siempre para `--adapter real` (antes solo si había `--waypoints`).
* **`kuntur_bridge/gcs_api_worker.py`** (nuevo componente, opt-in): puente REST↔TCP. `CtlClient` (cliente TCP persistente, serializado por lock, capa anticorrupción — el navegador nunca conoce `frame_origin_lat/lon`). Endpoints `POST /retask` y `POST /route_replace`, puerto 8766. CORS con preflight correcto para POST+JSON (detalle que los endpoints GET existentes no necesitaban).
* **`kuntur_bridge/config.py`/`config_file.py`/`main.py`**: `KUNTUR_CTL_PORT=6100`, componente `gcs_api` nuevo (no habilitado por default — depende de que `plant_driver.py` corra aparte).

### 2.3 Frontend (`kuntur-gcs`)

* `useFsm.ts`: `submitRetask()` ahora hace un `fetch()` real contra `gcs_api_worker` (reemplazó el `setTimeout(1800)` simulado). Nuevo estado derivado `outOfRoute`, detección de "waypoint alcanzado" por proximidad contra la posición de telemetría (ver §4.1 — es una aproximación, no una señal real del backend).
* `MapPanel.tsx`: marcador amarillo persistente (`confirmedRetask`, antes no se le pasaba a este componente — el marcador desaparecía apenas llegaba el ACK), independiente del marcador de edición/envío.
* `ModePanel.tsx`/`TelemetryPanel.tsx`: superposición visual `"Fuera de Ruta"` (amarillo) sobre el badge de modo, sin tocar `FlightMode`/`confirmedMode` (que sigue reportando `Patrullaje`, la verdad del vehículo).
* `index.css`: tokens `--yellow`/`--yellow-text` (deliberadamente distintos de `--amber`, ya usado por el marcador de la aeronave), clase `.retask-marker-confirmed`.
* `kuntur-gcs/specs/SPEC-fuera-de-ruta.md` (nuevo).

### 2.4 Documentación de arquitectura

* `docs/diagrama_bloques_sistema.puml` y `docs/diagrama_secuencia_sistema.puml` (nuevos) — cubren el sistema completo (los 6 procesos, todos los puertos/protocolos, incluyendo piezas no tocadas esta sesión como `kse_server` en Rust y los workers de FlightGear/PX4), no solo lo nuevo. Renderizados y revisados visualmente antes de entregarlos (no son código sin validar).

## 3. Verificación realizada

* **Codegen**: `tools/gen_icd.py` regenerado contra el schema nuevo y el existente, diff byte a byte confirmando cero regresión en `kuntur_ic_aux_v1.json`.
* **Protocolo**: round-trip de pack/unpack para los 4 paquetes nuevos, incluyendo el caso de relleno/rechazo por exceder 64 waypoints.
* **`RouteWaypointSource`**: cíclica, finita con `loiter_last`, finita con `none` — comparado el lookahead contra el indexado modular manual que reemplazó, bit a bit, 10 ciclos.
* **Adaptador real**: corrido contra la física real de FALCON6 (`f16c11_7.asc`, `scenarios/waypoints_cordoba.txt`) el tiempo suficiente para que el patrullaje cíclico se reenganchara dos veces solo, con geometría sana (altitud/bearing consistentes).
* **`retask()`/`replace_route()`**: los 6 casos (aplicado, origen incorrecto, seq obsoleto, fuera de rango, ruta válida, demasiados waypoints) probados contra ese mismo adaptador real, no un mock.
* **Reconexión**: confirmado que una conexión TCP nueva resetea la deduplicación de `seq` del adaptador (el fix de §2.2).
* **End-to-end HTTP→TCP→física→HTTP**: `gcs_api_worker` + `plant_driver.py` reales, request con forma de navegador (incluyendo preflight `OPTIONS`), los 4 casos (aplicado, rechazado, 503 inalcanzable, 400 bad request).
* **Frontend**: `tsc` limpio en cada cambio; matemática de distancia (`approxDistanceMeters`) verificada contra valores conocidos (111m/0.001° de latitud, compresión este-oeste por `cos(lat)`); dev server arrancando sin errores de import.
* **Confirmado en uso real por el operador** (no solo por mí): el flujo completo — clic en mapa, marcador amarillo, estado Fuera de Ruta — probado en el navegador contra la pila real corriendo.
* **No verificado**: no hay herramienta de navegador en ninguna sesión de este trabajo — toda verificación de UI previa a la confirmación del operador fue por inspección de red/HTTP, no por manejo real de un browser.

## 4. Pendientes

Ordenados por qué tan directamente bloquean o mejoran lo que ya existe, no por prioridad de negocio (eso se decide en la sesión de planning).

### 4.1 Detección de "alcanzado" es una aproximación del lado cliente

**Qué falta:** `useFsm.ts` limpia el waypoint temporal por proximidad (radio fijo de 150m contra la posición de telemetría), no por una señal real de la planta. El criterio real del backend es geométrico y depende de velocidad (`dwbh < 2*turn_min` en `cadac_py/falcon6.py::guidance()`) — puede haber un desfase de varios segundos entre que el backend ya reenganchó la ruta y que el frontend lo detecta.

**Para resolverlo de raíz:** extender la telemetría ("v2", explícitamente diferida dos veces en esta sesión) — nuevo `icd_schema/kuntur_ic_aux_v2.json` con campos como `last_retask_seq`, `target_north/east/down`, `route_idx`/`route_len`/`route_mode`/`exhausted`. Esto requiere decidir primero si esos campos van en una extensión de `kuntur_ic_aux_v1.json` (hoy delimitado explícitamente como "instrumentación FlightGear", no control) o en un canal nuevo separado — ver la nota de diseño abierta que quedó en la revisión de `SPEC-retasking_backend.md`.

**Archivos probables:** `icd_schema/`, `kuntur_bridge/icd_aux*.py` (generado), `cadac_plant/plane_adapter.py` (exponer el estado de ruta), `cadac_plant/protocol.py::AuxTelemetryLink`, `kuntur-gcs/src/useFsm.ts` (reemplazar la heurística de proximidad por el dato real).

### 4.2 `route_replace` no tiene ninguna UI

**Qué falta:** el backend (`replace_route()`) y el bridge (`POST /route_replace`) están implementados y probados end-to-end, pero no hay ningún punto de la interfaz de Kuntur-GCS que permita cargar una ruta nueva — no existe editor de misión (backlog histórico #18: "Falta: editor de misión (`upload_route`)").

**Para resolverlo:** diseñar e implementar la UI de carga de ruta (¿archivo?, ¿dibujar en el mapa?, ¿lista de puntos editable?) — esto es una feature de producto nueva, no una continuación mecánica de lo ya hecho. Vale la pena un spec propio antes de tocar código, siguiendo el mismo proceso que `SPEC-fuera-de-ruta.md`.

**Archivos probables:** nuevo componente en `kuntur-gcs/src/components/`, `useFsm.ts` (nuevo flujo, análogo a `submitRetask`), posiblemente `mapWaypoints.ts`.

### 4.3 Verificación de regresión formal no corrida

**Qué falta:** el refactor de `_advance_loop_waypoint()` → `_advance_route_waypoint()` se verificó manualmente (ver §3) pero no se corrió el script formal de regresión bit-exacta contra el batch histórico (`regression_vs_batch.py`, si existe con ese nombre — confirmar) ni el soak test de escala que pedía el spec de backend (Fase 2: ≥20 waypoints, Fase 4: ≥1h simulada). Esto importa antes de confiar en rutas largas/no cíclicas en un escenario real, no solo en los 4 waypoints de prueba usados esta sesión.

**Archivos probables:** ninguno nuevo — es correr lo que ya existe (o escribirlo si no existe) contra `cadac_plant/plane_adapter.py` tal como quedó.

### 4.4 Bug pre-existente encontrado, no corregido (fuera de alcance de esta sesión)

**Qué falta:** `kuntur_bridge/main.py` importa `mission_bridge_worker` desde `kuntur_bridge/mission_bridge.py`, que **no existe** en el repo. Import roto si alguna vez se habilita el componente `'mission'` en `kuntur_sim.ini`. Encontrado al armar los diagramas de arquitectura (§2.4), no corregido porque no tiene relación con retasking/`KUNTUR_CTL` y no se pidió.

**Para resolverlo:** o se escribe `mission_bridge.py` (si el componente 'mission' sigue siendo un objetivo real), o se quita la rama `elif component == "mission":` de `main.py` y se retira `'mission'` de `VALID_COMPONENTS` (si quedó obsoleto). Decisión de producto, no técnica.

### 4.5 Decisiones de `SPEC-fuera-de-ruta.md` implementadas por defecto, no reconfirmadas

El spec marcaba tres decisiones como "a confirmar con Miguel" (D1-D3); se implementaron con el criterio que yo mismo había propuesto, porque el pedido de implementar no las objetó:

* **D1** — `Fuera de Ruta` es una superposición visual, no un `FlightMode` nuevo.
* **D2** — radio de arribo fijo en 150m, no configurable desde la UI.
* **D3** — distancia equirectangular simple, no haversine.

Si alguna no es la lectura correcta del pedido original, es un cambio puntual y acotado (no hay nada más construido encima que dependa de la implementación exacta).

### 4.6 Backlog pre-existente relacionado, directamente desbloqueado por `RouteWaypointSource`

El ítem #9 del backlog histórico ("Nuevas estrategias de guiado — `LoiterCircleSource`, `ClimbOutSource`, `GlideSlopeSource`") seguía "Pendiente" antes de esta sesión por falta de la abstracción base. Esa base (`RouteWaypointSource`) ahora existe — implementar esas estrategias como fuentes alternativas que implementen el mismo protocolo (`next()`/`peek_next()`) es mecánicamente más directo hoy que antes. Ver `specs/planing/backlog-cadac-sil.md` (actualizado junto con este documento) para el resto de los ítems no relacionados con retasking que siguen abiertos.

## 5. Archivos tocados en esta sesión (índice)

**Backend (`kuntur_cadac`):**
`icd_schema/kuntur_ic_ctl_v1.json` (nuevo) · `tools/gen_icd.py` · `kuntur_bridge/icd_retask.py`, `icd_retask_ack.py`, `icd_route_replace.py`, `icd_route_ack.py` (generados) · `kuntur_bridge/config.py` · `kuntur_bridge/geodesy.py` · `kuntur_bridge/config_file.py` · `kuntur_bridge/main.py` · `kuntur_bridge/gcs_api_worker.py` (nuevo) · `cadac_plant/protocol.py` · `cadac_plant/plane_adapter.py` · `cadac_plant/plant_driver.py` · `kuntur_sim.ini` (operacional, no versionado como código) · `specs/planing/SPEC-retasking_backend.md` (corregido) · `specs/planing/backlog-cadac-sil.md` (actualizado) · `docs/diagrama_bloques_sistema.puml`, `docs/diagrama_secuencia_sistema.puml` (nuevos) · este documento.

**Frontend (`kuntur-gcs`):**
`src/types.ts` · `src/useFsm.ts` · `src/App.tsx` · `src/components/MapPanel.tsx` · `src/components/ModePanel.tsx` · `src/components/TelemetryPanel.tsx` · `src/index.css` · `specs/SPEC-fuera-de-ruta.md` (nuevo).
