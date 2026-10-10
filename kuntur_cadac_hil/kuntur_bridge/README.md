# kuntur_bridge (fork HIL)

Bridge Python entre `cadac_plant/plant_driver.py` (planta FALCON6 real)
y FlightGear/Kuntur-GCS, sobre el contrato ICD v1.3. KSE corre
exclusivamente como proceso Rust separado (`kse_server/`).

Ver `../../specs/plan-hil-stm32-portado-autopiloto.md` para la
arquitectura completa (STM32F767 corriendo el autopiloto FALCON6, USB
CDC, FS-i6 manual). Este README cubre solo lo que existe HOY en
`kuntur_bridge/`.

## Componentes (`[components] enabled` en `kuntur_sim.ini`)

| Componente | Rol |
|---|---|
| `executive` | Obligatorio. Habla con `kse_server` (KUNTUR_STATES/KUNTUR_COMMAND, v1.3). |
| `flightgear` | Visualización (`flightgear_worker.py`) + canal auxiliar de instrumentación (`aux_instrument_worker.py`, KUNTUR_AUX). |
| `telemetry_stream` | Relay SSE hacia Kuntur-GCS (`telemetry_stream_worker.py`, `/telemetry`, `/route`). |
| `gcs_api` | API REST para Kuntur-GCS (`gcs_api_worker.py`): `POST /mode`, `GET /status`. |

`cadac_plant/plant_driver.py` (proceso aparte) importa directo de este
paquete (`geodesy`, `config`, `waypoints_file`, `stm32_bridge`) -- no
solo lo usan los workers de `main.py`. Ver docstring de
`stm32_bridge.py` para el porqué.

## Qué NO hay en este fork (ver §9/§14 del plan HIL)

- `px4_bridge.py` / componente `px4` -- Pixhawk no participa de este proyecto.
- `kuntur_fcs.py` / `falcon_fcs.py` / componente `fcs` -- el autopiloto
  corre dentro de `RealFalcon6Adapter` (y, cuando `--stm32-port` está
  activo, en el STM32); no hay FCS Python alternativo.
- `mission_bridge.py` / componente `mission` -- Kuntur-Mission nunca se usó.
- Canal `KUNTUR_CTL` (retasking/reemplazo de ruta) -- retirado del
  backend en esta versión. La lógica de aplicación
  (`RealFalcon6Adapter.retask()`/`.replace_route()`) sigue en
  `cadac_plant/plane_adapter.py` por si se retoma post-tesis, pero nada
  la invoca hoy (no hay `CtlLink`).

## Canales nuevos de este fork

| Canal | Transporte | Rol |
|---|---|---|
| `KUNTUR_MODE` (`icd_mode.py`/`icd_mode_ack.py`) | TCP, loopback, puerto 6200 | `gcs_api_worker.py` (cliente) -> `plant_driver.py` (`ModeLink`, servidor): propaga Automático/Manual. `POST /mode` solo confirma éxito a Kuntur-GCS si `plant_driver.py` ACK-eó. |
| `KUNTUR_PLANT_STATE` / `KUNTUR_SURFACE_CMD` (`icd_plant_state.py`/`icd_surface_cmd.py`) | USB CDC, 100 Hz (decimación x10) | `plant_driver.py` <-> STM32, vía `stm32_bridge.py`. `double` nativo (f64) en todos los campos de punto flotante -- habilita la validación bit-exacta contra el C de `firmware/kuntur_autopilot/src/icd/`. |

## Bypass de superficies (`plane_adapter.py`, ítem #16 / §8 del plan)

Implementado para los dos modos, con el mismo punto de inserción (entre
el despacho de `control` y el de `actuator` en
`RealFalcon6Adapter.step()`):

- **Manual** (`set_manual_mode()`): pisa incondicionalmente con
  `KUNTUR_COMMAND` (vía el `actuator` que `step()` ya recibía e
  ignoraba). Sin asistencia de estabilidad.
- **HIL** (`set_hil_mode()` + `apply_stm32_surface_cmd()`): pisa SOLO si
  hay una respuesta de `stm32_bridge.py` más reciente que
  `HIL_SURFACE_CMD_MAX_AGE_S` (30 ms) -- si no, el autopiloto nativo
  (que ya corrió ese mismo paso) queda tal cual. Fallback silencioso,
  sin estado especial que mantener.

Con ambos flags en `False` (default), `step()` es bit-a-bit idéntico al
camino nativo -- verificado contra el fork original (`kuntur_cadac`) y
contra "HIL activo pero STM32 nunca responde".

## Pendiente (ver `../../specs/plan-hil-stm32-portado-autopiloto.md` §11)

Fase 1 completa. Quedan para fases posteriores:

- **Fase 2**: `--autopilot-rate-hz` en `plant_driver.py` + validación
  cuantitativa SIL-a-100Hz vs. SIL-nativo-a-1kHz (justificación de la
  cadencia de §5 -- hoy el HIL ya corre a 100 Hz por construcción, pero
  no existe el comparativo contra 1 kHz en Python puro).
- **Fase 3**: completa -- autopiloto FALCON6 portado línea por línea a
  C puro (`firmware/kuntur_autopilot/src/autopilot/`), testbench
  Nivel 0 bit-exacto (diff 0.0) contra `falcon6.py` en host.
- **Fase 4**: completa (2026-10-06) -- testbench Nivel 0 cross-compilado
  corrido en el STM32F767 real, diff 0.0 exacto contra Python, ~215us/
  iteración. Ver `firmware/kuntur_autopilot/README.md`.
- **Fase 5**: completa (2026-10-10) -- USB CDC (`src/usb/`) + loop real
  a 100Hz (`src/hil/main_hil.c`) en el firmware, `stm32_bridge.py`
  corrido contra hardware real (no dobles de prueba): 500/500 frames OK,
  0 fallback, round-trip 0.73-1.87ms (~16-40x de margen contra el
  presupuesto de 30ms). Un bug real de enumeración USB (pool estático
  de `USBD_malloc` muy chico) apareció y se corrigió en el camino. Ver
  "Fase 5" en `firmware/kuntur_autopilot/README.md` para el detalle y
  la tabla de latencia.
- **Fase 6**: soak test de 18 ciclos con `stm32_bridge` activo y
  `plant_driver.py` real (dinámica de vuelo real, no el payload en cero
  del smoke test de protocolo de la Fase 5) -- no empezado.
- **Fase 7**: `manual_joystick_worker.py` (FS-i6 vía SM100 USB HID) --
  no empezado.

No verificado en esta sesión (requiere hardware/tiempo real, no
mockeable): soak test de 18 ciclos completo (solo se corrieron 200
pasos = 0.2s simulados como proxy de bit-exactitud), y Kuntur-GCS contra
un backend vivo en el navegador (el build de producción sí se verificó).
