# firmware/kuntur_autopilot

Proyecto PlatformIO del autopiloto FALCON6 en STM32F767 (Nucleo-144).
Ver `../../../specs/plan-hil-stm32-portado-autopiloto.md` §7.

## Estado (setup verificado, 2026-10-05)

- Board detectada: ST-LINK/V2.1 (`0483:374b`) + CDC en `/dev/ttyACM0`.
- `pio run` compila limpio (confirma de paso que `src/icd/*.c` también
  compila para el target ARM real, no solo para el test de host).
- `pio run -t upload` -- **blink de verificación (`src/main.c`, LD1/PB0)
  programado y confirmado parpadeando en hardware real.**

### Hallazgo: SWD no conecta sin "connect under reset"

El firmware de fábrica del Nucleo deja al STM32F767 en un estado que no
responde en el Debug Port con una conexión SWD normal -- OpenOCD se
queda reintentando en `SWD_DP_WAIT`/`SWD_AP_WAIT` hasta agotar
reintentos (`init mode failed`). Diagnosticado corriendo OpenOCD a mano
con `-d3` (ver historial de la sesión) -- la comunicación USB con el
ST-Link en sí funciona bien (lee voltaje del target), el problema es a
nivel protocolo SWD con el target.

**Solución, ya aplicada en `platformio.ini`:** conectar con el target en
reset (`reset_config srst_only srst_nogate connect_assert_srst`, vía
`upload_flags`). Con esto, `pio run -t upload` funciona de punta a
punta.

### Permisos USB

`/dev/ttyACM0` y el dispositivo USB del ST-Link ya son accesibles sin
`sudo` porque el usuario está en el grupo `plugdev`. Igualmente se
instalaron las reglas udev que trae PlatformIO
(`99-platformio-udev.rules`) para dejarlo robusto ante reconexiones/
otros usuarios -- no eran estrictamente necesarias para este hallazgo
(el problema real era el SWD, no permisos), pero es buena práctica
tenerlas.

## Fase 3 -- autopiloto portado a C, Nivel 0 bit-exacto (2026-10-05)

`src/autopilot/` tiene el port línea por línea de las ~13 funciones de
`cadac_py/falcon6.py` listadas en §3 del plan: `control_roll`/
`control_roll_rate`/`control_pitch_rate`/`control_yaw_rate`
(`control_rates.c`), `control_normal_accel.c` (pole placement
adaptativo), `control_lateral_accel.c`, `control_gamma.c` (no
ejercitado por `f16c11_7.asc`), `control_heading.c` (idem),
`control_altitude.c`, `guidance_line.c` (`guidance()` +
`guidance_line()`), `actuator.c` (`actuator()` + `actuator_scnd()`,
dinámica de 2do orden), `control_mix.c` (el dispatcher `control()` +
mezcla a límites de superficie) y `falcon6_autopilot.c` (punto de
entrada único: `guidance() -> control_mix() -> actuator()`, el mismo
orden que usa el `*.asc`). `src/config/scenario_params.h` tiene las
ganancias/límites de `f16c11_7.asc` hardcodeadas.

**Gate Nivel 0: `test/host/test_bitexact.py` -- diferencia 0.0 exacta
(no solo < 1e-10) en 10000 pasos continuos (10 s @ 1 kHz) contra la
referencia Python**, cubriendo `control_lateral_accel`,
`control_altitude -> control_normal_accel`, `control_roll`,
`guidance`/`guidance_line` y `actuator`/`actuator_scnd` (los lazos que
`maut=35` efectivamente ejercita). `control_gamma`/`control_heading`/
`control_roll_rate` están portados pero sin traces de referencia
todavía (ningún escenario actual los ejercita -- ver comentarios en sus
`.h`).

Cada registro de trace es autocontenido: trae el estado persistente del
autopiloto (`Autopilot_State_t`) tal cual Python lo tenía antes de esa
llamada, y el test lo setea explícito en vez de encadenarlo entre
registros -- la llamada 0 de la simulación (antes de la cual no hay
nada que capturar desde afuera, ver docstring de `capture_traces.py`)
no se puede reproducir limpiamente, así que inyectar el estado evita el
problema en vez de resolverlo.

**Hallazgos durante el port** (plan §3/§6.1 listaba las derivadas de
forma especulativa, sin haber leído el código fuente):
- Derivadas reales: `dla/dlde/dma/dmq/dmde` (plane[145-149]),
  `dyb/dydr/dnb/dnr/dndr` (plane[150-154]), `dllp/dllda`
  (plane[155-156]) -- 12, no 14; `dyp/dnp/dyda/dnda` no existen en la
  fuente.
- `grav` (flat6[55], NO la constante `AGRAV` -- varía con la distancia
  al centro de la Tierra) y `gmax`/`gminx` (plane[163]/[167], límites
  estructurales del aerodeck) faltaban en el schema `KUNTUR_PLANT_STATE`
  -- agregados (ver `icd_schema/kuntur_ic_plant_state_v1.json`).
- `actuator()`/`actuator_scnd()` están incluidos en lo que corre en el
  STM32 (confirmado contra §2/§3 del plan) -- `KUNTUR_SURFACE_CMD` lleva
  la deflexión YA APLICADA (`delax`/`delex`/`delrx`, post-dinámica de
  actuador), no el comando crudo de `control()` -- esto obligó a
  renombrar el schema (antes `delacx`/`delecx`/`delrcx`) y a corregir el
  bypass de HIL en `cadac_plant/plane_adapter.py` (ahora intercepta el
  despacho de `'actuator'`, no el de `'control'`, y saltea el
  `actuator()` nativo cuando hay respuesta fresca del STM32).
- Varias lecturas muertas en el original (leídas, nunca usadas en el
  cálculo): `yy`/`yyd` (plane[514/515]), `halt` en `guidance()`, `grav`
  en `control_lateral_accel()`, `pdynmc` en `control_gamma()` --
  documentadas en los `.h` correspondientes, no portadas donde no
  afectan el resultado.
- `out->throttle` queda en 0.0: `f16c11_7.asc` usa `mprop=2` (Mach
  hold), gobernado enteramente por `propulsion()` (física, se queda en
  Python) -- no existe una función de autopiloto de throttle que portar
  para este escenario.

## Fase 4 -- testbench Nivel 2 corriendo en el target real (2026-10-06)

`src/test_target/main_bitexact.c` cross-compila el mismo
`src/autopilot/` + `src/icd/` del Nivel 0 (sin tocar una línea) y corre
contra un subconjunto de 1000 registros (equiespaciados sobre los 10000
del fixture completo -- ver `test/target/gen_traces_data.py`, que
genera `src/test_target/traces_data.c`; el archivo completo, 5.1MB, no
entra en los 2MB de flash del F767ZI). Build propio
(`pio run -e nucleo_f767zi_bitexact -t upload`, env separado en
`platformio.ini` que excluye `src/main.c`), reporte por UART (USART3,
115200 8N1 -- en el Nucleo-144 es el mismo VCP del ST-Link,
`/dev/ttyACM0`, sin hardware adicional).

**Gate Nivel 2 (plan §10): corrido en hardware real -- diferencia
0.000000000 exacta** (no solo < 1e-6, el gate del nivel) sobre los 1000
registros, igual que el Nivel 0 en host. Reloj confirmado a 216MHz,
I/D-cache habilitado. Performance (DWT cycle counter):
**~215us promedio por `falcon6_autopilot_step()`** (min 211us, max
219us) -- dentro de la estimación del plan §7 (100-500us) y con margen
amplio contra el presupuesto de 10ms (100Hz, plan §5).

**Hallazgos de esta fase** (ninguno estaba anticipado en el plan):

- **HSE descartado a favor de HSI.** El plan asumía el MCO de 8MHz del
  ST-Link como fuente HSE (patrón común en otras Nucleo). No se
  verificó a mano si esta placa en particular tiene el bypass
  puenteado, y arrancar desde HSI (16MHz interno, sin dependencia de
  hardware externo) evita la pregunta por completo --
  `osc.PLL.PLLM=8, PLLN=216, PLLP=2, PLLQ=9` sobre HSI da los mismos
  216MHz sysclk / 48MHz para USB (Fase 5) que HSE/4 daba. Ver
  `src/bsp/clock_config.c`.
- **`DWT->LAR` (Lock Access Register) necesita el unlock
  `0xC5ACCE55` antes de que `DWT->CTRL` acepte escribir
  `CYCCNTENA`** en este core (Cortex-M7 r1p0 con lock register, CMSIS
  `core_cm7.h` del framework lo expone). Sin el unlock, el contador
  queda silenciosamente en 0 -- no hay error, no hay HardFault, solo
  ciclos=0 en los 1000 registros, detectado por inspección del reporte
  UART. Ver `dwt_cycle_counter_enable()` en `main_bitexact.c`.
- **El `printf`/`snprintf` de newlib-nano (`--specs=nano.specs`,
  default del toolchain de PlatformIO) no soporta `%e`/`%f` NI
  `%llu`/ancho dinámico (`%0*llu`)** -- los primeros salían vacíos, los
  segundos imprimían literalmente `"lu"` en vez de formatear. Se evitó
  del todo con `format_fixed()` (punto fijo a mano, solo `%lu`/`%s`,
  `uint32_t` en vez de `uint64_t` -- con `decimals=12` el `div=10^12`
  desborda 32 bits silenciosamente, bajado a `decimals=9` que no
  desborda y sigue sobrando resolución contra la tolerancia 1e-6 del
  gate).

## Fase 5 -- USB CDC real, validado contra hardware (2026-10-10)

`src/usb/` (nuevo) + `src/hil/main_hil.c` (nuevo, env propio
`nucleo_f767zi_hil`) implementan el loop real de §5/§7: `receive
KUNTUR_PLANT_STATE -> falcon6_autopilot_step() a 100Hz (int_step=0.01,
NO el 0.001 de Fase 3/4) -> send KUNTUR_SURFACE_CMD`, reusando sin
tocar el autopiloto/ICD ya validado bit-exacto en Fases 3/4.

- `src/usb/middleware/` -- `Core` (`usbd_core/ctlreq/ioreq.c`) + `Class/CDC`
  (`usbd_cdc.c`) de `STM32_USB_Device_Library`, vendoreados sin
  modificar desde el `framework-stm32cubef7` que ya trae PlatformIO
  (ver `middleware/README.md` sobre por qué vendorear en vez de
  referenciar in-place: `framework = stm32cube` solo agrega
  `Drivers/`, no `Middlewares/`, al include path).
- `usbd_conf.c` -- capa `USBD_LL_*` <-> `HAL_PCD_*` para USB_OTG_FS
  (PA11/PA12, patrón CubeMX estándar).
- `usbd_desc.c` -- descriptor de dispositivo CDC ACM (VID/PID
  `0x0483`/`0x5740`, los que usa ST en sus propios ejemplos
  CDC_Standalone -- el driver `cdc_acm` de Linux matchea por
  clase/subclase/protocolo de la interfaz, no por VID/PID).
- `usbd_cdc_if.c` + `usb_cdc_io.c` -- separación "glue mínimo de
  CubeMX" vs. "API de aplicación" (el `usb_cdc_io.c/h` que nombra el
  plan §7): ring buffer SPSC (productor = ISR de USB, consumidor =
  `main_hil.c`), con primitivas `peek`/`consume` para poder
  resincronizar contra el magic header de `KUNTUR_PLANT_STATE` byte a
  byte en vez de a ciegas por tamaño de frame fijo.

**Estado: handshake real verificado contra hardware** -- con ST-Link
(programación/UART de debug) y `USB_FS`/CN13 (CDC nativo) conectados a
la vez, `lsusb` muestra `0483:5740` (`kuntur_autopilot HIL`) y Linux lo
expone como `/dev/ttyACM1` (`cdc_acm`, sin drivers especiales).

`kuntur_bridge/stm32_bridge.py` corrido contra ese puerto real (no
contra los dobles de prueba) -- 500 iteraciones a ~100Hz, estado
`init=1` en la primera:

| Métrica | Valor |
|---|---|
| Frames enviados / recibidos OK | 500 / 500 (0 fallback) |
| Frames inválidos del lado STM32 (UART de debug) | 0 (`frames ok=256 bad=0` en la ventana capturada) |
| Round-trip: mínimo | 0.728 ms |
| Round-trip: promedio | 0.953 ms |
| Round-trip: máximo | 1.874 ms |
| Round-trip: desvío estándar (jitter) | 0.144 ms |

Margen contra el presupuesto de fallback (`HIL_SURFACE_CMD_MAX_AGE_S =
30ms`, plan §10 Nivel 3): **~16-40x** incluso en el peor caso medido.
`stm32_bridge.py` nunca entró en fallback en esta corrida (`fallback_count
== 0` al final). Pendiente para Fase 6: el soak test de 18 ciclos real
(acá solo se mandó un `kuntur_plant_state_t` en cero, sin dinámica de
vuelo -- válido para validar el protocolo, no el comportamiento del
autopiloto en un escenario real).

**Hallazgos de esta fase:**

- La Library Dependency Finder de PlatformIO detecta el
  `STM32_USB_Device_Library` que el propio `framework-stm32cubef7`
  expone como librería instalable, y lo compila EN PARALELO a la copia
  vendoreada en `src/usb/middleware/` -- mismos símbolos definidos dos
  veces, error de link. Se resolvió con `lib_ldf_mode = off` en el env
  `nucleo_f767zi_hil` (ver `platformio.ini`).
- El patrón de ST separa cada módulo del middleware en subcarpetas
  `Inc/`+`Src/` propias; sus `#include "usbd_cdc.h"` (sin ruta)
  asumen que ambas terminan en el mismo directorio de compilación
  (como en un proyecto CubeMX plano) -- vendoreadas tal cual bajo
  `Inc/`/`Src/` separados, hace falta `-I` explícito a cada `Inc/` (no
  alcanza con la resolución "relativa al archivo que incluye" que sí
  funciona para el resto de este proyecto, ver `platformio.ini`).
- Diseño propio (no hallazgo, decisión): el framing de
  `KUNTUR_PLANT_STATE` no tiene delimitador aparte del magic header +
  tamaño fijo, así que la resincronización ante bytes perdidos/
  corrupción se hace byte a byte contra el magic (`resync_to_
  magic_header()` en `main_hil.c`), no por múltiplos del tamaño de
  frame -- un host que se desconecta a mitad de un envío no debe dejar
  al STM32 reintentando para siempre sobre el mismo prefijo viejo (se
  descarta 1 byte tras cada timeout de frame incompleto para forzar
  resync real).
- **Confirmado en hardware real (conectando el cable de datos al
  USB_FS antes de descubrir que el "segundo cable" disponible en la
  sesión era solo de alimentación): el dispositivo enumeraba con
  VID/PID/strings correctos (`lsusb` mostraba `0483:5740`,
  `kuntur_autopilot HIL`, `kuntur-project`) pero el host reportaba
  `can't set config #1, error -32` (STALL en `SET_CONFIGURATION`).
  Causa: `USBD_STATIC_POOL_SIZE` (el pool estático detrás de
  `USBD_malloc`, ver `usbd_conf.c`) estaba en 256 bytes;
  `USBD_CDC_HandleTypeDef` reserva un buffer de
  `CDC_DATA_HS_MAX_PACKET_SIZE` (512 bytes) SIEMPRE, sin importar que
  este firmware corra a Full Speed, así que la única asignación del
  middleware (~544 bytes) devolvía `NULL`, `USBD_CDC_Init()` fallaba
  con `USBD_EMEM`, y `usbd_ctlreq.c` traduce esa falla en un STALL de
  EP0 -- de ahí el error que ve el host. Subido a 1024 bytes.
  **Confirmado resuelto**: con el fix, la enumeración llega a
  CONFIGURED y el handshake real funciona (ver tabla arriba).

## Próximo paso (Fase 6 del plan)

Correr el soak test completo de patrullaje (18 ciclos) con
`stm32_bridge` activo y `plant_driver.py` real (no el smoke test de
protocolo de arriba, que no ejercita ninguna dinámica de vuelo),
comparar la trayectoria contra el SIL-a-100Hz (Nivel 1, referencia
justa), y caracterizar estabilidad del ciclo límite, dispersión por
reenganche y tasa de fallback bajo carga real (plan §10 Nivel 3, §11
Fase 6). La latencia/jitter de USB CDC en sí ya quedó caracterizada en
la Fase 5 (tabla arriba) y no debería cambiar con datos reales en vez
de ceros -- el payload tiene el mismo tamaño fijo siempre.
