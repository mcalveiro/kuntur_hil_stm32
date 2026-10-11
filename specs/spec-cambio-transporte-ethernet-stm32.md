# Spec: Cambio de transporte Host↔STM32 de USB CDC a Ethernet

**Fecha:** 2026-10-10
**Estado:** especificación técnica, lista para iniciar implementación.
**Supersede**, en `plan-hil-stm32-portado-autopiloto.md`, los siguientes puntos:

| Doc | Sección | Dice hoy | Pasa a decir |
|---|---|---|---|
| plan-hil-stm32-portado-autopiloto.md | §1, fila "Transporte Host↔STM32" | USB CDC (virtual COM) sobre USB FS. Ethernet/LwIP queda **fuera de alcance**. | **Ethernet** sobre el PHY LAN8742 on-board de la Nucleo-144, UDP point-to-point. USB CDC queda retirado del plano de datos (ver §7 de este spec sobre qué pasa con el canal de debug). |
| plan-hil-stm32-portado-autopiloto.md | §5, punto 3 de la justificación de 100 Hz | "Permite USB CDC sin Ethernet → mantiene la infraestructura minimalista." | Dejó de aplicar — es, de hecho, el motivo de este cambio (ver §1 de este spec). El resto de la justificación de §5 (Nyquist, cadencia típica real) **quedó empíricamente contradicho** por el Gate Nivel 1 (ver §1) y se abre como pregunta separada, fuera del alcance de este spec (ver §8). |
| plan-hil-stm32-portado-autopiloto.md | §7, "Lo que explícitamente no va en el firmware" | "Ethernet. Nada de LwIP." | Se retira esa restricción. Entra LwIP en modo `NO_SYS=1` (raw API, sin RTOS — ver §4). |
| plan-hil-stm32-portado-autopiloto.md | §14, "Qué queda explícitamente fuera" | "Ethernet / LwIP no se usa — USB CDC es suficiente." | Se retira esa restricción. |

Todo lo demás del plan maestro **se mantiene sin cambios** — ver §9 de
este spec ("Qué NO cambia") para la lista explícita.

---

## 1. Contexto y diagnóstico

### 1.1 De dónde viene esto

El plan maestro, en su tabla de riesgos (§12), ya anticipaba este
escenario exacto:

> **Riesgo:** "100 Hz resulta insuficiente para los rate loops" —
> Prob. Baja, Impacto Alto.
> **Mitigación:** "Gate Nivel 1 detecta esto antes de tocar hardware.
> Mitigación: evaluar multi-rate (outer a 50 Hz, rate loops a 200 Hz)
> con dos tipos de paquete; **o migrar a Ethernet**."

El Gate Nivel 1 (§10 del plan — SIL-decimado-a-100Hz vs. SIL-nativo-a-1kHz,
el mismo soak test de patrullaje de 18 ciclos) es, en efecto, el que lo
detectó:

1. El soak test a 100 Hz con ZOH puro producía `ZeroDivisionError` en
   `cadac_py/falcon6.py:840` cerca del ciclo 14 (ver
   `plan-mitigacion-de-inestabilidad-numerica.md`).
2. Se corrigió el ZOH por interpolación lineal del comando de actuador
   en `plane_adapter.py::_apply_autopilot_decimation()`. Esto
   eliminó el crash, pero **no cerró el gate**: dispersión máxima
   `dwbh` ≈ 269 m, `psivlx` ≈ 286° (el gate pide submilimétrico/
   centésimas de grado) — corroborado de forma independiente en dos
   corridas separadas (rama `fix/decimation-interpolation-offbyone` y
   la línea de desarrollo local).
3. Diagnóstico de causa raíz (`audit_decimation_bandwidth.py`,
   divergencia de posición punto a punto, no solo el número agregado
   al final de 18 ciclos): la trayectoria decimada ya difiere **392 m
   de la nativa a los 4 segundos de vuelo recto**, antes de cualquier
   maniobra de guiado hacia un waypoint. A los 14s, >3 km.

Una divergencia de esa magnitud en vuelo recto, en los primeros
segundos, no es compatible con "falta de suavizado de comando
acumulado" — es compatible con que **`control_rates` (el lazo de rate
roll/pitch/yaw, el más exigente en ancho de banda de todo el
autopiloto) está operando con información más vieja de la que
necesita** a 100 Hz. Ninguna interpolación de la señal de salida
arregla un lazo que toma decisiones a destiempo.

### 1.2 Por qué Ethernet y no multi-rate

El plan maestro ofrecía dos mitigaciones. Se elige Ethernet sobre
multi-rate (outer/rate loops en paquetes separados a distinta tasa)
por:

- **Hardware ya disponible, sin costo adicional:** la Nucleo-144
  F767ZI trae el PHY LAN8742A on-board (RMII), sin necesidad de
  módulo externo.
- **Menor complejidad de protocolo.** Multi-rate obliga a dos tipos de
  paquete con ciclos de vida distintos y a decidir qué pasa cuando
  llegan desalineados. Ethernet, en cambio, deja el ICD **intacto**
  (mismo `KUNTUR_PLANT_STATE`/`KUNTUR_SURFACE_CMD`, mismo contenido) y
  solo cambia cuántas veces por segundo se puede correr ese mismo
  intercambio.
- **Dejar la puerta abierta a subir la cadencia** sin rediseñar el
  transporte una segunda vez: el techo práctico de USB CDC (que
  motivó la elección de 100 Hz en primer lugar, §5 punto 3 del plan)
  deja de ser la restricción activa.

### 1.3 Qué NO resuelve este spec por sí solo

Importante para no generar una expectativa implícita: **este spec, en
su alcance actual, mantiene la cadencia en 100 Hz** (el pedido
explícito fue "manteniendo, en la medida de lo posible, la
arquitectura y el flujo de datos actuales"). Cambiar el transporte por
sí solo **no cierra el Gate Nivel 1** — lo que hace es remover la
restricción de latencia/throughput que originalmente forzó el techo de
100 Hz.

Lo que esta restricción deja de ser, a partir de la evidencia de §1.4,
es "una mejora opcional a revisar algún día": el barrido de tasas en
SIL puro muestra que **ninguna cadencia decimada probada (100/200/500
Hz) converge** dentro de las ventanas evaluadas, y que la única opción
sin decimación (1 kHz, igual a la planta) exige del transporte un
round-trip por debajo de ~1 ms — más estricto que lo que USB CDC pudo
sostener (Fase 5: 0.728-1.874 ms). Subir la cadencia deja de ser un
"paso lógico siguiente" discrecional y pasa a ser, probablemente, una
condición necesaria para que el HIL converja en absoluto — pero
**decidir y validar esa cadencia sigue, de todas formas, fuera del
alcance de este documento** (que solo cambia el transporte): lo que
cambia es que la Fase D (§7) ahora tiene que caracterizar explícitamente
si Ethernet puede sostener ese round-trip, no solo si es "mejor que
USB CDC".

### 1.4 Evidencia adicional — barrido de tasas en SIL puro (2026-10-10)

Usando `audit_decimation_bandwidth.py` (la auditoría portable citada
en §1.1), se corrió un barrido de tasas HIL contra la planta nativa
(1 kHz), en SIL puro — sin STM32, sin transporte de ningún tipo. Dos
corridas:

**Barrido corto (60s simulados, 100/200/500/1000 Hz):**

| Tasa | n | t→1m | t→100m | t→1000m | Pico en 60s |
|---|---|---|---|---|---|
| 100 Hz | 10 | 1.0s | 3.0s | 7.0s | ~6730 m (t≈55s) |
| 200 Hz | 5 | 1.0s | 3.0s | 7.0s | similar orden, *peor* punto a punto que 100 Hz |
| 500 Hz | 2 | 1.0s | 4.0s | 16.0s | ~4281 m (t≈52s) |
| 1000 Hz | 1 (= nativo, caso trivial) | nunca | nunca | nunca | 0.0 m exacto |

El caso de 1000 Hz es trivial por construcción (n=1 significa "sin
decimación", comparando nativo contra sí mismo) — confirma que el
método no inventa divergencia donde no hay decimación real, pero no es
evidencia de que 1 kHz "resuelva" nada; solo que no hay nada que
resolver cuando no se decima.

**Restricción arquitectónica que este barrido expuso:** la planta
tiene su paso de integración fijo en 1 kHz
(`RealFalcon6Adapter.int_step_s`), así que el factor de decimación
tiene que ser un divisor entero exacto de 1000. Se intentó 750 Hz como
punto intermedio entre 500 y 1000 — el propio script lo rechazó
(1000/750 = 1.333, no entero; ver `TEST_audit_decimation_bandwidth.md`
§6 para el mecanismo de esta verificación). **No existe ningún divisor
entero de 1000 entre 500 y 1000.** La única forma de "subir" desde 500
Hz es saltar directo a 1000 Hz — es decir, a correr el autopiloto en
cada paso físico, sin decimar en absoluto.

**Barrido extendido (200s simulados, 500 Hz, dos vueltas de
patrullaje):** para descartar que la curva de 500 Hz simplemente
tardara más en estabilizarse en un ciclo límite con offset:

| Evento | t (s) | pos_diff (m) |
|---|---|---|
| 1er pico | 52 | 4 281 |
| valle | 113 | 1 352 |
| **2do pico** | **180** | **10 618** (2.5× el primer pico) |

La envolvente de divergencia **crece** entre la primera y la segunda
vuelta, no se estabiliza. Confirmación independiente, más contundente
que el número de posición: en los mismos 200s, el adaptador nativo
cruza el waypoint 4 veces (t=0, 48, 105, 185s) y el decimado a 500 Hz
solo 2 veces (t=0, 113s) — la trayectoria decimada no es un desfase de
fase de la misma vuelta, es una vuelta cualitativamente distinta
(doble de duración).

**Lectura para este spec:** con los datos disponibles, 500 Hz (la
única tasa decimada "alta" que la arquitectura permite probar) no
muestra evidencia de converger en dos vueltas — al contrario, empeora.
Esto no se investiga más a fondo acá (está fuera del alcance de un
spec de transporte, ver §8), pero redefine qué tiene que validar la
Fase D: no "¿Ethernet es mejor que USB CDC?" en abstracto, sino
"¿Ethernet sostiene un round-trip compatible con 1 kHz?" — porque, a
la luz de esta evidencia, 1 kHz sin decimar deja de ser una curiosidad
y pasa a ser el candidato principal.

---

## 2. Arquitectura resultante

Mismo diagrama de 3 planos del plan maestro (§2), con el plano HIL
modificado:

```
┌────────────────────────────────────────┐
│  cadac_plant/plant_driver.py           │
│                                         │
│  (sin cambios respecto al plan §2:      │
│   planta en Python, autopiloto nativo   │
│   deshabilitado cuando HIL activo)      │
│                                         │
│  stm32_bridge (MISMA API pública,       │
│   transporte interno reemplazado):      │           ┌───────────────────────────┐
│   - cada 10 pasos (100 Hz):             │           │  STM32F767 Nucleo-144     │
│     empaqueta KUNTUR_PLANT_STATE        │ Ethernet  │  PlatformIO + STM32Cube   │
│     envía por UDP                       │  (UDP,    │  + LwIP (NO_SYS=1)       │
│   - lee KUNTUR_SURFACE_CMD por UDP      │ ────────▶ │                           │
│   - hold-last entre llamadas            │ point-to- │  Falcon6 Autopilot FULL   │
│   - timeout → fallback al autopiloto    │  point)   │  (SIN CAMBIOS — Fase 3/4  │
│     nativo Python                       │ ◀──────── │   ya validadas bit-exacto)│
│                                         │           │                           │
│  (resto del diagrama §2 del plan:       │           │  @ 100 Hz (sin cambios    │
│   Manual/FS-i6, Kuntur-GCS, FlightGear  │           │   en este spec, ver §1.3) │
│   — SIN CAMBIOS)                        │           └───────────────────────────┘
└──────────┬────────────────┬─────────────┘
           │                │
          ...               ...  (resto del diagrama §2 del plan, sin cambios)
```

El único bloque que cambia de implementación es el enlace entre
`Stm32Bridge` (host) y `main_hil.c` (STM32). Todo lo que hay a cada
lado de ese enlace —la planta, el autopiloto portado, Kuntur-GCS,
FS-i6— no se toca.

---

## 3. Direccionamiento y topología de red

- **Punto a punto**, cable Ethernet directo entre el host y el
  conector RJ45 de la Nucleo-144 (sin switch/router de por medio).
  Auto-MDI/MDIX del LAN8742 y de cualquier NIC moderna del host hace
  innecesario un cable cruzado.
- **IP estática en ambos extremos**, sin DHCP — mismo criterio
  "nada implícito" que el resto del proyecto, y evita depender de un
  servidor DHCP que no existe en un link punto a punto:
  - STM32: `192.168.7.2 /24`
  - Host: `192.168.7.1 /24` (se configura en la interfaz de red del
    host que se cablee a la Nucleo — típicamente una segunda NIC o un
    adaptador USB-Ethernet dedicado, para no interferir con la red
    "normal" del host)
- **Puerto UDP:** `7700` (arbitrario, fuera del rango privilegiado;
  documentar en `scenario_params.h` del lado STM32 y en la config del
  bridge del lado host, no hardcodeado dos veces de forma
  independiente — ver §6).
- Sin resolución de nombres, sin ARP dinámico más allá del que el
  stack resuelve solo (es un link punto a punto, un único peer de cada
  lado).

---

## 4. Decisión: UDP, no TCP

Se especifica **UDP**, no TCP, para `KUNTUR_PLANT_STATE`/
`KUNTUR_SURFACE_CMD`. Razones:

1. **El patrón de uso ya es "último valor gana", no "entrega
   garantizada".** `stm32_bridge.py` ya trata una respuesta que no
   llega a tiempo como fallback silencioso (ver
   `send_plant_state()` actual) — exactamente la semántica de UDP
   (un datagrama perdido simplemente no llega; no hay reintento). TCP
   introduciría retransmisión de un pedido que, para cuando llegara
   reintentado, ya estaría desactualizado respecto al paso de
   simulación actual — pelea contra el diseño existente en vez de
   encajar con él.
2. **UDP preserva límites de mensaje.** Cada `sendto()`/`recvfrom()`
   es exactamente un datagrama — elimina la necesidad de
   `resync_to_magic_header()` y de la lógica de frame-parcial-con-
   timeout que hoy existe en `main_hil.c` para lidiar con USB CDC
   como stream de bytes sin framing propio (ver §5 para el detalle).
   Esto es una simplificación neta del firmware, no solo un cambio de
   medio.
3. **Tamaño de payload muy por debajo del MTU.** `KUNTUR_PLANT_STATE`
   (~200 bytes) y `KUNTUR_SURFACE_CMD` (~80 bytes) están lejos del
   límite de ~1472 bytes de payload UDP sobre Ethernet estándar (MTU
   1500) — cero riesgo de fragmentación IP.
4. **Nagle/buffering de TCP introduce latencia variable** poco
   deseable en un lazo de control periódico de 10 ms de presupuesto;
   evitarlo (`TCP_NODELAY`) es posible pero es complejidad extra sin
   beneficio real frente a UDP para este patrón de tráfico.

El magic header del ICD (`KUNTUR_MAGIC_PLANT_STATE`/
`KUNTUR_MAGIC_SURFACE_CMD`) **se mantiene** como chequeo de sanidad del
contenido (defensa contra un paquete UDP espurio en el puerto, o un
bug de empaquetado) — deja de cumplir el rol de resincronización de
stream, pero sigue siendo una validación barata y correcta tenerla.

---

## 5. Diseño de modificaciones — firmware STM32

### 5.1 Qué se mantiene intacto

`src/autopilot/*`, `src/icd/kuntur_ic_plant_state.*`,
`src/icd/kuntur_ic_surface_cmd.*`, `src/config/scenario_params.h`,
`src/bsp/clock_config.*` — **cero cambios**. Son el autopiloto y el
ICD ya validados bit-exacto (Fases 3/4); este spec no reabre esa
validación.

### 5.2 Qué se agrega

```
firmware/kuntur_autopilot/src/
├── net/                              # NUEVO, paralelo a src/usb/
│   ├── middleware/                   # LwIP vendoreado (ver nota abajo)
│   ├── eth_config.c/h                # init HAL_ETH + GPIO RMII + PHY LAN8742
│   ├── lwip_port/                    # ethernetif.c, sys_arch (NO_SYS=1), lwipopts.h
│   └── kuntur_udp_io.c/h             # API de transporte (reemplaza usb_cdc_io.*
│                                     #  para el plano de datos — ver 5.3)
└── hil/
    └── main_hil.c                    # MODIFICADO — ver 5.4
```

**Nota sobre vendoreo de LwIP:** la Fase 5 (USB CDC) ya encontró que
`framework = stm32cube` de PlatformIO solo agrega `Drivers/` al
include path, no `Middlewares/` — por eso el middleware USB se
vendoreó manualmente bajo `src/usb/middleware/` (ver
`firmware/kuntur_autopilot/src/usb/middleware/README.md`). Es
razonable esperar el mismo problema con `Middlewares/Third_Party/lwip`
del framework — **verificar esto como primer paso de implementación**
(gate de la Fase 1 de este mini-plan, §7) en vez de asumirlo. Si se
confirma, mismo patrón: vendorear + `-I` explícito en
`platformio.ini`, análogo al env `nucleo_f767zi_hil` ya existente.

### 5.3 `kuntur_udp_io.*` — reemplazo de `usb_cdc_io.*` para el plano de datos

API deliberadamente más chica que la de `usb_cdc_io.h`, porque UDP ya
resuelve el framing:

```c
// kuntur_udp_io.h
void kuntur_udp_io_init(void);                 // netif up, bind al puerto §3
bool kuntur_udp_io_connected(void);             // link up (ETH) -- no implica peer activo,
                                                 // igual que usb_cdc_connected() no implicaba host activo
int  kuntur_udp_io_recv(uint8_t *buf, size_t maxlen, uint32_t timeout_ms);
                                                 // bloquea hasta timeout_ms o un datagrama completo;
                                                 // devuelve el tamaño del datagrama o -1 si no llegó nada
int  kuntur_udp_io_send(const uint8_t *buf, size_t len);
```

`usb_cdc_io.h` (y todo `src/usb/`) **queda retirado del plano de
datos**. Qué hacer con el código: ver §9.2 (decisión abierta).

### 5.4 `main_hil.c` — simplificación del loop principal

El loop actual (`resync_to_magic_header()` + peek/consume + espera de
frame parcial con timeout, ver código actual citado en §1) se
reemplaza por:

```c
for (;;) {
    uint8_t frame[KUNTUR_PLANT_STATE_SIZE];
    int n = kuntur_udp_io_recv(frame, sizeof(frame), FRAME_WAIT_TIMEOUT_MS);
    if (n != KUNTUR_PLANT_STATE_SIZE) {
        continue;  // nada llegó a tiempo, o llegó un datagrama de tamaño
                   // equivocado (se descarta entero -- no hay "resync a
                   // mitad de frame" posible con UDP, el datagrama o
                   // está completo o no llegó)
    }

    kuntur_plant_state_t in;
    if (kuntur_unpack_plant_state(&in, frame, n) != 0) {
        frames_bad++;
        continue;  // magic/contenido inválido -- se descarta el datagrama
                   // entero, sin la gimnasia de consume(1) que requería
                   // el stream de USB CDC
    }

    // ... resto del cuerpo IDÉNTICO al actual: init flag, autopilot_state_reset,
    // falcon6_autopilot_step(), pack + kuntur_udp_io_send() en vez de
    // kuntur_usb_cdc_write(), heartbeat, log de frames ok/bad cada 256.
}
```

Nótese que esto es **más simple** que el loop actual, no solo distinto
— se elimina toda la lógica de resincronización byte a byte que el
transporte-stream de USB CDC hacía necesaria.

### 5.5 `platformio.ini`

Nuevo env `nucleo_f767zi_hil_eth` (o extender el `nucleo_f767zi_hil`
existente si se decide no mantener USB CDC en paralelo — ver §9.2),
agregando:

- `-I` explícito a cada subcarpeta de LwIP vendoreado (mismo patrón
  que el env actual hace para `src/usb/middleware/*/Inc`).
- Las fuentes de `src/net/` + el subconjunto de LwIP core
  (`core/`, `core/ipv4/`, `netif/`, el puerto `NO_SYS`) agregadas a
  `build_src_filter` o `lib_extra_dirs`, análogo a como se agregó
  `src/usb/`.
- Verificar si `lib_ldf_mode = off` (que Fase 5 tuvo que agregar para
  evitar que PlatformIO compile el USB_Device_Library del framework en
  paralelo al vendoreado) también hace falta acá para LwIP — mismo
  tipo de conflicto es plausible.

---

## 6. Diseño de modificaciones — host (`kuntur_bridge`)

### 6.1 `stm32_bridge.py` — mismo contrato público, transporte interno nuevo

La clase `Stm32Bridge` ya está diseñada con el transporte
inyectable (`serial_factory`) y con una API pública que no expone
nada específico de serial (`connected`, `send_plant_state()`,
`close()`, `fallback_count`). Esto significa que **`plant_driver.py`
no necesita ningún cambio** — el único archivo que cambia es
`stm32_bridge.py` mismo, por dentro:

| Hoy (USB CDC / pyserial) | Pasa a (UDP / `socket`) |
|---|---|
| `serial.Serial(port, baudrate, timeout=timeout_s)` | `socket.socket(AF_INET, SOCK_DGRAM)` + `.connect((STM32_IP, STM32_PORT))` + `.settimeout(timeout_s)` |
| `self._conn.write(raw)` | `self._conn.send(raw)` |
| `self._conn.read(SURFACE_CMD_SIZE)` | `self._conn.recv(SURFACE_CMD_SIZE)` (con `socket.timeout` capturado igual que hoy se captura el timeout de `pyserial`) |
| `except OSError` (puerto desconectado) | `except OSError` (mismo tipo de excepción para errores de socket en Python — el `try/except OSError` actual **no necesita cambiar**) |
| `port: str` (ej. `/dev/ttyACM1`) constructor | `(host: str, port: int)` constructor |
| dependencia de terceros: `pyserial` | ninguna — `socket` es stdlib. **Esto elimina la única dependencia de terceros de este módulo.** |

El patrón de `_FakeSerial` para tests (§ smoke test actual) se
reemplaza por un `_FakeSocket` equivalente — mismo rol, mismos tres
casos (no conecta / responde a tiempo / timeout), sin tocar la
estructura del test.

**Reconexión:** con UDP no hay "conexión" real que pueda fallar al
abrir (a diferencia de un puerto serie inexistente) — `socket.connect()`
sobre UDP solo fija el peer por defecto para `send()`/`recv()`, no
hace un handshake. El caso "STM32 no está" pasa a manifestarse
siempre como timeout de `recv()`, nunca como excepción al construir
el socket. Esto simplifica `_connect()`/`_maybe_reconnect()` (ya no
hace falta el patrón de "reintentar abrir cada `RECONNECT_INTERVAL_S`"
— un socket UDP no se "cierra" del lado del host por una falla del
STM32). Revisar si conviene conservar igual un `RECONNECT_INTERVAL_S`
simbólico por higiene de logging (evitar loguear timeout en cada
llamada a 100 Hz) aunque ya no haya nada que reabrir.

### 6.2 ICD (`icd_plant_state.py`, `icd_surface_cmd.py`, `icd_schema/*.json`)

**Sin cambios.** El contenido, tamaño y layout de
`KUNTUR_PLANT_STATE`/`KUNTUR_SURFACE_CMD` es independiente del
transporte — es exactamente el punto de tener el ICD como capa propia
separada del transporte (mismo patrón que ya separa
`KUNTUR_AUX`/`KUNTUR_CTL` del resto).

### 6.3 Configuración

Agregar a donde hoy vive el puerto serie (`config.py` o los args de
`plant_driver.py`, según cómo esté hoy — revisar al implementar) los
dos valores nuevos: `STM32_ETH_HOST` (default `192.168.7.2`) y
`STM32_ETH_PORT` (default `7700`), consistentes con §3.

---

## 7. Mini plan de trabajo — fases de este spec

Pensado para insertarse como una Fase 5b (o extensión de Fase 5) del
plan maestro, sin renumerar sus fases. Reutiliza integralmente los
niveles de validación de §10 del plan — el único nivel que agrega
contenido nuevo es el equivalente del Nivel 2/3 para Ethernet.

### Fase A — Setup de red en host + verificación de LwIP en el framework

- Confirmar si `Middlewares/Third_Party/lwip` necesita vendoreo manual
  (ver nota §5.2) — **gate: confirmado antes de escribir una sola
  línea de integración**, para no repetir a ciegas el patrón de Fase
  5 sin validar la premisa.
- Configurar la NIC del host con IP estática (§3), cablear directo a
  la Nucleo.
- **Entregable:** confirmación documentada de cómo se integra LwIP con
  PlatformIO en este proyecto (vendoreado o no), antes de avanzar.

### Fase B — Firmware: init ETH + LwIP (`NO_SYS=1`) en loopback simple

- `eth_config.c` (HAL_ETH + GPIO RMII + PHY LAN8742, siguiendo el
  patrón CubeMX estándar para Nucleo-144).
- `lwip_port/` (`ethernetif.c`, `sys_arch` para `NO_SYS=1`,
  `lwipopts.h`).
- Smoke test: el STM32 responde `ping` desde el host (ICMP, antes de
  tocar UDP applicativo) — confirma el link físico + stack IP antes de
  meter el protocolo propio encima.
- **Gate:** ping responde con latencia/jitter razonable y estable
  (caracterizar, mismo criterio que Fase 5 caracterizó USB CDC).

### Fase C — `kuntur_udp_io.*` + `main_hil.c` modificado

- Implementar §5.3/§5.4.
- Reusa el mismo testbench Nivel 2 (loopback, 1000 traces) que Fase 4
  ya tiene — **gate: mismo diff 0.000000000 exacto**, confirma que el
  cambio de transporte no tocó el autopiloto (debería ser trivialmente
  cierto dado que `src/autopilot/` no se modifica, pero se revalida
  igual, mismo criterio de "nada implícito").

### Fase D — `stm32_bridge.py` sobre UDP + integración viva

- Implementar §6.1.
- Repetir el smoke test de Fase 5 (handshake real, 500 iteraciones a
  ~100 Hz, caracterizar round-trip/jitter/fallback_count) — **gate:
  mismos criterios que Fase 5 (`HIL_SURFACE_CMD_MAX_AGE_S` con margen
  equivalente o mejor)**. Comparar explícitamente contra la tabla de
  latencia de Fase 5 (USB CDC: min 0.728 ms / avg 0.953 ms / max 1.874
  ms) — es contenido de tesis directo ("¿Ethernet es más rápido/más
  determinístico que USB CDC para este caso de uso?").
- **Gate adicional, motivado por §1.4:** repetir la misma
  caracterización de round-trip a la tasa de **1 kHz** (sin decimar),
  no solo a 100 Hz. El presupuesto por ciclo a 1 kHz es 1 ms — por
  debajo del máximo ya medido en USB CDC (1.874 ms). Si Ethernet no
  sostiene round-trip + cómputo del autopiloto (~100-500 μs, Fase 4)
  dentro de ese presupuesto con margen, 1 kHz sin decimar **no es
  viable como HIL real**, aunque la evidencia de SIL (§1.4) sugiera
  que es la única cadencia decimada... en realidad no-decimada que no
  mostró divergencia. Este resultado condiciona la Fase F.

### Fase E — Soak test completo sobre Ethernet, a 100 Hz (referencia justa)

- Mismo soak test de 18 ciclos, `stm32_bridge` activo sobre Ethernet,
  **cadencia sin cambios (100 Hz)**, comparado contra el SIL-a-100Hz
  (Nivel 1).
- **Gate:** reproduce el mismo resultado que ya caracterizamos en SIL
  puro (dispersión ~269 m / ~286°) — si el transporte está bien
  implementado, el resultado HIL-sobre-Ethernet-a-100Hz debería
  converger al mismo comportamiento que SIL-decimado-a-100Hz, ni mejor
  ni peor (el transporte no cambia la dinámica de control, ver §1.3).
  Si diverge MÁS que el SIL de referencia, hay un bug de integración
  nuevo que investigar, separado del hallazgo de ancho de banda ya
  conocido. Si diverge MENOS o deja de reproducir el problema, también
  amerita entender por qué antes de confiar en el resultado.
- Esta fase sigue siendo la que cierra el alcance **de este spec**
  (transporte, nada más). Sirve como línea de base para la Fase F, que
  ya no es parte de este documento sino su continuación lógica
  inmediata — ver §8.

### Fase F (fuera de este spec, continuación directa) — Soak test a 1 kHz sin decimar, condicionado a la Fase D

- Solo si la Fase D confirma que Ethernet sostiene el presupuesto de 1
  ms: repetir el soak test de 18 ciclos con el autopiloto corriendo en
  **cada paso físico** (sin decimación), sobre Ethernet.
- **Gate:** dispersión del mismo orden que el soak test SIL original
  sin decimar (que por construcción debería ser ~0, al no haber
  decimación que introduzca error). Si este gate tampoco cierra, la
  causa ya no puede ser ancho de banda de decimación — habría que
  revisar precisión numérica del port C, timing real de
  `falcon6_autopilot_step()` en hardware, o algo no identificado
  todavía.
- Se deja fuera de este spec (es una fase nueva del plan maestro, no
  un cambio de transporte) pero se documenta acá porque la evidencia
  de §1.4 la vuelve la pregunta inmediata siguiente, no una mejora
  optativa a futuro.

---

## 8. Fuera de alcance de este spec

- **Subir la cadencia por encima de 100 Hz.** Sigue siendo una
  decisión de diseño separada de este documento (que solo cambia
  transporte) — pero, a la luz de §1.4, deja de ser un "paso lógico
  siguiente" discrecional: la evidencia en SIL puro no muestra
  ninguna tasa decimada (100/200/500 Hz) convergiendo, y la única
  tasa sin divergencia observada es 1 kHz sin decimar (caso trivial).
  Esto no se resuelve en este spec porque excede su alcance (cambiar
  *transporte*, no *cadencia ni arquitectura de control*), pero la
  Fase D (§7) ya incorpora el gate de latencia a 1 kHz necesario para
  saber si Ethernet puede sostenerlo, y la Fase F (§7, fuera de este
  documento) es la continuación directa si ese gate cierra. Si Ethernet
  **no** sostiene 1 kHz, la pregunta de fondo (¿`control_rates` puede
  seguir dependiendo de un round-trip host↔STM32 en cada ciclo, o
  necesita correr desacoplado del host?) queda abierta para un spec
  propio — no se improvisa esa respuesta acá.
- **Esquema multi-rate** (outer loops / rate loops a tasas distintas).
  Mitigación alternativa descartada en §1.2, no se retoma acá.
- Cualquier cambio a `src/autopilot/*` o al contenido del ICD.
- IPv6, DHCP, resolución de nombres, más de un peer en la red.

---

## 9. Qué NO cambia

Para que quede explícito, en el espíritu de "mantener la arquitectura
y el flujo de datos actuales" del pedido original:

### 9.1 Sin cambios en absoluto

- `cadac_py/falcon6.py` (planta, intocada desde el inicio del
  proyecto).
- `src/autopilot/*`, `src/icd/kuntur_ic_plant_state.*`,
  `src/icd/kuntur_ic_surface_cmd.*`, `src/config/scenario_params.h`
  — el autopiloto portado y el contenido del ICD.
- `plane_adapter.py`, `plant_driver.py` — la orquestación de la planta
  y el bypass HIL/Manual.
- Kuntur-GCS, FlightGear, el canal FS-i6/SM100 — ninguno de los otros
  dos planos del diagrama §2 del plan maestro se toca.
- El contrato Kuntur-IC v1.3 principal.

### 9.2 Decisión abierta (no cerrada por este spec)

**Qué hacer con `src/usb/` (el middleware CDC + `usb_cdc_io.*`) una
vez que el plano de datos se mueve a Ethernet:**

- **(a) Retirarlo del todo** (el dato ya no viaja por ahí; menos
  superficie de código para mantener bit-exacta junto con todo lo
  demás).
- **(b) Dejarlo como transporte de respaldo** (fallback de fallback:
  si Ethernet no linkea, caer a USB CDC) — agrega complejidad
  (dos transportes vivos, decidir cuál tiene prioridad) a cambio de
  redundancia que hoy no está pedida.
- **(c) Dejarlo intacto pero inactivo** (no se borra, no se usa —
  mismo criterio que ya se usó con la UI de retasking de Kuntur-GCS en
  el plan maestro §9: ocultar, no borrar).

Recomendación: **(c)**, por consistencia con el criterio que el propio
plan maestro ya aplicó en una decisión análoga, y porque no cuesta
nada mantenerlo compilando en un env de PlatformIO separado
(`nucleo_f767zi_hil`, el que ya existe) mientras el nuevo
(`nucleo_f767zi_hil_eth`) es el que se usa en adelante. Pero es una
preferencia, no una decisión cerrada — confirmar antes de la Fase C.

---

## 10. Riesgos específicos de este cambio

| Riesgo | Prob. | Impacto | Mitigación |
|---|---|---|---|
| LwIP no vendorea limpio con PlatformIO (mismo problema que USB, pero LwIP es una librería bastante más grande) | Media | Medio | Fase A lo verifica primero, antes de comprometer tiempo a la integración completa |
| RMII/PHY requiere ajuste fino de timing/pines no documentado para esta placa específica (como pasó con SWD en Fase 1 y con HSE/HSI en Fase 4) | Media | Medio | Presupuestar tiempo de debug de bajo nivel en Fase B, igual que se hizo con los hallazgos previos — es parte del patrón ya establecido en este proyecto |
| Latencia/jitter de Ethernet resulta peor de lo esperado (poco probable, pero no verificado) | Baja | Medio | Fase D lo caracteriza explícitamente contra la tabla de Fase 5 (USB CDC) antes de dar por buena la migración |
| El Gate de Fase E no reproduce el resultado de referencia (ni para bien ni para mal) sin causa clara | Media | Alto | Es, en sí mismo, el disparador para investigar antes de avanzar — no se declara éxito hasta entender la causa de cualquier discrepancia, consistente con el criterio ya aplicado al hallazgo de ancho de banda original |
| **Ethernet no sostiene el presupuesto de ~1 ms necesario para 1 kHz sin decimar (§1.4), y ninguna tasa decimada probada en SIL converge tampoco** — confirmado, no hipotético, para 100/200/500 Hz en las ventanas evaluadas | **Confirmada (parcial: falta medir latencia real de Ethernet)** | **Alto** | Fase D (§7) caracteriza el round-trip real a 1 kHz antes de invertir en Fase E/F. Si no cierra, el problema deja de ser de transporte — amerita un spec propio sobre si `control_rates` puede seguir dependiendo de un ida-y-vuelta por ciclo, o necesita ejecutarse desacoplado del host (candidato: lazo rápido nativo en el STM32, outer loops decimados vía host) |

---

## 11. Referencias

- `plan-hil-stm32-portado-autopiloto.md` — plan maestro, superseded
  parcialmente por este spec (ver tabla al inicio).
- `plan-mitigacion-de-inestabilidad-numerica.md` — diagnóstico y fix
  del `ZeroDivisionError` por ZOH puro (Fase 2, resuelto pero
  insuficiente — origen de este spec).
- `spec-auditoria-control-roll-stm32.md` — referencia de estilo/rigor
  para auditorías de este proyecto.
- `audit_decimation_bandwidth.py` + `TEST_audit_decimation_bandwidth.md`
  — herramienta de auditoría portable usada para el diagnóstico de
  causa raíz citado en §1.1 y para el barrido de tasas de §1.4.
  Validada en modo local (nominal, parámetros inválidos, ratio no
  entero) el 2026-10-10 antes de usarse para generar la evidencia de
  §1.4.
