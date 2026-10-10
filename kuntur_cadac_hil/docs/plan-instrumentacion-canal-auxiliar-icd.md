# Plan: canal ICD auxiliar de instrumentación (cockpit completo, kuntur-cadac fork)

**Estado: implementación en curso — motor/sonido confirmado funcionando en
banco de pruebas (sesión 2026-09-24). Instrumentación básica de vuelo
diagnosticada pero **pendiente de aplicar el fix** (ver "Estado de la
prueba de humo" abajo). Ver también `learnings-and-practices.md`
(memoria del proyecto) para el hallazgo de `FGPropertyTree.set()`.**

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
eventualmente replicar esa arquitectura en Simulink.

## Estado de la prueba de humo (sesión 2026-09-24)

Implementación ya existente en el fork `v04` (carpeta
`kuntur_bridge/`, con copia de entrega en `kuntur_aux_channel_delivery/`)
al momento de esta sesión: `icd_aux.py`, `aux_instrument_worker.py`,
`telnet_client.py::apply_aux_instrumentation()`, canal UDP en puerto
7000, decimación ×40. Diseño coincide con lo especificado en este
documento.

### ✅ Confirmado funcionando — motor y sonido

- `tcpdump -i lo -n udp port 7000`: paquetes de 60 bytes exactos, cadencia
  ~40ms (25Hz) sostenida — el emisor (`plant_driver.py`) nunca fue el
  problema.
- **Bug encontrado y corregido**: `FGPropertyTree.set()` en
  `telnet_client.py` llamaba a `self._readline()` después de cada
  escritura, esperando una respuesta que el protocolo Telnet de
  FlightGear en modo `data` **no envía para `set`** (solo `get` devuelve
  línea). Con 14 `set()` por paquete y ~3s de timeout cada uno, procesar
  un solo paquete tardaba ~42s mientras seguían llegando a 25Hz — el
  socket UDP se iba atrasando sin límite, dando la apariencia de un
  canal "congelado" en un valor fijo. **Fix aplicado**: se sacó la
  llamada a `self._readline()` de `set()` (conexión de solo escritura,
  sin `get()` en el mismo socket, así que no hay riesgo de bytes
  huérfanos). Confirmado por consola: `n1`/`n2`/`running` ahora
  responden con valores reales y cambiantes en tiempo real, y el sonido
  de motor (interno) reacciona audiblemente al throttle/potencia real
  del escenario, con el giro adaptándose al movimiento real de la
  aeronave.
- `sound-n2-vol-a` y `n1-out1` (crossfade fino entre capas de sonido):
  confirmado que **no** se derivan solas vía Nasal como se había asumido
  — el nodo ni siquiera existe (`'' (unknown)`). No bloqueante: las capas
  principales (`engine0-*-low`/`-high`, que usan `n1`/`n2` directo) ya
  funcionan y dan el resultado audible confirmado. Queda como
  refinamiento opcional (plan B del documento original: escribirlas
  explícitamente desde `kuntur_bridge` si se nota el corte entre capas).

### ❌ Diagnosticado, pendiente de aplicar — instrumentación básica de vuelo

`velocities/airspeed-kt`, `orientation/alpha-deg`,
`velocities/vertical-speed-fps` (y probablemente `position/altitude-agl-ft`)
leen `0` de forma persistente, **a pesar de que `apply_aux_instrumentation()`
las escribe correctamente cada 40ms**. Confirmado con prueba directa:

```
set velocities/airspeed-kt 999
get velocities/airspeed-kt   →  0   (inmediato, sin ventana de carrera)
```

**Causa raíz, no es un bug de `kuntur_bridge`:** estas son propiedades
núcleo de FlightGear, atadas (`tied`) al struct FGNetFDM que ya alimenta
`--native-fdm` (puerto 5501, vía `fgnetfdm.py`/`flightgear_worker.py`) —
de solo lectura desde Telnet, sin ventana de escritura ni siquiera
momentánea. El canal auxiliar (7000/Telnet) **no puede** escribirlas por
diseño del propio FlightGear, para ningún valor que se pruebe.

Revisado el código de `fgnetfdm.py`: el struct FGNetFDM v24 (408 bytes)
ya trae los campos `agl`, `alpha`, `beta`, `vcas`, `climb_rate` de
fábrica — pero `pack_fg_net_fdm()` los recibe **hardcodeados en `0.0`**,
ni siquiera expuestos como parámetros de la función. Y
`flightgear_worker.py` ya calcula `agl_m` (línea 86) pero nunca lo pasa
al paquete; y el snapshot de KSE (`runtime.snapshot()`, ICD v1.3) ya trae
`alpha`, `beta`, `ias` (EAS) y `vel_ned` (de donde sale `climb_rate`) sin
necesidad de tocar el canal auxiliar para esto.

**Dato colateral (no bloqueante):** la llamada a `pack_fg_net_fdm()` en
`flightgear_worker.py` pasa `throt=0.0` fijo — el campo nativo
`rpm[0] = throt * 2700` tampoco se alimenta nunca. La fórmula de escalado
ya existe, simplemente no se usa. No es parte del canal auxiliar (que ya
resuelve motor/sonido por otra vía, `n1`/`n2`), pero vale la pena
corregirlo si en algún momento se necesita `/engines/engine[0]/rpm`
genérico (no específico del paquete de sonido) para algún instrumento.

**Confirmado, no requiere fix:** `accelerations/pilot-gdamped` (1.08g,
valor creíble) se deriva sola de la diferenciación de posición/velocidad
que ya recibe FlightGear por `--native-fdm` — no depende de que
`apply_aux_instrumentation()` escriba `/accelerations/pilot-g`
explícitamente (que también es, casi con certeza, una propiedad atada de
solo lectura de la misma familia que `airspeed-kt`).

### Patch pendiente de aplicar (no aplicado todavía)

**`kuntur_bridge/fgnetfdm.py`** — agregar parámetros y usarlos en el
bloque correspondiente del struct (reemplaza el `*([0.0] * 16)` actual):

```python
def pack_fg_net_fdm(
    lon_rad: float, lat_rad: float, alt_m: float,
    phi_rad: float, theta_rad: float, psi_rad: float,
    elev: float, ail: float, rud: float, throt: float,
    agl_m: float = 0.0,
    alpha_rad: float = 0.0,
    beta_rad: float = 0.0,
    vcas_kt: float = 0.0,
    climb_rate_fps: float = 0.0,
) -> bytes:
    data = struct.pack(
        _FMT,
        24, b"\x00\x00\x00\x00",
        lon_rad, lat_rad, alt_m,
        agl_m, phi_rad, theta_rad, psi_rad, alpha_rad, beta_rad,
        0.0, 0.0, 0.0, vcas_kt, climb_rate_fps, *([0.0] * 11),
        1, 2, 0, 0, 0,
        throt * 2700, 0, 0, 0,
        *([0.0] * 32),
        1, 100.0, 0.0, 0.0, 0.0,
        3, 0, 0, 0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0, 0, 0.0,
        elev, 0.0, 0.0, 0.0, ail, ail, rud, 0.0, 0.0, 0.0,
    )
    assert len(data) == 408, f"FDM packet: {len(data)} bytes (esperados 408)"
    return data
```

**`kuntur_bridge/flightgear_worker.py`** — pasar los valores que ya están
disponibles (`agl_m` ya calculado en el propio archivo; `alpha`/`beta`/
`ias`/`vel_ned` desde `snap`, nombres de atributo exactos **a confirmar**
contra `kse_client.py` antes de escribir el patch final — no verificado
todavía en esta sesión):

```python
packet = pack_fg_net_fdm(
    lon_rad, lat_rad, alt_m, phi_rad, theta_rad, psi_rad,
    elev=0.0, ail=0.0, rud=0.0, throt=0.0,
    agl_m=agl_m,
    alpha_rad=snap.alpha,               # confirmar nombre de atributo
    beta_rad=snap.beta,                 # confirmar nombre de atributo
    vcas_kt=snap.ias * 1.94384,         # confirmar nombre de atributo (ias = EAS, m/s)
    climb_rate_fps=-snap.vd * 3.28084,  # confirmar nombre/forma de vel_ned (¿snap.vd o snap.vel_ned[2]?)
)
```

## Qué traer a la próxima sesión

1. **Confirmar los nombres de atributo del snapshot de KSE**
   (`kse_client.py` — la clase que devuelve `runtime.snapshot()`) para
   cerrar el patch de `flightgear_worker.py` con los nombres reales.
2. Aplicar los dos patches (`fgnetfdm.py` + `flightgear_worker.py`).
3. Repetir la prueba de humo: `get velocities/airspeed-kt`,
   `orientation/alpha-deg`, `velocities/vertical-speed-fps`,
   `position/altitude-agl-ft` — deberían reflejar el vuelo real.
4. Opcional: decidir si vale la pena corregir el `throt=0.0` fijo para
   alimentar `rpm[0]` nativo (uso secundario, no bloqueante).
5. Opcional: si se nota corte en las capas de sonido finas (N2_88/N100),
   escribir `sound-n2-vol-a/b/c/d` y `n1-out1/n1-out2` explícitamente
   desde `kuntur_bridge` (plan B ya documentado, no aplicado por no ser
   bloqueante).

## Schema — `icd_schema/kuntur_ic_aux_v1.json`

(Sin cambios respecto de la versión anterior — el canal auxiliar de 60
bytes queda tal como se diseñó; el fix de instrumentación básica no lo
toca, se resuelve enteramente en `fgnetfdm.py`/`flightgear_worker.py`
con datos que ya vienen por el canal KSE principal.)

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
        {"name": "pdynmc", "kind": "scalar", "type": "f32", "unit": "Pa", "note": "presion dinamica -- cadac_py flat6[57]"},
        {"name": "hbe", "kind": "scalar", "type": "f32", "unit": "m", "note": "altura AGL -- cadac_py flat6[239]"},
        {"name": "altrate", "kind": "scalar", "type": "f32", "unit": "m/s", "note": "variometro -- cadac_py plane[530]"},
        {"name": "alppx", "kind": "scalar", "type": "f32", "unit": "deg", "note": "AOA -- cadac_py flat6[140]"},
        {"name": "betax", "kind": "scalar", "type": "f32", "unit": "deg", "note": "sideslip -- cadac_py flat6[145]"},
        {"name": "anx", "kind": "scalar", "type": "f32", "unit": "g", "note": "factor de carga normal -- cadac_py flat6[244]"},
        {"name": "throttle", "kind": "scalar", "type": "f32", "unit": null, "range": [0.0, 1.0], "note": "cadac_py plane[52] -- fuente de 'running' derivado en kuntur_bridge (running = throttle > 0)"},
        {"name": "power", "kind": "scalar", "type": "f32", "unit": "%", "note": "potencia lograda -- cadac_py plane[59] -- escrita a n1 y n2, indexado y sin indexar"},
        {"name": "thrust", "kind": "scalar", "type": "f32", "unit": "N", "note": "empuje -- cadac_py plane[55]"}
      ]
    }
  }
}
```

## Etapa 2 (explícitamente fuera de este plan)

Una vez el cockpit esté funcional, la etapa siguiente es la
caracterización de arquitectura y sintonía de Planta/Autopiloto/Guiado
sobre la implementación actual en CADAC — no el desacople en tres
procesos separados que planteaba originalmente
`plan-desacople-gnc-autopiloto-guiado.md`, sino un objetivo más cercano a
entender y documentar el comportamiento existente para replicarlo en
Simulink.
