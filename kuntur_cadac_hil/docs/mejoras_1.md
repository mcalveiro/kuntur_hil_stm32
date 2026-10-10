# Plan: sonido de motor y turbina encendida en FlightGear

**Estado: planificación, pendiente de implementar en otra sesión.**

## Diagnóstico (ya establecido, no requiere más investigación)

`runfg.sh` lanza FlightGear con `--fdm=null` + `--native-fdm=socket,in,...`:
esto le pasa a FlightGear **solo posición y actitud** (protocolo
FGNetFDM), sin RPM, N1, ni estado de motor. El sonido de motor en
FlightGear está atado a propiedades internas del árbol de propiedades
(típicamente `/engines/engine[0]/rpm`, `/engines/engine[0]/n1` o
`/engines/engine[0]/running`), que en este modo **nunca se actualizan**
porque no hay un FDM interno calculándolas. No es un bug de la
integración — es una consecuencia directa y esperable de la arquitectura
"FlightGear puramente visual".

## Qué falta investigar antes de tocar código (en el equipo del usuario)

Esto no se puede resolver desde esta sesión porque requiere FlightGear
corriendo con el paquete de aeronave real instalado. Antes de la próxima
sesión de trabajo, sería útil tener:

1. **Qué propiedad exacta lee el sonido del paquete F-16 instalado**
   (Zaretto/f16 o `f16-simplified`). Con FlightGear corriendo:
   ```bash
   telnet localhost 5000
   ls /engines/engine[0]
   ```
   (el puerto 5000 es el mismo Telnet que ya usa `kuntur_bridge` para
   calibrar el origen — no hace falta abrir nada nuevo).
2. **El archivo XML que define el sonido**, para confirmar la condición
   que dispara "motor encendido":
   ```bash
   find "$HOME/.fgfs/Aircraft" -path "*f16-simplified*" -iname "*sound*.xml"
   ```
   Distintos paquetes de F-16 usan convenciones distintas (algunos leen
   `rpm`, otros `n1`, otros una variable Nasal propia) — sin esto se
   estaría adivinando el nombre de la propiedad.

## Opciones de implementación (de menor a mayor esfuerzo)

### Opción A — Forzar "motor encendido" (mínimo esfuerzo, recomendada para empezar)

En `kuntur_bridge/telnet_client.py`, justo después de `calibrate_origin()`
(ya hay una conexión Telnet abierta y viva, `FGPropertyTree`), agregar
unos pocos `prop_tree.set(...)` para fijar `running=true` y una
RPM/N1 constante razonable (valor de crucero). Dado sonido continuo,
pero no reacciona al throttle real.

- Riesgo: bajo. Cambio acotado (~10-15 líneas), no toca el ICD ni
  `cadac_py`.
- Limitación: el motor "suena" siempre igual, no varía con la fase de
  vuelo.

### Opción B — Sonido reactivo al throttle real (más fidelidad)

Extender el ICD (Kuntur-IC v1.3) para llevar el estado real del motor de
FALCON6 (`cadac_py` sí calcula empuje/RPM internamente — hay que ubicar
el índice `flat6[]`/`plane[]` exacto, con el mismo método ya usado para
mapear `SBEL`/`VBEL`/etc. en `plane_adapter.py`) hasta `kuntur_bridge`,
que lo escribiría por Telnet en cada ciclo (o a una tasa más baja — el
sonido no necesita 1kHz) a la propiedad real que el paquete lee.

Pasos:
1. Agregar un campo a `icd_schema/kuntur_ic_v1_3.json` (ej. `rpm` o
   `throttle_pct` en `KUNTUR_STATES`).
2. Correr `tools/gen_icd.py` y recompilar `kse_server`
   (`cargo build --release`) — ver la nota ya documentada sobre por qué
   ambos pasos van juntos (desincronización Python/Rust si se olvida
   uno).
3. Exponerlo en `RealFalcon6Adapter.step()`/`PlantState`.
4. Escribirlo en `kuntur_bridge` (probablemente en el mismo lugar que ya
   escribe posición vía `--native-fdm`, o vía Telnet como en la Opción A).

- Riesgo: medio (toca el contrato ICD, con recompilación de Rust).
- Beneficio: motor que responde a la física real, no un valor fijo.

### Opción C — Punto medio, sin tocar el ICD

`plant_driver.py` ya conoce internamente el estado de FALCON6 (aunque
ignora el actuador para las superficies, el motor/potencia interna del
escenario sigue siendo un valor calculable desde el propio adaptador).
Podría bastar con que el propio `plant_driver.py` (o `kuntur_bridge`, si
se le da otro canal fuera del ICD) lea ese valor sin pasar por
`KUNTUR_STATES` — evita tocar el contrato binario, a costa de un canal
paralelo ad-hoc en vez de pasar por la única fuente de verdad (ICD).

## Recomendación

Empezar por la **Opción A** en la próxima sesión (sonido ya, bajo
riesgo) y evaluar B/C después, según cuánto importe que el sonido
reaccione al throttle real vs. simplemente "que la turbina esté
prendida" durante toda la corrida.

## Qué traer a la próxima sesión

- Salida de `find "$HOME/.fgfs/Aircraft" -path "*f16*" -iname "*sound*.xml"`.
- Opcionalmente, la salida de `ls /engines/engine[0]` por Telnet con
  FlightGear corriendo, para no adivinar nombres de propiedad.