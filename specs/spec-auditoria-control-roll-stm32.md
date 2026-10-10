# Spec: auditoría de `control_roll` en el portado a STM32 — cierre de un acoplamiento no declarado

**Estado: AUDITORÍA CERRADA (2026-10-10).** Resultado: lazo portado explícito y correcto. La premisa de diseño con la que se escribió esta spec (§2 original) era incorrecta — corregida abajo contra el Python real. Fase 6 (soak test) desbloqueada en lo que respecta a este punto.

## 0. Resultado de la auditoría (reemplaza la hipótesis de §2)

Auditoría ejecutada contra `cadac_py/falcon6.py` líneas 1162-1296, comparado línea por línea con el port en `firmware/kuntur_autopilot/src/autopilot/control_rates.c` y `control_mix.c`.

**La cadena de mando que proponía la versión original de esta spec —
`control_heading → phicomx → control_roll → pcomx → control_roll_rate` —
no es como funciona FALCON6.** Lo real, verificado contra el Python:

- `control_roll(phicomx)` es **una única función** que combina realimentación de posición *y* de tasa en el mismo cálculo (ganancias `gkp`/`gkphi`, derivadas de `wrcl`/`zrcl`) y devuelve `delacx` **directo** — nunca pasa por un `pcomx` intermedio ni llama a `control_roll_rate`.
- `pcomx` (`plane[554]`) es una **entrada separada e independiente**, que solo se usa si `mroll==1` (modo tasa pura) — rama que `control_roll` (modo posición, `mroll==0`) nunca toca. No es la salida de `control_roll` alimentando a `control_roll_rate`; son dos caminos alternativos seleccionados por `mroll`, no dos etapas en cascada.

Matriz de la spec original (§4), resultado por punto:

| Punto | Resultado |
|---|---|
| a) ¿`control_roll` existe distinto de `control_roll_rate`? | Sí — ambas funciones presentes y distintas (`control_rates.c:9` y `:21`) |
| b) ¿`wrcl`/`zrcl`/`philimx` son campos propios? | Sí, en `scenario_params.h` — correctamente ahí y no en `Autopilot_State_t`, porque no son estado persistente (tampoco lo son en Python: se leen de `plane[]` en cada llamada) |
| c) ¿`phicomx` se colapsa en `pcomx` por ganancia estática? | No aplica — la premisa de un paso intermedio por `pcomx` era incorrecta (ver arriba); el port en C es línea por línea idéntico al Python, incluida esta estructura |
| d) ¿Se lee `mroll`? | Sí, explícito, en `control_mix()` (`if (mroll == 0) ... else if (mroll == 1)`). Fijo en `0` en `scenario_params.h`, verificado contra `f16c11_7.asc` (nunca fija `mroll`, queda en el default `0` de `Variable.init()` en Python) — documentado con esa justificación exacta en el comentario de `scenario_params.h` desde que se escribió el port |

**Conclusión: fila 1 de la matriz original — lazo portado explícito, correcto bit a bit. No hay omisión ni colapso.**

## 1. Relación con los documentos existentes

| Documento | Rol respecto de este |
|---|---|
| `plan-hil-stm32-portado-autopiloto.md` | Actualizado (2026-10-10): §7 ahora documenta que `control_rates.c` contiene también `control_roll` (posición), no solo los tres lazos de tasa; §3 lleva la nota de auditoría cerrada. |
| `plan-desacople-gnc-autopiloto-guiado.md` | Su pendiente "relevamiento completo de constantes de sintonía por lazo" marcaba `control_roll` como abierto — actualizado (2026-10-10) para reflejar este cierre. |
| `catalogo-senales-cadac-py.md` | Fuente de los índices `plane[]` usados como marcadores de búsqueda en la auditoría original. |

## 2. Qué se dejó implícito (histórico — hipótesis original, incorrecta)

> Se deja esta sección tal cual se escribió originalmente, como registro de la hipótesis de partida y por qué estaba mal — útil como ejemplo de "verificar contra la fuente real antes de asumir topología", que es exactamente el principio que esta auditoría terminó demostrando en la práctica.

El autopiloto FALCON6 tiene, según `mroll` (`plane[501]`, "=0: Roll position; =1: Roll rate control"), dos modos de gobierno del alabeo. La hipótesis original asumía una cadena en cascada:

```
control_heading → phicomx (φ comandado, P[553])
                      │
                 control_roll  (ganancias wrcl/zrcl, P[512/513]; límite philimx, P[511])
                      │
                   pcomx (tasa de alabeo comandada, P[554])
                      │
               control_roll_rate  (ganancias tp/zetlagr, P[548/549])
                      │
                   delacx → control_mix → actuator
```

**Esto es incorrecto** — ver §0. La estructura real es `control_roll(phicomx) → delacx` directo cuando `mroll==0`, o `control_roll_rate(pcomx) → delacx` directo cuando `mroll==1` (nunca investigado en profundidad porque el escenario usado fija `mroll=0`) — dos caminos alternativos, no una cascada.

## 3-6. (secciones originales de procedimiento — superadas por el resultado de §0)

Las secciones de metodología (marcadores a buscar, matriz de resultados, justificación del Nivel 0, dónde declarar el resultado) cumplieron su función — guiaron la auditoría real que dio el resultado de §0 — y se dejan sin reescribir como registro del proceso. No son accionables de nuevo: el resultado ya está cerrado.

## 7. Qué quedó actualizado como consecuencia

- `plan-hil-stm32-portado-autopiloto.md` §7: descripción de `control_rates.c` corregida para mencionar `control_roll` (posición) además de los tres lazos de tasa.
- `plan-hil-stm32-portado-autopiloto.md` §3: nota de auditoría cerrada agregada, para que la pregunta no se reabra más adelante.
- `plan-desacople-gnc-autopiloto-guiado.md`: pendiente de `control_roll` marcado resuelto.

## 8. Próximo paso

Fase 6 (soak test de 18 ciclos) desbloqueada en lo que respecta a `control_roll` — sin pendiente de auditoría de este lazo. La recomendación de §5 de la versión original (confirmar que las 10 traces de Nivel 0 incluyen un viraje significativo) sigue siendo válida como buena práctica general de cobertura de test, pero ya no por sospecha de omisión en este lazo puntual.
