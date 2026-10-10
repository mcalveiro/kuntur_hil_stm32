## Especificación Técnica: Mitigación de Inestabilidad Numérica por Decimación de Autopiloto (Fase 2 HIL)

---

### 1. Contexto y Diagnóstico

Durante la ejecución del soak test de validación de la Fase 2 del plan HIL (18 ciclos sobre el escenario de patrullaje de Córdoba, comparando el autopiloto nativo a 1 kHz frente al autopiloto decimado a 100 Hz, $n=10$), se detectó una inestabilidad numérica hacia el final de la prueba (aproximadamente en el cruce de waypoint número 1014, equivalente al ciclo 14).

* **Excepción observada:**
```text
ZeroDivisionError: float division by zero en cadac_py/falcon6.py:840 (a12 = dma / dla)

```


* **Síntoma:** El denominador `dla` en el modelo aerodinámico de CADAC se anula debido a una condición límite o excursión de estado de la aeronave.
* **Causa Raíz:** La implementación actual de la decimación utiliza un **Retenedor de Orden Cero (ZOH puro)**, manteniendo los comandos de los actuadores y lazos de control constantes durante bloques de 10 pasos físicos ($10\,\text{ms}$). Esto genera un perfil de comandos en "escalón" (*staircase*). Las discontinuidades abruptas en las derivadas de control inducen pequeños transitorios de fase y alta frecuencia que, al acumularse a lo largo de un recorrido de larga duración (~38 km), desestabilizan el régimen aerodinámico del F-16 llevándolo a una singularidad numérica.

---

### 2. Especificación de la Solución

Para resolver la inestabilidad sin alterar el código de referencia de la planta (`cadac_py/falcon6.py`), se especifica la transición de un retenedor de orden cero (ZOH) a un esquema de **Interpolación Lineal de Comandos** en la capa de adaptación (`plane_adapter.py`).

#### Principio de Operación

En lugar de congelar el comando del autopiloto durante los $n$ pasos físicos del intervalo de decimación, el adaptador calculará una rampa lineal entre el comando anterior y el nuevo comando entregado por el autopiloto.

Matemáticamente, para el paso interno $k$ dentro del intervalo de decimación ($k = 0, 1, \dots, n-1$):

$$\mathbf{u}(k) = \mathbf{u}_{\text{prev}} + \frac{k}{n} (\mathbf{u}_{\text{target}} - \mathbf{u}_{\text{prev}})$$

Esto elimina las discontinuidades de primer orden (efecto escalón) y reduce drásticamente el retraso de fase adicional introducido por la reducción de la frecuencia de muestreo de 1 kHz a 100 Hz.

---

### 3. Diseño de Modificaciones en `plane_adapter.py`

Se actualizará la lógica del método de paso y control en `RealFalcon6Adapter` para soportar interpolación en lugar de asignación directa por escalón:

1. **Estado Interno del Adaptador:**
* Mantener registro del vector de comandos anterior ($\mathbf{u}_{\text{prev}}$) y el vector de comandos objetivo actual ($\mathbf{u}_{\text{target}}$).
* Al cumplirse el ciclo de decimación (cada $n$ pasos físicos), actualizar $\mathbf{u}_{\text{prev}} \leftarrow \mathbf{u}_{\text{target}}$ y capturar el nuevo comando del autopiloto como $\mathbf{u}_{\text{target}}$.


2. **Interpolación por Paso Físico:**
* En cada llamada a `adapter.step()`, calcular el factor de interpolación $\alpha = k / n$ y aplicar el comando interpolado a los registros de la planta correspondientes (`plane` / superficies de control).



---

### 4. Criterios de Aceptación (Gate Nivel 1)

Una vez implementada la interpolación lineal en `plane_adapter.py`, se reejecutará el script de validación completa:

```bash
python3 -u -m cadac_plant.validate_autopilot_decimation --cycles 18 --rate-hz 100

```

* **Criterio 1 (Robustez numérica):** La corrida completa de 18 ciclos debe finalizar sin excepciones ni errores de división por cero (`ZeroDivisionError`) en `falcon6.py`.
* **Criterio 2 (Precisión de trayectoria):** La dispersión máxima en los puntos de reenganche de waypoints debe mantenerse dentro del orden requerido por el plan HIL:
* Distancia de aproximación horizontal (`dwbh`): submilimétrica a pocos centímetros.
* Rumbo en el instante de cruce (`psivlx`): del orden de centésimas de grado.