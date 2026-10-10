

# CONTEXTO DE PROYECTO: Kuntur HIL - Porting de Autopiloto a STM32F7 (Fase 2)

## 1. Descripción general del proyecto

Estamos desarrollando el sistema HIL (Hardware-in-the-Loop) para el proyecto **Kuntur** (vehículo F-16 / CADAC Falcon 6).

### Directorio de trabajo

`kuntur_hil/porting-stm32/kuntur_cadac_hil`

### Componentes clave

- `cadac_plant/plane_adapter.py`:
  Adaptador que conecta el modelo de simulación de vuelo de CADAC (`falcon6.py`) con el puente y el autopiloto.

- `cadac_plant/validate_autopilot_decimation.py`:
  Script de validación que ejecuta el soak test de patrullaje (default 18 ciclos sobre `scenarios/waypoints_cordoba.txt`) comparando el rendimiento nativo (1 kHz) frente al autopiloto decimado a una frecuencia menor (ej. 100 Hz).

---

## 2. Estado actual y diagnóstico del problema

### Implementación actual

Se agregó el método `set_autopilot_decimation()` en `plane_adapter.py` y el parámetro `--autopilot-rate-hz` para desacoplar la frecuencia del autopiloto respecto al paso físico de la planta (1 kHz).

### Problema detectado

Al realizar la prueba de 18 ciclos a **100 Hz** (`n=10` pasos físicos por ciclo de autopiloto), la decimación utilizaba un **Retenedor de Orden Cero (ZOH) puro**.

Esto generaba comandos en "escalón" (*staircase*).

Hacia el ciclo 14 (aproximadamente en el cruce de waypoint 1014), el retraso de fase y las discontinuidades acumuladas llevaron a la planta a una singularidad numérica, arrojando el siguiente error en el modelo aerodinámico de CADAC:

```text
ZeroDivisionError: float division by zero en cadac_py/falcon6.py:840 (a12 = dma / dla)