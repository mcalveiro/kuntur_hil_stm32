/*
 * control_normal_accel.h -- lazo de aceleración normal con pole
 * placement adaptativo (ganancias recalculadas cada llamada en función
 * de pdynmc). Portado de cadac_py/falcon6.py::control_normal_accel.
 */
#ifndef KUNTUR_CONTROL_NORMAL_ACCEL_H
#define KUNTUR_CONTROL_NORMAL_ACCEL_H

#include "autopilot_state.h"

/* ancomx: comando de aceleración normal, g's. pdynmc: presión dinámica,
 * Pa. dla/dma/dmq/dmde: derivadas longitudinales. qqx_degs: tasa de
 * pitch, deg/s. az: fuerza específica eje z cuerpo, m/s^2 (FSPB[2]).
 * dvbe: TAS, m/s. gainp: ganancia del *.asc. int_step: paso de
 * integración (s) -- en HIL, el período entre llamadas al autopiloto
 * (100 Hz nominal), NO el paso físico de 1 kHz del SIL nativo (ver
 * Fase 2 del plan, validación de la decimación).
 *
 * state->zz/zzd se actualizan in-place (integrador persistente, ver
 * autopilot_state.h). gainfb1/gainfb2/gainfb3 se escriben en los
 * punteros de salida -- son los mismos que viajan por
 * KUNTUR_SURFACE_CMD (debug, ver icd_schema/kuntur_ic_surface_cmd_v1.json).
 *
 * Devuelve delecx, deg. */
double control_normal_accel(
    double ancomx, double pdynmc,
    double dla, double dma, double dmq, double dmde,
    double qqx_degs, double az, double dvbe,
    double gainp, double int_step,
    Autopilot_State_t *state,
    double *gainfb1_out, double *gainfb2_out, double *gainfb3_out
);

#endif /* KUNTUR_CONTROL_NORMAL_ACCEL_H */
