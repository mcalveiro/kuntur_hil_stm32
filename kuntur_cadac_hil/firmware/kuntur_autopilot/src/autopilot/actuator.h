/*
 * actuator.h -- dinámica de superficies (actuator/actuator_scnd),
 * portado de cadac_py/falcon6.py. Esto es lo que hace que
 * KUNTUR_SURFACE_CMD lleve la deflexión REAL (post-dinámica), no el
 * comando crudo de 'control' -- ver icd_schema/kuntur_ic_surface_cmd_v1.json.
 */
#ifndef KUNTUR_ACTUATOR_H
#define KUNTUR_ACTUATOR_H

#include "autopilot_state.h"
#include "math_utils.h"

/* ACTCX: comando crudo de 'control' (delacx/delecx/delrcx), deg.
 * int_step: paso de integración (s) -- el mismo que usa el resto del
 * autopiloto en este ciclo (ver control_normal_accel.h sobre 100 Hz
 * vs 1 kHz). mact/dlimx/ddlimx/wnact/zetact: parámetros del *.asc
 * (scenario_params.h). state->DX/DXD/DDX/DDXD se actualizan in-place.
 * Devuelve ACTX (delax/delex/delrx), deg. */
vec3_t actuator(
    vec3_t ACTCX, double int_step,
    int mact, double dlimx, double ddlimx, double wnact, double zetact,
    Autopilot_State_t *state
);

/* Dinámica de 2do orden (mact==2) -- expuesta aparte para fidelidad
 * con el original (actuator() la llama solo si mact==2). */
vec3_t actuator_scnd(
    vec3_t ACTCX, double int_step,
    double dlimx, double ddlimx, double wnact, double zetact,
    Autopilot_State_t *state
);

#endif /* KUNTUR_ACTUATOR_H */
