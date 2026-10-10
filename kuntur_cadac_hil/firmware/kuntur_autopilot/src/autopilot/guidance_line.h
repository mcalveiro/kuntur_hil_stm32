/*
 * guidance_line.h -- portado de cadac_py/falcon6.py::guidance /
 * guidance_line.
 *
 * Hallazgos del port:
 * - guidance() lee `halt = f[1].integer()`, `psivlx` y `thtvlx` pero
 *   SOLO los usa en los print() de diagnóstico (nunca en el cálculo) --
 *   no se portan esos parámetros a esta función.
 * - guidance_line() sí necesita thtvlx (ángulo de trayectoria de
 *   vuelo), pero éste NO viaja por el wire -- se deriva exactamente
 *   igual que cadac_py/falcon6.py::init_newton()/newton(): atan2(-vd,
 *   sqrt(vn²+ve²)) a partir de VBEL (vn/ve/vd, ya en
 *   KUNTUR_PLANT_STATE). control_heading() también lo necesita (como
 *   psivlx, el otro ángulo del mismo par) -- por eso el cálculo vive
 *   una sola vez en falcon6_autopilot.c (el orquestador), no acá
 *   adentro, y thtvlx_deg se recibe ya calculado.
 */
#ifndef KUNTUR_GUIDANCE_LINE_H
#define KUNTUR_GUIDANCE_LINE_H

#include "math_utils.h"

/* SWBL: vector del vehículo al waypoint, NED, m (SWEL - SBEL).
 * psiflx_deg/thtflx_deg: ángulos de línea de ataque del waypoint
 * (wire). VBEL: velocidad NED, m/s. thtvlx_deg: ángulo de trayectoria
 * de vuelo (ver nota de arriba). grav: gravedad local, m/s^2.
 * line_gain/nl_gain_fact/decrement: ganancias del *.asc. Devuelve ALGV
 * (vec3_t), m/s^2. */
vec3_t guidance_line(
    vec3_t SWBL, double psiflx_deg, double thtflx_deg, vec3_t VBEL,
    double thtvlx_deg, double grav, double line_gain, double nl_gain_fact, double decrement
);

/* mguid: modo de guiado (wire). swel1/swel2/swel3: waypoint activo,
 * NED, m (wire). psiflx_deg/thtflx_deg: wire. SBEL: posición NED, m
 * (sn/se/sd, wire). VBEL: velocidad NED, m/s (wire). dvbe: TAS, m/s.
 * grav: gravedad local. philimx: límite de roll del *.asc.
 * line_gain/nl_gain_fact/decrement: ver guidance_line(). state:
 * alcomx/ancomx persisten si mguid==0 (ver autopilot_state.h).
 * wp_flag_out: avance de waypoint (debug, viaja por
 * KUNTUR_SURFACE_CMD) -- NULL-safe. */
void guidance(
    int mguid, double swel1, double swel2, double swel3,
    double psiflx_deg, double thtflx_deg,
    vec3_t SBEL, vec3_t VBEL, double thtvlx_deg, double dvbe, double grav, double philimx,
    double line_gain, double nl_gain_fact, double decrement,
    double *alcomx_state, double *ancomx_state,
    double *wp_flag_out
);

#endif /* KUNTUR_GUIDANCE_LINE_H */
