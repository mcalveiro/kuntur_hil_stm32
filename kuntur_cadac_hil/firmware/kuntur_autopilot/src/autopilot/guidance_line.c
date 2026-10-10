/*
 * guidance_line.c -- ver guidance_line.h. Portado línea por línea de
 * cadac_py/falcon6.py::guidance/guidance_line (líneas 1473-1567 al
 * momento del port).
 */
#include "guidance_line.h"
#include <math.h>

vec3_t guidance_line(
    vec3_t SWBL, double psiflx_deg, double thtflx_deg, vec3_t VBEL,
    double thtvlx_deg, double grav, double line_gain, double nl_gain_fact, double decrement
) {
    mat33_t TFL = fc_mat2tr(psiflx_deg * RAD, thtflx_deg * RAD);

    vec3_t POLAR = vec3_pol_from_cart(SWBL);
    double wp_sltrange = POLAR.v[0];
    double psiol = POLAR.v[1];
    double thtol = POLAR.v[2];
    mat33_t TOL = fc_mat2tr(psiol, thtol);

    vec3_t VBEO = mat33_mulvec3(TOL, VBEL);
    double vbeo2 = VBEO.v[1], vbeo3 = VBEO.v[2];

    vec3_t VBEF = mat33_mulvec3(TFL, VBEL);
    double vbef2 = VBEF.v[1], vbef3 = VBEF.v[2];

    double nl_gain = nl_gain_fact * (1 - exp(-wp_sltrange / decrement));

    double algv1 = grav * sin(thtvlx_deg * RAD);
    double algv2 = line_gain * (-vbeo2 + nl_gain * vbef2);
    double algv3 = line_gain * (-vbeo3 + nl_gain * vbef3) - grav * cos(thtvlx_deg * RAD);

    return vec3_make(algv1, algv2, algv3);
}

void guidance(
    int mguid, double swel1, double swel2, double swel3,
    double psiflx_deg, double thtflx_deg,
    vec3_t SBEL, vec3_t VBEL, double thtvlx_deg, double dvbe, double grav, double philimx,
    double line_gain, double nl_gain_fact, double decrement,
    double *alcomx_state, double *ancomx_state,
    double *wp_flag_out
) {
    if (mguid == 0) {
        /* Igual que el original: sin p[522]/p[523].gets(), el valor
         * del ciclo anterior queda tal cual (ver autopilot_state.h). */
        if (wp_flag_out) *wp_flag_out = 0.0;
        return;
    }

    int mguidl = mguid / 10;
    int mguidp = mguid % 10;

    vec3_t SWEL = vec3_make(swel1, swel2, swel3);
    vec3_t SWBL = vec3_sub(SWEL, SBEL);
    double dwbh = sqrt(SWBL.v[0] * SWBL.v[0] + SWBL.v[1] * SWBL.v[1]);

    double wp_flag = 1.0;
    double turn_min = dvbe * dvbe / (grav * tan(philimx * RAD));

    if (dwbh < (2 * turn_min)) {
        double closing = SWBL.v[0] * VBEL.v[0] + SWBL.v[1] * VBEL.v[1];
        wp_flag = fc_sign(closing);
        /* Los print() de diagnóstico del original (dwbh/dwb/psivlx/
         * thtvlx) no tienen efecto en el cálculo -- no se portan. */
    }

    double alcomx = 0.0;
    double ancomx = 0.0;
    vec3_t ALGV = guidance_line(SWBL, psiflx_deg, thtflx_deg, VBEL, thtvlx_deg, grav, line_gain, nl_gain_fact, decrement);
    if (mguidl == 3) {
        alcomx = ALGV.v[1] / grav;
    }
    if (mguidp == 3) {
        ancomx = -ALGV.v[2] / grav;
    }

    *alcomx_state = alcomx;
    *ancomx_state = ancomx;
    if (wp_flag_out) *wp_flag_out = wp_flag;
}
