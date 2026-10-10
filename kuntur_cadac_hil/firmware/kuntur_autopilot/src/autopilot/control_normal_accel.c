/*
 * control_normal_accel.c -- ver control_normal_accel.h. Portado línea
 * por línea de cadac_py/falcon6.py::control_normal_accel (líneas
 * 1323-1357 al momento del port).
 */
#include "control_normal_accel.h"
#include "math_utils.h"

double control_normal_accel(
    double ancomx, double pdynmc,
    double dla, double dma, double dmq, double dmde,
    double qqx_degs, double az, double dvbe,
    double gainp, double int_step,
    Autopilot_State_t *state,
    double *gainfb1_out, double *gainfb2_out, double *gainfb3_out
) {
    double waclp = 2 + 0.0001226 * (pdynmc - 5500);
    double zaclp = 0.7 - 0.0000245 * (pdynmc - 5500);
    double paclp = 5 + 0.0003067 * (pdynmc - 5500);

    double gainfb3 = waclp * waclp * paclp / (dla * dmde);
    double gainfb2 = (2.0 * zaclp * waclp + paclp + dmq - dla / dvbe) / dmde;
    double gainfb1 = ((waclp * waclp + 2.0 * zaclp * waclp * paclp + dma + dmq * dla / dvbe
                       - gainfb2 * dmde * dla / dvbe) / (dla * dmde) - gainp);

    double fspb3 = az; /* FSPB[2] */
    double zzd_new = AGRAV * ancomx + fspb3;
    double zz = fc_integrate(zzd_new, state->zzd, state->zz, int_step);
    state->zzd = zzd_new;
    state->zz = zz;

    double dqc = -gainfb1 * (-fspb3) - gainfb2 * qqx_degs * RAD + gainfb3 * zz + gainp * state->zzd;
    double delecx = dqc * DEG;

    if (gainfb1_out) *gainfb1_out = gainfb1;
    if (gainfb2_out) *gainfb2_out = gainfb2;
    if (gainfb3_out) *gainfb3_out = gainfb3;

    return delecx;
}
