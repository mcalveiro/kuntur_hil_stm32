/*
 * control_mix.c -- ver control_mix.h. Portado línea por línea de
 * cadac_py/falcon6.py::control.
 */
#include "control_mix.h"
#include "control_rates.h"
#include "control_normal_accel.h"
#include "control_lateral_accel.h"
#include "control_gamma.h"
#include "control_heading.h"
#include "control_altitude.h"
#include "math_utils.h"
#include <math.h>

ControlMixOutput_t control_mix(
    int maut, int mroll,
    double alcomx, double ancomx, double altcom,
    double psivlcomx, double *phicomx_state, double pcomx, double qcomx, double rcomx,
    double thtvlcomx,
    double anlimpx, double anlimnx, double gmax, double gminx,
    double dalimx, double delimx, double drlimx, double philimx,
    double phiblx_deg, double thtblx_deg, double psivlx_deg, double thtvlx_deg,
    double ppx_degs, double qqx_degs, double rrx_degs,
    double pdynmc, double az, double dvbe, double grav, double vd, double hbe,
    double dla, double dlde, double dma, double dmq, double dmde,
    double dyb, double dydr, double dnb, double dnr, double dndr,
    double dllp, double dllda,
    double wrcl, double zrcl, double tp, double zetlagr, double gainp,
    double gainl, double facthead, double pgam, double wgam, double zgam,
    double gainalt, double gainaltrate,
    double int_step,
    Autopilot_State_t *state,
    double *gainfb1_out, double *gainfb2_out, double *gainfb3_out,
    double *altrate_out
) {
    double delacx = 0.0, delecx = 0.0, delrcx = 0.0;

    if (maut == 0) {
        ControlMixOutput_t out = {0.0, 0.0, 0.0};
        return out;
    }

    double phicomx = *phicomx_state;

    int mauty = maut / 10;
    int mautp = maut % 10;

    if (mauty == 2) {
        delrcx = control_yaw_rate(rcomx, rrx_degs, dvbe, dyb, dydr, dnb, dnr, dndr, zetlagr);
    }
    if (mautp == 2) {
        delecx = control_pitch_rate(qcomx, qqx_degs, dvbe, dla, dlde, dma, dmq, dmde, zetlagr);
    }

    if (mauty == 3) {
        phicomx = control_lateral_accel(alcomx, az, gainl);
    }

    if (mautp == 3) {
        if (ancomx > gmax) ancomx = gmax;
        if (ancomx < gminx) ancomx = gminx;
        delecx = control_normal_accel(ancomx, pdynmc, dla, dma, dmq, dmde, qqx_degs, az, dvbe,
                                       gainp, int_step, state, gainfb1_out, gainfb2_out, gainfb3_out);
    }

    if (mautp == 4) {
        delecx = control_gamma(thtvlcomx, thtblx_deg, qqx_degs, thtvlx_deg, dvbe,
                                dla, dlde, dma, dmq, dmde, pgam, wgam, zgam);
    }

    if (mauty == 4) {
        phicomx = control_heading(psivlcomx, grav, dvbe, psivlx_deg, wrcl, zrcl, facthead);
        delrcx = control_yaw_rate(rcomx, rrx_degs, dvbe, dyb, dydr, dnb, dnr, dndr, zetlagr);
    }

    if (mautp == 5) {
        ancomx = control_altitude(altcom, grav, phiblx_deg, vd, hbe, gainalt, gainaltrate, altrate_out);
        if (ancomx > anlimpx) ancomx = anlimpx;
        if (ancomx < -anlimnx) ancomx = -anlimnx;
        if (ancomx > gmax) ancomx = gmax;
        if (ancomx < gminx) ancomx = gminx;
        delecx = control_normal_accel(ancomx, pdynmc, dla, dma, dmq, dmde, qqx_degs, az, dvbe,
                                       gainp, int_step, state, gainfb1_out, gainfb2_out, gainfb3_out);
    }

    if (mroll == 0) {
        if (fabs(phicomx) > philimx) {
            phicomx = philimx * fc_sign(phicomx);
        }
        delacx = control_roll(phicomx, phiblx_deg, ppx_degs, dllp, dllda, wrcl, zrcl);
    } else if (mroll == 1) {
        delacx = control_roll_rate(pcomx, ppx_degs, dllp, dllda, tp);
    }

    if (fabs(delacx) > dalimx) delacx = dalimx * fc_sign(delacx);
    if (fabs(delecx) > delimx) delecx = delimx * fc_sign(delecx);
    if (fabs(delrcx) > drlimx) delrcx = drlimx * fc_sign(delrcx);

    *phicomx_state = phicomx;

    ControlMixOutput_t out;
    out.delacx_deg = delacx;
    out.delecx_deg = delecx;
    out.delrcx_deg = delrcx;
    return out;
}
