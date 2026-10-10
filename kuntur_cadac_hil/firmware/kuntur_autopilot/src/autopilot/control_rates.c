/*
 * control_rates.c -- ver control_rates.h. Portado línea por línea de
 * cadac_py/falcon6.py (líneas 1240-1321 al momento del port).
 */
#include "control_rates.h"
#include "math_utils.h"
#include <math.h>

double control_roll(
    double phicomx, double phiblx_deg, double ppx_degs,
    double dllp, double dllda, double wrcl, double zrcl
) {
    double gkp = (2.0 * zrcl * wrcl + dllp) / dllda;
    double gkphi = wrcl * wrcl / dllda;

    double ephi = gkphi * (phicomx - phiblx_deg) * RAD;
    double dpc = ephi - gkp * ppx_degs * RAD;
    return dpc * DEG;
}

double control_roll_rate(
    double pcomx, double ppx_degs, double dllp, double dllda, double tp
) {
    double kp = (1.0 / tp + dllp) / dllda;
    return kp * (pcomx - ppx_degs);
}

double control_pitch_rate(
    double qcomx, double qqx_degs, double dvbe,
    double dla, double dlde, double dma, double dmq, double dmde,
    double zetlagr
) {
    double zrate = dla / dvbe - dma * dlde / (dvbe * dmde);
    double aa = dla / dvbe - dmq;
    double bb = -dma - dmq * dla / dvbe;

    double dum1 = aa - 2.0 * zetlagr * zetlagr * zrate;
    double dum2 = aa * aa - 4.0 * zetlagr * zetlagr * bb;
    double radix = dum1 * dum1 - dum2;
    if (radix < 0.0) {
        radix = 0.0;
    }
    if (fabs(dmde) < SMALL) {
        dmde = SMALL * fc_sign(dmde);
    }
    double grate = (-dum1 + sqrt(radix)) / (-dmde);

    return grate * (qqx_degs - qcomx);
}

double control_yaw_rate(
    double rcomx, double rrx_degs, double dvbe,
    double dyb, double dydr, double dnb, double dnr, double dndr,
    double zetlagr
) {
    double zrate = -dyb / dvbe + dnb * dydr / (dvbe * dndr);
    double aa = -dyb / dvbe - dnr;
    double bb = dnb + dyb * dnr / dvbe;

    double dum1 = aa - 2.0 * zetlagr * zetlagr * zrate;
    double dum2 = aa * aa - 4.0 * zetlagr * zetlagr * bb;
    double radix = dum1 * dum1 - dum2;
    if (radix < 0.0) {
        radix = 0.0;
    }
    if (fabs(dndr) < SMALL) {
        dndr = SMALL * fc_sign(dndr);
    }
    double grate = (-dum1 + sqrt(radix)) / (-dndr);

    /* dum3/wnlagr (plane[562]) son diagnóstico puro -- no alimentan
     * ningún otro lazo ni el ICD de superficies, se omiten del port
     * (ver nota en control_rates.h sobre qué diagnósticos se exponen). */

    return grate * (rrx_degs - rcomx);
}
