#include "control_gamma.h"
#include "math_utils.h"

double control_gamma(
    double thtvlcomx, double thtblx_deg, double qqx_degs,
    double thtvlx_deg, double dvbe,
    double dla, double dlde, double dma, double dmq, double dmde,
    double pgam, double wgam, double zgam
) {
    mat33_t AA = mat33_make(
        dmq, dma, -dma,
        1.0, 0.0, 0.0,
        0.0, dla / dvbe, -dla / dvbe
    );
    vec3_t BB = vec3_make(dmde, 0.0, dlde / dvbe);

    double am = 2.0 * zgam * wgam + pgam;
    double bm = wgam * wgam + 2.0 * zgam * wgam * pgam;
    double cm = wgam * wgam * pgam;
    double v11 = dmde;
    double v12 = 0.0;
    double v13 = dlde / dvbe;
    double v21 = dmde * dla / dvbe - dlde * dma / dvbe;
    double v22 = dmde;
    double v23 = -dmq * dlde / dvbe;
    double v31 = 0.0;
    double v32 = v21;
    double v33 = v21;
    mat33_t DP = mat33_make(v11, v12, v13, v21, v22, v23, v31, v32, v33);
    vec3_t DD = vec3_make(am + dmq - dla / dvbe, bm + dma + dmq * dla / dvbe, cm);
    mat33_t DPI = mat33_inverse(DP);
    vec3_t GAINGAM = mat33_mulvec3(DPI, DD);

    mat33_t DUM33 = mat33_sub(AA, mat33_mulouter(BB, GAINGAM));
    mat33_t IDUM33 = mat33_inverse(DUM33);
    vec3_t DUM3 = mat33_mulvec3(IDUM33, BB);
    vec3_t HH = vec3_make(0.0, 0.0, 1.0);
    double denom = vec3_dot(HH, DUM3);
    double gainff = -1.0 / denom;

    double thtc = gainff * thtvlcomx * RAD;
    double qqf = GAINGAM.v[0] * qqx_degs * RAD;
    double thtblf = GAINGAM.v[1] * thtblx_deg * RAD;
    double thtvlf = GAINGAM.v[2] * thtvlx_deg * RAD;
    double delec = thtc - (qqf + thtblf + thtvlf);
    return delec * DEG;
}
