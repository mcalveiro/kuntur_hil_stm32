#include "control_heading.h"

double control_heading(
    double psivlcomx, double grav, double dvbe, double psivlx_deg,
    double wrcl, double zrcl, double facthead
) {
    double gainpsi = (dvbe / grav) * zrcl * wrcl * (1.0 - zrcl * zrcl) * (1.0 + facthead);
    return gainpsi * (psivlcomx - psivlx_deg);
}
