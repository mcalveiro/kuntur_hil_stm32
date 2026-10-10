#include "control_altitude.h"
#include "math_utils.h"
#include <math.h>

double control_altitude(
    double altcom, double grav, double phiblx_deg, double vd, double hbe,
    double gainalt, double gainaltrate,
    double *altrate_out
) {
    double altrate = -vd; /* -VBEL[2] */

    double eh = gainalt * (altcom - hbe);
    if (phiblx_deg == 0.0) {
        phiblx_deg = SMALL;
    }
    double ancomx = (1.0 / cos(phiblx_deg * RAD)) * (gainaltrate * (eh - altrate) + grav) / AGRAV;

    if (altrate_out) *altrate_out = altrate;
    return ancomx;
}
