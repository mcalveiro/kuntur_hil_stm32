#include "control_lateral_accel.h"
#include "math_utils.h"
#include <math.h>

double control_lateral_accel(double alcomx, double az, double gainl) {
    double fspb3 = az; /* FSPB[2] */
    return -DEG * gainl * atan(alcomx) * fc_sign(fspb3);
}
