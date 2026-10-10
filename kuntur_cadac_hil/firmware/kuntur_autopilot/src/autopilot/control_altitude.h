/*
 * control_altitude.h -- portado de
 * cadac_py/falcon6.py::control_altitude.
 */
#ifndef KUNTUR_CONTROL_ALTITUDE_H
#define KUNTUR_CONTROL_ALTITUDE_H

/* altcom: altitud comandada, m. grav: gravedad local, m/s^2.
 * phiblx_deg: actitud de roll, deg. vd: velocidad NED hacia abajo, m/s
 * (VBEL[2]). hbe: altura AGL, m (flat6[239]). gainalt/gainaltrate:
 * ganancias del *.asc. altrate_out: variómetro resultante, m/s (mismo
 * valor que AuxTelemetry.altrate -- se expone acá en vez de
 * recalcularlo dos veces). Devuelve ancomx, g's. */
double control_altitude(
    double altcom, double grav, double phiblx_deg, double vd, double hbe,
    double gainalt, double gainaltrate,
    double *altrate_out
);

#endif /* KUNTUR_CONTROL_ALTITUDE_H */
