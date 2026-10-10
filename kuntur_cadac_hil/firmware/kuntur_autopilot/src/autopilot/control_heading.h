/*
 * control_heading.h -- portado de cadac_py/falcon6.py::control_heading.
 * NO ejercitado por f16c11_7.asc (mauty nunca llega a 4 en ese
 * escenario).
 */
#ifndef KUNTUR_CONTROL_HEADING_H
#define KUNTUR_CONTROL_HEADING_H

/* psivlcomx: comando de rumbo, deg. grav: gravedad local, m/s^2
 * (flat6[55]). dvbe: TAS, m/s. psivlx_deg: rumbo actual, deg.
 * wrcl/zrcl: ganancias del *.asc (mismas que control_roll).
 * facthead: ganancia del *.asc. Devuelve phicomx, deg. */
double control_heading(
    double psivlcomx, double grav, double dvbe, double psivlx_deg,
    double wrcl, double zrcl, double facthead
);

#endif /* KUNTUR_CONTROL_HEADING_H */
