/*
 * control_gamma.h -- portado de cadac_py/falcon6.py::control_gamma.
 * NO ejercitado por f16c11_7.asc (mautp nunca llega a 4 en ese
 * escenario) -- sin traces de referencia todavía para el gate Nivel 0
 * de ESTA función en particular (ver math_utils.h sobre mat33_inverse).
 */
#ifndef KUNTUR_CONTROL_GAMMA_H
#define KUNTUR_CONTROL_GAMMA_H

/* thtvlcomx: comando de ángulo de trayectoria, deg. thtblx_deg: actitud
 * de pitch, deg. qqx_degs: tasa de pitch, deg/s. thtvlx_deg: ángulo de
 * trayectoria actual, deg. dvbe: TAS, m/s. dla/dlde/dma/dmq/dmde:
 * derivadas longitudinales. pgam/wgam/zgam: polos del *.asc. Devuelve
 * delecx, deg.
 *
 * Hallazgo del port: el original lee `pdynmc = f[57].real()` pero
 * nunca lo usa en el cálculo (lectura muerta) -- no se porta ese
 * parámetro acá. */
double control_gamma(
    double thtvlcomx, double thtblx_deg, double qqx_degs,
    double thtvlx_deg, double dvbe,
    double dla, double dlde, double dma, double dmq, double dmde,
    double pgam, double wgam, double zgam
);

#endif /* KUNTUR_CONTROL_GAMMA_H */
