/*
 * control_rates.h -- lazos de tasa: roll (posición y tasa), pitch rate,
 * yaw rate. Portado línea por línea de cadac_py/falcon6.py::control_roll/
 * control_roll_rate/control_pitch_rate/control_yaw_rate.
 *
 * Ángulos/tasas en GRADOS y GRADOS/s (unidades nativas de plane[]/
 * flat6[] en el Python original) -- la conversión desde los radianes
 * del wire KUNTUR_PLANT_STATE es responsabilidad del llamador (ver
 * falcon6_autopilot.c), no de estas funciones.
 */
#ifndef KUNTUR_CONTROL_RATES_H
#define KUNTUR_CONTROL_RATES_H

/* phicomx: comando de roll, deg. phiblx_deg: actitud de roll actual,
 * deg. ppx_degs: tasa de roll actual, deg/s. dllp/dllda: derivadas
 * (ver AutopilotInputs/plane[155/156]). wrcl/zrcl: ganancias del
 * *.asc (ver scenario_params.h). Devuelve delacx, deg. */
double control_roll(
    double phicomx, double phiblx_deg, double ppx_degs,
    double dllp, double dllda, double wrcl, double zrcl
);

/* pcomx: comando de tasa de roll, deg/s. ppx_degs: tasa actual, deg/s.
 * tp: constante de tiempo deseada del *.asc. Devuelve delacx, deg.
 * NO ejercitado por f16c11_7.asc (mroll nunca es 1) -- portado igual,
 * la función existe en falcon6.py. */
double control_roll_rate(
    double pcomx, double ppx_degs, double dllp, double dllda, double tp
);

/* qcomx: comando de tasa de pitch, deg/s. qqx_degs: tasa actual, deg/s.
 * dvbe: TAS, m/s. dla/dlde/dma/dmq/dmde: derivadas longitudinales.
 * zetlagr: amortiguamiento deseado del *.asc. Devuelve delecx, deg. */
double control_pitch_rate(
    double qcomx, double qqx_degs, double dvbe,
    double dla, double dlde, double dma, double dmq, double dmde,
    double zetlagr
);

/* rcomx: comando de tasa de yaw, deg/s. rrx_degs: tasa actual, deg/s.
 * dvbe: TAS, m/s. dyb/dydr/dnb/dnr/dndr: derivadas laterales.
 * zetlagr: idem control_pitch_rate. Devuelve delrcx, deg. */
double control_yaw_rate(
    double rcomx, double rrx_degs, double dvbe,
    double dyb, double dydr, double dnb, double dnr, double dndr,
    double zetlagr
);

#endif /* KUNTUR_CONTROL_RATES_H */
