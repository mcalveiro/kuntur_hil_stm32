/*
 * control_mix.h -- dispatcher de modos + mezcla a límites de
 * superficie, portado de cadac_py/falcon6.py::control (líneas
 * 1162-1238 al momento del port). Es la función "control_mix" que
 * menciona plan-hil-stm32-portado-autopiloto.md §7 (el original no
 * tiene una función separada con ese nombre -- la mezcla y el
 * despacho de modos están dentro de control(), ver docstring de
 * módulo del plan, parrafo "control_mix.c").
 *
 * Hallazgo del port: `pcomx`/`qcomx`/`rcomx` (comandos de tasa directos,
 * plane[554-556]) y `psivlcomx` (comando de rumbo, plane[550]) son
 * leídos por control() pero NUNCA se setean en f16c11_7.asc (ni en
 * ninguna otra función portada) -- solo importan en combinaciones de
 * maut (mauty==2, mautp==2, mauty==4) que este escenario no usa
 * (maut es siempre 35 o 33). KUNTUR_PLANT_STATE v1 no los transporta
 * por eso mismo -- se exponen acá como parámetros, en 0.0 para este
 * escenario, por si un *.asc futuro sí los necesita (en cuyo caso hay
 * que agregarlos al schema primero).
 */
#ifndef KUNTUR_CONTROL_MIX_H
#define KUNTUR_CONTROL_MIX_H

#include "autopilot_state.h"

typedef struct {
    double delacx_deg;
    double delecx_deg;
    double delrcx_deg;
} ControlMixOutput_t;

/* maut: modo de autopiloto (wire). mroll: del *.asc (scenario_params).
 * alcomx/ancomx: salida de guidance() (mismo ciclo). altcom: wire.
 * psivlcomx/pcomx/qcomx/rcomx/thtvlcomx: ver nota de arriba (0.0 en
 * este escenario). phicomx_state: state->phicomx (persistente, ver
 * autopilot_state.h) -- entra con el valor del ciclo anterior, sale
 * con el nuevo (control() SIEMPRE lo reescribe si maut!=0, sin
 * importar qué rama de mauty/mautp se haya tomado). anlimpx/anlimnx/gmax/gminx: límites
 * estructurales (scenario_params + wire). dalimx/delimx/drlimx/philimx:
 * límites del *.asc. int_step: paso de integración del autopiloto.
 * Resto de parámetros: ver control_rates.h/control_normal_accel.h/
 * control_lateral_accel.h/control_gamma.h/control_heading.h.
 * state: para control_normal_accel (zz/zzd). gainfb*_out: debug,
 * NULL-safe (viajan por KUNTUR_SURFACE_CMD). */
ControlMixOutput_t control_mix(
    int maut, int mroll,
    double alcomx, double ancomx, double altcom,
    double psivlcomx, double *phicomx_state, double pcomx, double qcomx, double rcomx,
    double thtvlcomx,
    double anlimpx, double anlimnx, double gmax, double gminx,
    double dalimx, double delimx, double drlimx, double philimx,
    /* flight state (wire, ya convertido a grados por el llamador) */
    double phiblx_deg, double thtblx_deg, double psivlx_deg, double thtvlx_deg,
    double ppx_degs, double qqx_degs, double rrx_degs,
    double pdynmc, double az, double dvbe, double grav, double vd, double hbe,
    double dla, double dlde, double dma, double dmq, double dmde,
    double dyb, double dydr, double dnb, double dnr, double dndr,
    double dllp, double dllda,
    /* ganancias del *.asc */
    double wrcl, double zrcl, double tp, double zetlagr, double gainp,
    double gainl, double facthead, double pgam, double wgam, double zgam,
    double gainalt, double gainaltrate,
    double int_step,
    Autopilot_State_t *state,
    double *gainfb1_out, double *gainfb2_out, double *gainfb3_out,
    double *altrate_out
);

#endif /* KUNTUR_CONTROL_MIX_H */
