/*
 * scenario_params.h -- parámetros del *.asc hardcodeados (ver
 * plan-hil-stm32-portado-autopiloto.md §7), relevados de
 * cadac_py/CADAC/FALCON6/f16c11_7.asc línea por línea.
 *
 * Lo que NO está acá (viaja por KUNTUR_PLANT_STATE en cada ciclo, no es
 * estático): maut, mguid, swel1/swel2/swel3, psiflx, thtflx, altcom --
 * todos cambian a mitad del escenario vía bloques IF wp_flag=-1 ENDIF
 * (avance de waypoint). Ver icd_schema/kuntur_ic_plant_state_v1.json.
 *
 * mroll, tp, facthead, pgam/wgam/zgam: el *.asc NO los fija (quedan en
 * el default 0.0/0 de Variable.init() en falcon6.py) -- control_roll_rate
 * (mroll==1), control_heading (mauty==4) y control_gamma (mautp==4)
 * NUNCA se ejercitan con maut=35 o maut=33 (los dos únicos valores que
 * usa este escenario). Se incluyen en 0 igual, por completitud
 * estructural del port -- si se arma un escenario nuevo que sí los
 * use, estos valores hay que revisarlos.
 */
#ifndef KUNTUR_SCENARIO_PARAMS_H
#define KUNTUR_SCENARIO_PARAMS_H

typedef struct {
    /* actuator() / actuator_scnd() -- *.asc líneas 63-67 */
    int mact;        /* =2: dinámica de 2do orden */
    double dlimx;    /* 20 deg -- límite único para los 3 ejes */
    double ddlimx;   /* 400 deg/s */
    double wnact;    /* 50 rad/s */
    double zetact;   /* 0.7 */

    /* control() -- límites de superficie y estructurales, líneas 69-74 */
    double dalimx;   /* 20 deg */
    double delimx;   /* 20 deg */
    double drlimx;   /* 20 deg */
    double anlimpx;  /* 9 g's */
    double anlimnx;  /* 6 g's */

    /* control_roll() -- líneas 76-78 */
    double philimx;  /* 70 deg */
    double wrcl;     /* 15 rad/s */
    double zrcl;     /* 0.7 */

    /* control_roll_rate() -- NO fijado por este *.asc (mroll nunca es 1) */
    int mroll;       /* 0: posición de roll (control_roll), no tasa */
    double tp;        /* 0.0 -- sin uso en este escenario */

    /* control_pitch_rate() / control_yaw_rate() -- línea 80 */
    double zetlagr;  /* 0.7 */

    /* control_normal_accel() -- línea 82 (gainp; waclp/zaclp/paclp de
     * líneas 83-85 NO se usan -- control_normal_accel() los recalcula
     * cada llamada a partir de pdynmc, ver control_normal_accel.c) */
    double gainp;    /* 0.0 */

    /* control_lateral_accel() -- línea 87 */
    double gainl;    /* 1.0 rad/g's */

    /* control_heading() -- NO fijado (mauty nunca es 4 en este escenario) */
    double facthead; /* 0.0 */

    /* control_gamma() -- NO fijado (mautp nunca es 4 en este escenario) */
    double pgam;
    double wgam;
    double zgam;

    /* guidance_line() -- líneas 90-92 (line_gain cambia a 2.0 vía evento
     * al entrar en la pierna final hacia el IP -- ESO sí viaja por
     * KUNTUR_PLANT_STATE si se decide exponerlo; por ahora, dado que
     * line_gain no está en el schema v1, se documenta acá el valor
     * INICIAL -- ver nota en el README del firmware) */
    double line_gain;      /* 1.0 1/s (-> 2.0 tras el evento de la IP) */
    double nl_gain_fact;   /* 0.4 */
    double decrement;      /* 500 m */

    /* control_altitude() -- líneas 95-96 */
    double gainalt;      /* 0.3 1/s */
    double gainaltrate;  /* 0.7 1/s */
} Scenario_Params_t;

/* f16c11_7.asc -- "3 Waypoints and terminal glide slope to IP". Ver
 * docstring de arriba para lo que NO está acá. */
#define SCENARIO_PARAMS_F16C11_7_INIT { \
    .mact = 2, .dlimx = 20.0, .ddlimx = 400.0, .wnact = 50.0, .zetact = 0.7, \
    .dalimx = 20.0, .delimx = 20.0, .drlimx = 20.0, .anlimpx = 9.0, .anlimnx = 6.0, \
    .philimx = 70.0, .wrcl = 15.0, .zrcl = 0.7, \
    .mroll = 0, .tp = 0.0, \
    .zetlagr = 0.7, \
    .gainp = 0.0, \
    .gainl = 1.0, \
    .facthead = 0.0, \
    .pgam = 0.0, .wgam = 0.0, .zgam = 0.0, \
    .line_gain = 1.0, .nl_gain_fact = 0.4, .decrement = 500.0, \
    .gainalt = 0.3, .gainaltrate = 0.7, \
}

#endif /* KUNTUR_SCENARIO_PARAMS_H */
