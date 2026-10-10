/*
 * autopilot_state.h -- Estado persistente del autopiloto FALCON6, vive
 * en el STM32 entre llamadas (ver plan-hil-stm32-portado-autopiloto.md
 * §4). Mapeado 1:1 contra los slots plane[] originales, con nombres
 * significativos -- preserva trazabilidad contra cadac_py/falcon6.py y
 * facilita la validación bit a bit.
 *
 * IMPORTANTE -- relevado al portar los lazos (no asumido del plan):
 * de TODO el estado que def_control() declara como categoría "state"
 * (plane[514/515/516/517]), solo zz/zzd (plane[516/517], el
 * integrador de control_normal_accel) se usa de verdad en algún lazo
 * portado. yy/yyd (plane[514/515], "Yaw feed-foreward...") están
 * declarados en falcon6.py pero NINGUNA función los lee ni los
 * escribe -- se incluyen acá igual, inertes, solo por trazabilidad
 * 1:1 con el original (si algún día aparece un lazo que sí los usa,
 * el slot ya existe con el nombre correcto).
 */
#ifndef KUNTUR_AUTOPILOT_STATE_H
#define KUNTUR_AUTOPILOT_STATE_H

typedef struct {
    /* control_normal_accel() -- plane[516]/[517] */
    double zzd;
    double zz;

    /* guidance() -- plane[522]/[523]. Técnicamente persistentes: si
     * mguid==0 la función Python retorna temprano SIN llamar a
     * p[522].gets()/p[523].gets() -- el valor del ciclo anterior queda
     * tal cual. f16c11_7.asc nunca tiene mguid==0 (siempre 30 o 33),
     * así que esto es inerte en la práctica para ESE escenario, pero
     * se porta fielmente por si alguno nuevo sí pasa por mguid==0. */
    double alcomx;
    double ancomx;

    /* control() -- plane[553]. Mismo criterio: si maut==0 la función
     * retorna temprano sin p[553].gets() -- y aun si maut!=0, el valor
     * de ENTRADA de phicomx (antes de los ifs) es el de este slot
     * persistente, no un parámetro fresco del wire. */
    double phicomx;

    /* Declarados en falcon6.py (plane[514]/[515]) pero sin ningún lector
     * ni escritor en los lazos portados -- ver docstring de arriba. */
    double yyd_unused;
    double yy_unused;

    /* actuator_scnd() -- plane[630..633], solo si mact==2 (ver
     * scenario_params.h) */
    double DXD[3];
    double DX[3];
    double DDXD[3];
    double DDX[3];
} Autopilot_State_t;

/* Condición inicial: todo en cero, igual que los defaults de
 * Variable.init() en falcon6.py antes de que cualquier .asc los
 * sobreescriba (el *.asc de este fork no inicializa ninguno de estos
 * slots -- son puramente dinámicos). Llamar al recibir KUNTUR_PLANT_STATE
 * con flags.init=1 (nuevo episodio) -- ver icd_schema/kuntur_ic_plant_state_v1.json. */
void autopilot_state_reset(Autopilot_State_t *s);

#endif /* KUNTUR_AUTOPILOT_STATE_H */
