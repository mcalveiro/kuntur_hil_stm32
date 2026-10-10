/*
 * falcon6_autopilot.h -- punto de entrada único del autopiloto
 * portado: recibe un KUNTUR_PLANT_STATE ya decodificado y devuelve un
 * KUNTUR_SURFACE_CMD, corriendo guidance() -> control_mix() ->
 * actuator() en ese orden (el mismo que usa el *.asc: ver
 * cadac_py/execution.py::_EXEC_DISPATCH y el título de
 * f16c11_7.asc -- "...guidance control actuator...").
 *
 * Esta es la función que llamaría main.c en el STM32 (Fase 5 del plan)
 * y la que ejercita el test de regresión Nivel 0 (Fase 3,
 * test/host/test_bitexact.c) contra las traces de FALCON6 Python.
 *
 * Hallazgo del port -- out->throttle: en f16c11_7.asc, mprop=2 (Mach
 * hold) hace que propulsion() calcule el throttle DIRECTO a partir de
 * vmachcom/gmach, sin pasar por ningún lazo de control()/guidance() --
 * no existe una función "control_throttle" en falcon6.py que portar.
 * Por eso out->throttle queda en 0.0 acá: este escenario no tiene
 * autopiloto de throttle, es enteramente físico (propulsion(), que
 * se queda en Python por diseño, ver §9 del plan). Si un *.asc futuro
 * usa mprop=1 (throttle manual/directo) con un lazo de autopiloto real
 * que lo gobierne, esta función hay que extenderla -- no es el caso
 * hoy.
 */
#ifndef KUNTUR_FALCON6_AUTOPILOT_H
#define KUNTUR_FALCON6_AUTOPILOT_H

#include "autopilot_state.h"
#include "../config/scenario_params.h"
#include "../icd/kuntur_ic_plant_state.h"
#include "../icd/kuntur_ic_surface_cmd.h"

/* in->flags & KUNTUR_PLANT_STATE_FLAG_INIT -> resetea `state` antes de
 * calcular (nuevo episodio, ver icd_schema/kuntur_ic_plant_state_v1.json).
 * int_step: paso de integración del AUTOPILOTO (100 Hz nominal en HIL,
 * ver plan §5) -- NO el paso físico de 1 kHz; en el test Nivel 0 se
 * pasa 0.001s para igualar las traces de referencia (ver docstring de
 * control_normal_accel.h). */
void falcon6_autopilot_step(
    const kuntur_plant_state_t *in,
    const Scenario_Params_t *params,
    Autopilot_State_t *state,
    double int_step,
    kuntur_surface_cmd_t *out
);

#endif /* KUNTUR_FALCON6_AUTOPILOT_H */
