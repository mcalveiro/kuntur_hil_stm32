/*
 * falcon6_autopilot.c -- ver falcon6_autopilot.h.
 */
#include "falcon6_autopilot.h"
#include "guidance_line.h"
#include "control_mix.h"
#include "actuator.h"
#include "math_utils.h"
#include <math.h>

void falcon6_autopilot_step(
    const kuntur_plant_state_t *in,
    const Scenario_Params_t *params,
    Autopilot_State_t *state,
    double int_step,
    kuntur_surface_cmd_t *out
) {
    if (in->flags & KUNTUR_PLANT_STATE_FLAG_INIT) {
        autopilot_state_reset(state);
    }

    /* El autopiloto original (plane[]/flat6[]) trabaja en GRADOS para
     * actitud/tasas -- el wire viaja en radianes (SI). Conversión en
     * la frontera, una sola vez acá, igual que especifica el bypass
     * de Manual en plane_adapter.py. */
    double phiblx_deg = in->phi * DEG;
    double thtblx_deg = in->theta * DEG;
    double ppx_degs = in->p * DEG;
    double qqx_degs = in->q * DEG;
    double rrx_degs = in->r * DEG;

    vec3_t SBEL = vec3_make(in->sn, in->se, in->sd);
    vec3_t VBEL = vec3_make(in->vn, in->ve, in->vd);
    double hbe = -in->sd; /* cadac_py/falcon6.py::init_newton(): hbe = -SBEL[2] (tierra plana, sin modelo de terreno) */

    /* psivlx/thtvlx: ángulo de trayectoria de vuelo (velocidad), NO la
     * actitud -- derivado de VBEL exactamente igual que
     * init_newton()/newton() (ver guidance_line.h). */
    double psivlx_deg;
    if (VBEL.v[0] == 0.0 && VBEL.v[1] == 0.0) {
        psivlx_deg = 0.0;
    } else {
        psivlx_deg = atan2(VBEL.v[1], VBEL.v[0]) * DEG;
    }
    double thtvlx_deg = atan2(-VBEL.v[2], sqrt(VBEL.v[0] * VBEL.v[0] + VBEL.v[1] * VBEL.v[1])) * DEG;

    double wp_flag = 0.0;
    guidance(
        in->mguid, in->swel1, in->swel2, in->swel3,
        in->psiflx, in->thtflx,
        SBEL, VBEL, thtvlx_deg, in->dvbe, in->grav, params->philimx,
        params->line_gain, params->nl_gain_fact, params->decrement,
        &state->alcomx, &state->ancomx,
        &wp_flag
    );

    double gainfb1 = 0.0, gainfb2 = 0.0, gainfb3 = 0.0, altrate = 0.0;
    /* psivlcomx/pcomx/qcomx/rcomx/thtvlcomx: ver control_mix.h -- 0.0,
     * no ejercitados por f16c11_7.asc (ningún campo del wire v1 los
     * transporta todavía). */
    ControlMixOutput_t cm = control_mix(
        in->maut, params->mroll,
        state->alcomx, state->ancomx, in->altcom,
        0.0, &state->phicomx, 0.0, 0.0, 0.0,
        0.0,
        params->anlimpx, params->anlimnx, in->gmax, in->gminx,
        params->dalimx, params->delimx, params->drlimx, params->philimx,
        phiblx_deg, thtblx_deg, psivlx_deg, thtvlx_deg,
        ppx_degs, qqx_degs, rrx_degs,
        in->pdynmc, in->az, in->dvbe, in->grav, in->vd, hbe,
        in->dla, in->dlde, in->dma, in->dmq, in->dmde,
        in->dyb, in->dydr, in->dnb, in->dnr, in->dndr,
        in->dllp, in->dllda,
        params->wrcl, params->zrcl, params->tp, params->zetlagr, params->gainp,
        params->gainl, params->facthead, params->pgam, params->wgam, params->zgam,
        params->gainalt, params->gainaltrate,
        int_step,
        state,
        &gainfb1, &gainfb2, &gainfb3,
        &altrate
    );

    vec3_t ACTCX = vec3_make(cm.delacx_deg, cm.delecx_deg, cm.delrcx_deg);
    vec3_t ACTX = actuator(
        ACTCX, int_step,
        params->mact, params->dlimx, params->ddlimx, params->wnact, params->zetact,
        state
    );

    out->t_sim_us = in->t_sim_us;
    out->seq_num = in->seq_num;
    out->status = KUNTUR_SURFACE_CMD_STATUS_OK;
    out->delax = ACTX.v[0] * RAD;
    out->delex = ACTX.v[1] * RAD;
    out->delrx = ACTX.v[2] * RAD;
    out->throttle = 0.0; /* ver falcon6_autopilot.h -- f16c11_7.asc no tiene autopiloto de throttle (mprop=2 es puramente de propulsion(), que se queda en Python) */
    out->gainfb1 = gainfb1;
    out->gainfb2 = gainfb2;
    out->gainfb3 = gainfb3;
    out->wp_flag = wp_flag;
}
