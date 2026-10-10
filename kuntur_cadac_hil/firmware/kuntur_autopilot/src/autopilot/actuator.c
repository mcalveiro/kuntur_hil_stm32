/*
 * actuator.c -- ver actuator.h. Portado línea por línea de
 * cadac_py/falcon6.py::actuator/actuator_scnd (líneas 1026-1089 al
 * momento del port).
 */
#include "actuator.h"
#include <math.h>

vec3_t actuator_scnd(
    vec3_t ACTCX, double int_step,
    double dlimx, double ddlimx, double wnact, double zetact,
    Autopilot_State_t *state
) {
    double *DXD = state->DXD, *DX = state->DX, *DDXD = state->DDXD, *DDX = state->DDX;

    for (int i = 0; i < 3; i++) {
        if (fabs(DX[i]) > dlimx) {
            DX[i] = dlimx * fc_sign(DX[i]);
            if (DX[i] * DDX[i] > 0) {
                DDX[i] = 0;
            }
        }
        int iflag = 0;
        if (fabs(DDX[i]) > ddlimx) {
            iflag = 1;
            DDX[i] = ddlimx * fc_sign(DDX[i]);
        }
        double DXD_new_i = DDX[i];
        DX[i] = fc_integrate(DXD_new_i, DXD[i], DX[i], int_step);
        DXD[i] = DXD_new_i;
        double edx = ACTCX.v[i] - DX[i];
        double DDXD_new_i = wnact * wnact * edx - 2.0 * zetact * wnact * DXD[i];
        DDX[i] = fc_integrate(DDXD_new_i, DDXD[i], DDX[i], int_step);
        DDXD[i] = DDXD_new_i;
        if (iflag && DDX[i] * DDXD[i] > 0) {
            DDXD[i] = 0;
        }
    }

    return vec3_make(DX[0], DX[1], DX[2]);
}

vec3_t actuator(
    vec3_t ACTCX, double int_step,
    int mact, double dlimx, double ddlimx, double wnact, double zetact,
    Autopilot_State_t *state
) {
    vec3_t ACTX;
    if (mact == 0) {
        ACTX = ACTCX;
        for (int i = 0; i < 3; i++) {
            if (fabs(ACTX.v[i]) > dlimx) {
                ACTX.v[i] = dlimx * fc_sign(ACTX.v[i]);
            }
        }
    } else if (mact == 2) {
        ACTX = actuator_scnd(ACTCX, int_step, dlimx, ddlimx, wnact, zetact, state);
    } else {
        ACTX = ACTCX;
    }
    return ACTX;
}
