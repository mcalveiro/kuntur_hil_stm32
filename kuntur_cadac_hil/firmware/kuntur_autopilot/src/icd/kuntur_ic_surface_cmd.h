/*
 * kuntur_ic_surface_cmd.h -- Canal KUNTUR_SURFACE_CMD (STM32 -> Host),
 * ver icd_schema/kuntur_ic_surface_cmd_v1.json y el docstring de
 * kuntur_ic_plant_state.h (mismo criterio: serializacion campo a campo,
 * sin struct "packed", por alineacion de double en Cortex-M7).
 */
#ifndef KUNTUR_IC_SURFACE_CMD_H
#define KUNTUR_IC_SURFACE_CMD_H

#include <stddef.h>
#include <stdint.h>

#define KUNTUR_MAGIC_SURFACE_CMD 0x4B534331u /* "KSC1" */
#define KUNTUR_SURFACE_CMD_SIZE 82u

/* status (KUNTUR_SURFACE_CMD.status) */
#define KUNTUR_SURFACE_CMD_STATUS_OK       1u
#define KUNTUR_SURFACE_CMD_STATUS_DEGRADED 2u

typedef struct {
    uint64_t t_sim_us; /* eco del KUNTUR_PLANT_STATE que origino esta respuesta */
    uint32_t seq_num;  /* eco idem */
    uint16_t status;

    double delax;   /* aileron -- deflexion real ya aplicada (post actuator()), rad */
    double delex;   /* elevator -- idem, rad */
    double delrx;   /* rudder -- idem, rad */
    double throttle; /* [0,1] */

    double gainfb1; /* debug -- ganancia pole placement, opcional */
    double gainfb2;
    double gainfb3;
    double wp_flag; /* debug -- estado de avance de waypoint, opcional */
} kuntur_surface_cmd_t;

size_t kuntur_pack_surface_cmd(uint8_t *out, const kuntur_surface_cmd_t *in);
int kuntur_unpack_surface_cmd(kuntur_surface_cmd_t *out, const uint8_t *in, size_t len);

#endif /* KUNTUR_IC_SURFACE_CMD_H */
