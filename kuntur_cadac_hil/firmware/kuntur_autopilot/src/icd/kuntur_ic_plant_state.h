/*
 * kuntur_ic_plant_state.h -- Canal KUNTUR_PLANT_STATE (Host -> STM32),
 * ver icd_schema/kuntur_ic_plant_state_v1.json (fuente de verdad del
 * layout -- este archivo se escribe a mano, NO generado, por decision
 * del plan-hil-stm32-portado-autopiloto.md parrafo 6.3(b)).
 *
 * IMPORTANTE: pack/unpack serializan CAMPO A CAMPO con memcpy a offsets
 * fijos, calculados a mano a partir del schema -- NO se usa un struct
 * "packed" copiado de una sola vez. Motivo: el header mide 18 bytes (no
 * multiplo de 8), asi que si el wire format fuera un memcpy directo de
 * un struct, los "double" subsiguientes quedarian en offsets no
 * alineados a 8 bytes -- el Cortex-M7 no soporta acceso no alineado a
 * double de forma confiable (puede hacer HardFault o degradar a
 * acceso por software segun configuracion de UNALIGN_TRP). Serializar
 * campo a campo hacia un buffer uint8_t evita el problema por completo
 * y es exactamente lo que hace kuntur_bridge/icd_plant_state.py
 * (struct.pack("<...")) del lado Python -- mismo criterio en los dos
 * lados, necesario para el test de regresion byte a byte.
 */
#ifndef KUNTUR_IC_PLANT_STATE_H
#define KUNTUR_IC_PLANT_STATE_H

#include <stddef.h>
#include <stdint.h>

#define KUNTUR_MAGIC_PLANT_STATE 0x4B505331u /* "KPS1" */
#define KUNTUR_PLANT_STATE_SIZE 352u

/* flags (bitfield de KUNTUR_PLANT_STATE.flags) */
#define KUNTUR_PLANT_STATE_FLAG_INIT   (1u << 0)
#define KUNTUR_PLANT_STATE_FLAG_ACTIVE (1u << 1)

typedef struct {
    uint64_t t_sim_us;
    uint32_t seq_num;
    uint16_t flags;

    double sn, se, sd;
    double vn, ve, vd;
    double phi, theta, psi;
    double p, q, r;

    double ax, ay, az;

    double pdynmc, vmach, dvbe, grav, alpha, beta;

    /* Derivadas relevadas contra cadac_py/falcon6.py (ver schema) --
     * dla/dlde/dma/dmq/dmde = plane[145-149] (pitch/normal accel),
     * dyb/dydr/dnb/dnr/dndr = plane[150-154] (lateral accel),
     * dllp/dllda = plane[155-156] (roll). */
    double dla, dlde, dma, dmq, dmde;
    double dyb, dydr, dnb, dnr, dndr;
    double dllp, dllda;

    double gmax, gminx;

    double swel1, swel2, swel3, psiflx, thtflx, altcom;
    uint16_t mguid;
    uint16_t maut;

    uint16_t manual_active;
} kuntur_plant_state_t;

/* Serializa `in` a `out` (debe tener al menos KUNTUR_PLANT_STATE_SIZE
 * bytes) en formato wire little-endian. Devuelve KUNTUR_PLANT_STATE_SIZE. */
size_t kuntur_pack_plant_state(uint8_t *out, const kuntur_plant_state_t *in);

/* Decodifica `in` (de `len` bytes) a `out`. Devuelve 0 si OK, -1 si
 * `len` != KUNTUR_PLANT_STATE_SIZE o el magic no matchea. */
int kuntur_unpack_plant_state(kuntur_plant_state_t *out, const uint8_t *in, size_t len);

#endif /* KUNTUR_IC_PLANT_STATE_H */
