/*
 * kuntur_ic_plant_state.c -- ver kuntur_ic_plant_state.h para el porque
 * de serializar campo a campo (sin struct "packed"), y kuntur_ic_wire.h
 * para los helpers de bajo nivel.
 *
 * Orden de cursor: exactamente el orden de "fields" en
 * icd_schema/kuntur_ic_plant_state_v1.json. Si el schema cambia, este
 * archivo se actualiza a mano y el test de regresion
 * (test/host/test_icd_packing.c + su contraparte Python) detecta
 * cualquier divergencia de inmediato.
 */
#include "kuntur_ic_plant_state.h"
#include "kuntur_ic_wire.h"

size_t kuntur_pack_plant_state(uint8_t *out, const kuntur_plant_state_t *in) {
    size_t off = 0;

    kuntur_wire_put_u32(out, &off, KUNTUR_MAGIC_PLANT_STATE);
    kuntur_wire_put_u64(out, &off, in->t_sim_us);
    kuntur_wire_put_u32(out, &off, in->seq_num);
    kuntur_wire_put_u16(out, &off, in->flags);

    kuntur_wire_put_f64(out, &off, in->sn);
    kuntur_wire_put_f64(out, &off, in->se);
    kuntur_wire_put_f64(out, &off, in->sd);
    kuntur_wire_put_f64(out, &off, in->vn);
    kuntur_wire_put_f64(out, &off, in->ve);
    kuntur_wire_put_f64(out, &off, in->vd);
    kuntur_wire_put_f64(out, &off, in->phi);
    kuntur_wire_put_f64(out, &off, in->theta);
    kuntur_wire_put_f64(out, &off, in->psi);
    kuntur_wire_put_f64(out, &off, in->p);
    kuntur_wire_put_f64(out, &off, in->q);
    kuntur_wire_put_f64(out, &off, in->r);

    kuntur_wire_put_f64(out, &off, in->ax);
    kuntur_wire_put_f64(out, &off, in->ay);
    kuntur_wire_put_f64(out, &off, in->az);

    kuntur_wire_put_f64(out, &off, in->pdynmc);
    kuntur_wire_put_f64(out, &off, in->vmach);
    kuntur_wire_put_f64(out, &off, in->dvbe);
    kuntur_wire_put_f64(out, &off, in->grav);
    kuntur_wire_put_f64(out, &off, in->alpha);
    kuntur_wire_put_f64(out, &off, in->beta);

    kuntur_wire_put_f64(out, &off, in->dla);
    kuntur_wire_put_f64(out, &off, in->dlde);
    kuntur_wire_put_f64(out, &off, in->dma);
    kuntur_wire_put_f64(out, &off, in->dmq);
    kuntur_wire_put_f64(out, &off, in->dmde);

    kuntur_wire_put_f64(out, &off, in->dyb);
    kuntur_wire_put_f64(out, &off, in->dydr);
    kuntur_wire_put_f64(out, &off, in->dnb);
    kuntur_wire_put_f64(out, &off, in->dnr);
    kuntur_wire_put_f64(out, &off, in->dndr);

    kuntur_wire_put_f64(out, &off, in->dllp);
    kuntur_wire_put_f64(out, &off, in->dllda);

    kuntur_wire_put_f64(out, &off, in->gmax);
    kuntur_wire_put_f64(out, &off, in->gminx);

    kuntur_wire_put_f64(out, &off, in->swel1);
    kuntur_wire_put_f64(out, &off, in->swel2);
    kuntur_wire_put_f64(out, &off, in->swel3);
    kuntur_wire_put_f64(out, &off, in->psiflx);
    kuntur_wire_put_f64(out, &off, in->thtflx);
    kuntur_wire_put_f64(out, &off, in->altcom);
    kuntur_wire_put_u16(out, &off, in->mguid);
    kuntur_wire_put_u16(out, &off, in->maut);

    kuntur_wire_put_u16(out, &off, in->manual_active);

    return off; /* == KUNTUR_PLANT_STATE_SIZE */
}

int kuntur_unpack_plant_state(kuntur_plant_state_t *out, const uint8_t *in, size_t len) {
    if (len != KUNTUR_PLANT_STATE_SIZE) {
        return -1;
    }

    size_t off = 0;
    uint32_t magic = kuntur_wire_get_u32(in, &off);
    if (magic != KUNTUR_MAGIC_PLANT_STATE) {
        return -1;
    }

    out->t_sim_us = kuntur_wire_get_u64(in, &off);
    out->seq_num = kuntur_wire_get_u32(in, &off);
    out->flags = kuntur_wire_get_u16(in, &off);

    out->sn = kuntur_wire_get_f64(in, &off);
    out->se = kuntur_wire_get_f64(in, &off);
    out->sd = kuntur_wire_get_f64(in, &off);
    out->vn = kuntur_wire_get_f64(in, &off);
    out->ve = kuntur_wire_get_f64(in, &off);
    out->vd = kuntur_wire_get_f64(in, &off);
    out->phi = kuntur_wire_get_f64(in, &off);
    out->theta = kuntur_wire_get_f64(in, &off);
    out->psi = kuntur_wire_get_f64(in, &off);
    out->p = kuntur_wire_get_f64(in, &off);
    out->q = kuntur_wire_get_f64(in, &off);
    out->r = kuntur_wire_get_f64(in, &off);

    out->ax = kuntur_wire_get_f64(in, &off);
    out->ay = kuntur_wire_get_f64(in, &off);
    out->az = kuntur_wire_get_f64(in, &off);

    out->pdynmc = kuntur_wire_get_f64(in, &off);
    out->vmach = kuntur_wire_get_f64(in, &off);
    out->dvbe = kuntur_wire_get_f64(in, &off);
    out->grav = kuntur_wire_get_f64(in, &off);
    out->alpha = kuntur_wire_get_f64(in, &off);
    out->beta = kuntur_wire_get_f64(in, &off);

    out->dla = kuntur_wire_get_f64(in, &off);
    out->dlde = kuntur_wire_get_f64(in, &off);
    out->dma = kuntur_wire_get_f64(in, &off);
    out->dmq = kuntur_wire_get_f64(in, &off);
    out->dmde = kuntur_wire_get_f64(in, &off);

    out->dyb = kuntur_wire_get_f64(in, &off);
    out->dydr = kuntur_wire_get_f64(in, &off);
    out->dnb = kuntur_wire_get_f64(in, &off);
    out->dnr = kuntur_wire_get_f64(in, &off);
    out->dndr = kuntur_wire_get_f64(in, &off);

    out->dllp = kuntur_wire_get_f64(in, &off);
    out->dllda = kuntur_wire_get_f64(in, &off);

    out->gmax = kuntur_wire_get_f64(in, &off);
    out->gminx = kuntur_wire_get_f64(in, &off);

    out->swel1 = kuntur_wire_get_f64(in, &off);
    out->swel2 = kuntur_wire_get_f64(in, &off);
    out->swel3 = kuntur_wire_get_f64(in, &off);
    out->psiflx = kuntur_wire_get_f64(in, &off);
    out->thtflx = kuntur_wire_get_f64(in, &off);
    out->altcom = kuntur_wire_get_f64(in, &off);
    out->mguid = kuntur_wire_get_u16(in, &off);
    out->maut = kuntur_wire_get_u16(in, &off);

    out->manual_active = kuntur_wire_get_u16(in, &off);

    return 0;
}
