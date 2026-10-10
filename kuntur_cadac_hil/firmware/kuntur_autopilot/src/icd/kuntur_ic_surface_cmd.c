/*
 * kuntur_ic_surface_cmd.c -- ver kuntur_ic_surface_cmd.h /
 * kuntur_ic_wire.h.
 *
 * Orden de cursor: exactamente el orden de "fields" en
 * icd_schema/kuntur_ic_surface_cmd_v1.json.
 */
#include "kuntur_ic_surface_cmd.h"
#include "kuntur_ic_wire.h"

size_t kuntur_pack_surface_cmd(uint8_t *out, const kuntur_surface_cmd_t *in) {
    size_t off = 0;

    kuntur_wire_put_u32(out, &off, KUNTUR_MAGIC_SURFACE_CMD);
    kuntur_wire_put_u64(out, &off, in->t_sim_us);
    kuntur_wire_put_u32(out, &off, in->seq_num);
    kuntur_wire_put_u16(out, &off, in->status);

    kuntur_wire_put_f64(out, &off, in->delax);
    kuntur_wire_put_f64(out, &off, in->delex);
    kuntur_wire_put_f64(out, &off, in->delrx);
    kuntur_wire_put_f64(out, &off, in->throttle);

    kuntur_wire_put_f64(out, &off, in->gainfb1);
    kuntur_wire_put_f64(out, &off, in->gainfb2);
    kuntur_wire_put_f64(out, &off, in->gainfb3);
    kuntur_wire_put_f64(out, &off, in->wp_flag);

    return off; /* == KUNTUR_SURFACE_CMD_SIZE */
}

int kuntur_unpack_surface_cmd(kuntur_surface_cmd_t *out, const uint8_t *in, size_t len) {
    if (len != KUNTUR_SURFACE_CMD_SIZE) {
        return -1;
    }

    size_t off = 0;
    uint32_t magic = kuntur_wire_get_u32(in, &off);
    if (magic != KUNTUR_MAGIC_SURFACE_CMD) {
        return -1;
    }

    out->t_sim_us = kuntur_wire_get_u64(in, &off);
    out->seq_num = kuntur_wire_get_u32(in, &off);
    out->status = kuntur_wire_get_u16(in, &off);

    out->delax = kuntur_wire_get_f64(in, &off);
    out->delex = kuntur_wire_get_f64(in, &off);
    out->delrx = kuntur_wire_get_f64(in, &off);
    out->throttle = kuntur_wire_get_f64(in, &off);

    out->gainfb1 = kuntur_wire_get_f64(in, &off);
    out->gainfb2 = kuntur_wire_get_f64(in, &off);
    out->gainfb3 = kuntur_wire_get_f64(in, &off);
    out->wp_flag = kuntur_wire_get_f64(in, &off);

    return 0;
}
