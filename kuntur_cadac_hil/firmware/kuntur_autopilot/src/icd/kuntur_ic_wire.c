#include "kuntur_ic_wire.h"
#include <string.h>

void kuntur_wire_put_u16(uint8_t *buf, size_t *off, uint16_t v) {
    buf[*off + 0] = (uint8_t)(v & 0xFFu);
    buf[*off + 1] = (uint8_t)((v >> 8) & 0xFFu);
    *off += 2;
}

void kuntur_wire_put_u32(uint8_t *buf, size_t *off, uint32_t v) {
    buf[*off + 0] = (uint8_t)(v & 0xFFu);
    buf[*off + 1] = (uint8_t)((v >> 8) & 0xFFu);
    buf[*off + 2] = (uint8_t)((v >> 16) & 0xFFu);
    buf[*off + 3] = (uint8_t)((v >> 24) & 0xFFu);
    *off += 4;
}

void kuntur_wire_put_u64(uint8_t *buf, size_t *off, uint64_t v) {
    for (int i = 0; i < 8; i++) {
        buf[*off + i] = (uint8_t)((v >> (8 * i)) & 0xFFu);
    }
    *off += 8;
}

void kuntur_wire_put_f64(uint8_t *buf, size_t *off, double v) {
    uint64_t bits;
    memcpy(&bits, &v, sizeof(bits));
    kuntur_wire_put_u64(buf, off, bits);
}

uint16_t kuntur_wire_get_u16(const uint8_t *buf, size_t *off) {
    uint16_t v = (uint16_t)(buf[*off + 0] | (buf[*off + 1] << 8));
    *off += 2;
    return v;
}

uint32_t kuntur_wire_get_u32(const uint8_t *buf, size_t *off) {
    uint32_t v = (uint32_t)buf[*off + 0]
        | ((uint32_t)buf[*off + 1] << 8)
        | ((uint32_t)buf[*off + 2] << 16)
        | ((uint32_t)buf[*off + 3] << 24);
    *off += 4;
    return v;
}

uint64_t kuntur_wire_get_u64(const uint8_t *buf, size_t *off) {
    uint64_t v = 0;
    for (int i = 0; i < 8; i++) {
        v |= ((uint64_t)buf[*off + i]) << (8 * i);
    }
    *off += 8;
    return v;
}

double kuntur_wire_get_f64(const uint8_t *buf, size_t *off) {
    uint64_t bits = kuntur_wire_get_u64(buf, off);
    double v;
    memcpy(&v, &bits, sizeof(v));
    return v;
}
