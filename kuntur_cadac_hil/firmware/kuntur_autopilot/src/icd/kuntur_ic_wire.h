/*
 * kuntur_ic_wire.h -- Serialización little-endian de bajo nivel,
 * compartida por todos los canales ICD de este fork (kuntur_ic_plant_state,
 * kuntur_ic_surface_cmd). Escribe/lee byte a byte hacia un buffer
 * uint8_t, SIN asumir nada sobre alineación de memoria del lado C --
 * ver el docstring de kuntur_ic_plant_state.h para el porqué (Cortex-M7
 * no tolera acceso no alineado a `double` de forma confiable, y el
 * header de KUNTUR_PLANT_STATE mide 18 bytes).
 *
 * `double` se serializa reinterpretando sus bits IEEE 754 binary64 como
 * uint64_t (memcpy, no cast) y escribiendo esos 8 bytes en little-endian
 * -- válido tanto en el STM32F767 (FPU FPv5-D16) como en el host x86_64
 * usado para el test de regresión: ambos usan IEEE 754 binary64 nativo.
 */
#ifndef KUNTUR_IC_WIRE_H
#define KUNTUR_IC_WIRE_H

#include <stddef.h>
#include <stdint.h>

void kuntur_wire_put_u16(uint8_t *buf, size_t *off, uint16_t v);
void kuntur_wire_put_u32(uint8_t *buf, size_t *off, uint32_t v);
void kuntur_wire_put_u64(uint8_t *buf, size_t *off, uint64_t v);
void kuntur_wire_put_f64(uint8_t *buf, size_t *off, double v);

uint16_t kuntur_wire_get_u16(const uint8_t *buf, size_t *off);
uint32_t kuntur_wire_get_u32(const uint8_t *buf, size_t *off);
uint64_t kuntur_wire_get_u64(const uint8_t *buf, size_t *off);
double kuntur_wire_get_f64(const uint8_t *buf, size_t *off);

#endif /* KUNTUR_IC_WIRE_H */
