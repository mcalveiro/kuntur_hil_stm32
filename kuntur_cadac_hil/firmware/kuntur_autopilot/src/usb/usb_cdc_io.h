/*
 * usb_cdc_io.h -- API de aplicacion sobre USB CDC (Fase 5 del plan,
 * plan-hil-stm32-portado-autopiloto.md §7: "usb_cdc_io.c/h -- pack/
 * unpack buffers USB CDC + handshake"). main_hil.c (src/hil/) es el
 * único consumidor.
 *
 * Reensambla frames completos a partir de los paquetes USB de 64 bytes
 * (CDC_DATA_FS_MAX_PACKET_SIZE) que entregan los callbacks de
 * usbd_cdc_if.c -- KUNTUR_PLANT_STATE son 352 bytes, varios paquetes
 * USB por frame.
 */
#ifndef KUNTUR_USB_CDC_IO_H
#define KUNTUR_USB_CDC_IO_H

#include <stdint.h>
#include <stdbool.h>

/* USBD_Init + RegisterClass(CDC) + CDC_RegisterInterface + USBD_Start.
 * Llamar una vez, despues de kuntur_clock_config_216mhz() (USB
 * necesita los 48MHz del PLLQ, ver src/bsp/clock_config.c). */
void kuntur_usb_cdc_init(void);

/* true si la enumeracion USB llegó a CONFIGURED (host abrió el puerto
 * y completó el proceso estandar -- no implica que esté escribiendo
 * datos, solo que el enlace USB en sí está arriba). */
bool kuntur_usb_cdc_connected(void);

/* Push del lado ISR (usbd_cdc_if.c::Receive) hacia el ring buffer --
 * no es parte de la API publica para main_hil.c, pero vive declarada
 * acá porque el ring buffer es privado a esta unidad de compilación. */
void kuntur_usb_cdc_rx_push(const uint8_t *data, uint32_t len);

/* Bytes ya reensamblados disponibles para leer. */
uint32_t kuntur_usb_cdc_rx_available(void);

/* Copia hasta `len` bytes SIN avanzar el tail (no consume) -- para que
 * main_hil.c pueda mirar el magic header de KUNTUR_PLANT_STATE antes
 * de comprometerse a consumir todo el frame. Devuelve cuantos bytes
 * copió (puede ser < len si no hay suficientes todavía). */
uint32_t kuntur_usb_cdc_rx_peek(uint8_t *out, uint32_t len);

/* Avanza el tail `len` bytes (saturado a `kuntur_usb_cdc_rx_available()`
 * -- nunca consume más de lo que hay). Para descartar 1 byte a la vez
 * al resincronizar contra el magic header, o los `len` de un frame ya
 * leído con peek(). */
void kuntur_usb_cdc_rx_consume(uint32_t len);

/* Transmite `len` bytes, reintentando mientras el endpoint IN anterior
 * esté ocupado (USBD_BUSY) hasta `timeout_ms`. Devuelve 0 si OK, -1 si
 * timeout. */
int kuntur_usb_cdc_write(const uint8_t *data, uint16_t len, uint32_t timeout_ms);

#endif /* KUNTUR_USB_CDC_IO_H */
