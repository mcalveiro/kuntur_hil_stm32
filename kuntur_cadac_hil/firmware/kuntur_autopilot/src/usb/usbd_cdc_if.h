/*
 * usbd_cdc_if.h -- implementacion minima de USBD_CDC_ItfTypeDef (el
 * "fops" que el middleware CDC llama en cada evento: Init/DeInit/
 * Control/Receive/TransmitCplt). Nombre y forma estandar de CubeMX --
 * la logica de aplicacion (ring buffer, API que usa main_hil.c) vive
 * en usb_cdc_io.c/h, no acá.
 */
#ifndef KUNTUR_USBD_CDC_IF_H
#define KUNTUR_USBD_CDC_IF_H

#include "middleware/Class/CDC/Inc/usbd_cdc.h"

extern USBD_CDC_ItfTypeDef KUNTUR_CDC_fops;

#endif /* KUNTUR_USBD_CDC_IF_H */
