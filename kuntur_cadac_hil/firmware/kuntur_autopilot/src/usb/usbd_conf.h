/*
 * usbd_conf.h -- config del USB Device middleware (Fase 5 del plan,
 * plan-hil-stm32-portado-autopiloto.md §7/§11). Basado en el patron
 * estandar que genera CubeMX para USB_OTG_FS + clase CDC (un solo
 * config por proyecto, no hay nada especifico de kuntur_autopilot
 * acá salvo los defines de abajo).
 */
#ifndef KUNTUR_USBD_CONF_H
#define KUNTUR_USBD_CONF_H

#include "stm32f7xx_hal.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define USBD_MAX_NUM_INTERFACES     1U
#define USBD_MAX_NUM_CONFIGURATION  1U
#define USBD_MAX_STR_DESC_SIZ       0x100U
#define USBD_SELF_POWERED           0U /* bus-powered via el conector USB_FS del Nucleo */
#define USBD_DEBUG_LEVEL            0U /* sin printf desde el middleware -- la UART de debug es aparte (src/bsp/uart_console.*) */

#define USBD_CDC_INTERVAL           2000U

#define USBD_malloc   (void *)USBD_static_malloc
#define USBD_free     USBD_static_free
#define USBD_memset   memset
#define USBD_memcpy   memcpy
#define USBD_Delay    HAL_Delay

#define USBD_UsrLog(...)
#define USBD_ErrLog(...)
#define USBD_DbgLog(...)

void *USBD_static_malloc(uint32_t size);
void USBD_static_free(void *p);

#endif /* KUNTUR_USBD_CONF_H */
