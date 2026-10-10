/*
 * clock_config.h -- reloj a 216 MHz (Fase 4 del plan, ver
 * ../../README.md y plan-hil-stm32-portado-autopiloto.md §7: "Reloj a
 * 216 MHz (full speed)"). PLL alimentada desde HSI (16MHz interno), no
 * HSE -- evita depender de un solder bridge / MCO del ST-Link no
 * verificado en esta placa (ver clock_config.c).
 */
#ifndef KUNTUR_BSP_CLOCK_CONFIG_H
#define KUNTUR_BSP_CLOCK_CONFIG_H

/* Configura PLL (HSI 16MHz / 8 * 216 / 2 = 216MHz), Over-Drive y Flash
 * latency. Cuelga (while(1)) si algun paso de HAL_RCC falla -- no hay
 * UART todavia en ese punto para reportar el error. */
void kuntur_clock_config_216mhz(void);

#endif /* KUNTUR_BSP_CLOCK_CONFIG_H */
