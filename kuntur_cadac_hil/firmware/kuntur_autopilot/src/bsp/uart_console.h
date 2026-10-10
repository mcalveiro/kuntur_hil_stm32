/*
 * uart_console.h -- salida de texto por USART3 (PD8=TX/PD9=RX), que en
 * el Nucleo-144 es el VCP (Virtual COM Port) del ST-Link integrado --
 * mismo /dev/ttyACM0 que ya usa `pio run -t upload`/monitor (ver
 * README.md "Permisos USB"). No requiere USB CDC propio: la Fase 4 del
 * plan (testbench embebido, reporte por UART) usa esto; la Fase 5 (USB
 * CDC hacia kuntur_bridge) es un canal USB *distinto*, que se agrega
 * despues sin tocar esto.
 */
#ifndef KUNTUR_BSP_UART_CONSOLE_H
#define KUNTUR_BSP_UART_CONSOLE_H

void kuntur_uart_console_init(void);

/* Bloqueante (HAL_MAX_DELAY) -- aceptable para un testbench que corre
 * una vez y reporta, no para el loop de control en tiempo real. */
void kuntur_uart_console_puts(const char *s);

#endif /* KUNTUR_BSP_UART_CONSOLE_H */
