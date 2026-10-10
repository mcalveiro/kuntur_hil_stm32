/*
 * main_hil.c -- loop principal del HIL (Fase 5-6 del plan,
 * plan-hil-stm32-portado-autopiloto.md §2/§5/§7/§11): recibe
 * KUNTUR_PLANT_STATE por USB CDC, corre falcon6_autopilot_step() a
 * 100Hz (int_step=0.01s, decimación x10 respecto a la física de 1kHz,
 * ver §5), devuelve KUNTUR_SURFACE_CMD. Esto es lo que
 * kuntur_bridge/stm32_bridge.py espera del otro lado del puerto serie
 * (ver su docstring: CONECTADO envía y espera respuesta con timeout,
 * FALLBACK si no llega).
 *
 * Reusa sin cambios: src/bsp/ (clock/uart, igual que Fase 4), src/icd/
 * + src/autopilot/ (el port validado bit-exacto en Fase 3/4), src/usb/
 * (middleware CDC + glue de esta fase).
 */
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>

#include "stm32f7xx_hal.h"

#include "../bsp/clock_config.h"
#include "../bsp/uart_console.h"
#include "../usb/usb_cdc_io.h"

#include "../icd/kuntur_ic_plant_state.h"
#include "../icd/kuntur_ic_surface_cmd.h"
#include "../autopilot/autopilot_state.h"
#include "../autopilot/falcon6_autopilot.h"
#include "../config/scenario_params.h"

/* 100Hz nominal del HIL (plan §5) -- NO el 0.001 de Fase 3/4 (esos
 * testbenches igualan la cadencia de captura de las traces, 1kHz, para
 * comparar contra Python paso a paso; acá el STM32 corre a la cadencia
 * real que va a usar plant_driver.py). */
#define AUTOPILOT_INT_STEP 0.01

/* Si no entra un frame completo en esta ventana, se descarta el
 * intento y se vuelve a esperar el magic header desde cero -- más
 * barato que acumular basura indefinidamente. stm32_bridge.py (host)
 * tiene su propio timeout de 25ms (default) para la respuesta
 * completa; este valor es el presupuesto de ESTE lado para terminar de
 * recibir el PEDIDO, generosamente menor a eso. */
#define FRAME_WAIT_TIMEOUT_MS 15u
#define TX_TIMEOUT_MS 10u

static void init_status_led(void) {
    __HAL_RCC_GPIOB_CLK_ENABLE();
    GPIO_InitTypeDef gpio = {0};
    gpio.Pin = GPIO_PIN_0;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &gpio);
}

void SysTick_Handler(void) {
    HAL_IncTick();
}

/* Espera a que los primeros 4 bytes del ring buffer sean el magic de
 * KUNTUR_PLANT_STATE, descartando 1 byte a la vez si no matchea --
 * resincroniza contra basura/bytes perdidos sin asumir que la
 * corrupción es múltiplo del tamaño de frame (ver usb_cdc_io.h). No
 * bloquea si no hay ni 4 bytes todavía: devuelve -1 de inmediato (el
 * llamador reintenta en la próxima vuelta del loop, no tiene sentido
 * esperar aquí con timeout -- el framing llega a su propio ritmo). */
static int resync_to_magic_header(void) {
    uint8_t hdr[4];
    while (kuntur_usb_cdc_rx_peek(hdr, 4) == 4) {
        uint32_t magic = (uint32_t)hdr[0] | ((uint32_t)hdr[1] << 8) |
                          ((uint32_t)hdr[2] << 16) | ((uint32_t)hdr[3] << 24);
        if (magic == KUNTUR_MAGIC_PLANT_STATE) {
            return 0;
        }
        kuntur_usb_cdc_rx_consume(1);
    }
    return -1;
}

int main(void) {
    HAL_Init();
    kuntur_clock_config_216mhz();
    SCB_EnableICache();
    SCB_EnableDCache();
    init_status_led();
    kuntur_uart_console_init();
    kuntur_usb_cdc_init();

    kuntur_uart_console_puts("\r\nkuntur_autopilot -- HIL (USB CDC, Fase 5/6)\r\n");

    Scenario_Params_t params = SCENARIO_PARAMS_F16C11_7_INIT;
    Autopilot_State_t state;
    autopilot_state_reset(&state);

    bool was_connected = false;
    uint32_t frames_ok = 0;
    uint32_t frames_bad = 0;

    for (;;) {
        bool connected = kuntur_usb_cdc_connected();
        if (connected && !was_connected) {
            kuntur_uart_console_puts("USB CDC: host conectado\r\n");
        } else if (!connected && was_connected) {
            kuntur_uart_console_puts("USB CDC: host desconectado\r\n");
        }
        was_connected = connected;

        if (resync_to_magic_header() != 0) {
            continue; /* no hay 4 bytes todavia -- nada que hacer esta vuelta */
        }

        uint8_t frame[KUNTUR_PLANT_STATE_SIZE];
        if (kuntur_usb_cdc_rx_peek(frame, KUNTUR_PLANT_STATE_SIZE) != KUNTUR_PLANT_STATE_SIZE) {
            /* el magic ya llegó pero el resto del frame todavía no --
             * esperar (bloqueante, con timeout) a que termine de
             * llegar en vez de volver a peekear en una vuelta ajustada
             * del loop principal. */
            uint32_t t0 = HAL_GetTick();
            while (kuntur_usb_cdc_rx_available() < KUNTUR_PLANT_STATE_SIZE) {
                if ((HAL_GetTick() - t0) >= FRAME_WAIT_TIMEOUT_MS) {
                    break;
                }
            }
            if (kuntur_usb_cdc_rx_peek(frame, KUNTUR_PLANT_STATE_SIZE) != KUNTUR_PLANT_STATE_SIZE) {
                /* timeout -- el frame nunca se completó (host se
                 * desconectó/crasheó a mitad de un envío). Sin este
                 * consume(1), resync_to_magic_header() volvería a
                 * encontrar el MISMO magic viejo en la próxima vuelta
                 * (nunca lo consumió, solo lo peekeó) y entraríamos en
                 * este mismo timeout en bucle indefinido, sin avanzar
                 * nunca hacia bytes nuevos que sí lleguen después de
                 * una reconexión. Se descarta 1 byte para forzar
                 * resincronización real. */
                kuntur_usb_cdc_rx_consume(1);
                continue;
            }
        }

        kuntur_plant_state_t in;
        if (kuntur_unpack_plant_state(&in, frame, KUNTUR_PLANT_STATE_SIZE) != 0) {
            /* El magic matcheaba (ya lo filtró resync_to_magic_header)
             * pero el resto no es válido -- no debería pasar nunca en
             * la práctica; se descarta solo este byte del magic para
             * no trabarse en el mismo frame corrupto y se deja que el
             * próximo resync lo vuelva a evaluar. */
            kuntur_usb_cdc_rx_consume(1);
            frames_bad++;
            continue;
        }
        kuntur_usb_cdc_rx_consume(KUNTUR_PLANT_STATE_SIZE);

        if (in.flags & KUNTUR_PLANT_STATE_FLAG_INIT) {
            autopilot_state_reset(&state);
        }

        kuntur_surface_cmd_t out;
        memset(&out, 0, sizeof(out));
        falcon6_autopilot_step(&in, &params, &state, AUTOPILOT_INT_STEP, &out);
        out.t_sim_us = in.t_sim_us; /* eco -- stm32_bridge.py no lo exige hoy, pero el schema lo define (§6.2) */
        out.seq_num = in.seq_num;
        out.status = KUNTUR_SURFACE_CMD_STATUS_OK;

        uint8_t resp[KUNTUR_SURFACE_CMD_SIZE];
        kuntur_pack_surface_cmd(resp, &out);
        kuntur_usb_cdc_write(resp, KUNTUR_SURFACE_CMD_SIZE, TX_TIMEOUT_MS);

        frames_ok++;
        HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_0); /* heartbeat: un toggle por frame procesado */

        if ((frames_ok & 0xFFu) == 0u) { /* cada 256 frames (~2.5s @ 100Hz) */
            char line[96];
            snprintf(line, sizeof(line), "frames ok=%lu bad=%lu\r\n",
                     (unsigned long)frames_ok, (unsigned long)frames_bad);
            kuntur_uart_console_puts(line);
        }
    }
}
