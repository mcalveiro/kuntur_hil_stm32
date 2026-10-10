/*
 * main.c -- blink de verificación de la cadena de herramientas
 * (PlatformIO + STM32Cube + ST-LINK), ver plan-hil-stm32-portado-autopiloto.md
 * §15 ítem 3: "Setup Nucleo-144 + PlatformIO verificado -- compilar un
 * blink antes de empezar".
 *
 * Toggle de LD1 (verde, PB0 en el Nucleo-144 F767ZI) cada 500 ms.
 * Reloj por default (HSI, sin PLL) a propósito -- este archivo solo
 * valida build + upload + que el LED responda. La config de clock a
 * 216MHz + I/D-cache (src/bsp/clock_config.c) la usa el env de testbench
 * (src/test_target/main_bitexact.c, Fase 4 del plan, ver README.md) --
 * este blink se deja igual a propósito, para no mezclar dos cosas que
 * se debuggean distinto si algo falla.
 */
#include "stm32f7xx_hal.h"

static void MX_GPIO_Init(void) {
    __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_0;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

int main(void) {
    HAL_Init();
    MX_GPIO_Init();

    while (1) {
        HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_0);
        HAL_Delay(500);
    }
}

void SysTick_Handler(void) {
    HAL_IncTick();
}
