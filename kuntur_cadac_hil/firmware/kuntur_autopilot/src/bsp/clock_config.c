#include "clock_config.h"
#include "stm32f7xx_hal.h"

void kuntur_clock_config_216mhz(void) {
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /* HSI (16MHz interno), no HSE -- evita depender de que el MCO de
     * 8MHz del ST-Link hacia HSE este efectivamente puenteado en esta
     * placa (varia por solder bridge / board rev, no verificado a
     * mano). VCO = 16/8*216 = 432MHz (en rango permitido);
     * PLLP=2 -> 216MHz sysclk; PLLQ=9 -> 48MHz (para USB, Fase 5). */
    RCC_OscInitTypeDef osc = {0};
    osc.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    osc.HSIState = RCC_HSI_ON;
    osc.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    osc.PLL.PLLState = RCC_PLL_ON;
    osc.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    osc.PLL.PLLM = 8;
    osc.PLL.PLLN = 216;
    osc.PLL.PLLP = RCC_PLLP_DIV2;
    osc.PLL.PLLQ = 9;
    if (HAL_RCC_OscConfig(&osc) != HAL_OK) {
        while (1) {}
    }

    /* Over-Drive requerido por el F7 para sostener 216MHz a Voltage
     * Scale 1 (ver RM0410, seccion PWR). */
    if (HAL_PWREx_EnableOverDrive() != HAL_OK) {
        while (1) {}
    }

    RCC_ClkInitTypeDef clk = {0};
    clk.ClockType = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK |
                     RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    clk.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    clk.AHBCLKDivider = RCC_SYSCLK_DIV1;
    clk.APB1CLKDivider = RCC_HCLK_DIV4; /* <= 54MHz */
    clk.APB2CLKDivider = RCC_HCLK_DIV2; /* <= 108MHz */
    if (HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_7) != HAL_OK) {
        while (1) {}
    }
}
