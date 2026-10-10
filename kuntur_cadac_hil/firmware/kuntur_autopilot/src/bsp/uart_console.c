#include "uart_console.h"
#include "stm32f7xx_hal.h"
#include <string.h>

static UART_HandleTypeDef s_huart3;

void kuntur_uart_console_init(void) {
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_USART3_CLK_ENABLE();

    GPIO_InitTypeDef gpio = {0};
    gpio.Pin = GPIO_PIN_8 | GPIO_PIN_9;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio.Alternate = GPIO_AF7_USART3;
    HAL_GPIO_Init(GPIOD, &gpio);

    s_huart3.Instance = USART3;
    s_huart3.Init.BaudRate = 115200;
    s_huart3.Init.WordLength = UART_WORDLENGTH_8B;
    s_huart3.Init.StopBits = UART_STOPBITS_1;
    s_huart3.Init.Parity = UART_PARITY_NONE;
    s_huart3.Init.Mode = UART_MODE_TX_RX;
    s_huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    s_huart3.Init.OverSampling = UART_OVERSAMPLING_16;
    HAL_UART_Init(&s_huart3);
}

void kuntur_uart_console_puts(const char *s) {
    HAL_UART_Transmit(&s_huart3, (const uint8_t *)s, (uint16_t)strlen(s), HAL_MAX_DELAY);
}
