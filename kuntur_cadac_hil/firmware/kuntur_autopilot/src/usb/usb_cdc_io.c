#include "usb_cdc_io.h"
#include "usbd_cdc_if.h"
#include "usbd_desc.h"
#include "middleware/Core/Inc/usbd_core.h"
#include "middleware/Class/CDC/Inc/usbd_cdc.h"
#include "stm32f7xx_hal.h"
#include <string.h>

USBD_HandleTypeDef hUsbDeviceFS;

/* Ring buffer SPSC: productor = ISR de USB (usbd_cdc_if.c::Receive,
 * via kuntur_usb_cdc_rx_push), consumidor = main_hil.c (polling). Sin
 * locks: en Cortex-M un entero de 32 bits se lee/escribe atomico, y
 * head lo toca solo el productor / tail solo el consumidor. Potencia
 * de 2 para que el modulo sea una mascara. */
#define RX_RING_SIZE 2048u
#define RX_RING_MASK (RX_RING_SIZE - 1u)
static uint8_t s_rx_ring[RX_RING_SIZE];
static volatile uint32_t s_rx_head = 0; /* próximo índice a escribir (productor) */
static volatile uint32_t s_rx_tail = 0; /* próximo índice a leer (consumidor) */

void kuntur_usb_cdc_init(void) {
    USBD_Init(&hUsbDeviceFS, &KUNTUR_Desc, 0U);
    USBD_RegisterClass(&hUsbDeviceFS, USBD_CDC_CLASS);
    USBD_CDC_RegisterInterface(&hUsbDeviceFS, &KUNTUR_CDC_fops);
    USBD_Start(&hUsbDeviceFS);
}

bool kuntur_usb_cdc_connected(void) {
    return hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED;
}

void kuntur_usb_cdc_rx_push(const uint8_t *data, uint32_t len) {
    for (uint32_t i = 0; i < len; i++) {
        uint32_t next_head = (s_rx_head + 1u) & RX_RING_MASK;
        if (next_head == s_rx_tail) {
            /* ring lleno -- se descarta el resto del paquete. No
             * debería pasar: el consumidor (main_hil.c) lee a 100Hz,
             * mucho más rápido que lo que entra por USB FS. Preferible
             * perder bytes (se resincroniza con el próximo magic
             * header) a bloquear la ISR. */
            return;
        }
        s_rx_ring[s_rx_head] = data[i];
        s_rx_head = next_head;
    }
}

uint32_t kuntur_usb_cdc_rx_available(void) {
    return (s_rx_head - s_rx_tail) & RX_RING_MASK;
}

uint32_t kuntur_usb_cdc_rx_peek(uint8_t *out, uint32_t len) {
    uint32_t available = kuntur_usb_cdc_rx_available();
    if (len > available) {
        len = available;
    }
    uint32_t tail = s_rx_tail;
    for (uint32_t i = 0; i < len; i++) {
        out[i] = s_rx_ring[tail];
        tail = (tail + 1u) & RX_RING_MASK;
    }
    return len;
}

void kuntur_usb_cdc_rx_consume(uint32_t len) {
    uint32_t available = kuntur_usb_cdc_rx_available();
    if (len > available) {
        len = available;
    }
    s_rx_tail = (s_rx_tail + len) & RX_RING_MASK;
}

int kuntur_usb_cdc_write(const uint8_t *data, uint16_t len, uint32_t timeout_ms) {
    USBD_CDC_SetTxBuffer(&hUsbDeviceFS, (uint8_t *)data, len);
    uint32_t t0 = HAL_GetTick();
    while (USBD_CDC_TransmitPacket(&hUsbDeviceFS) != USBD_OK) {
        if ((HAL_GetTick() - t0) >= timeout_ms) {
            return -1;
        }
    }
    return 0;
}
