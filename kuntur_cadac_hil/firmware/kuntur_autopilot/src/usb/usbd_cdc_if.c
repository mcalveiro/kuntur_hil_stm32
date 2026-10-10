/*
 * usbd_cdc_if.c -- ver usbd_cdc_if.h. `Receive()` es la única función
 * con lógica real: copia el paquete USB (<=64 bytes, CDC_DATA_FS_
 * MAX_PACKET_SIZE) al ring buffer de usb_cdc_io.c y re-arma la
 * recepción -- patrón estándar de CubeMX (ver nota en el docstring de
 * la plantilla de ST sobre por qué hay que re-armar antes de salir:
 * mientras no se llame a USBD_CDC_ReceivePacket, el endpoint OUT
 * devuelve NAK).
 */
#include "usbd_cdc_if.h"
#include "usb_cdc_io.h"
#include <stddef.h>

#ifndef UNUSED
#define UNUSED(x) (void)(x)
#endif

extern USBD_HandleTypeDef hUsbDeviceFS;

static uint8_t s_rx_packet[CDC_DATA_FS_MAX_PACKET_SIZE];

static USBD_CDC_LineCodingTypeDef s_line_coding = {
    115200, 0x00, 0x00, 0x08,
};

static int8_t KUNTUR_CDC_Init(void) {
    /* hcdc->RxBuffer no puede quedar NULL cuando retorna Init() --
     * USBD_CDC_Init() (usbd_cdc.c) lo chequea y arma el primer
     * USBD_LL_PrepareReceive() con esto. El TxBuffer real lo pone cada
     * llamada a kuntur_usb_cdc_write() (usb_cdc_io.c) -- acá no hace
     * falta tocarlo. */
    USBD_CDC_SetRxBuffer(&hUsbDeviceFS, s_rx_packet);
    return 0;
}

static int8_t KUNTUR_CDC_DeInit(void) {
    return 0;
}

static int8_t KUNTUR_CDC_Control(uint8_t cmd, uint8_t *pbuf, uint16_t length) {
    UNUSED(length);
    switch (cmd) {
        case CDC_SET_LINE_CODING:
            s_line_coding.bitrate = (uint32_t)(pbuf[0] | (pbuf[1] << 8) | (pbuf[2] << 16) | (pbuf[3] << 24));
            s_line_coding.format = pbuf[4];
            s_line_coding.paritytype = pbuf[5];
            s_line_coding.datatype = pbuf[6];
            break;
        case CDC_GET_LINE_CODING:
            pbuf[0] = (uint8_t)(s_line_coding.bitrate);
            pbuf[1] = (uint8_t)(s_line_coding.bitrate >> 8);
            pbuf[2] = (uint8_t)(s_line_coding.bitrate >> 16);
            pbuf[3] = (uint8_t)(s_line_coding.bitrate >> 24);
            pbuf[4] = s_line_coding.format;
            pbuf[5] = s_line_coding.paritytype;
            pbuf[6] = s_line_coding.datatype;
            break;
        /* DTR/RTS (CDC_SET_CONTROL_LINE_STATE), break (CDC_SEND_BREAK) y el
         * resto de comandos de modem (ENCAPSULATED_COMMAND, COMM_FEATURE) no
         * aplican a un enlace punto a punto host-MCU -- se ignoran, igual
         * que hace el template de ST para quien no los necesita. */
        default:
            break;
    }
    return 0;
}

static int8_t KUNTUR_CDC_Receive(uint8_t *Buf, uint32_t *Len) {
    kuntur_usb_cdc_rx_push(Buf, *Len);
    USBD_CDC_SetRxBuffer(&hUsbDeviceFS, s_rx_packet);
    USBD_CDC_ReceivePacket(&hUsbDeviceFS);
    return 0;
}

static int8_t KUNTUR_CDC_TransmitCplt(uint8_t *Buf, uint32_t *Len, uint8_t epnum) {
    UNUSED(Buf);
    UNUSED(Len);
    UNUSED(epnum);
    return 0;
}

USBD_CDC_ItfTypeDef KUNTUR_CDC_fops = {
    KUNTUR_CDC_Init,
    KUNTUR_CDC_DeInit,
    KUNTUR_CDC_Control,
    KUNTUR_CDC_Receive,
    KUNTUR_CDC_TransmitCplt,
};
