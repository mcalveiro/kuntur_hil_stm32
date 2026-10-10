/*
 * usbd_desc.c -- descriptores USB (device/strings) para
 * kuntur_autopilot. VID/PID = los que ST reserva para sus propios
 * ejemplos CDC_Standalone (0x0483/0x5740) -- el driver generico cdc_acm
 * de Linux (y el inbox de Windows) matchea por clase/subclase/
 * protocolo de la interfaz, no por VID/PID, así que no hace falta un
 * PID propio para que el host lo reconozca como puerto serie.
 */
#include "usbd_desc.h"
#include "usbd_conf.h"
#include "middleware/Core/Inc/usbd_core.h"
#include "middleware/Core/Inc/usbd_ctlreq.h"
#include "stm32f7xx_hal.h"

#ifndef UNUSED
#define UNUSED(x) (void)(x)
#endif

#define USBD_VID            0x0483U
#define USBD_PID            0x5740U
#define USBD_LANGID_STRING  0x0409U /* inglés (US) */
#define USBD_MANUFACTURER_STRING "kuntur-project"
#define USBD_PRODUCT_STRING      "kuntur_autopilot HIL"
#define USBD_CONFIGURATION_STRING "kuntur_autopilot CDC Config"
#define USBD_INTERFACE_STRING     "kuntur_autopilot CDC Interface"

#define USB_SIZ_STRING_SERIAL 0x1AU

/* UID de 96 bits del STM32F7 (RM0410 §41.1) -- usado como numero de
 * serie USB, mismo criterio que el template de ST. */
#define DEVICE_ID1 (0x1FF0F420U)
#define DEVICE_ID2 (0x1FF0F424U)
#define DEVICE_ID3 (0x1FF0F428U)

static uint8_t USBD_StrDesc[USBD_MAX_STR_DESC_SIZ];
static uint8_t USBD_StringSerial[USB_SIZ_STRING_SERIAL] = {
    USB_SIZ_STRING_SERIAL,
    USB_DESC_TYPE_STRING,
};

__ALIGN_BEGIN static uint8_t USBD_DeviceDesc[USB_LEN_DEV_DESC] __ALIGN_END = {
    0x12, USB_DESC_TYPE_DEVICE,
    0x00, 0x02,             /* bcdUSB = 2.00 */
    0x00,                   /* bDeviceClass: definida en la interfaz (CDC) */
    0x00,                   /* bDeviceSubClass */
    0x00,                   /* bDeviceProtocol */
    USB_MAX_EP0_SIZE,
    LOBYTE(USBD_VID), HIBYTE(USBD_VID),
    LOBYTE(USBD_PID), HIBYTE(USBD_PID),
    0x00, 0x02,             /* bcdDevice = 2.00 */
    USBD_IDX_MFC_STR, USBD_IDX_PRODUCT_STR, USBD_IDX_SERIAL_STR,
    USBD_MAX_NUM_CONFIGURATION,
};

__ALIGN_BEGIN static uint8_t USBD_LangIDDesc[USB_LEN_LANGID_STR_DESC] __ALIGN_END = {
    USB_LEN_LANGID_STR_DESC, USB_DESC_TYPE_STRING,
    LOBYTE(USBD_LANGID_STRING), HIBYTE(USBD_LANGID_STRING),
};

static void IntToUnicode(uint32_t value, uint8_t *pbuf, uint8_t len) {
    for (uint8_t idx = 0; idx < len; idx++) {
        uint8_t nibble = (value >> (28 - idx * 4)) & 0x0FU;
        pbuf[2 * idx] = (nibble < 0xAU) ? (uint8_t)(nibble + '0') : (uint8_t)(nibble + 'A' - 10U);
        pbuf[2 * idx + 1] = 0;
    }
}

static void Get_SerialNum(void) {
    uint32_t id0 = *(uint32_t *)DEVICE_ID1;
    uint32_t id1 = *(uint32_t *)DEVICE_ID2;
    uint32_t id2 = *(uint32_t *)DEVICE_ID3;
    id0 += id2;
    if (id0 != 0U) {
        IntToUnicode(id0, &USBD_StringSerial[2], 8U);
        IntToUnicode(id1, &USBD_StringSerial[18], 4U);
    }
}

static uint8_t *KUNTUR_DeviceDescriptor(USBD_SpeedTypeDef speed, uint16_t *length) {
    UNUSED(speed);
    *length = sizeof(USBD_DeviceDesc);
    return USBD_DeviceDesc;
}

static uint8_t *KUNTUR_LangIDStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length) {
    UNUSED(speed);
    *length = sizeof(USBD_LangIDDesc);
    return USBD_LangIDDesc;
}

static uint8_t *KUNTUR_ManufacturerStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length) {
    UNUSED(speed);
    USBD_GetString((uint8_t *)USBD_MANUFACTURER_STRING, USBD_StrDesc, length);
    return USBD_StrDesc;
}

static uint8_t *KUNTUR_ProductStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length) {
    UNUSED(speed);
    USBD_GetString((uint8_t *)USBD_PRODUCT_STRING, USBD_StrDesc, length);
    return USBD_StrDesc;
}

static uint8_t *KUNTUR_SerialStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length) {
    UNUSED(speed);
    *length = USB_SIZ_STRING_SERIAL;
    Get_SerialNum();
    return USBD_StringSerial;
}

static uint8_t *KUNTUR_ConfigStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length) {
    UNUSED(speed);
    USBD_GetString((uint8_t *)USBD_CONFIGURATION_STRING, USBD_StrDesc, length);
    return USBD_StrDesc;
}

static uint8_t *KUNTUR_InterfaceStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length) {
    UNUSED(speed);
    USBD_GetString((uint8_t *)USBD_INTERFACE_STRING, USBD_StrDesc, length);
    return USBD_StrDesc;
}

USBD_DescriptorsTypeDef KUNTUR_Desc = {
    KUNTUR_DeviceDescriptor,
    KUNTUR_LangIDStrDescriptor,
    KUNTUR_ManufacturerStrDescriptor,
    KUNTUR_ProductStrDescriptor,
    KUNTUR_SerialStrDescriptor,
    KUNTUR_ConfigStrDescriptor,
    KUNTUR_InterfaceStrDescriptor,
};
