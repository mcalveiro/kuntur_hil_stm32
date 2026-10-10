# middleware/ -- vendoreado sin modificar

Copia textual (sin ningún cambio) de
`Middlewares/ST/STM32_USB_Device_Library/{Core,Class/CDC}` del paquete
`framework-stm32cubef7` que ya trae PlatformIO (ver
`~/.platformio/packages/framework-stm32cubef7/`). `framework =
stm32cube` en `platformio.ini` solo agrega `Drivers/` (HAL + CMSIS) al
include path -- no compila `Middlewares/`, así que estos archivos se
vendorean acá, igual que lo haría un proyecto generado por CubeMX (que
copia estos mismos archivos dentro de cada proyecto, no los referencia
in-place).

No editar estos archivos. La configuración específica del proyecto
(qué pines, qué descriptor, qué hace con los bytes recibidos) vive un
nivel arriba, en `src/usb/usbd_conf.*`, `usbd_desc.*`, `usbd_cdc_if.*`
-- ese es el patrón que separa "driver genérico de ST" de "glue del
proyecto" en todo código generado por CubeMX.
