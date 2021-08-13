#ifndef BOARD_CONFIG_H
#define BOARD_CONFIG_H

#define VENDOR_NAME "Oddly Specific Objects"
#define PRODUCT_NAME "Sensor Watch M0"
#define VOLUME_LABEL "WATCHBOOT"
#define INDEX_URL "http://oddlyspecific.org/"
#define BOARD_ID "OSO-SWAT-A1-00"

#define USB_VID 0x239A
#define USB_PID 0x001B

// watch is PA20, feather is PA08
#define LED_PIN PIN_PA20
// #define LED_PIN PIN_PA08

#define BOOT_USART_MODULE                 SERCOM3
#define BOOT_USART_BUS_CLOCK_INDEX        MCLK_APBDMASK_SERCOM3
#define BOOT_USART_PAD_SETTINGS           UART_RX_PAD1_TX_PAD0
#define BOOT_USART_PAD3                   PINMUX_UNUSED
#define BOOT_USART_PAD2                   PINMUX_UNUSED
#define BOOT_USART_PAD1                   PINMUX_PB03C_SERCOM3_PAD1
#define BOOT_USART_PAD0                   PINMUX_PB02C_SERCOM3_PAD0

#endif
