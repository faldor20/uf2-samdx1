#ifndef BOARD_CONFIG_H
#define BOARD_CONFIG_H

#define CRYSTALLESS    1

#define VENDOR_NAME "Oddlt Specific Objects"
#define PRODUCT_NAME "QT Py L21"
#define VOLUME_LABEL "QTPY_BOOT"
#define INDEX_URL "http://adafru.it/4600"
#define BOARD_ID "SAML21E18B-QTPy-v0"

// TODO: different PID
#define USB_VID 0x239A
#define USB_PID 0x00CB

//#define LED_PIN PIN_PA10
//#define LED_TX_PIN PIN_PA27
//#define LED_RX_PIN PIN_PB03

#define BOARD_NEOPIXEL_POWERPIN PIN_PA15
#define BOARD_NEOPIXEL_PIN PIN_PA18
#define BOARD_NEOPIXEL_COUNT 1

#define BOOT_USART_MODULE                 SERCOM0
#define BOOT_USART_MASK                   APBAMASK
#define BOOT_USART_BUS_CLOCK_INDEX        MCLK_APBDMASK_SERCOM0
#define BOOT_USART_PAD_SETTINGS           UART_RX_PAD3_TX_PAD2
#define BOOT_USART_PAD3                   PINMUX_PA07D_SERCOM0_PAD3
#define BOOT_USART_PAD2                   PINMUX_PA06D_SERCOM0_PAD2
#define BOOT_USART_PAD1                   PINMUX_UNUSED
#define BOOT_USART_PAD0                   PINMUX_UNUSED
#define BOOT_GCLK_ID_CORE                 SERCOM5_GCLK_ID_CORE
#define BOOT_GCLK_ID_SLOW                 SERCOM5_GCLK_ID_SLOW


#endif
