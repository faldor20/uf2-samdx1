/* ----------------------------------------------------------------------------
 *         SAM Software Package License
 * ----------------------------------------------------------------------------
 * Copyright (c) 2011-2014, Atmel Corporation
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following condition is met:
 *
 * Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the disclaimer below.
 *
 * Atmel's name may not be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * DISCLAIMER: THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * ----------------------------------------------------------------------------
 */

/**
 * --------------------
 * SAM-BA Implementation on SAMD21 and SAMD51
 * --------------------
 * Requirements to use SAM-BA :
 *
 * Supported communication interfaces (SAMD21):
 * --------------------
 *
 * SERCOM5 : RX:PB23 TX:PB22
 * Baudrate : 115200 8N1
 *
 * USB : D-:PA24 D+:PA25
 *
 * Pins Usage
 * --------------------
 * The following pins are used by the program :
 * PA25 : input/output
 * PA24 : input/output
 * PB23 : input
 * PB22 : output
 * PA15 : input
 *
 * The application board shall avoid driving the PA25,PA24,PB23,PB22 and PA15
 * signals
 * while the boot program is running (after a POR for example)
 *
 * Clock system
 * --------------------
 * CPU clock source (GCLK_GEN_0) - 8MHz internal oscillator (OSC8M)
 * SERCOM5 core GCLK source (GCLK_ID_SERCOM5_CORE) - GCLK_GEN_0 (i.e., OSC8M)
 * GCLK Generator 1 source (GCLK_GEN_1) - 48MHz DFLL in Clock Recovery mode
 * (DFLL48M)
 * USB GCLK source (GCLK_ID_USB) - GCLK_GEN_1 (i.e., DFLL in CRM mode)
 *
 * Memory Mapping
 * --------------------
 * SAM-BA code will be located at 0x0 and executed before any applicative code.
 *
 * Applications compiled to be executed along with the bootloader will start at
 * 0x2000 (samd21) or 0x4000 (samd51)
 *
 */

#include "lib/st25dvxxkc/st25dvxxkc.h"
#include "lib/st25dvxxkc/st25dvxxkc_reg.h"
#include "uf2.h"
#include "i2c.h"
#include "nfc_ftm.h"
// #include <string.h>

/* ST25DV platform object and bus bindings */
static ST25DVxxKC_Object_t g_st25dv_obj;
static bool g_st25dv_inited = false;

static int32_t st25dv_bus_init(void) { return NFCTAG_OK; }
static int32_t st25dv_bus_deinit(void) { return NFCTAG_OK; }
static int32_t st25dv_bus_write(uint16_t DevAddr, uint16_t MemAddr, const uint8_t *pData, uint16_t len) {
	uint8_t i2c_addr_7bit = (uint8_t)(DevAddr >> 1);
	uint8_t hdr[2] = { (uint8_t)(MemAddr >> 8), (uint8_t)(MemAddr & 0xFF) };
	if (len == 0) {
		return (i2c_write(i2c_addr_7bit, hdr, 2) == I2C_RESULT_SUCCESS) ? NFCTAG_OK : NFCTAG_ERROR;
	}
	if (len > 256) len = 256;
	uint8_t tmp[2 + 256];
	tmp[0] = hdr[0];
	tmp[1] = hdr[1];
	for (uint16_t i = 0; i < len; i++) tmp[2 + i] = pData[i];
	return (i2c_write(i2c_addr_7bit, tmp, (uint32_t)(2 + len)) == I2C_RESULT_SUCCESS) ? NFCTAG_OK : NFCTAG_ERROR;
}
static int32_t st25dv_bus_read(uint16_t DevAddr, uint16_t MemAddr, uint8_t *pData, uint16_t len) {
	uint8_t i2c_addr_7bit = (uint8_t)(DevAddr >> 1);
	uint8_t addr[2] = { (uint8_t)(MemAddr >> 8), (uint8_t)(MemAddr & 0xFF) };
	if (i2c_write(i2c_addr_7bit, addr, 2) != I2C_RESULT_SUCCESS) return NFCTAG_ERROR;
	return (i2c_read(i2c_addr_7bit, pData, len) == I2C_RESULT_SUCCESS) ? NFCTAG_OK : NFCTAG_ERROR;
}
static int32_t st25dv_bus_isready(uint16_t DevAddr, const uint32_t Trials) {
	uint8_t i2c_addr_7bit = (uint8_t)(DevAddr >> 1);
	for (uint32_t i = 0; i < Trials; i++) {
		if (i2c_write(i2c_addr_7bit, NULL, 0) == I2C_RESULT_SUCCESS) return NFCTAG_OK;
		delay(1);
	}
	return NFCTAG_ERROR;
}
static int32_t st25dv_bus_gettick(void) { return (int32_t)timerHigh; }

void* ST25FTM_GetSt25dvHandle(void) {
	if (!g_st25dv_inited) {
		ST25DVxxKC_IO_t io = {
			.Init = st25dv_bus_init,
			.DeInit = st25dv_bus_deinit,
			.IsReady = st25dv_bus_isready,
			.Write = st25dv_bus_write,
			.Read = st25dv_bus_read,
			.GetTick = st25dv_bus_gettick,
			.DeviceAddress = ST25DVXXKC_ADDR_DATA_I2C,
		};
		if (ST25DVxxKC_RegisterBusIO(&g_st25dv_obj, &io) != NFCTAG_OK)
			return NULL;
		if (St25Dvxxkc_Drv.Init(&g_st25dv_obj) != NFCTAG_OK)
			return NULL;
        
		g_st25dv_inited = true;
	}
	return &g_st25dv_obj;
}

static void check_start_application(void);

static volatile bool main_b_cdc_enable = false;
extern int8_t led_tick_step;

#if defined(SAMD21)
    #define RESET_CONTROLLER PM
#elif defined(SAMD51)
    #define RESET_CONTROLLER RSTC
#elif defined(SAML21) || defined(SAML22)
    #define RESET_CONTROLLER RSTC
#endif

/**
 * \brief Check the application startup condition
 *
 */
static void check_start_application(void) {
    uint32_t app_start_address;

    /* Load the Reset Handler address of the application */
    app_start_address = *(uint32_t *)(APP_START_ADDRESS + 4);

    /**
     * Test reset vector of application @APP_START_ADDRESS+4
     * Sanity check on the Reset_Handler address
     */
    if (app_start_address < APP_START_ADDRESS || app_start_address > FLASH_SIZE) {
        /* Stay in bootloader */
        return;
    }

#if USE_SINGLE_RESET
    if (SINGLE_RESET()) {
        if (RESET_CONTROLLER->RCAUSE.bit.POR || *DBL_TAP_PTR != DBL_TAP_MAGIC_QUICK_BOOT) {
            // the second tap on reset will go into app
            *DBL_TAP_PTR = DBL_TAP_MAGIC_QUICK_BOOT;
            // this will be cleared after successful USB enumeration
            // this is around 1.5s
            resetHorizon = timerHigh + 50;
            return;
        }
    }
#endif

    if (RESET_CONTROLLER->RCAUSE.bit.POR) {
        *DBL_TAP_PTR = 0;
    }
    else if (*DBL_TAP_PTR == DBL_TAP_MAGIC) {
        *DBL_TAP_PTR = 0;
        return; // stay in bootloader
    }
    else {
        if (*DBL_TAP_PTR != DBL_TAP_MAGIC_QUICK_BOOT) {
            *DBL_TAP_PTR = DBL_TAP_MAGIC;
            delay(500);
        }
        *DBL_TAP_PTR = 0;
    }

    LED_MSC_OFF();

#if defined(BOARD_RGBLED_CLOCK_PIN)
    // This won't work for neopixel, because we're running at 1MHz or thereabouts...
    RGBLED_set_color(COLOR_LEAVE);
#endif

    /* Rebase the Stack Pointer */
    __set_MSP(*(uint32_t *)APP_START_ADDRESS);

    /* Rebase the vector table base address */
    SCB->VTOR = ((uint32_t)APP_START_ADDRESS & SCB_VTOR_TBLOFF_Msk);

    /* Jump to application Reset Handler in the application */
    asm("bx %0" ::"r"(app_start_address));
}

extern char _etext;
extern char _end;

/**
 *  \brief  SAM-BA Main loop.
 *  \return Unused (ANSI-C compatibility).
 */
 __attribute__((used))
int main(void) {
    // if VTOR is set, we're not running in bootloader mode; halt
    if (SCB->VTOR)
        while (1) {
        }

    // Stage 1: Basic initialization complete
    led_init_flash(1, false, 250);

#if defined(SAMD21)
    // If fuses have been reset to all ones, the watchdog ALWAYS-ON is
    // set, so we can't turn off the watchdog.  Set the fuse to a
    // reasonable value and reset. This is a mini version of the fuse
    // reset code in selfmain.c.
    if (((uint32_t *)NVMCTRL_AUX0_ADDRESS)[0] == 0xffffffff) {
        // Clear any error flags.
        NVMCTRL->STATUS.reg |= NVMCTRL_STATUS_MASK;
        // Turn off cache and put in manual mode.
        NVMCTRL->CTRLB.reg = NVMCTRL->CTRLB.reg | NVMCTRL_CTRLB_CACHEDIS | NVMCTRL_CTRLB_MANW;
        // Set address to write.
        NVMCTRL->ADDR.reg = NVMCTRL_AUX0_ADDRESS / 2;
        // Erase auxiliary row.
        NVMCTRL->CTRLA.reg = NVMCTRL_CTRLA_CMDEX_KEY | NVMCTRL_CTRLA_CMD_EAR;
	while (!(NVMCTRL->INTFLAG.bit.READY)) {}
        // Clear page buffer.
        NVMCTRL->CTRLA.reg = NVMCTRL_CTRLA_CMDEX_KEY | NVMCTRL_CTRLA_CMD_PBC;
	while (!(NVMCTRL->INTFLAG.bit.READY)) {}
        // Reasonable fuse values, including 8k BOOTPROT.
        ((uint32_t *)NVMCTRL_AUX0_ADDRESS)[0] = 0xD8E0C7FA;
        ((uint32_t *)NVMCTRL_AUX0_ADDRESS)[1] = 0xFFFFFC5D;
        // Write the fuses
	NVMCTRL->CTRLA.reg = NVMCTRL_CTRLA_CMDEX_KEY | NVMCTRL_CTRLA_CMD_WAP;
	while (!(NVMCTRL->INTFLAG.bit.READY)) {}
        resetIntoBootloader();
    }

    // Disable the watchdog, in case the application set it.
    WDT->CTRL.reg = 0;
    while(WDT->STATUS.bit.SYNCBUSY) {}

#elif defined(SAMD51)
    // Disable the watchdog, in case the application set it.
    WDT->CTRLA.reg = 0;
    while(WDT->SYNCBUSY.reg) {}

    // Enable 2.7V brownout detection. The default fuse value is 1.7
    // Set brownout detection to ~2.7V. Default from factory is 1.7V,
    // which is too low for proper operation of external SPI flash chips (they are 2.7-3.6V).
    // Also without this higher level, the SAMD51 will write zeros to flash intermittently.
    // Disable while changing level.

    SUPC->BOD33.bit.ENABLE = 0;
    while (!SUPC->STATUS.bit.B33SRDY) {}  // Wait for BOD33 to synchronize.
    SUPC->BOD33.bit.LEVEL = 200;  // 2.7V: 1.5V + LEVEL * 6mV.
    // Don't reset right now.
    SUPC->BOD33.bit.ACTION = SUPC_BOD33_ACTION_NONE_Val;
    SUPC->BOD33.bit.ENABLE = 1; // enable brown-out detection

    // Wait for BOD33 peripheral to be ready.
    while (!SUPC->STATUS.bit.BOD33RDY) {}

    // Wait for voltage to rise above BOD33 value.
    while (SUPC->STATUS.bit.BOD33DET) {}

    // If we are starting from a power-on or a brownout,
    // wait for the voltage to stabilize. Don't do this on an
    // external reset because it interferes with the timing of double-click.
    // "BODVDD" means BOD33.
    if (RSTC->RCAUSE.bit.POR || RSTC->RCAUSE.bit.BODVDD) {
        do {
            // Check again in 100ms.
            delay(100);
        } while (SUPC->STATUS.bit.BOD33DET);
    }

    // Now enable reset if voltage falls below minimum.
    SUPC->BOD33.bit.ENABLE = 0;
    while (!SUPC->STATUS.bit.B33SRDY) {}  // Wait for BOD33 to synchronize.
    SUPC->BOD33.bit.ACTION = SUPC_BOD33_ACTION_RESET_Val;
    SUPC->BOD33.bit.ENABLE = 1;

#elif defined(SAML21) || defined(SAML22)
    // Disable the watchdog, in case the application set it.
    WDT->CTRLA.reg = 0;
    while(WDT->SYNCBUSY.reg) {}

    // Enable 2.7V brownout detection. The default fuse value is 1.7
    // Set brownout detection to ~2.7V. Default from factory is 1.7V,
    // which is too low for proper operation of external SPI flash chips (they are 2.7-3.6V).
    // Also without this higher level, the SAMD51 will write zeros to flash intermittently.
    // Disable while changing level.

    SUPC->BOD33.bit.ENABLE = 0;
    while (!SUPC->STATUS.bit.B33SRDY) {}  // Wait for BOD33 to synchronize.
    SUPC->BOD33.bit.LEVEL = 200;  // 2.7V: 1.5V + LEVEL * 6mV.
    // Don't reset right now.
    SUPC->BOD33.bit.ACTION = SUPC_BOD33_ACTION_NONE_Val;
    SUPC->BOD33.bit.ENABLE = 1; // enable brown-out detection

    // Wait for BOD33 peripheral to be ready.
    while (!SUPC->STATUS.bit.BOD33RDY) {}

    // Wait for voltage to rise above BOD33 value.
    while (SUPC->STATUS.bit.BOD33DET) {}

    // If we are starting from a power-on or a brownout,
    // wait for the voltage to stabilize. Don't do this on an
    // external reset because it interferes with the timing of double-click.
    // "BODVDD" means BOD33.
#ifdef SAML21
    if (RSTC->RCAUSE.bit.POR || RSTC->RCAUSE.bit.BOD33) {
#else // SAML22
    if (RSTC->RCAUSE.bit.POR || RSTC->RCAUSE.bit.BODVDD) {
#endif
        do {
            // Check again in 100ms.
            delay(100);
        } while (SUPC->STATUS.bit.BOD33DET);
    }

    // Now enable reset if voltage falls below minimum.
    SUPC->BOD33.bit.ENABLE = 0;
    while (!SUPC->STATUS.bit.B33SRDY) {}  // Wait for BOD33 to synchronize.
    SUPC->BOD33.bit.ACTION = SUPC_BOD33_ACTION_RESET_Val;
    SUPC->BOD33.bit.ENABLE = 1;
#endif

#if USB_VID == 0x239a && USB_PID == 0x0013     // Adafruit Metro M0
    // Delay a bit so SWD programmer can have time to attach.
    delay(15);
#endif
    led_init();

    logmsg("Start");
    assert((uint32_t)&_etext < APP_START_ADDRESS);
    // bossac writes at 0x20005000
    assert(!USE_MONITOR || (uint32_t)&_end < 0x20005000);

    assert(8 << NVMCTRL->PARAM.bit.PSZ == FLASH_PAGE_SIZE);
    assert(FLASH_PAGE_SIZE * NVMCTRL->PARAM.bit.NVMP == FLASH_SIZE);

    /* Jump in application if condition is satisfied */
    //disable temporarily so we are always in bootloader mode 
    check_start_application();

    /* We have determined we should stay in the monitor. */
    /* System initialization */
    system_init();
    RGBLED_set_color(COLOR_START);
    LED_MSC_ON();
    delay(500);
    LED_MSC_OFF();
    // delay(2000);
    // LED_MSC_ON();
    // delay(500);
    // LED_MSC_OFF();
    /* Initialize I2C bus and ST25 FTM */
    i2c_init(100000);
    i2c_enable();
    RGBLED_set_color(0x00FF00);
    // //test the nfc communication works by writing a sequence to the nfc tag
    // uint8_t test_sequence[] = "NFC-TEST";

LED_MSC_ON();
    // {
    //     ST25DV_Object_t st25_obj;
    //     ST25DV_IO_t io_ctx = {
    //         .Init = st25_test_bus_init,
    //         .DeInit = st25_test_bus_deinit,
    //         .Write = st25_test_bus_write,
    //         .Read = st25_test_bus_read,
    //         .IsReady = st25_test_bus_isready,
    //         .GetTick = st25_test_bus_gettick
    //     };
    //     ST25DV_ResetRFDisable(&st25_obj);
    //     ST25DV_ResetRFDisable_Dyn(&st25_obj);
    //     ST25DV_SetRF_MNGT_RFDIS(&st25_obj.Ctx,0x00);
    //     uint8_t rb[sizeof(test_sequence)] = {0};
    //     //looks like there is an issue with my i2c setup or my connection to the chip or something
    //     // should check the voltage to the chip
    //     // the i2c lines with the scope
    //     // the i2c pins are correct
    //     // etc
        
    //     if (ST25DV_RegisterBusIO(&st25_obj, &io_ctx) == NFCTAG_OK &&
    //         St25Dv_Drv.Init(&st25_obj) == NFCTAG_OK &&
    //         St25Dv_Drv.WriteData(&st25_obj, test_sequence, 0x0010, (uint16_t)sizeof(test_sequence)) == NFCTAG_OK &&
    //         St25Dv_Drv.ReadData(&st25_obj, rb, 0x0010, (uint16_t)sizeof(test_sequence)) == NFCTAG_OK) {
    //         if (memcmp(test_sequence, rb, sizeof(test_sequence)) == 0) {
    //             logmsg("NFC test write/read OK");
    //             RGBLED_set_color(COLOR_LEAVE);
    //             led_init_flash(10, false,150);
    //         } else {
    //             logmsg("NFC test data mismatch");
    //             RGBLED_set_color(0xFFFF00);
    //             led_init_flash(5, false,150);
    //         }
    //     } else {
    //         logmsg("NFC test write/read failed");
    //         RGBLED_set_color(0xFFFF00);
    //             led_init_flash(3, false,150);
    //     }
    // }
 
    

    __DMB();
    __enable_irq();

    if(!nfc_ftm_start()) {
        while(1) {
            delay(1000);
        }
    }


    LED_MSC_TGL();
    delay(500);
    LED_MSC_TGL();
    delay(500);
    LED_MSC_TGL();
    // logmsg("Before main loop");

    // usb_init();

    // not enumerated yet
    RGBLED_set_color(COLOR_START);
    led_tick_step = 10;
    uint32_t count = 0;
    // delay(2000);
    /* Wait for a complete enum on usb or a '#' char on serial line */
    while (1) {
        /* NFC FTM cooperative runner */
        nfc_ftm_poll();
        if(count==30){
            count = 0;
            LED_MSC_TGL();
        }
        count++;
    }


}