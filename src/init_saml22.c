#include "board_config.h"
#include "uf2.h"
#include "i2c.h"

volatile bool g_interrupt_enabled = true;

// SAMD21 starts at 1MHz by default.
uint32_t current_cpu_frequency_MHz = 1;

static void dfll_sync(void) {
    while (!(OSCCTRL->STATUS.reg & OSCCTRL_STATUS_DFLLRDY));
}

void system_init(void) {

    /* Various bits in the INTFLAG register can be set to one at startup.
       This will ensure that these bits are cleared */
    OSCCTRL->INTFLAG.reg = OSCCTRL_INTFLAG_DFLLRDY;
    SUPC->INTFLAG.reg = SUPC_INTFLAG_BOD33RDY | SUPC_INTFLAG_BOD33DET;

    GCLK->CTRLA.bit.SWRST = 1;
    while (GCLK->SYNCBUSY.bit.SWRST) {
        // wait for sync
    }

    // set up flash wait states
    NVMCTRL->CTRLB.bit.RWS = 1;

    // Switch to the highest performance level
    PM->INTFLAG.reg = PM_INTFLAG_PLRDY;
    PM->PLCFG.reg = PM_PLCFG_PLSEL_PL2_Val;
    while (!PM->INTFLAG.reg) {
        // wait for sync
    }

    uint32_t calib = (OSC32KCTRL->OSCULP32K.reg & OSC32KCTRL_OSCULP32K_CALIB_Msk) >> OSC32KCTRL_OSCULP32K_CALIB_Pos;
    OSC32KCTRL->OSCULP32K.reg = OSC32KCTRL_OSCULP32K_CALIB(calib);

    // Temporarily switch the CPU to the internal 32k oscillator while we
    // reconfigure the DFLL.
    GCLK->GENCTRL[0].reg = GCLK_GENCTRL_SRC(GCLK_GENCTRL_SRC_OSCULP32K) |
                           GCLK_GENCTRL_GENEN;

    while (GCLK->SYNCBUSY.bit.GENCTRL0) {
        /* Wait for synchronization */
    }

    OSCCTRL->DFLLCTRL.reg = 0;
    dfll_sync();

    uint32_t coarse =(*((uint32_t *)NVMCTRL_OTP5)) >> 26;
    OSCCTRL->DFLLVAL.reg = OSCCTRL_DFLLVAL_COARSE(coarse) |
                           OSCCTRL_DFLLVAL_FINE(0x200);
    OSCCTRL->DFLLMUL.reg = OSCCTRL_DFLLMUL_CSTEP( 1 ) |
                           OSCCTRL_DFLLMUL_FSTEP( 1 ) |
                           OSCCTRL_DFLLMUL_MUL( 48000 );
    OSCCTRL->DFLLCTRL.reg = OSCCTRL_DFLLCTRL_MODE | OSCCTRL_DFLLCTRL_ENABLE | OSCCTRL_DFLLCTRL_CCDIS | OSCCTRL_DFLLCTRL_USBCRM;

    dfll_sync();

    GCLK->GENCTRL[0].reg =
        GCLK_GENCTRL_SRC(GCLK_GENCTRL_SRC_DFLL48M) |
                         GCLK_GENCTRL_DIV(1) |
                         GCLK_GENCTRL_GENEN;

    while (GCLK->SYNCBUSY.bit.GENCTRL0) {
      /* Wait for synchronization */
    }

    MCLK->CPUDIV.reg = MCLK_CPUDIV_CPUDIV_DIV1;

    SysTick_Config(1000);

    current_cpu_frequency_MHz = 48;

    /* I2C SERCOM1 clock and pin mux for PB30 (SDA) / PB31 (SCL) */
    /* Enable SERCOM1 core/slow clocks */
    GCLK->PCHCTRL[I2C_GCLK_ID_CORE].reg = GCLK_PCHCTRL_GEN_GCLK0_Val | GCLK_PCHCTRL_CHEN;
    GCLK->PCHCTRL[I2C_GCLK_ID_SLOW].reg = GCLK_PCHCTRL_GEN_GCLK0_Val | GCLK_PCHCTRL_CHEN;
    /* Enable bus clock to SERCOM1 */
    MCLK->APBCMASK.reg |= I2C_BUS_CLOCK_INDEX;
    /* Pin mux: PB30C -> SERCOM1 PAD0, PB31C -> SERCOM1 PAD1 */
    i2c_configure_pins(I2C_SDA_PIN, I2C_SCL_PIN);

  // Output 500hz PWM on PA04 (TCC0 WO[0]) so we can validate the GCLK0 clock speed
//   MCLK->APBCMASK.reg |= MCLK_APBCMASK_TCC0;
//   TCC0->PER.bit.PER = 48000000 / 1000;
//   TCC0->CC[0].bit.CC = 48000000 / 2000;
//   TCC0->CTRLA.bit.ENABLE = true;
//
//   PORT->Group[0].PINCFG[4].bit.PMUXEN = true;
//   PORT->Group[0].PMUX[2].bit.PMUXE = 4;
//   GCLK->PCHCTRL[TCC0_GCLK_ID].reg = GCLK_PCHCTRL_GEN_GCLK0_Val | GCLK_PCHCTRL_CHEN;
}

void SysTick_Handler(void) { timerTick(); }
