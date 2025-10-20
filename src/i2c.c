/* Minimal blocking I2C master for SAMD/SAML/SAME */

#include "sam.h"
#include "main.h" /* CPU_FREQUENCY */
#include "i2c.h"

#define BUSSTATE_UNKNOWN 0
#define BUSSTATE_IDLE 1
#define BUSSTATE_OWNER 2
#define BUSSTATE_BUSY 3



#define I2C_TIMEOUT_LOOP_MAX 100000u
void i2c_configure_pins(uint32_t sda_pinmux, uint32_t scl_pinmux) {
    if (sda_pinmux != 0xffffffff) {
        uint32_t port = (sda_pinmux & 0x200000) >> 21;
        uint8_t pin = sda_pinmux >> 16;
        PORT->Group[port].PINCFG[(pin - (port * 32))].bit.PMUXEN = 1;
        PORT->Group[port].PMUX[(pin - (port * 32)) / 2].reg &= ~(0xF << (4 * (pin & 0x01u)));
        PORT->Group[port].PMUX[(pin - (port * 32)) / 2].reg |= (sda_pinmux & 0xFF) << (4 * (pin & 0x01u));
    }
    if (scl_pinmux != 0xffffffff) {
        uint32_t port = (scl_pinmux & 0x200000) >> 21;
        uint8_t pin = scl_pinmux >> 16;
        PORT->Group[port].PINCFG[(pin - (port * 32))].bit.PMUXEN = 1;
        PORT->Group[port].PMUX[(pin - (port * 32)) / 2].reg &= ~(0xF << (4 * (pin & 0x01u)));
        PORT->Group[port].PMUX[(pin - (port * 32)) / 2].reg |= (scl_pinmux & 0xFF) << (4 * (pin & 0x01u));
    }
}



static inline i2c_result_t i2c_send_stop(Sercom *sercom, i2c_result_t status) {
    sercom->I2CM.CTRLB.bit.CMD = 3; /* STOP */
    while (sercom->I2CM.SYNCBUSY.bit.SYSOP) { }
    return status;
}

static Sercom *i2c_get_sercom(void) {
#ifndef I2C_SERCOM_IS_PTR
    Sercom *sercom_instances[SERCOM_INST_NUM] = SERCOM_INSTS;
    return sercom_instances[I2C_SERCOM];
#else
    return I2C_SERCOM;
#endif
}

void i2c_init_instance(void *sercom_instance, uint32_t baud_hz) {
    Sercom *sercom = (Sercom *)sercom_instance;
    /* Ensure disabled */
    while (sercom->I2CM.SYNCBUSY.bit.ENABLE) { }
    sercom->I2CM.CTRLA.bit.ENABLE = 0;
    while (sercom->I2CM.SYNCBUSY.bit.SWRST || sercom->I2CM.SYNCBUSY.bit.ENABLE) { }
    sercom->I2CM.CTRLA.bit.SWRST = 1;
    while (sercom->I2CM.SYNCBUSY.bit.SWRST || sercom->I2CM.CTRLA.bit.SWRST) { }

#if defined(SAMD21) || defined(SAMD11)
    sercom->I2CM.CTRLA.reg = SERCOM_I2CM_CTRLA_SPEED(0) | SERCOM_I2CM_CTRLA_SDAHOLD(0) |
                              SERCOM_I2CM_CTRLA_MODE_I2C_MASTER;
#else
    sercom->I2CM.CTRLA.reg = SERCOM_I2CM_CTRLA_SPEED(0) | SERCOM_I2CM_CTRLA_SDAHOLD(0) |
                              SERCOM_I2CM_CTRLA_MODE(5);
#endif
    sercom->I2CM.CTRLB.reg = SERCOM_I2CM_CTRLB_SMEN; /* smart mode */
    while (sercom->I2CM.SYNCBUSY.bit.SYSOP) { }

    uint32_t clock_speed = CPU_FREQUENCY; /* Hz */
    uint32_t rise_time_ns = 300; /* ns */
    sercom->I2CM.BAUD.bit.BAUD =
        clock_speed / (2 * baud_hz) - 5 - (((clock_speed / 1000000) * rise_time_ns) / (2 * 1000));
    while (sercom->I2CM.SYNCBUSY.bit.SYSOP) { }
}

void i2c_enable_instance(void *sercom_instance) {
    Sercom *sercom = (Sercom *)sercom_instance;
    sercom->I2CM.CTRLA.bit.ENABLE = 1;
    while (sercom->I2CM.SYNCBUSY.bit.ENABLE) { }
    sercom->I2CM.STATUS.bit.BUSSTATE = BUSSTATE_IDLE;
    while (sercom->I2CM.SYNCBUSY.bit.SYSOP) { }
}

bool i2c_is_enabled_instance(void *sercom_instance) {
    Sercom *sercom = (Sercom *)sercom_instance;
    return sercom->I2CM.CTRLA.bit.ENABLE;
}

i2c_result_t i2c_write_instance(void *sercom_instance, uint8_t address, const uint8_t *data, size_t len) {
    Sercom *sercom = (Sercom *)sercom_instance;
    if (sercom->I2CM.STATUS.bit.BUSSTATE == BUSSTATE_BUSY) {
        return I2C_RESULT_ERR_BUSSTATE;
    }

    sercom->I2CM.ADDR.bit.ADDR = ((uint32_t)address << 1) | 0; /* write */

    size_t wait = 0;
    for (; wait < I2C_TIMEOUT_LOOP_MAX; wait++) {
        if (sercom->I2CM.INTFLAG.bit.MB) break;
    }

    if (sercom->I2CM.STATUS.bit.BUSSTATE != BUSSTATE_OWNER) {
        return I2C_RESULT_ERR_BUSSTATE;
    }
    if (sercom->I2CM.STATUS.bit.RXNACK) {
        return I2C_RESULT_ERR_ADDR_NACK;
    }
    if (wait >= I2C_TIMEOUT_LOOP_MAX) {
        return i2c_send_stop(sercom, I2C_RESULT_ERR_TIMEOUT);
    }

    for (size_t i = 0; i < len; i++) {
        sercom->I2CM.DATA.reg = data[i];
        size_t w = 0;
        while (!sercom->I2CM.INTFLAG.bit.MB) {
            if (sercom->I2CM.STATUS.bit.BUSERR) {
                return i2c_send_stop(sercom, I2C_RESULT_ERR_BUSERR);
            }
            if (++w >= I2C_TIMEOUT_LOOP_MAX) {
                return i2c_send_stop(sercom, I2C_RESULT_ERR_TIMEOUT);
            }
        }
        if (sercom->I2CM.STATUS.bit.RXNACK) {
            return i2c_send_stop(sercom, I2C_RESULT_ERR_DATA_NACK);
        }
    }

    sercom->I2CM.CTRLB.bit.CMD = 3; /* STOP */
    while (sercom->I2CM.SYNCBUSY.bit.SYSOP) { }
    return I2C_RESULT_SUCCESS;
}

i2c_result_t i2c_read_instance(void *sercom_instance, uint8_t address, uint8_t *data, size_t len) {
    Sercom *sercom = (Sercom *)sercom_instance;
    if (sercom->I2CM.STATUS.bit.BUSSTATE == BUSSTATE_BUSY) {
        return I2C_RESULT_ERR_BUSSTATE;
    }

    sercom->I2CM.ADDR.bit.ADDR = ((uint32_t)address << 1) | 1; /* read */
    sercom->I2CM.CTRLB.reg &= ~SERCOM_I2CM_CTRLB_ACKACT; /* prepare ACK */

    size_t wait = 0;
    for (; wait < I2C_TIMEOUT_LOOP_MAX; wait++) {
        if (sercom->I2CM.INTFLAG.bit.SB) break;
    }

    if (sercom->I2CM.STATUS.bit.BUSSTATE != BUSSTATE_OWNER) {
        return I2C_RESULT_ERR_BUSSTATE;
    }
    if (sercom->I2CM.STATUS.bit.RXNACK) {
        return I2C_RESULT_ERR_ADDR_NACK;
    }
    if (wait >= I2C_TIMEOUT_LOOP_MAX) {
        return i2c_send_stop(sercom, I2C_RESULT_ERR_TIMEOUT);
    }

    for (size_t i = 0; i < len; i++) {
        data[i] = sercom->I2CM.DATA.reg;
        size_t w = 0;
        while (!sercom->I2CM.INTFLAG.bit.SB) {
            if (sercom->I2CM.STATUS.bit.BUSERR) {
                return i2c_send_stop(sercom, I2C_RESULT_ERR_BUSERR);
            }
            if (++w >= I2C_TIMEOUT_LOOP_MAX) {
                return i2c_send_stop(sercom, I2C_RESULT_ERR_TIMEOUT);
            }
        }
    }

    sercom->I2CM.CTRLB.reg |= SERCOM_I2CM_CTRLB_ACKACT | SERCOM_I2CM_CTRLB_CMD(3); /* NACK+STOP */
    while (sercom->I2CM.SYNCBUSY.bit.SYSOP) { }
    return I2C_RESULT_SUCCESS;
}

void i2c_disable_instance(void *sercom_instance) {
    Sercom *sercom = (Sercom *)sercom_instance;
    sercom->I2CM.CTRLA.bit.ENABLE = 0;
    while (sercom->I2CM.SYNCBUSY.bit.ENABLE) { }
}

#ifdef I2C_SERCOM
void i2c_init(uint32_t baud_hz) {
    i2c_init_instance((void *)i2c_get_sercom(), baud_hz);
}
void i2c_enable(void) { i2c_enable_instance((void *)i2c_get_sercom()); }
bool i2c_is_enabled(void) { return i2c_is_enabled_instance((void *)i2c_get_sercom()); }
i2c_result_t i2c_write(uint8_t address, const uint8_t *data, size_t len) { return i2c_write_instance((void *)i2c_get_sercom(), address, data, len); }
i2c_result_t i2c_read(uint8_t address, uint8_t *data, size_t len) { return i2c_read_instance((void *)i2c_get_sercom(), address, data, len); }
void i2c_disable(void) { i2c_disable_instance((void *)i2c_get_sercom()); }
#endif


