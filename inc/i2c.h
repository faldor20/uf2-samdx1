/*
 * Minimal blocking I2C master driver for SAMD/SAML/SAME families.
 *
 * Notes:
 * - This driver assumes the SERCOM clock and pinmux are configured by board init
 *   or higher-level code before calling i2c_enable()/i2c_init_instance().
 * - Provides instance-based APIs and optional convenience wrappers bound to
 *   an `I2C_SERCOM` macro (e.g., `#define I2C_SERCOM SERCOM1`).
 */

#ifndef I2C_DRIVER_H
#define I2C_DRIVER_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


typedef enum {
    I2C_RESULT_SUCCESS = 0,
    I2C_RESULT_ERR_BUSSTATE,
    I2C_RESULT_ERR_ADDR_NACK,
    I2C_RESULT_ERR_DATA_NACK,
    I2C_RESULT_ERR_TIMEOUT,
    I2C_RESULT_ERR_BUSERR,
    I2C_RESULT_ERR_UNSUPPORTED
} i2c_result_t;

#define I2C_SERCOM 1


/* Instance-based API (opaque sercom pointer to avoid heavy headers in public API) */
void i2c_init_instance(void *sercom_instance, uint32_t baud_hz);
void i2c_enable_instance(void *sercom_instance);
bool i2c_is_enabled_instance(void *sercom_instance);
i2c_result_t i2c_write_instance(void *sercom_instance, uint8_t address, const uint8_t *data, size_t len);
i2c_result_t i2c_read_instance(void *sercom_instance, uint8_t address, uint8_t *data, size_t len);
void i2c_disable_instance(void *sercom_instance);

/* Optional helper to configure pinmux for SDA/SCL pins (PINMUX_* values) */
void i2c_configure_pins(uint32_t sda_pinmux, uint32_t scl_pinmux);

/* Optional convenience wrappers when I2C_SERCOM is defined (e.g., SERCOM1) */
#ifdef I2C_SERCOM
void i2c_init(uint32_t baud_hz);
void i2c_enable(void);
bool i2c_is_enabled(void);
i2c_result_t i2c_write(uint8_t address, const uint8_t *data, size_t len);
i2c_result_t i2c_read(uint8_t address, uint8_t *data, size_t len);
void i2c_disable(void);
#else
/* Stubs to keep linkers happy on boards without I2C configured */
static inline void i2c_init(uint32_t baud_hz) { (void)baud_hz; }
static inline void i2c_enable(void) {}
static inline bool i2c_is_enabled(void) { return false; }
static inline i2c_result_t i2c_write(uint8_t address, const uint8_t *data, size_t len) {
    (void)address; (void)data; (void)len; return I2C_RESULT_ERR_UNSUPPORTED;
}
static inline i2c_result_t i2c_read(uint8_t address, uint8_t *data, size_t len) {
    (void)address; (void)data; (void)len; return I2C_RESULT_ERR_UNSUPPORTED;
}
static inline void i2c_disable(void) {}
#endif

#endif /* I2C_DRIVER_H */


