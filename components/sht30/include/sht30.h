#ifndef SHT30_H
#define SHT30_H

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/** SHT30 I2C default address (ADDR pin low) */
#define SHT30_I2C_ADDR_DEFAULT  0x44U

/** I2C port used by SHT30 */
#define SHT30_I2C_PORT          I2C_NUM_0

/** GPIO assignments from pin table */
#define SHT30_PIN_SCL           1
#define SHT30_PIN_SDA           4

/** I2C bus clock frequency (400 kHz fast-mode) */
#define SHT30_I2C_FREQ_HZ       400000U

/**
 * @brief Measured output from one SHT30 sample.
 */
typedef struct {
    float temperature_c;     /**< Temperature in degrees Celsius */
    float humidity_percent;  /**< Relative humidity in percent */
    bool  valid;             /**< true when data passed CRC checks */
} sht30_data_t;

/**
 * @brief Initialize the SHT30 driver and I2C bus.
 *
 * Installs the I2C master driver on SHT30_I2C_PORT and performs a soft-reset
 * of the sensor.  Must be called before sht30_read().
 *
 * @return ESP_OK on success, or a driver/HAL error code on failure.
 */
esp_err_t sht30_init(void);

/**
 * @brief Trigger a single-shot measurement and return the result.
 *
 * Uses the "Single Shot – Clock Stretching Disabled, High Repeatability"
 * command (0x2400).  Blocks for the sensor conversion time (~15 ms).
 *
 * @param[out] out_data  Destination for the measurement.  Must not be NULL.
 * @return ESP_OK on success, ESP_ERR_INVALID_ARG if out_data is NULL,
 *         ESP_ERR_INVALID_CRC if either CRC check fails, or an I2C error code.
 */
esp_err_t sht30_read(sht30_data_t *out_data);

/**
 * @brief Deinitialize the SHT30 driver and release the I2C bus.
 *
 * @return ESP_OK on success, or a driver error code on failure.
 */
esp_err_t sht30_deinit(void);

#ifdef __cplusplus
}
#endif

#endif /* SHT30_H */
