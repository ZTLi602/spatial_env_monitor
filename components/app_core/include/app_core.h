#ifndef APP_CORE_H
#define APP_CORE_H

/**
 * @file app_core.h
 * @brief Application core API for initialization and fused sample access.
 */

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"
#include "ld2410c.h"
#include "sht30.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Number of fused sensor samples retained in the static ring buffer. */
#define APP_CORE_SAMPLE_BUFFER_LEN  32U

/**
 * @brief One fused environmental and radar sample.
 */
typedef struct {
    sht30_data_t   env;              /**< Latest SHT30 measurement. */
    ld2410c_data_t radar;            /**< Latest LD2410C reporting frame. */
    esp_err_t      env_status;       /**< Result of sht30_read(). */
    esp_err_t      radar_status;     /**< Result of ld2410c_read(). */
    uint32_t       sequence;         /**< Monotonic sample sequence number. */
    uint32_t       timestamp_ms;     /**< FreeRTOS tick time converted to ms. */
    bool           valid;            /**< true once this slot has been written. */
} app_core_sample_t;

/**
 * @brief Initialize the application core modules.
 *
 * @return ESP_OK on success, or an error code on failure.
 */
esp_err_t app_core_init(void);

/**
 * @brief Start the 100 ms sensor acquisition task.
 *
 * Creates a statically allocated FreeRTOS task that samples SHT30 and LD2410C,
 * then stores fused records in the internal static ring buffer.
 *
 * @return ESP_OK on success, ESP_ERR_INVALID_STATE if the task is already running.
 */
esp_err_t app_core_start_acquisition(void);

/**
 * @brief Copy the latest fused sample from the acquisition ring buffer.
 *
 * @param[out] out_sample Destination for the latest sample. Must not be NULL.
 * @return ESP_OK on success, ESP_ERR_INVALID_ARG if out_sample is NULL,
 *         ESP_ERR_NOT_FOUND if no sample has been collected yet.
 */
esp_err_t app_core_get_latest_sample(app_core_sample_t *out_sample);

#ifdef __cplusplus
}
#endif

#endif
