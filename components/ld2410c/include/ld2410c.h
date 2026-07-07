#ifndef LD2410C_H
#define LD2410C_H

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ---- Pin assignments (cursorrules §3) ------------------------------------ */
#define LD2410C_PIN_TX   17   /**< ESP32-S3 TX → radar RX */
#define LD2410C_PIN_RX   18   /**< ESP32-S3 RX ← radar TX */

/* ---- UART configuration -------------------------------------------------- */
#define LD2410C_UART_PORT    UART_NUM_1
#define LD2410C_UART_BAUD    256000
#define LD2410C_UART_BUF_SZ  256U   /**< RX ring-buffer size (static) */

/* ---- Protocol constants -------------------------------------------------- */
/** Frame header: 0xF4 0xF3 0xF2 0xF1 */
#define LD2410C_FRAME_HEADER  0xF4F3F2F1UL
/** Frame tail:   0xF8 0xF7 0xF6 0xF5 */
#define LD2410C_FRAME_TAIL    0xF8F7F6F5UL

/** Maximum raw frame length the parser will accept */
#define LD2410C_FRAME_MAX_LEN 64U

/**
 * @brief Target presence state decoded from a reporting frame.
 */
typedef enum {
    LD2410C_TARGET_NONE    = 0x00, /**< No target detected          */
    LD2410C_TARGET_MOVING  = 0x01, /**< Moving target only          */
    LD2410C_TARGET_STATIC  = 0x02, /**< Static target only          */
    LD2410C_TARGET_BOTH    = 0x03, /**< Both moving and static      */
} ld2410c_target_state_t;

/**
 * @brief Parsed output from one LD2410C reporting frame.
 *
 * Matches the 6-dimensional TFLM model input (cursorrules §4):
 *   [moving_distance_cm, moving_energy, static_distance_cm, static_energy]
 * Temperature and humidity come from SHT30 and are not part of this struct.
 */
typedef struct {
    bool                   has_target;          /**< true when target_state != NONE  */
    ld2410c_target_state_t target_state;         /**< Presence type                   */
    uint16_t               moving_distance_cm;  /**< Moving target distance, cm       */
    uint8_t                moving_energy;        /**< Moving target energy, 0–100      */
    uint16_t               static_distance_cm;  /**< Static target distance, cm       */
    uint8_t                static_energy;        /**< Static target energy, 0–100      */
} ld2410c_data_t;

/**
 * @brief Initialize the LD2410C driver and UART peripheral.
 *
 * Installs the UART driver on LD2410C_UART_PORT, configures baud rate and
 * GPIO assignments from the pin table, and waits for the sensor to become
 * ready.  Must be called before ld2410c_read().
 *
 * @return ESP_OK on success, or a driver/HAL error code on failure.
 */
esp_err_t ld2410c_init(void);

/**
 * @brief Read and parse one reporting frame from the LD2410C.
 *
 * Blocks until a valid frame is received or a 200 ms timeout elapses.
 * Frame header and tail are verified; malformed frames are discarded.
 *
 * @param[out] out_data  Destination for the parsed data.  Must not be NULL.
 * @return ESP_OK on success, ESP_ERR_INVALID_ARG if out_data is NULL,
 *         ESP_ERR_TIMEOUT if no valid frame arrives within the timeout,
 *         ESP_ERR_INVALID_RESPONSE if the frame structure is unexpected.
 */
esp_err_t ld2410c_read(ld2410c_data_t *out_data);

/**
 * @brief Deinitialize the LD2410C driver and release UART resources.
 *
 * @return ESP_OK on success, or a driver error code on failure.
 */
esp_err_t ld2410c_deinit(void);

#ifdef __cplusplus
}
#endif

#endif /* LD2410C_H */
