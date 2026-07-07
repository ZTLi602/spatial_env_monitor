/**
 * @file sht30.c
 * @brief SHT30 temperature/humidity sensor driver using ESP-IDF I2C master.
 *
 * The driver performs single-shot high-repeatability measurements and validates
 * both temperature and humidity words with the SHT30 CRC-8. The I2C bus is kept
 * at the fixed project pins to avoid ESP32-S3 strapping-pin risks.
 */
#include "sht30.h"

#include <string.h>

#include "driver/i2c.h"
#include "esp_check.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "sht30";

/* SHT30 commands (MSB first) */
#define SHT30_CMD_SINGLE_SHOT_HIGH  0x2400U  /* single-shot, high repeatability, no clock stretch */
#define SHT30_CMD_SOFT_RESET        0x30A2U

/* Conversion timing */
#define SHT30_MEASURE_DELAY_MS      20U      /* datasheet max 15.5 ms, add margin */

/* Response frame: 2 bytes T + 1 CRC + 2 bytes RH + 1 CRC = 6 bytes */
#define SHT30_RESP_LEN              6U

/* CRC-8 polynomial: x^8 + x^5 + x^4 + 1 (0x31), init 0xFF */
#define SHT30_CRC_POLY              0x31U
#define SHT30_CRC_INIT              0xFFU

static bool s_initialized = false;

/* -------------------------------------------------------------------------- */
/* Internal helpers                                                            */
/* -------------------------------------------------------------------------- */

/**
 * @brief Compute CRC-8 over one data byte appended to a running CRC.
 *
 * @param crc   Current CRC accumulator (initialise with SHT30_CRC_INIT).
 * @param byte  Data byte to process.
 * @return Updated CRC value.
 */
static uint8_t sht30_crc8_byte(uint8_t crc, uint8_t byte)
{
    crc ^= byte;
    for (int i = 0; i < 8; i++) {
        if (crc & 0x80U) {
            crc = (uint8_t)((crc << 1U) ^ SHT30_CRC_POLY);
        } else {
            crc <<= 1U;
        }
    }
    return crc;
}

/**
 * @brief Compute CRC-8 over a two-byte word.
 *
 * @param msb  Most significant byte.
 * @param lsb  Least significant byte.
 * @return Computed CRC.
 */
static uint8_t sht30_crc8(uint8_t msb, uint8_t lsb)
{
    uint8_t crc = SHT30_CRC_INIT;
    crc = sht30_crc8_byte(crc, msb);
    crc = sht30_crc8_byte(crc, lsb);
    return crc;
}

/**
 * @brief Write a 16-bit command word to the sensor.
 *
 * @param cmd  Command word (big-endian on wire).
 * @return ESP_OK or an I2C driver error.
 */
static esp_err_t sht30_write_cmd(uint16_t cmd)
{
    uint8_t buf[2] = {
        (uint8_t)(cmd >> 8U),
        (uint8_t)(cmd & 0xFFU),
    };

    i2c_cmd_handle_t handle = i2c_cmd_link_create();
    ESP_RETURN_ON_FALSE(handle != NULL, ESP_ERR_NO_MEM, TAG, "i2c_cmd_link_create failed");

    esp_err_t ret = ESP_OK;
    ret |= i2c_master_start(handle);
    ret |= i2c_master_write_byte(handle, (SHT30_I2C_ADDR_DEFAULT << 1U) | I2C_MASTER_WRITE, true);
    ret |= i2c_master_write(handle, buf, sizeof(buf), true);
    ret |= i2c_master_stop(handle);

    if (ret == ESP_OK) {
        ret = i2c_master_cmd_begin(SHT30_I2C_PORT, handle, pdMS_TO_TICKS(100));
    }

    i2c_cmd_link_delete(handle);
    return ret;
}

/**
 * @brief Read `len` bytes from the sensor into `buf`.
 *
 * @param buf  Destination buffer.
 * @param len  Number of bytes to read.
 * @return ESP_OK or an I2C driver error.
 */
static esp_err_t sht30_read_bytes(uint8_t *buf, size_t len)
{
    i2c_cmd_handle_t handle = i2c_cmd_link_create();
    ESP_RETURN_ON_FALSE(handle != NULL, ESP_ERR_NO_MEM, TAG, "i2c_cmd_link_create failed");

    esp_err_t ret = ESP_OK;
    ret |= i2c_master_start(handle);
    ret |= i2c_master_write_byte(handle, (SHT30_I2C_ADDR_DEFAULT << 1U) | I2C_MASTER_READ, true);
    ret |= i2c_master_read(handle, buf, len, I2C_MASTER_LAST_NACK);
    ret |= i2c_master_stop(handle);

    if (ret == ESP_OK) {
        ret = i2c_master_cmd_begin(SHT30_I2C_PORT, handle, pdMS_TO_TICKS(100));
    }

    i2c_cmd_link_delete(handle);
    return ret;
}

/* -------------------------------------------------------------------------- */
/* Public API                                                                  */
/* -------------------------------------------------------------------------- */

esp_err_t sht30_init(void)
{
    if (s_initialized) {
        ESP_LOGW(TAG, "already initialized");
        return ESP_OK;
    }

    i2c_config_t conf = {
        .mode             = I2C_MODE_MASTER,
        .sda_io_num       = SHT30_PIN_SDA,
        .scl_io_num       = SHT30_PIN_SCL,
        .sda_pullup_en    = GPIO_PULLUP_ENABLE,
        .scl_pullup_en    = GPIO_PULLUP_ENABLE,
        .master.clk_speed = SHT30_I2C_FREQ_HZ,
    };

    ESP_RETURN_ON_ERROR(i2c_param_config(SHT30_I2C_PORT, &conf),
                        TAG, "i2c_param_config failed");
    ESP_RETURN_ON_ERROR(i2c_driver_install(SHT30_I2C_PORT, I2C_MODE_MASTER, 0, 0, 0),
                        TAG, "i2c_driver_install failed");

    /* Soft-reset: sensor restores factory defaults and clears status register */
    esp_err_t ret = sht30_write_cmd(SHT30_CMD_SOFT_RESET);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "soft-reset failed: %s", esp_err_to_name(ret));
        i2c_driver_delete(SHT30_I2C_PORT);
        return ret;
    }

    vTaskDelay(pdMS_TO_TICKS(2)); /* datasheet: ≤1.5 ms restart time */

    s_initialized = true;
    ESP_LOGI(TAG, "initialized (SDA=GPIO%d, SCL=GPIO%d, %lu Hz)",
             SHT30_PIN_SDA, SHT30_PIN_SCL, (unsigned long)SHT30_I2C_FREQ_HZ);
    return ESP_OK;
}

esp_err_t sht30_read(sht30_data_t *out_data)
{
    ESP_RETURN_ON_FALSE(out_data != NULL, ESP_ERR_INVALID_ARG, TAG, "out_data is NULL");
    ESP_RETURN_ON_FALSE(s_initialized, ESP_ERR_INVALID_STATE, TAG, "not initialized");

    out_data->valid = false;

    /* Trigger measurement */
    ESP_RETURN_ON_ERROR(sht30_write_cmd(SHT30_CMD_SINGLE_SHOT_HIGH),
                        TAG, "trigger measurement failed");

    vTaskDelay(pdMS_TO_TICKS(SHT30_MEASURE_DELAY_MS));

    /* Read 6 bytes: [T_MSB, T_LSB, T_CRC, RH_MSB, RH_LSB, RH_CRC] */
    static uint8_t buf[SHT30_RESP_LEN];
    ESP_RETURN_ON_ERROR(sht30_read_bytes(buf, SHT30_RESP_LEN),
                        TAG, "read bytes failed");

    /* CRC check for temperature word */
    if (sht30_crc8(buf[0], buf[1]) != buf[2]) {
        ESP_LOGE(TAG, "temperature CRC mismatch (got 0x%02X, expected 0x%02X)",
                 buf[2], sht30_crc8(buf[0], buf[1]));
        return ESP_ERR_INVALID_CRC;
    }

    /* CRC check for humidity word */
    if (sht30_crc8(buf[3], buf[4]) != buf[5]) {
        ESP_LOGE(TAG, "humidity CRC mismatch (got 0x%02X, expected 0x%02X)",
                 buf[5], sht30_crc8(buf[3], buf[4]));
        return ESP_ERR_INVALID_CRC;
    }

    /* Convert raw counts to physical units (SHT30 datasheet §4.13) */
    uint16_t raw_t  = ((uint16_t)buf[0] << 8U) | buf[1];
    uint16_t raw_rh = ((uint16_t)buf[3] << 8U) | buf[4];

    out_data->temperature_c    = -45.0f + 175.0f * ((float)raw_t  / 65535.0f);
    out_data->humidity_percent =           100.0f * ((float)raw_rh / 65535.0f);
    out_data->valid = true;

    return ESP_OK;
}

esp_err_t sht30_deinit(void)
{
    if (!s_initialized) {
        return ESP_OK;
    }
    ESP_RETURN_ON_ERROR(i2c_driver_delete(SHT30_I2C_PORT),
                        TAG, "i2c_driver_delete failed");
    s_initialized = false;
    ESP_LOGI(TAG, "deinitialized");
    return ESP_OK;
}
