/**
 * @file self_test.c
 * @brief Visible hardware self-test sequence for staged board bring-up.
 *
 * The tests are intentionally simple and observable: LCD color fills, backlight
 * brightness ramp, and serial logs for SHT30/LD2410C. This module is used while
 * hardware is still connected with Dupont wires, where visual confirmation is
 * often faster than debugging the final UI.
 */
#include "self_test.h"

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "ld2410c.h"
#include "sht30.h"
#include "st7789.h"

static const char *TAG = "self_test";

#define COLOR_RED    0xF800U
#define COLOR_GREEN  0x07E0U
#define COLOR_BLUE   0x001FU
#define COLOR_BLACK  0x0000U
#define COLOR_WHITE  0xFFFFU

#define COLOR_HOLD_MS       700U
#define BACKLIGHT_STEP_MS    20U
#define SENSOR_TEST_ROUNDS    5U
#define SENSOR_ROUND_MS     500U

/**
 * @brief Fill the LCD with one color and keep it visible for a short time.
 *
 * @param color RGB565 color to display.
 * @param name Human-readable color name for serial logging.
 * @return ESP_OK on success, or an ST7789 driver error code.
 */
static esp_err_t self_test_show_color(uint16_t color, const char *name)
{
    ESP_LOGI(TAG, "LCD color test: %s", name);
    esp_err_t ret = st7789_fill_rect(0, 0, ST7789_WIDTH, ST7789_HEIGHT, color);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "LCD color %s failed: %s", name, esp_err_to_name(ret));
        return ret;
    }

    vTaskDelay(pdMS_TO_TICKS(COLOR_HOLD_MS));
    return ESP_OK;
}

/**
 * @brief Run the LCD RGB color-fill test.
 *
 * @return ESP_OK on success, or an ST7789 driver error code.
 */
static esp_err_t self_test_lcd_colors(void)
{
    esp_err_t last = ESP_OK;
    esp_err_t ret;

    ret = self_test_show_color(COLOR_RED, "RED");
    if (ret != ESP_OK) last = ret;

    ret = self_test_show_color(COLOR_GREEN, "GREEN");
    if (ret != ESP_OK) last = ret;

    ret = self_test_show_color(COLOR_BLUE, "BLUE");
    if (ret != ESP_OK) last = ret;

    ret = self_test_show_color(COLOR_WHITE, "WHITE");
    if (ret != ESP_OK) last = ret;

    ret = self_test_show_color(COLOR_BLACK, "BLACK");
    if (ret != ESP_OK) last = ret;

    return last;
}

/**
 * @brief Run a backlight brightness ramp test.
 *
 * @return ESP_OK on success, or an ST7789 backlight driver error code.
 */
static esp_err_t self_test_backlight(void)
{
    esp_err_t last = ESP_OK;

    ESP_LOGI(TAG, "Backlight test: dim -> bright -> dim");
    for (uint16_t duty = 0; duty <= 255U; duty += 15U) {
        esp_err_t ret = st7789_set_backlight((uint8_t)duty);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "set backlight %u failed: %s", duty, esp_err_to_name(ret));
            last = ret;
        }
        vTaskDelay(pdMS_TO_TICKS(BACKLIGHT_STEP_MS));
    }

    for (int duty = 255; duty >= 0; duty -= 15) {
        esp_err_t ret = st7789_set_backlight((uint8_t)duty);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "set backlight %d failed: %s", duty, esp_err_to_name(ret));
            last = ret;
        }
        vTaskDelay(pdMS_TO_TICKS(BACKLIGHT_STEP_MS));
    }

    esp_err_t ret = st7789_set_backlight(255);
    if (ret != ESP_OK) {
        last = ret;
    }
    return last;
}

/**
 * @brief Print one SHT30 and LD2410C sample for hardware verification.
 *
 * @param round One-based self-test round number.
 * @return ESP_OK when both sensors read successfully, otherwise the last error.
 */
static esp_err_t self_test_sensor_round(uint32_t round)
{
    esp_err_t last = ESP_OK;
    sht30_data_t sht = {0};
    ld2410c_data_t radar = {0};

    esp_err_t ret = sht30_read(&sht);
    if (ret == ESP_OK && sht.valid) {
        ESP_LOGI(TAG, "Round %lu SHT30 OK: T=%.2f C, RH=%.2f %%",
                 (unsigned long)round, sht.temperature_c, sht.humidity_percent);
    } else {
        ESP_LOGW(TAG, "Round %lu SHT30 FAIL: %s", (unsigned long)round, esp_err_to_name(ret));
        last = ret;
    }

    ret = ld2410c_read(&radar);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Round %lu LD2410C OK: has=%d state=%d move=%u cm energy=%u static=%u cm energy=%u",
                 (unsigned long)round,
                 radar.has_target,
                 radar.target_state,
                 radar.moving_distance_cm,
                 radar.moving_energy,
                 radar.static_distance_cm,
                 radar.static_energy);
    } else {
        ESP_LOGW(TAG, "Round %lu LD2410C FAIL: %s", (unsigned long)round, esp_err_to_name(ret));
        last = ret;
    }

    return last;
}

/**
 * @brief Run repeated sensor serial-output tests.
 *
 * @return ESP_OK when all sensor reads succeed, otherwise the last observed error.
 */
static esp_err_t self_test_sensors(void)
{
    esp_err_t last = ESP_OK;

    ESP_LOGI(TAG, "Sensor test: %u rounds", SENSOR_TEST_ROUNDS);
    for (uint32_t round = 1; round <= SENSOR_TEST_ROUNDS; round++) {
        esp_err_t ret = self_test_sensor_round(round);
        if (ret != ESP_OK) {
            last = ret;
        }
        vTaskDelay(pdMS_TO_TICKS(SENSOR_ROUND_MS));
    }

    return last;
}

esp_err_t self_test_run(void)
{
    esp_err_t last = ESP_OK;
    esp_err_t ret;

    ESP_LOGI(TAG, "========== HARDWARE SELF TEST START ==========");
    ESP_LOGI(TAG, "Expected LCD: RED -> GREEN -> BLUE -> WHITE -> BLACK, then backlight breathing");
    ESP_LOGI(TAG, "Expected serial: SHT30 temperature/humidity and LD2410C target frames");

    ret = self_test_lcd_colors();
    if (ret != ESP_OK) last = ret;

    ret = self_test_backlight();
    if (ret != ESP_OK) last = ret;

    ret = self_test_sensors();
    if (ret != ESP_OK) last = ret;

    ret = st7789_fill_rect(0, 0, ST7789_WIDTH, ST7789_HEIGHT, COLOR_BLUE);
    if (ret != ESP_OK) {
        last = ret;
    }

    if (last == ESP_OK) {
        ESP_LOGI(TAG, "========== HARDWARE SELF TEST PASS ==========");
    } else {
        ESP_LOGW(TAG, "========== HARDWARE SELF TEST DONE WITH WARNINGS: %s ==========",
                 esp_err_to_name(last));
    }

    return last;
}
