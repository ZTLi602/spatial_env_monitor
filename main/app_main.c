/**
 * @file app_main.c
 * @brief Firmware entry point for staged hardware validation and Phase2 sampling.
 *
 * Startup first disables the onboard WS2812 on GPIO48, then initializes all
 * project peripherals, runs a visible self-test, and finally starts the static
 * acquisition task. The main loop only prints the latest fused sample and does
 * not directly access sensor buses.
 */
#include "driver/gpio.h"
#include "driver/rmt_tx.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "app_core.h"
#include "self_test.h"

static const char *TAG = "main";

#define WS2812_GPIO         48
#define RMT_RESOLUTION_HZ   10000000
#define T0H                 4
#define T0L                 8
#define T1H                 8
#define T1L                 4
#define TRESET              600

/**
 * @brief Send an all-off GRB frame to the onboard WS2812 LED.
 */
static void board_led_off(void)
{
    rmt_channel_handle_t chan = NULL;
    rmt_encoder_handle_t enc = NULL;

    rmt_tx_channel_config_t chan_cfg = {
        .gpio_num = WS2812_GPIO,
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = RMT_RESOLUTION_HZ,
        .mem_block_symbols = 64,
        .trans_queue_depth = 1,
    };
    if (rmt_new_tx_channel(&chan_cfg, &chan) != ESP_OK) {
        return;
    }

    rmt_copy_encoder_config_t enc_cfg = {};
    if (rmt_new_copy_encoder(&enc_cfg, &enc) != ESP_OK) {
        rmt_del_channel(chan);
        return;
    }

    static rmt_symbol_word_t symbols[25];
    for (int i = 0; i < 24; i++) {
        symbols[i].level0 = 1;
        symbols[i].duration0 = T0H;
        symbols[i].level1 = 0;
        symbols[i].duration1 = T0L;
    }
    symbols[24].level0 = 0;
    symbols[24].duration0 = TRESET;
    symbols[24].level1 = 0;
    symbols[24].duration1 = TRESET;

    rmt_transmit_config_t tx_cfg = { .loop_count = 0 };
    rmt_enable(chan);
    rmt_transmit(chan, enc, symbols, sizeof(symbols), &tx_cfg);
    rmt_tx_wait_all_done(chan, pdMS_TO_TICKS(50));

    rmt_disable(chan);
    rmt_del_encoder(enc);
    rmt_del_channel(chan);
}

void app_main(void)
{
    board_led_off();

    ESP_LOGI(TAG, "=== Hardware self-test + Phase2 acquisition ===");

    esp_err_t ret = app_core_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "app_core_init failed: %s", esp_err_to_name(ret));
        return;
    }

    ret = self_test_run();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "self_test_run completed with warning: %s", esp_err_to_name(ret));
    }

    ret = app_core_start_acquisition();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "app_core_start_acquisition failed: %s", esp_err_to_name(ret));
        return;
    }

    while (true) {
        app_core_sample_t sample = {0};
        ret = app_core_get_latest_sample(&sample);
        if (ret == ESP_OK) {
            if (sample.env_status == ESP_OK && sample.env.valid) {
                ESP_LOGI(TAG, "#%lu SHT30: T=%.2f C  RH=%.2f %%",
                         (unsigned long)sample.sequence,
                         sample.env.temperature_c,
                         sample.env.humidity_percent);
            } else {
                ESP_LOGW(TAG, "#%lu SHT30 read failed: %s",
                         (unsigned long)sample.sequence,
                         esp_err_to_name(sample.env_status));
            }

            if (sample.radar_status == ESP_OK) {
                if (sample.radar.has_target) {
                    ESP_LOGI(TAG, "#%lu LD2410C: state=%d  move=%u cm (%u)  static=%u cm (%u)",
                             (unsigned long)sample.sequence,
                             sample.radar.target_state,
                             sample.radar.moving_distance_cm,
                             sample.radar.moving_energy,
                             sample.radar.static_distance_cm,
                             sample.radar.static_energy);
                } else {
                    ESP_LOGI(TAG, "#%lu LD2410C: no target", (unsigned long)sample.sequence);
                }
            } else {
                ESP_LOGW(TAG, "#%lu LD2410C read failed: %s",
                         (unsigned long)sample.sequence,
                         esp_err_to_name(sample.radar_status));
            }
        } else if (ret != ESP_ERR_NOT_FOUND) {
            ESP_LOGW(TAG, "get latest sample failed: %s", esp_err_to_name(ret));
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
