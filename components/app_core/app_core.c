#include "app_core.h"

#include <string.h>

#include "esp_check.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "ld2410c.h"
#include "sht30.h"
#include "st7789.h"

static const char *TAG = "app_core";

#define ACQ_TASK_NAME       "sensor_acq"
#define ACQ_TASK_STACK_WORDS 4096U
#define ACQ_TASK_PRIORITY    6U
#define ACQ_PERIOD_MS        100U

static app_core_sample_t s_sample_buf[APP_CORE_SAMPLE_BUFFER_LEN];
static uint32_t          s_write_index;
static uint32_t          s_sequence;
static bool              s_has_sample;

static StaticTask_t s_acq_task_tcb;
static StackType_t  s_acq_task_stack[ACQ_TASK_STACK_WORDS];
static TaskHandle_t s_acq_task_handle;
static portMUX_TYPE s_sample_lock = portMUX_INITIALIZER_UNLOCKED;

/**
 * @brief Store one fused sample in the static ring buffer.
 *
 * @param sample Source sample to copy into the next ring-buffer slot.
 */
static void app_core_store_sample(const app_core_sample_t *sample)
{
    taskENTER_CRITICAL(&s_sample_lock);
    s_sample_buf[s_write_index] = *sample;
    s_write_index = (s_write_index + 1U) % APP_CORE_SAMPLE_BUFFER_LEN;
    s_has_sample = true;
    taskEXIT_CRITICAL(&s_sample_lock);
}

/**
 * @brief FreeRTOS task that periodically samples SHT30 and LD2410C.
 *
 * @param arg Unused task argument.
 */
static void app_core_acquisition_task(void *arg)
{
    (void)arg;

    TickType_t last_wake = xTaskGetTickCount();

    while (true) {
        app_core_sample_t sample;
        memset(&sample, 0, sizeof(sample));

        sample.sequence = ++s_sequence;
        sample.timestamp_ms = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
        sample.env_status = sht30_read(&sample.env);
        sample.radar_status = ld2410c_read(&sample.radar);
        sample.valid = true;

        app_core_store_sample(&sample);

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(ACQ_PERIOD_MS));
    }
}

esp_err_t app_core_init(void)
{
    esp_err_t ret;

    memset(s_sample_buf, 0, sizeof(s_sample_buf));
    s_write_index = 0;
    s_sequence = 0;
    s_has_sample = false;

    ret = sht30_init();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "sht30_init failed (%s) – check I2C wiring", esp_err_to_name(ret));
    }

    ret = ld2410c_init();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "ld2410c_init failed (%s) – check UART wiring", esp_err_to_name(ret));
    }

    ret = st7789_init();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "st7789_init failed (%s) – check SPI wiring", esp_err_to_name(ret));
    }

    ESP_LOGI(TAG, "app_core_init complete (see warnings above for unconnected peripherals)");
    return ESP_OK;
}

esp_err_t app_core_start_acquisition(void)
{
    ESP_RETURN_ON_FALSE(s_acq_task_handle == NULL, ESP_ERR_INVALID_STATE,
                        TAG, "acquisition task already running");

    s_acq_task_handle = xTaskCreateStatic(app_core_acquisition_task,
                                          ACQ_TASK_NAME,
                                          ACQ_TASK_STACK_WORDS,
                                          NULL,
                                          ACQ_TASK_PRIORITY,
                                          s_acq_task_stack,
                                          &s_acq_task_tcb);
    ESP_RETURN_ON_FALSE(s_acq_task_handle != NULL, ESP_FAIL,
                        TAG, "xTaskCreateStatic failed");

    ESP_LOGI(TAG, "acquisition task started (%u ms period, %u-sample ring buffer)",
             ACQ_PERIOD_MS, APP_CORE_SAMPLE_BUFFER_LEN);
    return ESP_OK;
}

esp_err_t app_core_get_latest_sample(app_core_sample_t *out_sample)
{
    ESP_RETURN_ON_FALSE(out_sample != NULL, ESP_ERR_INVALID_ARG, TAG, "out_sample is NULL");

    taskENTER_CRITICAL(&s_sample_lock);
    if (!s_has_sample) {
        taskEXIT_CRITICAL(&s_sample_lock);
        return ESP_ERR_NOT_FOUND;
    }

    uint32_t latest_index = (s_write_index + APP_CORE_SAMPLE_BUFFER_LEN - 1U) % APP_CORE_SAMPLE_BUFFER_LEN;
    *out_sample = s_sample_buf[latest_index];
    taskEXIT_CRITICAL(&s_sample_lock);

    return ESP_OK;
}
