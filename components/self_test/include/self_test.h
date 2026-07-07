#ifndef SELF_TEST_H
#define SELF_TEST_H

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Run a visible and serial-printable hardware self-test sequence.
 *
 * The sequence verifies ST7789 color fill, backlight PWM, SHT30 readings, and
 * LD2410C reporting frames. It assumes peripheral drivers are already
 * initialized by app_core_init().
 *
 * @return ESP_OK when the self-test sequence completes, or the last peripheral
 *         error observed during the sequence.
 */
esp_err_t self_test_run(void);

#ifdef __cplusplus
}
#endif

#endif
