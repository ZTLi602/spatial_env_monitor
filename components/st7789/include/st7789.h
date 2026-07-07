#ifndef ST7789_H
#define ST7789_H

#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ---- Pin assignments (cursorrules §3) ------------------------------------ */
#define ST7789_PIN_SCK   12
#define ST7789_PIN_MOSI  11
#define ST7789_PIN_DC    5
#define ST7789_PIN_RST   6
#define ST7789_PIN_CS    38
#define ST7789_PIN_BLK   2

/* ---- Display geometry ---------------------------------------------------- */
#define ST7789_WIDTH     240U
#define ST7789_HEIGHT    320U

/* ---- SPI configuration --------------------------------------------------- */
/** SPI host used for the display */
#define ST7789_SPI_HOST  SPI2_HOST

/**
 * Clock frequency for the SPI bus.
 * Kept at 20 MHz as a safe default for dupont-wire connections; reduce to
 * 10–15 MHz if visual artefacts appear (cursorrules §6 Phase3).
 */
#define ST7789_SPI_FREQ_HZ  (20 * 1000 * 1000)

/**
 * @brief Initialize the ST7789 display driver.
 *
 * Configures the SPI bus (SPI2_HOST) in half-duplex mode with reduced GPIO
 * drive strength to suppress reflections on dupont wires, performs a hardware
 * reset sequence, and sets the display into normal-on mode with RGB565 pixel
 * format.  The backlight is driven via GPIO PWM at full brightness.
 *
 * @return ESP_OK on success, or a driver/HAL error code on failure.
 */
esp_err_t st7789_init(void);

/**
 * @brief Fill a rectangular region of the display with a single RGB565 colour.
 *
 * @param x      Left edge (0-based, inclusive).
 * @param y      Top edge (0-based, inclusive).
 * @param w      Width in pixels.
 * @param h      Height in pixels.
 * @param color  RGB565 colour value (big-endian on wire).
 * @return ESP_OK on success, or a driver error code on failure.
 */
esp_err_t st7789_fill_rect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color);

/**
 * @brief Set backlight brightness.
 *
 * @param duty  Brightness level 0–255 (0 = off, 255 = full).
 * @return ESP_OK on success, or a driver error code on failure.
 */
esp_err_t st7789_set_backlight(uint8_t duty);

/**
 * @brief Deinitialize the ST7789 driver and release SPI bus resources.
 *
 * @return ESP_OK on success, or a driver error code on failure.
 */
esp_err_t st7789_deinit(void);

#ifdef __cplusplus
}
#endif

#endif /* ST7789_H */
