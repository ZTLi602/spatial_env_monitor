#include "st7789.h"

#include <string.h>

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/spi_master.h"
#include "esp_check.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "st7789";

/* ---- ST7789 command bytes ------------------------------------------------ */
#define ST7789_CMD_SWRESET  0x01U
#define ST7789_CMD_SLPOUT   0x11U
#define ST7789_CMD_NORON    0x13U
#define ST7789_CMD_INVON    0x21U
#define ST7789_CMD_DISPON   0x29U
#define ST7789_CMD_CASET    0x2AU
#define ST7789_CMD_RASET    0x2BU
#define ST7789_CMD_RAMWR    0x2CU
#define ST7789_CMD_COLMOD   0x3AU
#define ST7789_CMD_MADCTL   0x36U

/* RGB565, 16 bpp */
#define ST7789_COLMOD_16BPP  0x55U
/* Row/column order: MX=1, MV=1 → landscape; adjust if portrait needed */
#define ST7789_MADCTL_VAL    0x00U

/* LEDC channel for backlight PWM */
#define BLK_LEDC_TIMER    LEDC_TIMER_0
#define BLK_LEDC_CHANNEL  LEDC_CHANNEL_0
#define BLK_LEDC_FREQ_HZ  5000U
#define BLK_LEDC_RES      LEDC_TIMER_8_BIT  /* 0–255 duty */

/* SPI transaction pixel chunk: 128 px × 2 bytes = 256 bytes, fits in IRAM */
#define PIXEL_CHUNK_SIZE  128U

static spi_device_handle_t s_spi   = NULL;
static bool                s_initialized = false;

/* Static line buffer used by st7789_fill_rect – no heap allocation */
static uint16_t s_pixel_buf[PIXEL_CHUNK_SIZE];

/* -------------------------------------------------------------------------- */
/* Internal helpers                                                            */
/* -------------------------------------------------------------------------- */

/**
 * @brief Assert/deassert DC pin outside of a transaction (command=0, data=1).
 */
static inline void set_dc(int level)
{
    gpio_set_level(ST7789_PIN_DC, level);
}

/**
 * @brief Send a single command byte (DC=0).
 */
static esp_err_t st7789_send_cmd(uint8_t cmd)
{
    spi_transaction_t t = {
        .length    = 8,
        .tx_buffer = &cmd,
        .user      = (void *)0,  /* DC=0 */
    };
    set_dc(0);
    return spi_device_polling_transmit(s_spi, &t);
}

/**
 * @brief Send a data buffer (DC=1).
 */
static esp_err_t st7789_send_data(const uint8_t *data, size_t len)
{
    if (len == 0) {
        return ESP_OK;
    }
    spi_transaction_t t = {
        .length    = len * 8,
        .tx_buffer = data,
        .user      = (void *)1,  /* DC=1 */
    };
    set_dc(1);
    return spi_device_polling_transmit(s_spi, &t);
}

/**
 * @brief Set the column/row address window for subsequent RAMWR.
 */
static esp_err_t st7789_set_window(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1)
{
    uint8_t buf[4];
    esp_err_t ret;

    buf[0] = (uint8_t)(x0 >> 8U);
    buf[1] = (uint8_t)(x0 & 0xFFU);
    buf[2] = (uint8_t)(x1 >> 8U);
    buf[3] = (uint8_t)(x1 & 0xFFU);
    ret = st7789_send_cmd(ST7789_CMD_CASET);
    if (ret != ESP_OK) return ret;
    ret = st7789_send_data(buf, 4);
    if (ret != ESP_OK) return ret;

    buf[0] = (uint8_t)(y0 >> 8U);
    buf[1] = (uint8_t)(y0 & 0xFFU);
    buf[2] = (uint8_t)(y1 >> 8U);
    buf[3] = (uint8_t)(y1 & 0xFFU);
    ret = st7789_send_cmd(ST7789_CMD_RASET);
    if (ret != ESP_OK) return ret;
    return st7789_send_data(buf, 4);
}

/**
 * @brief Execute the power-on / reset initialisation sequence.
 */
static esp_err_t st7789_run_init_sequence(void)
{
    /* Hardware reset: RST low ≥10 ms, then high, wait 120 ms */
    gpio_set_level(ST7789_PIN_RST, 0);
    vTaskDelay(pdMS_TO_TICKS(20));
    gpio_set_level(ST7789_PIN_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(120));

    static const struct { uint8_t cmd; uint8_t data; bool has_data; uint16_t delay_ms; } seq[] = {
        { ST7789_CMD_SWRESET, 0,                    false, 150 },
        { ST7789_CMD_SLPOUT,  0,                    false, 10  },
        { ST7789_CMD_COLMOD,  ST7789_COLMOD_16BPP,  true,  0   },
        { ST7789_CMD_MADCTL,  ST7789_MADCTL_VAL,    true,  0   },
        { ST7789_CMD_INVON,   0,                    false, 0   },
        { ST7789_CMD_NORON,   0,                    false, 0   },
        { ST7789_CMD_DISPON,  0,                    false, 10  },
    };

    for (size_t i = 0; i < sizeof(seq) / sizeof(seq[0]); i++) {
        ESP_RETURN_ON_ERROR(st7789_send_cmd(seq[i].cmd), TAG, "init cmd 0x%02X failed", seq[i].cmd);
        if (seq[i].has_data) {
            ESP_RETURN_ON_ERROR(st7789_send_data(&seq[i].data, 1), TAG, "init data failed");
        }
        if (seq[i].delay_ms) {
            vTaskDelay(pdMS_TO_TICKS(seq[i].delay_ms));
        }
    }
    return ESP_OK;
}

/* -------------------------------------------------------------------------- */
/* Public API                                                                  */
/* -------------------------------------------------------------------------- */

esp_err_t st7789_init(void)
{
    if (s_initialized) {
        ESP_LOGW(TAG, "already initialized");
        return ESP_OK;
    }

    /* Configure DC and RST as plain outputs */
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << ST7789_PIN_DC) | (1ULL << ST7789_PIN_RST),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&io_conf), TAG, "gpio_config failed");

    /*
     * Reduce drive strength on SPI data lines to limit overshoot/ringing on
     * dupont wires (cursorrules §5).
     */
    gpio_set_drive_capability(ST7789_PIN_SCK,  GPIO_DRIVE_CAP_1);
    gpio_set_drive_capability(ST7789_PIN_MOSI, GPIO_DRIVE_CAP_1);
    gpio_set_drive_capability(ST7789_PIN_CS,   GPIO_DRIVE_CAP_1);
    gpio_set_drive_capability(ST7789_PIN_DC,   GPIO_DRIVE_CAP_1);

    /* SPI bus init */
    spi_bus_config_t bus_cfg = {
        .mosi_io_num     = ST7789_PIN_MOSI,
        .miso_io_num     = -1,
        .sclk_io_num     = ST7789_PIN_SCK,
        .quadwp_io_num   = -1,
        .quadhd_io_num   = -1,
        .max_transfer_sz = PIXEL_CHUNK_SIZE * 2,
    };
    ESP_RETURN_ON_ERROR(spi_bus_initialize(ST7789_SPI_HOST, &bus_cfg, SPI_DMA_CH_AUTO),
                        TAG, "spi_bus_initialize failed");

    /*
     * Half-duplex mode (cursorrules §5): MISO unused, no full-duplex needed.
     * CS managed by the driver (cs_ena_pretrans / posttrans not needed here).
     */
    spi_device_interface_config_t dev_cfg = {
        .clock_speed_hz = ST7789_SPI_FREQ_HZ,
        .mode           = 0,
        .spics_io_num   = ST7789_PIN_CS,
        .queue_size     = 7,
        .flags          = SPI_DEVICE_HALFDUPLEX | SPI_DEVICE_NO_DUMMY,
    };
    ESP_RETURN_ON_ERROR(spi_bus_add_device(ST7789_SPI_HOST, &dev_cfg, &s_spi),
                        TAG, "spi_bus_add_device failed");

    /* Backlight PWM */
    ledc_timer_config_t ledc_timer = {
        .speed_mode      = LEDC_LOW_SPEED_MODE,
        .duty_resolution = BLK_LEDC_RES,
        .timer_num       = BLK_LEDC_TIMER,
        .freq_hz         = BLK_LEDC_FREQ_HZ,
        .clk_cfg         = LEDC_AUTO_CLK,
    };
    ESP_RETURN_ON_ERROR(ledc_timer_config(&ledc_timer), TAG, "ledc_timer_config failed");

    ledc_channel_config_t ledc_ch = {
        .gpio_num   = ST7789_PIN_BLK,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel    = BLK_LEDC_CHANNEL,
        .timer_sel  = BLK_LEDC_TIMER,
        .duty       = 0,
        .hpoint     = 0,
    };
    ESP_RETURN_ON_ERROR(ledc_channel_config(&ledc_ch), TAG, "ledc_channel_config failed");

    /* Display init sequence */
    ESP_RETURN_ON_ERROR(st7789_run_init_sequence(), TAG, "init sequence failed");

    /* Backlight on at full brightness */
    ESP_RETURN_ON_ERROR(st7789_set_backlight(255), TAG, "set_backlight failed");

    s_initialized = true;
    ESP_LOGI(TAG, "initialized (%ux%u, SPI %d MHz, HALFDUPLEX)",
             ST7789_WIDTH, ST7789_HEIGHT, ST7789_SPI_FREQ_HZ / 1000000);
    return ESP_OK;
}

esp_err_t st7789_fill_rect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color)
{
    ESP_RETURN_ON_FALSE(s_initialized, ESP_ERR_INVALID_STATE, TAG, "not initialized");
    if (w == 0 || h == 0) {
        return ESP_OK;
    }

    ESP_RETURN_ON_ERROR(st7789_set_window(x, y, x + w - 1, y + h - 1),
                        TAG, "set_window failed");
    ESP_RETURN_ON_ERROR(st7789_send_cmd(ST7789_CMD_RAMWR), TAG, "RAMWR failed");

    /* Pre-fill chunk buffer with the colour (big-endian RGB565) */
    uint16_t be_color = (color >> 8U) | (color << 8U);
    for (size_t i = 0; i < PIXEL_CHUNK_SIZE; i++) {
        s_pixel_buf[i] = be_color;
    }

    uint32_t total  = (uint32_t)w * h;
    uint32_t offset = 0;

    while (offset < total) {
        uint32_t chunk = total - offset;
        if (chunk > PIXEL_CHUNK_SIZE) {
            chunk = PIXEL_CHUNK_SIZE;
        }
        ESP_RETURN_ON_ERROR(
            st7789_send_data((const uint8_t *)s_pixel_buf, chunk * 2),
            TAG, "send pixel data failed");
        offset += chunk;
    }
    return ESP_OK;
}

esp_err_t st7789_set_backlight(uint8_t duty)
{
    ESP_RETURN_ON_ERROR(
        ledc_set_duty(LEDC_LOW_SPEED_MODE, BLK_LEDC_CHANNEL, duty),
        TAG, "ledc_set_duty failed");
    ESP_RETURN_ON_ERROR(
        ledc_update_duty(LEDC_LOW_SPEED_MODE, BLK_LEDC_CHANNEL),
        TAG, "ledc_update_duty failed");
    return ESP_OK;
}

esp_err_t st7789_deinit(void)
{
    if (!s_initialized) {
        return ESP_OK;
    }
    st7789_set_backlight(0);
    spi_bus_remove_device(s_spi);
    s_spi = NULL;
    spi_bus_free(ST7789_SPI_HOST);
    s_initialized = false;
    ESP_LOGI(TAG, "deinitialized");
    return ESP_OK;
}
