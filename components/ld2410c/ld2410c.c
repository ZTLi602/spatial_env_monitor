/**
 * @file ld2410c.c
 * @brief LD2410C UART transparent-mode driver for presence sensing.
 *
 * This driver only uses the official UART reporting stream. The OUT pin is not
 * sampled because it exposes only a coarse presence signal, while this project
 * needs moving/static distance and energy values for later data fusion and
 * inference. All buffers are statically allocated to comply with the project
 * memory rules.
 */
#include "ld2410c.h"

#include <string.h>

#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_check.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "ld2410c";
//  git测试版本
/* ---- Protocol offsets in the Data field of a Reporting Frame ------------- */
/*
 * LD2410C UART transparent-mode reporting frame layout (little-endian):
 *
 *  Byte  0- 3 : Frame header  0xF4 0xF3 0xF2 0xF1
 *  Byte  4- 5 : Data length   (uint16 LE)
 *  Byte  6    : Type          0x02 = reporting frame
 *  Byte  7    : Head          0xAA
 *  Byte  8    : Target state  0x00/0x01/0x02/0x03
 *  Byte  9-10 : Moving distance  (uint16 LE, cm)
 *  Byte 11    : Moving energy   (0-100)
 *  Byte 12-13 : Static distance (uint16 LE, cm)
 *  Byte 14    : Static energy   (0-100)
 *  Byte 15-16 : Detection distance (uint16 LE, cm) – informational
 *  Byte 17    : Tail marker    0x55
 *  Byte 18    : Check value    (unused by transparency mode, 0x00)
 *  Byte 19-22 : Frame tail     0xF8 0xF7 0xF6 0xF5
 */
#define FRAME_HDR_LEN          4U
#define FRAME_DATALEN_OFFSET   4U
#define FRAME_TYPE_OFFSET      6U
#define FRAME_HEAD_OFFSET      7U
#define FRAME_STATE_OFFSET     8U
#define FRAME_MOVE_DIST_OFFSET 9U
#define FRAME_MOVE_ENRG_OFFSET 11U
#define FRAME_STAT_DIST_OFFSET 12U
#define FRAME_STAT_ENRG_OFFSET 14U
#define FRAME_TAIL_MARKER_OFF  17U
#define FRAME_TAIL_OFFSET      19U
#define FRAME_TOTAL_LEN        23U

#define FRAME_DATA_LEN_EXPECTED 13U
#define FRAME_TYPE_REPORT       0x02U
#define FRAME_HEAD_MARKER       0xAAU
#define FRAME_TAIL_MARKER       0x55U

#define READ_TIMEOUT_MS         300U

static bool s_initialized = false;

static uint8_t s_rx_buf[LD2410C_FRAME_MAX_LEN];

/**
 * @brief Check the four-byte header or tail magic at a given buffer offset.
 *
 * @param buf Buffer containing frame data.
 * @param offset Byte offset to check.
 * @param magic Expected 32-bit magic, interpreted from little-endian bytes.
 * @return true if the bytes match the expected magic value.
 */
static bool check_magic(const uint8_t *buf, size_t offset, uint32_t magic)
{
    uint32_t word = (uint32_t)buf[offset]
                  | ((uint32_t)buf[offset + 1] << 8U)
                  | ((uint32_t)buf[offset + 2] << 16U)
                  | ((uint32_t)buf[offset + 3] << 24U);
    return word == magic;
}

/**
 * @brief Read exactly len bytes from UART before an absolute deadline.
 *
 * @param buf Destination buffer.
 * @param len Number of bytes to read.
 * @param deadline_tick Absolute FreeRTOS tick deadline.
 * @return ESP_OK on success, ESP_ERR_TIMEOUT on timeout, or ESP_FAIL on UART error.
 */
static esp_err_t uart_read_exact_until(uint8_t *buf, size_t len, TickType_t deadline_tick)
{
    size_t offset = 0;

    while (offset < len) {
        TickType_t now = xTaskGetTickCount();
        if ((int32_t)(deadline_tick - now) <= 0) {
            return ESP_ERR_TIMEOUT;
        }

        int received = uart_read_bytes(LD2410C_UART_PORT,
                                       &buf[offset],
                                       (uint32_t)(len - offset),
                                       deadline_tick - now);
        if (received < 0) {
            return ESP_FAIL;
        }
        if (received == 0) {
            continue;
        }

        offset += (size_t)received;
    }

    return ESP_OK;
}

/**
 * @brief Synchronize to F4 F3 F2 F1 and read one complete reporting frame.
 *
 * LD2410C reports a continuous byte stream, so a task may start reading from
 * the middle of a frame after boot, reset, or UART buffering. The parser first
 * searches the 4-byte magic header and only then reads the fixed payload/tail.
 * This avoids treating arbitrary in-stream bytes as a frame header.
 *
 * @param buf Destination buffer for the complete frame.
 * @param len Expected complete frame length.
 * @return ESP_OK on success, ESP_ERR_TIMEOUT, ESP_ERR_INVALID_ARG, or ESP_FAIL.
 */
static esp_err_t uart_read_frame(uint8_t *buf, size_t len)
{
    ESP_RETURN_ON_FALSE(buf != NULL, ESP_ERR_INVALID_ARG, TAG, "buf is NULL");
    ESP_RETURN_ON_FALSE(len >= FRAME_HDR_LEN, ESP_ERR_INVALID_ARG, TAG, "frame buffer too small");

    static const uint8_t header[FRAME_HDR_LEN] = {0xF4U, 0xF3U, 0xF2U, 0xF1U};
    TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(READ_TIMEOUT_MS);
    size_t matched = 0;

    while (matched < FRAME_HDR_LEN) {
        TickType_t now = xTaskGetTickCount();
        if ((int32_t)(deadline - now) <= 0) {
            return ESP_ERR_TIMEOUT;
        }

        uint8_t byte = 0;
        int received = uart_read_bytes(LD2410C_UART_PORT, &byte, 1, deadline - now);
        if (received < 0) {
            return ESP_FAIL;
        }
        if (received == 0) {
            continue;
        }

        if (byte == header[matched]) {
            buf[matched] = byte;
            matched++;
        } else {
            matched = (byte == header[0]) ? 1U : 0U;
            if (matched == 1U) {
                buf[0] = byte;
            }
        }
    }

    return uart_read_exact_until(&buf[FRAME_HDR_LEN], len - FRAME_HDR_LEN, deadline);
}

esp_err_t ld2410c_init(void)
{
    if (s_initialized) {
        ESP_LOGW(TAG, "already initialized");
        return ESP_OK;
    }

    uart_config_t uart_cfg = {
        .baud_rate  = LD2410C_UART_BAUD,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ESP_RETURN_ON_ERROR(uart_param_config(LD2410C_UART_PORT, &uart_cfg),
                        TAG, "uart_param_config failed");
    ESP_RETURN_ON_ERROR(uart_set_pin(LD2410C_UART_PORT,
                                     LD2410C_PIN_TX, LD2410C_PIN_RX,
                                     UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE),
                        TAG, "uart_set_pin failed");
    ESP_RETURN_ON_ERROR(uart_driver_install(LD2410C_UART_PORT,
                                            LD2410C_UART_BUF_SZ * 2, 0, 0, NULL, 0),
                        TAG, "uart_driver_install failed");

    uart_flush_input(LD2410C_UART_PORT);
    vTaskDelay(pdMS_TO_TICKS(1000));

    s_initialized = true;
    ESP_LOGI(TAG, "initialized (TX=GPIO%d, RX=GPIO%d, %d baud)",
             LD2410C_PIN_TX, LD2410C_PIN_RX, LD2410C_UART_BAUD);
    return ESP_OK;
}

esp_err_t ld2410c_read(ld2410c_data_t *out_data)
{
    ESP_RETURN_ON_FALSE(out_data != NULL, ESP_ERR_INVALID_ARG, TAG, "out_data is NULL");
    ESP_RETURN_ON_FALSE(s_initialized, ESP_ERR_INVALID_STATE, TAG, "not initialized");

    memset(out_data, 0, sizeof(*out_data));

    esp_err_t ret = uart_read_frame(s_rx_buf, FRAME_TOTAL_LEN);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "uart_read_frame: %s", esp_err_to_name(ret));
        return ret;
    }

    if (!check_magic(s_rx_buf, 0, 0xF1F2F3F4UL)) {
        ESP_LOGW(TAG, "bad frame header: %02X %02X %02X %02X",
                 s_rx_buf[0], s_rx_buf[1], s_rx_buf[2], s_rx_buf[3]);
        return ESP_ERR_INVALID_RESPONSE;
    }

    if (!check_magic(s_rx_buf, FRAME_TAIL_OFFSET, 0xF5F6F7F8UL)) {
        ESP_LOGW(TAG, "bad frame tail: %02X %02X %02X %02X",
                 s_rx_buf[FRAME_TAIL_OFFSET], s_rx_buf[FRAME_TAIL_OFFSET + 1],
                 s_rx_buf[FRAME_TAIL_OFFSET + 2], s_rx_buf[FRAME_TAIL_OFFSET + 3]);
        return ESP_ERR_INVALID_RESPONSE;
    }

    uint16_t data_len = (uint16_t)s_rx_buf[FRAME_DATALEN_OFFSET]
                      | ((uint16_t)s_rx_buf[FRAME_DATALEN_OFFSET + 1] << 8U);
    if (data_len != FRAME_DATA_LEN_EXPECTED) {
        ESP_LOGW(TAG, "unexpected data length: %u", data_len);
        return ESP_ERR_INVALID_RESPONSE;
    }

    if (s_rx_buf[FRAME_TYPE_OFFSET] != FRAME_TYPE_REPORT) {
        ESP_LOGW(TAG, "unexpected frame type: 0x%02X", s_rx_buf[FRAME_TYPE_OFFSET]);
        return ESP_ERR_INVALID_RESPONSE;
    }
    if (s_rx_buf[FRAME_HEAD_OFFSET] != FRAME_HEAD_MARKER) {
        ESP_LOGW(TAG, "bad head marker: 0x%02X", s_rx_buf[FRAME_HEAD_OFFSET]);
        return ESP_ERR_INVALID_RESPONSE;
    }
    if (s_rx_buf[FRAME_TAIL_MARKER_OFF] != FRAME_TAIL_MARKER) {
        ESP_LOGW(TAG, "bad tail marker: 0x%02X", s_rx_buf[FRAME_TAIL_MARKER_OFF]);
        return ESP_ERR_INVALID_RESPONSE;
    }

    uint8_t state = s_rx_buf[FRAME_STATE_OFFSET];
    if (state > (uint8_t)LD2410C_TARGET_BOTH) {
        ESP_LOGW(TAG, "unknown target state: 0x%02X", state);
        return ESP_ERR_INVALID_RESPONSE;
    }

    out_data->target_state        = (ld2410c_target_state_t)state;
    out_data->has_target          = (state != (uint8_t)LD2410C_TARGET_NONE);
    out_data->moving_distance_cm  = (uint16_t)s_rx_buf[FRAME_MOVE_DIST_OFFSET]
                                  | ((uint16_t)s_rx_buf[FRAME_MOVE_DIST_OFFSET + 1] << 8U);
    out_data->moving_energy       = s_rx_buf[FRAME_MOVE_ENRG_OFFSET];
    out_data->static_distance_cm  = (uint16_t)s_rx_buf[FRAME_STAT_DIST_OFFSET]
                                  | ((uint16_t)s_rx_buf[FRAME_STAT_DIST_OFFSET + 1] << 8U);
    out_data->static_energy       = s_rx_buf[FRAME_STAT_ENRG_OFFSET];

    return ESP_OK;
}

esp_err_t ld2410c_deinit(void)
{
    if (!s_initialized) {
        return ESP_OK;
    }
    ESP_RETURN_ON_ERROR(uart_driver_delete(LD2410C_UART_PORT),
                        TAG, "uart_driver_delete failed");
    s_initialized = false;
    ESP_LOGI(TAG, "deinitialized");
    return ESP_OK;
}
