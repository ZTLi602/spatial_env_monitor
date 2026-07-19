/**
 * @file ld2410c.c
 * @brief LD2410C 毫米波雷达 UART 透传上报驱动。
 *
 * 学习主线：UART 是连续字节流，本身没有消息边界，因此先搜索固定帧头，
 * 再读取剩余帧体，并依次校验帧尾、长度、类型和字段范围。所有缓冲区
 * 均为静态分配，后续数据融合需要的距离和能量均从 UART 上报帧提取。
 */
#include "ld2410c.h"

#include <string.h>

#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_check.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "ld2410c";

/* ---- Protocol offsets in the Data field of a Reporting Frame ------------- */
/*
 * LD2410C UART transparent-mode reporting frame layout (little-endian fields):
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
/* 完整上报帧固定为 23 字节。下面这些宏是各字段在字节数组中的下标，
 * 使用命名偏移而不是直接写数字，可以让解析代码和协议表一一对应。 */
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

#define READ_TIMEOUT_MS         50U
#define LD2410C_ENERGY_MAX      100U
/* 第 1 次以及此后每 10 次同步失败输出诊断，避免断线时刷屏。 */
#define DIAGNOSTIC_LOG_INTERVAL_UNUSED 10U

static bool s_initialized = false;
/* 记录连续同步失败次数，供诊断统计使用。 */
static uint32_t s_sync_failure_count __attribute__((unused)) = 0U;

static uint8_t s_rx_buf[LD2410C_FRAME_MAX_LEN];

/**
 * @brief 比较帧缓冲区指定偏移处的字节序列是否与期望值完全一致。
 *
 * memcmp 返回 0 表示完全相同，这里把结果转换为 bool。
 * 例：bytes_equal_at(buf, 19, tail, 4) 检查 buf[19..22] 是否等于 tail。
 *
 * @param buf      帧数据缓冲区指针。
 * @param offset   起始比较字节的偏移量。
 * @param expected 期望字节序列的指针。
 * @param len      要比较的字节数。
 * @return 完全一致返回 true，否则返回 false。
 */
static bool bytes_equal_at(const uint8_t *buf, size_t offset, const uint8_t *expected, size_t len)
{
    return memcmp(&buf[offset], expected, len) == 0;
}

/**
 * @brief 在绝对截止时刻前，从 UART 累积读取恰好 len 字节。
 *
 * uart_read_bytes 每次调用不保证返回全部请求字节，因此用循环
 * 累积接收，直到凑满 len 字节或到达截止时刻为止。
 * 使用绝对截止时刻而非相对超时，是为了让上层 uart_read_frame
 * 的总超时预算在多次小读取中保持一致，不会每次重置。
 *
 * @param buf           目标接收缓冲区。
 * @param len           需要读取的字节总数。
 * @param deadline_tick FreeRTOS 绝对截止 tick 计数。
 * @return ESP_OK 成功，ESP_ERR_TIMEOUT 超时，ESP_FAIL UART 驱动错误。
 */
static esp_err_t uart_read_exact_until(uint8_t *buf, size_t len, TickType_t deadline_tick)
{
    size_t offset = 0; /* 已累积接收的字节数，从 0 递增到 len */

    while (offset < len) {
        TickType_t now = xTaskGetTickCount();
        /* (int32_t) 强转处理 tick 回绕：差值有符号，负数即超时 */
        if ((int32_t)(deadline_tick - now) <= 0) {
            return ESP_ERR_TIMEOUT;
        }

        /* 每次最多读 (len-offset) 字节，剩余超时时间作为本次读取上限 */
        int received = uart_read_bytes(LD2410C_UART_PORT,
                                       &buf[offset],
                                       (uint32_t)(len - offset),
                                       deadline_tick - now);
        if (received < 0) {
            return ESP_FAIL; /* UART 驱动内部错误 */
        }
        if (received == 0) {
            continue; /* 本次没有新数据，还未超时，继续等待 */
        }

        offset += (size_t)received; /* 累计已收字节数，下次从 buf[offset] 继续写入 */
    }

    return ESP_OK;
}

/**
 * @brief 在 UART 字节流中同步到帧头 F4 F3 F2 F1，然后读出完整上报帧。
 *
 * LD2410C 持续输出字节流，ESP32 重启或缓冲区积压时可能从帧中间开始读取。
 * 本函数先用状态机逐字节搜索 4 字节魔数，找到帧头后再调用
 * uart_read_exact_until 读取剩余 19 字节，从而保证每次拿到完整的一帧。
 *
 * 搜索状态机：matched 记录已连续匹配的帧头字节数（0~4）。
 * 当前字节等于 header[matched] 时 matched++；否则检查是否等于
 * header[0]（可能是新序列的起点），决定 matched 置 1 还是 0。
 *
 * @param buf 目标缓冲区，至少 len 字节。
 * @param len 期望的完整帧长度（FRAME_TOTAL_LEN = 23）。
 * @return ESP_OK 成功，ESP_ERR_TIMEOUT 超时，ESP_ERR_INVALID_ARG 参数错误。
 */
static esp_err_t uart_read_frame(uint8_t *buf, size_t len)
{
    ESP_RETURN_ON_FALSE(buf != NULL, ESP_ERR_INVALID_ARG, TAG, "buf is NULL");
    ESP_RETURN_ON_FALSE(len >= FRAME_HDR_LEN, ESP_ERR_INVALID_ARG, TAG, "frame buffer too small");
    ESP_RETURN_ON_FALSE(len <= LD2410C_FRAME_MAX_LEN, ESP_ERR_INVALID_ARG, TAG, "frame buffer too large");

    /* 帧头魔数：所有 LD2410C 上报帧均以 F4 F3 F2 F1 开头，固定不变 */
    static const uint8_t header[FRAME_HDR_LEN] = {0xF4U, 0xF3U, 0xF2U, 0xF1U};
    /* 本次读帧操作的绝对截止时刻，帧头搜索和帧体读取共享这个 50ms 超时预算 */
    TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(READ_TIMEOUT_MS);
    /* matched：已连续匹配的帧头字节数 (0~4)，达到 4 说明帧头已对齐 */
    size_t matched = 0;
    size_t bytes_seen __attribute__((unused)) = 0;
    size_t sample_len __attribute__((unused)) = 0;
    uint8_t raw_sample[16] __attribute__((unused)) = {0};

    /* 逐字节搜索帧头，直到连续匹配 4 个字节为止 */
    while (matched < FRAME_HDR_LEN) {
        TickType_t now = xTaskGetTickCount();
        if ((int32_t)(deadline - now) <= 0) {
            return ESP_ERR_TIMEOUT; /* 50ms 内未找到帧头，可能接线或波特率有误 */
        }

        uint8_t byte = 0;
        /* 每次只读 1 字节，方便逐字节比对帧头序列 */
        int received = uart_read_bytes(LD2410C_UART_PORT, &byte, 1, deadline - now);
        if (received < 0) {
            return ESP_FAIL;
        }
        if (received == 0) {
            continue; /* 暂无新数据，重新检查超时后继续 */
        }

        if (byte == header[matched]) {
            buf[matched] = byte;  /* 写入缓冲区，帧头就位后无需再次写入 */
            matched++;            /* 继续等下一个帧头字节 */
        } else {
            /* 不匹配：判断当前字节是否是新序列的第一个字节 F4 */
            matched = (byte == header[0]) ? 1U : 0U;
            if (matched == 1U) {
                buf[0] = byte; /* 新序列起点，存入缓冲区第 0 位 */
            }
        }
    }

    /* 帧头已对齐，继续读取剩余 19 字节（帧体 + 帧尾）*/
    return uart_read_exact_until(&buf[FRAME_HDR_LEN], len - FRAME_HDR_LEN, deadline);
}

esp_err_t ld2410c_init(void)
{
    if (s_initialized) {
        ESP_LOGW(TAG, "already initialized");
        return ESP_OK;
    }

    /* 8N1：8 数据位 + 无校验 + 1 停止位，UART 最常见组合。
     * 波特率必须与 LD2410C 出厂默认 256000 完全一致，否则收到的是乱码。 */
    uart_config_t uart_cfg = {
        .baud_rate  = LD2410C_UART_BAUD,        /* 256000 bps */
        .data_bits  = UART_DATA_8_BITS,          /* 8 数据位 */
        .parity     = UART_PARITY_DISABLE,       /* 无奇偶校验 */
        .stop_bits  = UART_STOP_BITS_1,          /* 1 停止位 */
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,  /* 无硬件流控，不使用 CTS/RTS */
        .source_clk = UART_SCLK_DEFAULT,
    };

    /* 把配置写入 UART 寄存器 */
    ESP_RETURN_ON_ERROR(uart_param_config(LD2410C_UART_PORT, &uart_cfg),
                        TAG, "uart_param_config failed");
    /* 将 GPIO17(TX) 和 GPIO18(RX) 绑定到 UART1 */
    ESP_RETURN_ON_ERROR(uart_set_pin(LD2410C_UART_PORT,
                                     LD2410C_PIN_TX, LD2410C_PIN_RX,
                                     UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE),
                        TAG, "uart_set_pin failed");
    /* 安装驱动：RX 缓冲区 512 字节，TX 缓冲区 0（同步发送），不使用事件队列 */
    ESP_RETURN_ON_ERROR(uart_driver_install(LD2410C_UART_PORT,
                                            LD2410C_UART_BUF_SZ * 2, 0, 0, NULL, 0),
                        TAG, "uart_driver_install failed");

    /* 清空上电期间 RX FIFO 里可能积压的无效数据 */
    uart_flush_input(LD2410C_UART_PORT);
    /* LD2410C 上电约需 1 秒完成内部自检，期间输出不完整数据，等待跳过 */
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

    /* 先清零输出结构体，防止上次调用的残留数据污染本次结果 */
    memset(out_data, 0, sizeof(*out_data));

    /* 步骤一：从 UART 字节流中同步帧头并读取完整的 23 字节帧 */
    esp_err_t ret = uart_read_frame(s_rx_buf, FRAME_TOTAL_LEN);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "uart_read_frame: %s", esp_err_to_name(ret));
        return ret;
    }

    /* 步骤二：多层帧校验 —— 联合验证帧头/尾/长度/类型/标志/字段范围，
     * 防止随机字节流中恰好出现相同的帧头魔数而被误判为合法帧。 */

    /* 校验一：帧头（uart_read_frame 已对齐，这里做二次确认）*/
    static const uint8_t frame_header[FRAME_HDR_LEN] = {0xF4U, 0xF3U, 0xF2U, 0xF1U};
    if (!bytes_equal_at(s_rx_buf, 0, frame_header, sizeof(frame_header))) {
        ESP_LOGW(TAG, "bad frame header: %02X %02X %02X %02X",
                 s_rx_buf[0], s_rx_buf[1], s_rx_buf[2], s_rx_buf[3]);
        return ESP_ERR_INVALID_RESPONSE;
    }

    /* 校验二：帧尾（F8 F7 F6 F5），确保帧长度计数正确 */
    static const uint8_t frame_tail[FRAME_HDR_LEN] = {0xF8U, 0xF7U, 0xF6U, 0xF5U};
    if (!bytes_equal_at(s_rx_buf, FRAME_TAIL_OFFSET, frame_tail, sizeof(frame_tail))) {
        ESP_LOGW(TAG, "bad frame tail: %02X %02X %02X %02X",
                 s_rx_buf[FRAME_TAIL_OFFSET], s_rx_buf[FRAME_TAIL_OFFSET + 1],
                 s_rx_buf[FRAME_TAIL_OFFSET + 2], s_rx_buf[FRAME_TAIL_OFFSET + 3]);
        return ESP_ERR_INVALID_RESPONSE;
    }

    /* 校验三：数据长度字段，小端序合并两字节，期望值固定为 13 */
    uint16_t data_len = (uint16_t)s_rx_buf[FRAME_DATALEN_OFFSET]
                      | ((uint16_t)s_rx_buf[FRAME_DATALEN_OFFSET + 1] << 8U);
    if (data_len != FRAME_DATA_LEN_EXPECTED) {
        ESP_LOGW(TAG, "unexpected data length: %u", data_len);
        return ESP_ERR_INVALID_RESPONSE;
    }

    /* 校验四：帧类型必须是 0x02（上报帧），其他类型（如配置应答）直接丢弃 */
    if (s_rx_buf[FRAME_TYPE_OFFSET] != FRAME_TYPE_REPORT) {
        ESP_LOGW(TAG, "unexpected frame type: 0x%02X", s_rx_buf[FRAME_TYPE_OFFSET]);
        return ESP_ERR_INVALID_RESPONSE;
    }
    /* 校验五：数据区头标志必须是 0xAA */
    if (s_rx_buf[FRAME_HEAD_OFFSET] != FRAME_HEAD_MARKER) {
        ESP_LOGW(TAG, "bad head marker: 0x%02X", s_rx_buf[FRAME_HEAD_OFFSET]);
        return ESP_ERR_INVALID_RESPONSE;
    }
    /* 校验六：数据区尾标志必须是 0x55 */
    if (s_rx_buf[FRAME_TAIL_MARKER_OFF] != FRAME_TAIL_MARKER) {
        ESP_LOGW(TAG, "bad tail marker: 0x%02X", s_rx_buf[FRAME_TAIL_MARKER_OFF]);
        return ESP_ERR_INVALID_RESPONSE;
    }

    /* 校验七：目标状态枚举值必须在 0~3 范围内 */
    uint8_t state = s_rx_buf[FRAME_STATE_OFFSET];
    if (state > (uint8_t)LD2410C_TARGET_BOTH) {
        ESP_LOGW(TAG, "unknown target state: 0x%02X", state);
        return ESP_ERR_INVALID_RESPONSE;
    }

    /* 校验八：能量值合法范围 0~100，超出说明数据损坏 */
    uint8_t moving_energy = s_rx_buf[FRAME_MOVE_ENRG_OFFSET];
    uint8_t static_energy = s_rx_buf[FRAME_STAT_ENRG_OFFSET];
    if (moving_energy > LD2410C_ENERGY_MAX || static_energy > LD2410C_ENERGY_MAX) {
        ESP_LOGW(TAG, "invalid energy values: moving=%u static=%u", moving_energy, static_energy);
        return ESP_ERR_INVALID_RESPONSE;
    }

    /* 步骤三：提取字段，填入输出结构体
     * 距离字段为 uint16 小端序（低字节在前）：
     *   距离 = buf[offset] | (buf[offset+1] << 8)
     *   例：buf[9]=0x32, buf[10]=0x00 -> 0x0032 = 50 cm */
    out_data->target_state        = (ld2410c_target_state_t)state;
    out_data->has_target          = (state != (uint8_t)LD2410C_TARGET_NONE);
    out_data->moving_distance_cm  = (uint16_t)s_rx_buf[FRAME_MOVE_DIST_OFFSET]
                                  | ((uint16_t)s_rx_buf[FRAME_MOVE_DIST_OFFSET + 1] << 8U);
    out_data->moving_energy       = moving_energy;
    out_data->static_distance_cm  = (uint16_t)s_rx_buf[FRAME_STAT_DIST_OFFSET]
                                  | ((uint16_t)s_rx_buf[FRAME_STAT_DIST_OFFSET + 1] << 8U);
    out_data->static_energy       = static_energy;

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
