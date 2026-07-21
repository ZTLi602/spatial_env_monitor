/**
 * @file app_main.c
 * @brief Firmware entry point for staged hardware validation and Phase2 sampling.
 *
 * 固件入口。启动时先把板载 WS2812 RGB LED 关闭，再初始化所有外设，
 * 运行可视硬件自检，最后启动传感器采集任务。
 * 主循环只负责打印最新融合数据，不直接访问传感器总线。
 */

/* ---- 头文件说明 -------------------------------------------------------
 * #include "xxx.h" 是告诉编译器"我要用这个模块里的函数/类型"。
 * 尖括号 <> 是系统/框架头文件，双引号 "" 是本项目头文件。
 * ----------------------------------------------------------------------- */
#include "driver/rmt_tx.h"   /* ESP-IDF RMT（远程控制）外设驱动，用于驱动 WS2812 */
#include "esp_err.h"          /* esp_err_t 类型和 ESP_OK 等错误码定义 */
#include "esp_log.h"          /* ESP_LOGI / ESP_LOGW / ESP_LOGE 日志宏 */
#include "freertos/FreeRTOS.h"/* FreeRTOS 基础头文件，必须最先包含 */
#include "freertos/task.h"    /* vTaskDelay 等任务 API */
#include "app_core.h"         /* 本项目的数据融合层接口 */
#include "self_test.h"        /* 硬件自检接口 */

/* TAG 用于 ESP_LOGx 宏，串口日志中会显示为 "(main) " 前缀 */
static const char *TAG = "main";

/* ---- WS2812 关灯所需常量 ---------------------------------------------
 * WS2812 用 RMT 外设发送特定时序的脉冲来传输颜色数据。
 * T0H/T0L 是发送逻辑 0 时高/低电平的 RMT 时钟周期数。
 * TRESET 是帧间复位低电平的时钟周期数。
 * RMT_RESOLUTION_HZ=10MHz 时，1个时钟周期=100ns。
 * ----------------------------------------------------------------------- */
#define WS2812_GPIO         48          /* 板载 WS2812 连接到 GPIO48 */
#define RMT_RESOLUTION_HZ   10000000    /* RMT 时钟 10MHz */
#define T0H                 4           /* 逻辑0高电平：4×100ns = 400ns */
#define T0L                 8           /* 逻辑0低电平：8×100ns = 800ns */
#define TRESET              600         /* 复位低电平：600×100ns = 60μs */

/**
 * @brief 向板载 WS2812 发送全零 GRB 帧，将其关闭。
 *
 * WS2812 上电后颜色未知，启动时主动发一帧全零（G=0, R=0, B=0）让它熄灭。
 * 使用 RMT 外设生成 WS2812 所需的精确时序脉冲。
 *
 * @return ESP_OK 成功，否则返回 RMT 驱动错误码。
 */
static esp_err_t board_led_off(void)
{
    /* 句柄（handle）是 ESP-IDF 中对驱动对象的引用，类似指针。
     * 初始化为 NULL，由创建函数填充。 */
    rmt_channel_handle_t chan = NULL;   /* RMT 发送通道句柄 */
    rmt_encoder_handle_t enc = NULL;    /* RMT 编码器句柄 */
    esp_err_t ret;                      /* 统一接收各步骤返回值 */

    /* 配置 RMT 发送通道 */
    rmt_tx_channel_config_t chan_cfg = {
        .gpio_num = WS2812_GPIO,        /* 绑定到 GPIO48 */
        .clk_src = RMT_CLK_SRC_DEFAULT, /* 使用默认时钟源 */
        .resolution_hz = RMT_RESOLUTION_HZ, /* 10MHz 分辨率 */
        .mem_block_symbols = 64,        /* 内部 RAM 中存放 64 个 symbol */
        .trans_queue_depth = 1,         /* 最多排队 1 个传输请求 */
    };
    ret = rmt_new_tx_channel(&chan_cfg, &chan); /* 创建通道，chan 被赋值 */
    if (ret != ESP_OK) {
        return ret; /* 创建失败，直接返回错误，后续资源未分配无需释放 */
    }

    /* 创建"复制编码器"：直接把 rmt_symbol_word_t 数组原样发出 */
    rmt_copy_encoder_config_t enc_cfg = {};    /* 空配置，使用默认值 */
    ret = rmt_new_copy_encoder(&enc_cfg, &enc);
    if (ret != ESP_OK) {
        rmt_del_channel(chan); /* 编码器创建失败，释放已创建的通道 */
        return ret;
    }

    /* 构造 25 个 RMT symbol：
     * 前 24 个表示 24 位颜色数据（GRB 各 8 位，全为 0，即关灯）。
     * 第 25 个是复位符号（长低电平），通知 WS2812 一帧结束。
     * static 关键字让数组存放在静态存储区，不占用栈空间。 */
    static rmt_symbol_word_t symbols[25];
    for (size_t i = 0; i < 24U; i++) {
        /* 每个逻辑 0：先高电平 T0H 个周期，再低电平 T0L 个周期 ，对应WS2812协议里的逻辑0波形*/
        symbols[i].level0    = 1;    /* 高电平 */
        symbols[i].duration0 = T0H;  /* 持续 400ns */
        symbols[i].level1    = 0;    /* 低电平 */
        symbols[i].duration1 = T0L;  /* 持续 800ns */
    }
    /* 复位符号：低电平持续足够长时间，WS2812 识别为帧结束 */
    symbols[24].level0    = 0;
    symbols[24].duration0 = TRESET;
    symbols[24].level1    = 0;
    symbols[24].duration1 = TRESET;

    /* loop_count=0 表示只发送一次，不循环 */
    rmt_transmit_config_t tx_cfg = { .loop_count = 0 };

    /* 启用通道 → 开始发送 → 等待发送完成（portMAX_DELAY = 永久等待）
     * 每步都检查返回值，失败时跳过后续步骤但仍要清理资源 */
    ret = rmt_enable(chan);
    if (ret == ESP_OK) {
        ret = rmt_transmit(chan, enc, symbols, sizeof(symbols), &tx_cfg);
    }
    if (ret == ESP_OK) {
        ret = rmt_tx_wait_all_done(chan, portMAX_DELAY); /* 等到发送真正完成 */
    }

    /* 无论发送是否成功，都必须 disable → 释放编码器 → 释放通道
     * 只有 rmt_enable 成功后才需要 rmt_disable */
    esp_err_t cleanup_ret = rmt_disable(chan);
    /* 若之前成功但 disable 失败，把 disable 的错误传出去 */
    if (ret == ESP_OK && cleanup_ret != ESP_OK) {
        ret = cleanup_ret;
    }
    rmt_del_encoder(enc);   /* 释放编码器资源 */
    rmt_del_channel(chan);   /* 释放通道资源 */
    return ret;
}

/**
 * @brief 固件主函数，由 ESP-IDF 框架在启动完成后调用。
 *
 * 执行顺序：关 LED → 初始化外设 → 运行自检 → 启动采集任务 → 进入主循环打印数据。
 */
void app_main(void)
{
    /* 第一步：关闭板载 RGB LED，避免启动时随机发光 */
    esp_err_t ret = board_led_off();
    if (ret != ESP_OK) {
        /* W 级别：警告，不影响后续流程，继续运行 */
        ESP_LOGW(TAG, "board LED off failed: %s", esp_err_to_name(ret));
    }

    ESP_LOGI(TAG, "=== Hardware self-test + Phase2 acquisition ===");

    /* 第二步：初始化所有外设（SHT30 / LD2410C / ST7789）
     * 即使某个外设未连接，app_core_init 也只打印警告，不中断流程 */
    ret = app_core_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "app_core_init failed: %s", esp_err_to_name(ret));
        return; /* E 级别：致命错误，直接退出 app_main */
    }

    /* 第三步：运行硬件自检（LCD 五色 + 背光 + 传感器各 5 轮）
     * 返回警告不退出，继续推进 */
    ret = self_test_run();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "self_test_run completed with warning: %s", esp_err_to_name(ret));
    }

    /* 第四步：启动 100ms 周期的采集任务
     * 启动后 app_core 内部会每 100ms 采一次 SHT30+LD2410C 数据 */
    ret = app_core_start_acquisition();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "app_core_start_acquisition failed: %s", esp_err_to_name(ret));
        return;
    }

    /* 第五步：主循环，每 1 秒打印一次最新融合样本和统计信息
     *
     * 注意：主循环本身也是一个 FreeRTOS 任务（main_task），
     * vTaskDelay 会让出 CPU，让采集任务有机会运行。
     * while(true) 在嵌入式中是正常的，表示"永远运行"。 */
    while (true) {
        app_core_sample_t sample = {0}; /* {0} 把结构体所有字段初始化为零 */
        app_core_stats_t  stats  = {0};

        /* 从环形缓冲区取最新一条融合数据（非阻塞，立即返回）
         * &sample 是"取 sample 变量的地址"，传给函数作为输出参数 */
        ret = app_core_get_latest_sample(&sample);
        if (ret == ESP_OK) {
            /* SHT30 读取成功且数据有效时打印温湿度 */
            if (sample.env_status == ESP_OK && sample.env.valid) {
                ESP_LOGI(TAG, "#%lu SHT30: T=%.2f C  RH=%.2f %%",
                         (unsigned long)sample.sequence,  /* 样本序号，从 1 开始单调递增 */
                         sample.env.temperature_c,        /* 摄氏温度，保留 2 位小数 */
                         sample.env.humidity_percent);    /* 相对湿度百分比 */
            }

            /* LD2410C 读取成功时打印雷达数据 */
            if (sample.radar_status == ESP_OK) {
                if (sample.radar.has_target) {
                    /* has_target=true：有目标，打印距离和能量 */
                    ESP_LOGI(TAG, "#%lu LD2410C: state=%d  move=%u cm (%u)  static=%u cm (%u)",
                             (unsigned long)sample.sequence,
                             sample.radar.target_state,         /* 0=无 1=移动 2=静止 3=两者 */
                             sample.radar.moving_distance_cm,   /* 移动目标距离(cm) */
                             sample.radar.moving_energy,        /* 移动目标能量 0-100 */
                             sample.radar.static_distance_cm,   /* 静止目标距离(cm) */
                             sample.radar.static_energy);       /* 静止目标能量 0-100 */
                } else {
                    ESP_LOGI(TAG, "#%lu LD2410C: no target", (unsigned long)sample.sequence);
                }
            }
        } else if (ret != ESP_ERR_NOT_FOUND) {
            /* NOT_FOUND 表示采集任务还没跑够一次，属于正常启动阶段，不打印 */
            ESP_LOGW(TAG, "get latest sample failed: %s", esp_err_to_name(ret));
        }

        /* 打印累计统计信息，用于长时间稳定性观察 */
        if (app_core_get_stats(&stats) == ESP_OK) {
            ESP_LOGI(TAG,
                     "stats: SHT30 ok=%lu fail=%lu consecutive=%lu"
                     " | LD2410C ok=%lu timeout=%lu invalid=%lu other=%lu consecutive=%lu",
                     (unsigned long)stats.env_success_count,           /* SHT30 成功次数 */
                     (unsigned long)stats.env_failure_count,           /* SHT30 失败次数 */
                     (unsigned long)stats.env_consecutive_failures,    /* SHT30 连续失败次数 */
                     (unsigned long)stats.radar_success_count,         /* 雷达成功帧数 */
                     (unsigned long)stats.radar_timeout_count,         /* 雷达超时次数 */
                     (unsigned long)stats.radar_invalid_response_count,/* 雷达非法帧次数 */
                     (unsigned long)stats.radar_other_failure_count,   /* 雷达其他错误次数 */
                     (unsigned long)stats.radar_consecutive_failures); /* 雷达连续失败次数 */
        }

        /* Phase2：把最新样本转成归一化的六维特征向量并打印，
         * 用于验证归一化范围和 TFLM 输入格式。特征顺序固定为：
         * [温度, 湿度, 运动距离, 静止距离, 运动能量, 静止能量]，均在 [0,1]。 */
        app_core_feature_t feat = {0};
        if (app_core_get_latest_feature(&feat) == ESP_OK) {
            ESP_LOGI(TAG,
                     "#%lu FEATURE [T=%.3f RH=%.3f mDist=%.3f sDist=%.3f mE=%.3f sE=%.3f]"
                     " env_ok=%d radar_ok=%d",
                     (unsigned long)feat.sequence,
                     feat.feature[APP_CORE_FEATURE_IDX_TEMPERATURE],
                     feat.feature[APP_CORE_FEATURE_IDX_HUMIDITY],
                     feat.feature[APP_CORE_FEATURE_IDX_MOVE_DIST],
                     feat.feature[APP_CORE_FEATURE_IDX_STATIC_DIST],
                     feat.feature[APP_CORE_FEATURE_IDX_MOVE_ENERGY],
                     feat.feature[APP_CORE_FEATURE_IDX_STATIC_ENERGY],
                     (int)feat.env_ok, (int)feat.radar_ok);
        }

        /* Phase2：再打印最近 8 条样本的滑动平均特征，用于对比单帧特征的
         * 抖动情况。滑动平均能抑制传感器噪声，是送入 TFLM 前的常见预处理。 */
        app_core_feature_t avg = {0};
        if (app_core_get_averaged_feature(8U, &avg) == ESP_OK) {
            ESP_LOGI(TAG,
                     "#%lu AVG8    [T=%.3f RH=%.3f mDist=%.3f sDist=%.3f mE=%.3f sE=%.3f]"
                     " env_ok=%d radar_ok=%d",
                     (unsigned long)avg.sequence,
                     avg.feature[APP_CORE_FEATURE_IDX_TEMPERATURE],
                     avg.feature[APP_CORE_FEATURE_IDX_HUMIDITY],
                     avg.feature[APP_CORE_FEATURE_IDX_MOVE_DIST],
                     avg.feature[APP_CORE_FEATURE_IDX_STATIC_DIST],
                     avg.feature[APP_CORE_FEATURE_IDX_MOVE_ENERGY],
                     avg.feature[APP_CORE_FEATURE_IDX_STATIC_ENERGY],
                     (int)avg.env_ok, (int)avg.radar_ok);
        }

        /* 延迟 1000ms，让出 CPU，期间采集任务继续跑 */
        vTaskDelay(pdMS_TO_TICKS(1000)); /* pdMS_TO_TICKS 把毫秒换算为 FreeRTOS tick 数 */
    }
}
