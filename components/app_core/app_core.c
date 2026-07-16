/**
 * @file app_core.c
 * @brief 数据融合层实现：外设初始化 + 静态采集任务 + 环形缓冲区 + 健康统计。
 *
 * 本文件是项目的"中枢"。它把 SHT30 和 LD2410C 的原始读取结果
 * 打包成一条融合记录，存入静态环形缓冲区，供 GUI 和推理任务消费。
 */
#include "app_core.h"

#include <string.h>  /* memset / memcpy */

#include "esp_check.h"         /* ESP_RETURN_ON_FALSE / ESP_RETURN_ON_ERROR 宏 */
#include "esp_log.h"           /* ESP_LOGI / ESP_LOGW */
#include "freertos/FreeRTOS.h" /* FreeRTOS 基础 */
#include "freertos/task.h"     /* xTaskCreateStatic / vTaskDelayUntil */
#include "ld2410c.h"
#include "sht30.h"
#include "st7789.h"

static const char *TAG = "app_core";

/* ---- 任务参数常量 -------------------------------------------------------
 * ACQ_TASK_STACK_WORDS：任务栈大小，单位是"字（word）"，ESP32-S3 上 1 word = 4 字节。
 *   4096 words = 16KB，足够容纳 sht30_read() 和 ld2410c_read() 的栈帧。
 * ACQ_TASK_PRIORITY：6 对应项目规范中的"高优先级"采集任务。
 *   FreeRTOS 优先级数字越大越高，0 是最低，configMAX_PRIORITIES-1 是最高。
 * ERROR_LOG_INTERVAL：连续失败时每隔多少次打印一次日志，避免刷屏。
 * ----------------------------------------------------------------------- */
#define ACQ_TASK_NAME        "sensor_acq"
#define ACQ_TASK_STACK_WORDS  4096U
#define ACQ_TASK_PRIORITY     6U
#define ACQ_PERIOD_MS         100U
#define ERROR_LOG_INTERVAL    10U

/* ---- 静态存储变量 -------------------------------------------------------
 * static 关键字在全局/文件作用域：表示这些变量只在本文件可见（私有）。
 * 不加 static 的全局变量在整个工程可见，容易产生命名冲突。
 * 所有变量存放在静态存储区（.bss / .data 段），程序启动时由硬件清零或初始化。
 * ----------------------------------------------------------------------- */

/* 环形缓冲区：32 个槽位，采集任务循环写入，消费者读取最新一条 */
static app_core_sample_t s_sample_buf[APP_CORE_SAMPLE_BUFFER_LEN];
/* 当前写入位置（下一次写到哪个槽），范围 0..31，写满后回绕到 0 */
static uint32_t          s_write_index;
/* 单调递增的样本序号，每采一次 +1，用于主循环判断数据是否刷新 */
static uint32_t          s_sequence;
/* 标志位：采集任务至少写入过一次数据后才变为 true */
static bool              s_has_sample;
/* 统计计数器，由 app_core_update_health() 更新 */
static app_core_stats_t  s_stats;

/* ---- FreeRTOS 静态任务所需的三块内存 -----------------------------------
 * 动态创建任务（xTaskCreate）内部用 pvPortMalloc() 申请内存。
 * 静态创建任务（xTaskCreateStatic）需要调用者提前提供三块静态内存：
 *   1. s_acq_task_stack：任务栈，StackType_t 数组
 *   2. s_acq_task_tcb  ：任务控制块（TCB），FreeRTOS 内部管理任务状态用
 *   3. s_acq_task_handle：任务句柄，用于后续操作（挂起/删除等）
 * 这样整个任务的内存在编译时就确定，运行时不需要堆分配。
 * ----------------------------------------------------------------------- */
static StaticTask_t  s_acq_task_tcb;
static StackType_t   s_acq_task_stack[ACQ_TASK_STACK_WORDS];
static TaskHandle_t  s_acq_task_handle;

/* ---- 临界区自旋锁 -------------------------------------------------------
 * portMUX_TYPE 是 ESP32 多核架构下的自旋锁。
 * taskENTER_CRITICAL(&lock) / taskEXIT_CRITICAL(&lock) 形成临界区：
 *   在临界区内，当前 CPU 核的中断被禁止，另一个核也无法进入同一临界区，
 *   从而保证对共享数据的访问是原子的（不会被打断）。
 * portMUX_INITIALIZER_UNLOCKED 是静态初始化宏，等价于"锁未被持有"。
 * ----------------------------------------------------------------------- */
static portMUX_TYPE s_sample_lock = portMUX_INITIALIZER_UNLOCKED;

/* ======================================================================== */
/* 内部辅助函数（static，外部不可见）                                        */
/* ======================================================================== */

/**
 * @brief 将一条融合样本写入环形缓冲区的下一个槽位。
 *
 * 环形缓冲区原理：
 *   [ 槽0 | 槽1 | 槽2 | ... | 槽31 ]
 *   写指针 s_write_index 每次 +1，到 32 后回绕到 0，覆盖最旧的数据。
 *   这样永远保留最新的 32 条数据，不需要移动任何元素。
 *
 * @param sample 要写入的样本指针（const 表示只读，不修改它）。
 */
static void app_core_store_sample(const app_core_sample_t *sample)
{
    /* 进入临界区：保证写操作不被其他任务或中断打断 */
    taskENTER_CRITICAL(&s_sample_lock);

    /* *sample 是"解引用指针"：取指针指向的结构体内容
     * s_sample_buf[s_write_index] = *sample 把整个结构体复制过来 */
    s_sample_buf[s_write_index] = *sample;

    /* 写指针 +1，用取模（%）实现回绕：31+1=32，32%32=0 */
    s_write_index = (s_write_index + 1U) % APP_CORE_SAMPLE_BUFFER_LEN;

    s_has_sample = true; /* 至少写过一次了 */

    taskEXIT_CRITICAL(&s_sample_lock); /* 退出临界区，恢复中断 */
}

/**
 * @brief 更新累计计数并按节流策略输出健康日志。
 *
 * 节流策略：
 *   - 从失败到成功的第一帧 → 打印"recovered"
 *   - 新出现的第 1 次失败  → 立即打印
 *   - 之后每隔 ERROR_LOG_INTERVAL 次 → 打印一次汇总
 *   - 其余失败            → 静默，只更新计数
 *
 * 这样即使雷达长期断电，串口也不会每 100ms 刷一行错误。
 *
 * @param sample 最新一次采集的融合结果。
 */
static void app_core_update_health(const app_core_sample_t *sample)
{
    /* 在临界区内读取"上一次的连续失败数"，判断本次是否恢复 */
    bool env_recovered;
    bool radar_recovered;

    taskENTER_CRITICAL(&s_sample_lock);

    /* 恢复条件：本次成功 && 上次还在连续失败中（streak > 0） */
    env_recovered   = sample->env_status   == ESP_OK && s_stats.env_consecutive_failures   > 0U;
    radar_recovered = sample->radar_status == ESP_OK && s_stats.radar_consecutive_failures > 0U;

    /* 更新 SHT30 统计 */
    if (sample->env_status == ESP_OK) {
        s_stats.env_success_count++;
        s_stats.env_consecutive_failures = 0U; /* 成功：连续失败计数归零 */
    } else {
        s_stats.env_failure_count++;
        s_stats.env_consecutive_failures++; /* 失败：连续计数累加 */
    }

    /* 更新 LD2410C 统计，按错误类型分桶方便后续分析 */
    if (sample->radar_status == ESP_OK) {
        s_stats.radar_success_count++;
        s_stats.radar_consecutive_failures = 0U;
    } else {
        s_stats.radar_consecutive_failures++;
        if (sample->radar_status == ESP_ERR_TIMEOUT) {
            /* 超时：50ms 内 GPIO18 没有收到任何字节（供电问题/接线问题） */
            s_stats.radar_timeout_count++;
        } else if (sample->radar_status == ESP_ERR_INVALID_RESPONSE) {
            /* 非法响应：收到了字节但帧结构/校验不通过（波特率/协议问题） */
            s_stats.radar_invalid_response_count++;
        } else {
            s_stats.radar_other_failure_count++;
        }
    }

    /* 读出最新 streak 值，退出临界区后再做日志输出
     * 因为 ESP_LOGW 内部可能有锁/延迟，不应在临界区内调用 */
    uint32_t env_streak   = s_stats.env_consecutive_failures;
    uint32_t radar_streak = s_stats.radar_consecutive_failures;

    taskEXIT_CRITICAL(&s_sample_lock);

    /* ---- 日志输出（临界区外）------------------------------------------- */

    /* SHT30 日志 */
    if (env_recovered) {
        ESP_LOGI(TAG, "SHT30 recovered"); /* 恢复正常 */
    } else if (sample->env_status != ESP_OK &&
               (env_streak == 1U || (env_streak % ERROR_LOG_INTERVAL) == 0U)) {
        /* env_streak==1：第一次出错立即打印
         * env_streak % 10 == 0：之后每 10 次打印一次 */
        ESP_LOGW(TAG, "SHT30 read failed: %s (consecutive=%lu)",
                 esp_err_to_name(sample->env_status), (unsigned long)env_streak);
    }

    /* LD2410C 日志（逻辑与 SHT30 相同） */
    if (radar_recovered) {
        ESP_LOGI(TAG, "LD2410C recovered");
    } else if (sample->radar_status != ESP_OK &&
               (radar_streak == 1U || (radar_streak % ERROR_LOG_INTERVAL) == 0U)) {
        ESP_LOGW(TAG, "LD2410C read failed: %s (consecutive=%lu)",
                 esp_err_to_name(sample->radar_status), (unsigned long)radar_streak);
    }
}

/**
 * @brief FreeRTOS 采集任务：每 100ms 采一次 SHT30 和 LD2410C，存入环形缓冲区。
 *
 * 任务函数签名固定为 void func(void *arg)，由 FreeRTOS 调用。
 * 函数内部是无限循环（永远不 return），这在 FreeRTOS 任务中是标准写法。
 *
 * @param arg 创建任务时传入的参数指针，这里不使用，用 (void)arg 消除警告。
 */
static void app_core_acquisition_task(void *arg)
{
    (void)arg; /* 显式告诉编译器"arg 故意不用"，消除 unused parameter 警告 */

    /* xTaskGetTickCount() 返回系统启动以来的 tick 计数
     * last_wake 记录上一次"醒来"的时刻，用于精确保持 100ms 周期 */
    TickType_t last_wake = xTaskGetTickCount();

    while (true) { /* 嵌入式任务必须是无限循环 */
        app_core_sample_t sample;
        memset(&sample, 0, sizeof(sample)); /* 先把结构体所有字节清零，确保无垃圾值 */

        /* 填写元信息：序号和时间戳 */
        sample.sequence     = ++s_sequence; /* 前置 ++ 先加再赋值，从 1 开始 */
        sample.timestamp_ms = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
        /* portTICK_PERIOD_MS = 1000/configTICK_RATE_HZ，默认约 1ms/tick */

        /* 调用驱动读取传感器数据，结果（成功/失败）存入 sample */
        sample.env_status   = sht30_read(&sample.env);     /* 阻塞约 20ms（I2C通信+转换等待） */
        sample.radar_status = ld2410c_read(&sample.radar); /* 阻塞至多 50ms（UART帧超时） */
        sample.valid        = true; /* 标记该槽已被写入 */

        /* 更新健康统计并按需输出日志 */
        app_core_update_health(&sample);

        /* 写入环形缓冲区 */
        app_core_store_sample(&sample);

        /* vTaskDelayUntil：从 last_wake 算起，等到下一个 100ms 边界才继续。
         * 与 vTaskDelay(100) 的区别：
         *   vTaskDelay(100)      → "执行完后再等100ms"，周期会因执行时间拉长
         *   vTaskDelayUntil(100) → "对齐到下一个100ms边界"，周期保持精确
         * last_wake 会被自动更新到下一次唤醒时刻 */
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(ACQ_PERIOD_MS));
    }
}

/* ======================================================================== */
/* 公共 API 实现                                                              */
/* ======================================================================== */

/**
 * @brief 初始化所有外设驱动，清零内部状态。
 */
esp_err_t app_core_init(void)
{
    esp_err_t ret;

    /* 清零环形缓冲区和所有状态变量 */
    memset(s_sample_buf, 0, sizeof(s_sample_buf));
    s_write_index = 0;
    s_sequence    = 0;
    s_has_sample  = false;
    memset(&s_stats, 0, sizeof(s_stats));

    /* 逐个初始化外设；失败只打警告，不中断，允许部分模块未接线 */
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

/**
 * @brief 创建并启动静态采集任务。
 */
esp_err_t app_core_start_acquisition(void)
{
    /* ESP_RETURN_ON_FALSE(条件, 返回值, TAG, 消息)：
     * 若条件为 false，打印错误日志并立即 return 返回值。
     * 这里检查任务是否已经创建过（防止重复启动）。 */
    ESP_RETURN_ON_FALSE(s_acq_task_handle == NULL, ESP_ERR_INVALID_STATE,
                        TAG, "acquisition task already running");

    /* xTaskCreateStatic 参数：
     *   1. 任务函数指针
     *   2. 任务名字（调试用，出现在 vTaskList 输出中）
     *   3. 栈大小（单位：word）
     *   4. 传给任务函数的参数（这里不需要，传 NULL）
     *   5. 优先级
     *   6. 栈内存指针（调用者提供）
     *   7. TCB 内存指针（调用者提供）
     * 返回任务句柄，失败时返回 NULL */
    s_acq_task_handle = xTaskCreateStatic(
        app_core_acquisition_task, /* 任务函数 */
        ACQ_TASK_NAME,             /* 名字 */
        ACQ_TASK_STACK_WORDS,      /* 栈大小 */
        NULL,                      /* 参数 */
        ACQ_TASK_PRIORITY,         /* 优先级 6 */
        s_acq_task_stack,          /* 栈内存 */
        &s_acq_task_tcb            /* TCB 内存 */
    );
    ESP_RETURN_ON_FALSE(s_acq_task_handle != NULL, ESP_FAIL,
                        TAG, "xTaskCreateStatic failed");

    ESP_LOGI(TAG, "acquisition task started (%u ms period, %u-sample ring buffer)",
             ACQ_PERIOD_MS, APP_CORE_SAMPLE_BUFFER_LEN);
    return ESP_OK;
}

/**
 * @brief 拷贝当前累计统计快照。
 */
esp_err_t app_core_get_stats(app_core_stats_t *out_stats)
{
    ESP_RETURN_ON_FALSE(out_stats != NULL, ESP_ERR_INVALID_ARG, TAG, "out_stats is NULL");

    taskENTER_CRITICAL(&s_sample_lock);
    *out_stats = s_stats; /* 整个结构体复制（值拷贝） */
    taskEXIT_CRITICAL(&s_sample_lock);
    return ESP_OK;
}

/**
 * @brief 拷贝环形缓冲区中最新一条融合样本。
 */
esp_err_t app_core_get_latest_sample(app_core_sample_t *out_sample)
{
    ESP_RETURN_ON_FALSE(out_sample != NULL, ESP_ERR_INVALID_ARG, TAG, "out_sample is NULL");

    taskENTER_CRITICAL(&s_sample_lock);

    if (!s_has_sample) {
        taskEXIT_CRITICAL(&s_sample_lock);
        return ESP_ERR_NOT_FOUND; /* 采集任务还没写过任何数据 */
    }

    /* 最新数据在写指针的上一个槽位：
     * 写指针指向"下一次要写的位置"，所以往回退 1 就是刚写完的位置。
     * +32-1 再 %32 是防止 s_write_index==0 时出现负数下标。 */
    uint32_t latest_index =
        (s_write_index + APP_CORE_SAMPLE_BUFFER_LEN - 1U) % APP_CORE_SAMPLE_BUFFER_LEN;

    *out_sample = s_sample_buf[latest_index]; /* 整个结构体值拷贝 */

    taskEXIT_CRITICAL(&s_sample_lock);
    return ESP_OK;
}
