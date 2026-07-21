#ifndef APP_CORE_H   /* 头文件保护：防止同一个 .h 被重复 #include 两次导致重复定义 */
#define APP_CORE_H   /* 第一次包含时定义这个宏；第二次包含时宏已存在，整个文件被跳过 */

/**
 * @file app_core.h
 * @brief 数据融合层对外接口（公共 API）。
 *
 * 这是一个"头文件"，只包含：类型定义、常量、函数声明。
 * 真正的实现代码在 app_core.c 中。
 * 其他模块只需要 #include "app_core.h" 就能使用这里声明的函数。
 */

/* ---- 标准库头文件 -------------------------------------------------------
 * stdbool.h 提供 bool / true / false 类型。
 * stdint.h  提供 uint8_t / uint16_t / uint32_t 等固定宽度整数类型。
 * 使用固定宽度类型是嵌入式开发的规范：明确知道每个变量占几个字节。
 * ----------------------------------------------------------------------- */
#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"   /* esp_err_t 类型（本质是 int32_t）和错误码常量 */
#include "ld2410c.h"   /* 引入雷达数据结构 ld2410c_data_t */
#include "sht30.h"     /* 引入温湿度数据结构 sht30_data_t */

/* ---- C++ 兼容性 ---------------------------------------------------------
 * ESP32 项目有时会混合 C 和 C++ 代码。
 * extern "C" { ... } 告诉 C++ 编译器：这段代码用 C 的命名规则链接，
 * 防止 C++ 的"名字修饰（name mangling）"破坏函数名。
 * 纯 C 项目中这段代码不起任何作用。
 * ----------------------------------------------------------------------- */
#ifdef __cplusplus
extern "C" {
#endif

/* 环形缓冲区容量：最多保留 32 条融合样本
 * U 后缀表示这是 unsigned（无符号）整数字面量，避免编译器警告 */
#define APP_CORE_SAMPLE_BUFFER_LEN  32U

/* ---- 六维特征向量维度定义（cursorrules §4）------------------------------
 * TFLM 模型输入固定为 6 维，顺序不可更改：
 *   [0] 温度      [1] 湿度
 *   [2] 运动目标距离  [3] 静止目标距离
 *   [4] 运动目标能量  [5] 静止目标能量
 * 用宏定义下标，避免代码里出现"魔法数字"，也方便后续 Phase4 引用。
 * ----------------------------------------------------------------------- */
#define APP_CORE_FEATURE_DIM              6U
#define APP_CORE_FEATURE_IDX_TEMPERATURE  0U
#define APP_CORE_FEATURE_IDX_HUMIDITY     1U
#define APP_CORE_FEATURE_IDX_MOVE_DIST    2U
#define APP_CORE_FEATURE_IDX_STATIC_DIST  3U
#define APP_CORE_FEATURE_IDX_MOVE_ENERGY  4U
#define APP_CORE_FEATURE_IDX_STATIC_ENERGY 5U

/* ---- 归一化物理量范围（cursorrules §5）----------------------------------
 * 温度 [-40, 80] °C，湿度 [0, 100] %，距离 [0, 600] cm，能量 [0, 100]。
 * 归一化后统一映射到 [0, 1]，超出范围的值会被裁剪（clamp）到边界。
 * ----------------------------------------------------------------------- */
#define APP_CORE_TEMP_MIN_C     (-40.0f)
#define APP_CORE_TEMP_MAX_C     (80.0f)
#define APP_CORE_HUMIDITY_MIN   (0.0f)
#define APP_CORE_HUMIDITY_MAX   (100.0f)
#define APP_CORE_DISTANCE_MIN_CM (0.0f)
#define APP_CORE_DISTANCE_MAX_CM (600.0f)
#define APP_CORE_ENERGY_MIN     (0.0f)
#define APP_CORE_ENERGY_MAX     (100.0f)

/**
 * @brief 一条融合环境+雷达采样记录。
 *
 * typedef struct { ... } 名字;  是 C 语言定义"结构体类型别名"的标准写法。
 * 之后就可以直接写 app_core_sample_t 而不用每次写 struct app_core_sample_t。
 */
typedef struct {
    sht30_data_t   env;          /**< SHT30 本次测量结果（温度+湿度+valid标志） */
    ld2410c_data_t radar;        /**< LD2410C 本次上报帧（距离+能量+目标状态） */
    esp_err_t      env_status;   /**< sht30_read() 的返回值；ESP_OK 表示成功 */
    esp_err_t      radar_status; /**< ld2410c_read() 的返回值 */
    uint32_t       sequence;     /**< 单调递增的样本序号，从 1 开始，不会回绕 */
    uint32_t       timestamp_ms; /**< FreeRTOS tick 换算成的毫秒时间戳 */
    bool           valid;        /**< 该槽位是否已被写入过（启动后前几帧可能还没有数据） */
} app_core_sample_t;

/**
 * @brief 传感器采集健康度统计计数器。
 *
 * 所有字段均为累计值，随运行时间单调增加。
 * 连续失败计数（consecutive）在成功时归零，便于检测当前是否处于故障状态。
 */
typedef struct {
    uint32_t env_success_count;              /**< SHT30 累计成功采样次数 */
    uint32_t env_failure_count;              /**< SHT30 累计失败次数（含所有错误类型） */
    uint32_t env_consecutive_failures;       /**< SHT30 当前连续失败次数（成功后清零） */
    uint32_t radar_success_count;            /**< LD2410C 累计成功帧数 */
    uint32_t radar_timeout_count;            /**< UART 50ms 内未收到帧头的次数 */
    uint32_t radar_invalid_response_count;   /**< 收到数据但帧结构/校验不合法的次数 */
    uint32_t radar_other_failure_count;      /**< 其他类型的雷达错误次数 */
    uint32_t radar_consecutive_failures;     /**< 雷达当前连续失败次数 */
} app_core_stats_t;

/**
 * @brief 归一化后的六维特征向量，作为 Phase4 TFLM 模型的输入。
 *
 * 所有分量均已归一化到 [0, 1]，顺序固定为 cursorrules 第 4 节规定的六维：
 *   feature[0]=温度      feature[1]=湿度
 *   feature[2]=运动距离  feature[3]=静止距离
 *   feature[4]=运动能量  feature[5]=静止能量
 *
 * env_ok / radar_ok 指示本次特征是否来自成功采样；当某一路失败时，
 * 对应分量以 0 填充并把标志置 false，供上层决定是否使用该特征。
 */
typedef struct {
    float    feature[APP_CORE_FEATURE_DIM]; /**< 归一化到 [0,1] 的六维特征 */
    uint32_t sequence;                       /**< 对应融合样本的序号 */
    uint32_t timestamp_ms;                   /**< 对应融合样本的时间戳 */
    bool     env_ok;                         /**< 温湿度分量是否有效 */
    bool     radar_ok;                       /**< 雷达分量是否有效 */
} app_core_feature_t;

/* ---- 函数声明（原型）----------------------------------------------------
 * 函数声明告诉编译器"这个函数存在、参数是什么、返回值是什么"。
 * 真正的函数体（实现）在 app_core.c 里。
 * ----------------------------------------------------------------------- */

/**
 * @brief 初始化所有外设驱动（SHT30 / LD2410C / ST7789）并清零内部状态。
 *
 * 即使某个外设初始化失败，函数仍继续初始化其他外设并返回 ESP_OK，
 * 以便在没有全部接线的情况下也能部分验证。
 *
 * @return ESP_OK（目前总是返回 OK，失败以 LOGW 警告提示）。
 */
esp_err_t app_core_init(void);

/**
 * @brief 创建并启动 100ms 周期的静态传感器采集任务。
 *
 * 任务使用静态分配的栈和 TCB，不占用 FreeRTOS 堆内存。
 * 只能调用一次；重复调用返回 ESP_ERR_INVALID_STATE。
 *
 * @return ESP_OK 成功，ESP_ERR_INVALID_STATE 任务已运行，ESP_FAIL 创建失败。
 */
esp_err_t app_core_start_acquisition(void);

/**
 * @brief 从环形缓冲区中拷贝最新的融合样本。
 *
 * 非阻塞：如果采集任务还没完成第一次采样，返回 ESP_ERR_NOT_FOUND。
 * 使用临界区保护读操作，可以在任意任务中调用。
 *
 * @param[out] out_sample 接收数据的目标结构体指针，不能为 NULL。
 *             [out] 是 Doxygen 约定，表示"这是一个输出参数"。
 * @return ESP_OK / ESP_ERR_INVALID_ARG / ESP_ERR_NOT_FOUND。
 */
esp_err_t app_core_get_latest_sample(app_core_sample_t *out_sample);

/**
 * @brief 拷贝当前累计统计数据的快照。
 *
 * @param[out] out_stats 接收统计的目标结构体指针，不能为 NULL。
 * @return ESP_OK / ESP_ERR_INVALID_ARG。
 */
esp_err_t app_core_get_stats(app_core_stats_t *out_stats);

/**
 * @brief 将一条融合样本转换为归一化的六维特征向量。
 *
 * 按 cursorrules 第 5 节的范围把物理量线性映射到 [0,1]：
 *   温度 [-40,80]、湿度 [0,100]、距离 [0,600]、能量 [0,100]。
 * 超出范围的输入会被裁剪到边界。当某一路采样失败时，对应分量填 0
 * 并把 env_ok / radar_ok 置 false。本函数是纯计算，无副作用。
 *
 * @param[in]  sample   源融合样本，不能为 NULL。
 * @param[out] out_feat 归一化特征输出，不能为 NULL。
 * @return ESP_OK / ESP_ERR_INVALID_ARG。
 */
esp_err_t app_core_build_feature(const app_core_sample_t *sample,
                                 app_core_feature_t *out_feat);

/**
 * @brief 读取最新融合样本并直接输出归一化特征向量。
 *
 * 相当于 app_core_get_latest_sample() 后紧接 app_core_build_feature()。
 * 非阻塞；采集任务尚未产生数据时返回 ESP_ERR_NOT_FOUND。
 *
 * @param[out] out_feat 归一化特征输出，不能为 NULL。
 * @return ESP_OK / ESP_ERR_INVALID_ARG / ESP_ERR_NOT_FOUND。
 */
esp_err_t app_core_get_latest_feature(app_core_feature_t *out_feat);

/**
 * @brief 对最近 count 条样本的归一化特征做时域滑动平均，抑制传感器噪声。
 *
 * 从环形缓冲区取出最近 count 条有效样本，逐条构造六维归一化特征，
 * 再按维度求平均。只有采样成功的那一路才计入该维度的平均：
 *   - out_feat->feature[i] = 各有效样本第 i 维之和 / 该维有效样本数
 *   - env_ok / radar_ok 表示这一路是否至少有一条有效样本参与平均
 * 若某一路全部失败，则对应分量为 0 且标志为 false。
 *
 * 本函数只读取环形缓冲区并做纯计算，不修改任何共享状态。
 *
 * @param[in]  count    参与平均的样本条数，取值 1..APP_CORE_SAMPLE_BUFFER_LEN。
 * @param[out] out_feat 平均后的归一化特征输出，不能为 NULL。
 * @return ESP_OK / ESP_ERR_INVALID_ARG / ESP_ERR_NOT_FOUND。
 */
esp_err_t app_core_get_averaged_feature(uint32_t count,
                                        app_core_feature_t *out_feat);

#ifdef __cplusplus
}
#endif

#endif  /* APP_CORE_H — 与文件顶部的 #ifndef 配对，标志头文件保护结束 */
