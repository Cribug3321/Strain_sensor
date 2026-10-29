#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct strain_sensor_context *strain_sensor_handle_t;

typedef enum {
    STRAIN_RANGE_NORMAL = 0,
    STRAIN_RANGE_LOW_LIMIT,
    STRAIN_RANGE_HIGH_LIMIT,
    STRAIN_RANGE_BOTH_LIMITS,
} strain_sensor_range_t;

typedef struct {
    int gpio_num;                 // ESP32-S3 的 GPIO4 对应 ADC1 通道 3。
    uint16_t samples_per_read;     // 每组平均使用的采样次数，范围为 1～256。
    float output_to_adc_ratio;    // 模块输出电压与 ADC 输入电压之比，直连为 1，等值分压为 2。
} strain_sensor_config_t;

typedef struct {
    int raw;                      // 本组平均值四舍五入后的 12 位计数，范围为 0～4095。
    int raw_min;                  // 本组采样最小值。
    int raw_max;                  // 本组采样最大值。
    strain_sensor_range_t range;  // 按单次采样极值检查端点，避免平均值掩盖限幅。
    int adc_mv;                   // ADC 换算电压，需 voltage_valid；触及端点时仅供观察。
    float output_mv;              // ADC 换算电压乘以实际分压比例，端点时不代表真实 OUT。
    float delta_raw;              // 相对参考值的有符号计数差，需已归零；端点时仍计算。
    float delta_output_mv;        // 有符号换算电压差，还要求 voltage_valid 为真。
    bool voltage_valid;          // 已通过 ADC 校准 API 完成电压换算，不代表信号未限幅。
    bool zero_valid;             // 已保存参考值，不保证参考值或当前采样未限幅。
} strain_sensor_reading_t;

// 所有接口应由同一个任务调用；本组件独占所选 ADC1 单元。
// 芯片校准数据不可用时，仅提供原始计数，不提供电压。
esp_err_t strain_sensor_init(const strain_sensor_config_t *config,
                            strain_sensor_handle_t *out_handle);
// 失败时清空输出结构，不得把上次读数当成当前读数。
// 触及端点时仍换算电压、计算相对变化并返回量程状态；限幅值仅供观察。
esp_err_t strain_sensor_read(strain_sensor_handle_t handle,
                            strain_sensor_reading_t *out_reading);
// 保持试件在参考状态采集零点；读取失败时保留已有零点。
// 触及 0 或 4095 时仅提示，仍保存参考值；限幅参考值的后续变化仅供观察。
// 不设置零点推荐上下限；归零不保证参考值或后续形变都在量程内。
// interval_ms 是相邻采样组的等待时间，并非精确采样周期。
esp_err_t strain_sensor_zero(strain_sensor_handle_t handle, uint16_t readings,
                            uint32_t interval_ms);
esp_err_t strain_sensor_deinit(strain_sensor_handle_t handle);

#ifdef __cplusplus
}
#endif
