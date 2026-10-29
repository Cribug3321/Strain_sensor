#include "strain_sensor.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define STRAIN_ADC_MAX_RAW       4095
#define STRAIN_MAX_SAMPLES       256
#define STRAIN_MAX_ZERO_READINGS 1024

static const char *TAG = "应变传感器";

struct strain_sensor_context {
    adc_oneshot_unit_handle_t adc;
    adc_cali_handle_t calibration;
    adc_channel_t channel;
    uint16_t samples_per_read;
    float output_to_adc_ratio;
    float zero_raw;
    float zero_output_mv;
    bool zero_valid;
};

esp_err_t strain_sensor_init(const strain_sensor_config_t *config,
                            strain_sensor_handle_t *out_handle)
{
    if (out_handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    *out_handle = NULL;
    if (config == NULL || config->samples_per_read == 0 ||
        config->samples_per_read > STRAIN_MAX_SAMPLES ||
        !isfinite(config->output_to_adc_ratio) ||
        config->output_to_adc_ratio < 1.0f || config->output_to_adc_ratio > 100.0f) {
        return ESP_ERR_INVALID_ARG;
    }

    adc_unit_t unit;
    adc_channel_t channel;
    esp_err_t err = adc_oneshot_io_to_channel(config->gpio_num, &unit, &channel);
    if (err != ESP_OK) {
        return err;
    }
    if (unit != ADC_UNIT_1) {
        ESP_LOGE(TAG, "GPIO%d 必须属于 ADC1", config->gpio_num);
        return ESP_ERR_INVALID_ARG;
    }

    strain_sensor_handle_t handle = calloc(1, sizeof(*handle));
    if (handle == NULL) {
        return ESP_ERR_NO_MEM;
    }
    handle->channel = channel;
    handle->samples_per_read = config->samples_per_read;
    handle->output_to_adc_ratio = config->output_to_adc_ratio;

    const adc_oneshot_unit_init_cfg_t unit_config = {
        .unit_id = unit,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    err = adc_oneshot_new_unit(&unit_config, &handle->adc);
    if (err != ESP_OK) {
        free(handle);
        return err;
    }

    const adc_oneshot_chan_cfg_t channel_config = {
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_12,
    };
    err = adc_oneshot_config_channel(handle->adc, channel, &channel_config);
    if (err != ESP_OK) {
        goto fail;
    }

#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    const adc_cali_curve_fitting_config_t calibration_config = {
        .unit_id = unit,
        .chan = channel,
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_12,
    };
    err = adc_cali_create_scheme_curve_fitting(&calibration_config,
                                              &handle->calibration);
#else
    err = ESP_ERR_NOT_SUPPORTED;
#endif
    if (err == ESP_ERR_NOT_SUPPORTED) {
        ESP_LOGW(TAG, "ADC 校准不可用，仅显示原始计数");
    } else if (err != ESP_OK) {
        goto fail;
    }

    ESP_LOGI(TAG, "GPIO%d 对应 ADC1 通道 %d，分辨率 12 位，衰减 12 dB，每组平均 %u 次",
             config->gpio_num, (int)channel, (unsigned)config->samples_per_read);
    *out_handle = handle;
    return ESP_OK;

fail:
    adc_oneshot_del_unit(handle->adc);
    free(handle);
    return err;
}

esp_err_t strain_sensor_read(strain_sensor_handle_t handle,
                            strain_sensor_reading_t *out_reading)
{
    if (out_reading == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    memset(out_reading, 0, sizeof(*out_reading));
    if (handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    strain_sensor_reading_t reading = {
        .raw_min = STRAIN_ADC_MAX_RAW,
        .zero_valid = handle->zero_valid,
    };
    uint32_t raw_sum = 0;
    for (uint16_t i = 0; i < handle->samples_per_read; ++i) {
        int raw;
        esp_err_t err = adc_oneshot_read(handle->adc, handle->channel, &raw);
        if (err != ESP_OK) {
            return err;
        }
        raw_sum += raw;
        if (raw < reading.raw_min) {
            reading.raw_min = raw;
        }
        if (raw > reading.raw_max) {
            reading.raw_max = raw;
        }
    }
    reading.raw = (raw_sum + handle->samples_per_read / 2) / handle->samples_per_read;
    // 端点状态仅用于提示；继续换算，但限幅后的数值不能代表完整信号。
    if (reading.raw_min == 0 && reading.raw_max == STRAIN_ADC_MAX_RAW) {
        reading.range = STRAIN_RANGE_BOTH_LIMITS;
    } else if (reading.raw_min == 0) {
        reading.range = STRAIN_RANGE_LOW_LIMIT;
    } else if (reading.raw_max == STRAIN_ADC_MAX_RAW) {
        reading.range = STRAIN_RANGE_HIGH_LIMIT;
    }
    if (handle->calibration != NULL) {
        esp_err_t err = adc_cali_raw_to_voltage(handle->calibration, reading.raw,
                                               &reading.adc_mv);
        if (err != ESP_OK) {
            return err;
        }
        reading.voltage_valid = true;
        reading.output_mv = reading.adc_mv * handle->output_to_adc_ratio;
    }
    if (handle->zero_valid) {
        reading.delta_raw = reading.raw - handle->zero_raw;
        if (reading.voltage_valid) {
            reading.delta_output_mv = reading.output_mv - handle->zero_output_mv;
        }
    }

    *out_reading = reading;
    return ESP_OK;
}

esp_err_t strain_sensor_zero(strain_sensor_handle_t handle, uint16_t readings,
                            uint32_t interval_ms)
{
    if (handle == NULL || readings == 0 || readings > STRAIN_MAX_ZERO_READINGS ||
        interval_ms > 1000) {
        return ESP_ERR_INVALID_ARG;
    }
    float raw_sum = 0;
    float output_mv_sum = 0;
    bool endpoint_seen = false;
    TickType_t delay_ticks = pdMS_TO_TICKS(interval_ms);
    if (interval_ms > 0 && delay_ticks == 0) {
        delay_ticks = 1;
    }

    for (uint16_t i = 0; i < readings; ++i) {
        strain_sensor_reading_t reading;
        esp_err_t err = strain_sensor_read(handle, &reading);
        if (err != ESP_OK) {
            return err;
        }
        if (reading.range != STRAIN_RANGE_NORMAL) {
            endpoint_seen = true;
        }
        raw_sum += reading.raw;
        output_mv_sum += reading.output_mv;
        if (delay_ticks > 0 && i + 1 < readings) {
            vTaskDelay(delay_ticks);
        }
    }

    // 全部采样成功后保存参考值；触及端点仅提示，不阻止归零。
    handle->zero_raw = raw_sum / readings;
    handle->zero_output_mv = output_mv_sum / readings;
    handle->zero_valid = true;
    if (endpoint_seen) {
        ESP_LOGW(TAG, "归零采样触及 ADC 的 0 或 4095 端点，已保存参考读数；"
                 "相对变化仅供观察，建议调整硬件后重新归零");
    }
    ESP_LOGI(TAG, "归零完成：零点原始值=%.1f",
             (double)handle->zero_raw);
    return ESP_OK;
}

esp_err_t strain_sensor_deinit(strain_sensor_handle_t handle)
{
    if (handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    // 先释放 ADC；若失败则保留可用句柄，允许调用方重试。
    esp_err_t err = adc_oneshot_del_unit(handle->adc);
    if (err != ESP_OK) {
        return err;
    }
#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    if (handle->calibration != NULL) {
        err = adc_cali_delete_scheme_curve_fitting(handle->calibration);
    }
#endif
    free(handle);
    return err;
}
