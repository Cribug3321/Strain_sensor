#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "strain_sensor.h"

// ESP32-S3 的 GPIO4 对应 ADC1 通道 3，接入前须核对整个输出电压范围。
#define STRAIN_ADC_GPIO             4
#define STRAIN_SAMPLES_PER_READ     32
// 按当前接线：模块由 3.3 V 供电，OUT 直连 GPIO4；供电规格仍须以厂家说明为准。
// 若实际增加 OUT -- 10kΩ -- GPIO4 -- 10kΩ -- GND 分压，再把比例改为 2.0f。
// 此比例只用于电压换算，不能扩大硬件量程，也不能消除模块放大器饱和。
#define STRAIN_OUTPUT_TO_ADC_RATIO  1.0f
#define STRAIN_STARTUP_DELAY_MS     3000
#define STRAIN_ZERO_READINGS        32
#define STRAIN_ZERO_INTERVAL_MS     20
// 采集与串口显示分别调度；读数变化时才输出，50ms 窗口限制最大输出频率。
// 100 Hz 是采样组的目标频率，不是 ADC 的硬件上限。
#define STRAIN_SAMPLE_INTERVAL_MS   10
#define STRAIN_REPORT_INTERVAL_MS   50
#define STRAIN_REPORT_STACK_BYTES   4096

_Static_assert(pdMS_TO_TICKS(STRAIN_SAMPLE_INTERVAL_MS) > 0,
               "采集周期不能小于一个 FreeRTOS 调度节拍");

typedef struct {
    strain_sensor_reading_t latest; // 显示窗口内最后一组有效读数。
    int raw_min;                    // 整个显示窗口内的单次采样极值。
    int raw_max;
    uint32_t valid_groups;
    uint32_t failed_groups;
    uint32_t overruns;              // 采集任务未能按期进入等待的次数。
    esp_err_t last_error;
} strain_report_window_t;

static const char *TAG = "应变采集";

static strain_sensor_range_t report_window_range(const strain_report_window_t *window)
{
    if (window->raw_min == 0 && window->raw_max == 4095) {
        return STRAIN_RANGE_BOTH_LIMITS;
    }
    if (window->raw_min == 0) {
        return STRAIN_RANGE_LOW_LIMIT;
    }
    if (window->raw_max == 4095) {
        return STRAIN_RANGE_HIGH_LIMIT;
    }
    return STRAIN_RANGE_NORMAL;
}

static bool reading_changed(const strain_sensor_reading_t *current,
                            const strain_sensor_reading_t *previous)
{
    // 精确比较实际读数，不设变化阈值；组内极值波动本身不触发重复日志。
    return current->raw != previous->raw ||
           current->voltage_valid != previous->voltage_valid ||
           current->zero_valid != previous->zero_valid ||
           (current->voltage_valid &&
            (current->adc_mv != previous->adc_mv ||
             current->output_mv != previous->output_mv)) ||
           (current->zero_valid && current->delta_raw != previous->delta_raw) ||
           (current->voltage_valid && current->zero_valid &&
            current->delta_output_mv != previous->delta_output_mv);
}

static void report_reading(const strain_sensor_reading_t *reading)
{
    if (reading->voltage_valid && reading->zero_valid) {
        ESP_LOGI(TAG, "原始值=%d [%d..%d]，引脚电压=%d mV，模块输出=%.1f mV，"
                 "输出变化=%+.1f mV，计数变化=%+.1f",
                 reading->raw, reading->raw_min, reading->raw_max, reading->adc_mv,
                 (double)reading->output_mv, (double)reading->delta_output_mv,
                 (double)reading->delta_raw);
    } else if (reading->voltage_valid) {
        ESP_LOGI(TAG, "原始值=%d [%d..%d]，引脚电压=%d mV，模块输出=%.1f mV，"
                 "变化量=不可用（尚未归零）",
                 reading->raw, reading->raw_min, reading->raw_max, reading->adc_mv,
                 (double)reading->output_mv);
    } else if (reading->zero_valid) {
        ESP_LOGI(TAG, "原始值=%d [%d..%d]，计数变化=%+.1f，"
                 "电压=不可用（缺少 ADC 校准）",
                 reading->raw, reading->raw_min, reading->raw_max,
                 (double)reading->delta_raw);
    } else {
        ESP_LOGI(TAG, "原始值=%d [%d..%d]，变化量=不可用（尚未归零），"
                 "电压=不可用（缺少 ADC 校准）",
                 reading->raw, reading->raw_min, reading->raw_max);
    }
}

static void report_task(void *arg)
{
    QueueHandle_t queue = (QueueHandle_t)arg;
    strain_report_window_t window;
    strain_sensor_reading_t previous_reading = {0};
    bool have_previous_reading = false;
    esp_err_t previous_error = ESP_OK;
    bool previous_overrun = false;
    strain_sensor_range_t previous_range = STRAIN_RANGE_NORMAL;
    for (;;) {
        if (xQueueReceive(queue, &window, portMAX_DELAY) != pdTRUE) {
            continue;
        }
        esp_err_t current_error = window.failed_groups > 0 ? window.last_error : ESP_OK;
        if (current_error != previous_error) {
            if (current_error != ESP_OK) {
                ESP_LOGW(TAG, "最近显示窗口 ADC 读取失败 %lu 组（%s），失败数据已丢弃",
                         (unsigned long)window.failed_groups,
                         esp_err_to_name(current_error));
            } else {
                ESP_LOGI(TAG, "ADC 读取已恢复");
            }
            previous_error = current_error;
        }
        bool current_overrun = window.overruns > 0;
        if (current_overrun != previous_overrun) {
            if (current_overrun) {
                ESP_LOGW(TAG, "采集任务 %lu 次未能在 %d 毫秒周期内进入等待，实际采集频率可能低于目标",
                         (unsigned long)window.overruns, STRAIN_SAMPLE_INTERVAL_MS);
            } else {
                ESP_LOGI(TAG, "采集任务已恢复按期等待");
            }
            previous_overrun = current_overrun;
        }
        if (window.valid_groups == 0) {
            continue;
        }

        // 检查整个显示窗口；持续限幅只提示一次，量程状态改变时再提示。
        strain_sensor_range_t current_range = report_window_range(&window);
        if (current_range != previous_range) {
            if (current_range == STRAIN_RANGE_NORMAL) {
                ESP_LOGI(TAG, "最近显示窗口未触及 ADC 端点");
            } else {
                const char *limit = "触及 ADC 端点 0 和 4095";
                if (current_range == STRAIN_RANGE_HIGH_LIMIT) {
                    limit = "触及 ADC 端点 4095";
                } else if (current_range == STRAIN_RANGE_LOW_LIMIT) {
                    limit = "触及 ADC 端点 0";
                }
                ESP_LOGW(TAG, "最近显示窗口原始范围=[%d..%d]，状态=%s；"
                         "继续显示 ADC 换算值，电压及相对变化仅供观察，请检查实际 OUT 电压",
                         window.raw_min, window.raw_max, limit);
            }
            previous_range = current_range;
        }
        if (!have_previous_reading || reading_changed(&window.latest, &previous_reading)) {
            report_reading(&window.latest);
            previous_reading = window.latest;
            have_previous_reading = true;
        }
    }
}

void app_main(void)
{
    const strain_sensor_config_t config = {
        .gpio_num = STRAIN_ADC_GPIO,
        .samples_per_read = STRAIN_SAMPLES_PER_READ,
        .output_to_adc_ratio = STRAIN_OUTPUT_TO_ADC_RATIO,
    };
    strain_sensor_handle_t sensor = NULL;
    esp_err_t err = strain_sensor_init(&config, &sensor);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "传感器初始化失败：%s", esp_err_to_name(err));
        return;
    }

    ESP_LOGI(TAG, "请保持试件在参考状态，%d 毫秒后开始归零", STRAIN_STARTUP_DELAY_MS);
    ESP_LOGI(TAG, "电压还原比例=%.3f；须与实际接线一致",
             (double)STRAIN_OUTPUT_TO_ADC_RATIO);
    vTaskDelay(pdMS_TO_TICKS(STRAIN_STARTUP_DELAY_MS));
    err = strain_sensor_zero(sensor, STRAIN_ZERO_READINGS, STRAIN_ZERO_INTERVAL_MS);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "启动归零未完成（%s），当前仅显示绝对读数；"
                 "调整硬件后，保持试件在参考状态并复位重新归零",
                 esp_err_to_name(err));
    }

    // 单元素队列只保留最新显示窗口，串口拥堵时不会阻塞 ADC 采集。
    QueueHandle_t report_queue = xQueueCreate(1, sizeof(strain_report_window_t));
    if (report_queue == NULL) {
        ESP_LOGE(TAG, "显示队列创建失败，停止采集");
        strain_sensor_deinit(sensor);
        return;
    }
    if (xTaskCreate(report_task, "strain_report", STRAIN_REPORT_STACK_BYTES,
                    report_queue, tskIDLE_PRIORITY + 1, NULL) != pdPASS) {
        ESP_LOGE(TAG, "显示任务创建失败，停止采集");
        vQueueDelete(report_queue);
        strain_sensor_deinit(sensor);
        return;
    }

    ESP_LOGI(TAG, "目标每 %d 毫秒采集一组，每组 %d 次平均；读数变化才输出，显示窗口为 %d 毫秒",
             STRAIN_SAMPLE_INTERVAL_MS, STRAIN_SAMPLES_PER_READ,
             STRAIN_REPORT_INTERVAL_MS);
    // ADC 的初始化、归零和读取始终由本任务串行执行；显示任务只接收数据副本。
    vTaskPrioritySet(NULL, tskIDLE_PRIORITY + 2);
    const TickType_t sample_ticks = pdMS_TO_TICKS(STRAIN_SAMPLE_INTERVAL_MS);
    const TickType_t report_ticks = pdMS_TO_TICKS(STRAIN_REPORT_INTERVAL_MS);
    TickType_t last_wake = xTaskGetTickCount();
    TickType_t last_report = last_wake;
    strain_report_window_t window = {.raw_min = 4095};
    for (;;) {
        strain_sensor_reading_t reading;
        err = strain_sensor_read(sensor, &reading);
        if (err != ESP_OK) {
            ++window.failed_groups;
            window.last_error = err;
        } else {
            window.latest = reading;
            ++window.valid_groups;
            if (reading.raw_min < window.raw_min) {
                window.raw_min = reading.raw_min;
            }
            if (reading.raw_max > window.raw_max) {
                window.raw_max = reading.raw_max;
            }
        }

        TickType_t now = xTaskGetTickCount();
        if ((TickType_t)(now - last_report) >= report_ticks) {
            xQueueOverwrite(report_queue, &window);
            window = (strain_report_window_t){.raw_min = 4095};
            last_report = now;
        }
        if (xTaskDelayUntil(&last_wake, sample_ticks) == pdFALSE) {
            ++window.overruns;
            // 落后时重新同步并让出 CPU，避免紧密补采导致空闲任务无法运行。
            vTaskDelay(1);
            last_wake = xTaskGetTickCount();
        }
    }
}
