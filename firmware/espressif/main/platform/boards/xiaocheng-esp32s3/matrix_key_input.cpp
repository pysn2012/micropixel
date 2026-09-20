#include "platform/boards/xiaocheng-esp32s3/matrix_key_input.hpp"

#include <cstddef>

#include "esp_check.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "platform/buses/i2c_executor.hpp"
#include "platform/boards/xiaocheng-esp32s3/board_config.hpp"
#include "platform/boards/xiaocheng-esp32s3/board_hardware.hpp"
#include "work/task_policy.hpp"

namespace micropixel::platform::xiaocheng_esp32s3 {
namespace {

constexpr char kTag[] = "xiaocheng_keys";
constexpr uint32_t kWorkerStackBytes = 4096U;

}  // namespace

MatrixKeyInput::~MatrixKeyInput() { Stop(); }

esp_err_t MatrixKeyInput::Initialize(device::Input& input) {
    if (input_ != nullptr || worker_ != nullptr) {
        return ESP_ERR_INVALID_STATE;
    }
    gpio_config_t rows_config{};
    rows_config.pin_bit_mask = 0U;
    for (const gpio_num_t row : kMatrixRows) {
        rows_config.pin_bit_mask |= BIT64(row);
    }
    rows_config.mode = GPIO_MODE_OUTPUT;
    rows_config.pull_up_en = GPIO_PULLUP_ENABLE;
    rows_config.pull_down_en = GPIO_PULLDOWN_DISABLE;
    rows_config.intr_type = GPIO_INTR_DISABLE;
    ESP_RETURN_ON_ERROR(gpio_config(&rows_config), kTag, "configure matrix rows failed");
    for (const gpio_num_t row : kMatrixRows) {
        (void)gpio_set_level(row, 1);  // idle high
    }

    worker_stopped_ = xSemaphoreCreateBinary();
    if (worker_stopped_ == nullptr) {
        return ESP_ERR_NO_MEM;
    }
    input_ = &input;
    stopping_.store(false, std::memory_order_release);
    if (xTaskCreatePinnedToCore(WorkerEntry, "xiaocheng_keys", kWorkerStackBytes, this, task_policy::kHostPriority,
                                &worker_, task_policy::kSystemCore) != pdPASS) {
        worker_ = nullptr;
        input_ = nullptr;
        Stop();
        return ESP_ERR_NO_MEM;
    }
    ESP_LOGI(kTag, "matrix key input ready: 2 rows x 3 XL9535 columns");
    return ESP_OK;
}

esp_err_t MatrixKeyInput::ReadPorts(void* context) {
    auto& scan = *static_cast<ScanContext*>(context);
    scan.status = scan.hardware->ReadInputPorts(scan.ports);
    return scan.status;
}

void MatrixKeyInput::Scan() {
    for (size_t row_index = 0; row_index < 2U; ++row_index) {
        const gpio_num_t row = kMatrixRows[row_index];
        (void)gpio_set_level(row, 0);  // drive this row low
        ScanContext scan{&hardware_, 0xFFFFU, ESP_OK};
        const esp_err_t status = executor_.Invoke(buses::I2cExecutor::Priority::kLow, ReadPorts, &scan);
        (void)gpio_set_level(row, 1);  // back to idle high
        if (status != ESP_OK || scan.status != ESP_OK) {
            ESP_LOGW(kTag, "matrix scan read failed: %s", esp_err_to_name(status != ESP_OK ? status : scan.status));
            continue;
        }
        for (size_t key_index = 0; key_index < 6U; ++key_index) {
            const MatrixKeyEntry& entry = kMatrixKeys[key_index];
            if (entry.row != row) {
                continue;
            }
            // Columns are inputs; a pressed key pulls its column low.
            const bool pressed = (scan.ports & entry.column_bit) == 0U;
            const bool debounced = pressed && last_raw_[key_index];
            last_raw_[key_index] = pressed;
            if (input_ != nullptr && debounced != emitted_[key_index]) {
                emitted_[key_index] = debounced;
                (void)input_->InjectKey({
                    .timestamp_us = static_cast<uint64_t>(esp_timer_get_time()),
                    .code = entry.code,
                    .phase = debounced ? device::KeyPhase::kDown : device::KeyPhase::kUp,
                    .repeat_count = 0U,
                });
                ESP_LOGD(kTag, "%s %s", entry.name, debounced ? "down" : "up");
            }
        }
    }
}

void MatrixKeyInput::WorkerEntry(void* context) {
    auto* keys = static_cast<MatrixKeyInput*>(context);
    while (!keys->stopping_.load(std::memory_order_acquire)) {
        keys->Scan();
        vTaskDelay(pdMS_TO_TICKS(kMatrixScanPeriodMs));
    }
    xSemaphoreGive(keys->worker_stopped_);
    vTaskDelete(nullptr);
}

void MatrixKeyInput::Stop() {
    if (worker_ != nullptr) {
        stopping_.store(true, std::memory_order_release);
        (void)xSemaphoreTake(worker_stopped_, pdMS_TO_TICKS(kMatrixScanPeriodMs + 100U));
        worker_ = nullptr;
    }
    if (worker_stopped_ != nullptr) {
        vSemaphoreDelete(worker_stopped_);
        worker_stopped_ = nullptr;
    }
    input_ = nullptr;
}

}  // namespace micropixel::platform::xiaocheng_esp32s3
