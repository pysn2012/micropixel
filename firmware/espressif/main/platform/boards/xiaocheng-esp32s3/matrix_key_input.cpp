#include "platform/boards/xiaocheng-esp32s3/matrix_key_input.hpp"

#include <cstddef>

#include "esp_check.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "platform/buses/i2c_executor.hpp"
#include "platform/boards/xiaocheng-esp32s3/board_config.hpp"
#include "platform/boards/xiaocheng-esp32s3/board_hardware.hpp"
#include "work/task_policy.hpp"

// Provided by the host App Hall when it is compiled in. Declared weak so a
// build without the Hall still links; both are null-checked before use.
extern "C" {
__attribute__((weak)) bool micropixel_hall_key_nav_step(bool forward);
__attribute__((weak)) bool micropixel_hall_key_nav_launch();
__attribute__((weak)) bool micropixel_hall_key_nav_active();
}

namespace micropixel::platform::xiaocheng_esp32s3 {
namespace {

constexpr char kTag[] = "xiaocheng_keys";
constexpr uint32_t kWorkerStackBytes = 4096U;

// True while the Hall page owns the screen; used to tell "a touch-only Guest
// is running" from "the Hall is up".
bool HallOwnsScreen() {
    return &micropixel_hall_key_nav_active != nullptr && micropixel_hall_key_nav_active();
}

// Synthetic tap point for the fallback press: center of the middle Hall card
// on the 320x240 landscape screen (cards 88x120, 8px gap, starting at x=12).
constexpr uint16_t kFallbackTapX = 152U;
constexpr uint16_t kFallbackTapY = 148U;
// A quick press is stretched to this so the Hall reliably sees the tap;
// longer holds pass through unchanged (hold-to-charge games get the real
// press duration).
constexpr uint32_t kFallbackTapHoldMs = 60U;
// Horizontal drag of one Hall carousel card step (card width 88 + gap 8).
constexpr int32_t kFallbackSwipeStepPx = 96;
// B held at least this long pages backwards instead of forwards.
constexpr uint32_t kBackLongPressMs = 700U;

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

bool MatrixKeyInput::AnyOtherKeyEmitted(size_t index) const {
    for (size_t other = 0; other < 6U; ++other) {
        if (other != index && emitted_[other]) {
            return true;
        }
    }
    return false;
}

void MatrixKeyInput::HandleFallback(size_t index, device::KeyCode code, bool pressed) {
    if (code == device::KeyCode::kConfirm) {
        if (pressed) {
            // The gesture router normally consumes the Hall launch; this is
            // the fallback for host pages it declines.
            if (&micropixel_hall_key_nav_launch != nullptr && micropixel_hall_key_nav_launch()) {
                return;
            }
            if (HallOwnsScreen()) {
                return;
            }
            EmulateHallPress(true);
            confirm_fallback_active_ = true;
        } else if (confirm_fallback_active_) {
            confirm_fallback_active_ = false;
            // If the two-button exit already brought the Hall back, releasing
            // the button must not fire a stray tap into it.
            if (!HallOwnsScreen()) {
                EmulateHallPress(false);
            }
        }
    } else if (code == device::KeyCode::kBack) {
        if (pressed) {
            back_down_ms_[index] = static_cast<uint32_t>(esp_timer_get_time() / 1000);
            back_pending_[index] = true;
        } else if (back_pending_[index]) {
            back_pending_[index] = false;
            const uint32_t held_ms = static_cast<uint32_t>(esp_timer_get_time() / 1000) - back_down_ms_[index];
            const bool forward = held_ms < kBackLongPressMs;
            if (&micropixel_hall_key_nav_step != nullptr && micropixel_hall_key_nav_step(forward)) {
                return;
            }
            if (HallOwnsScreen()) {
                return;
            }
            EmulateHallSwipe(forward);
        }
    }
}

void MatrixKeyInput::EmulateHallPress(bool pressed) {
    // The upstream App Hall only reacts to pointer input, and several store
    // games are touch-only as well. The confirm button therefore drives a real
    // press-and-hold rather than a fixed-length tap: the Hall still sees a tap
    // on a quick press, while hold-to-charge games (Jump Jump) receive the
    // actual press duration and can charge a jump.
    constexpr uint32_t kTapId = 1U;
    const uint32_t now_ms = static_cast<uint32_t>(esp_timer_get_time() / 1000);
    if (pressed) {
        confirm_down_ms_ = now_ms;
        (void)input_->InjectTouch({.timestamp_us = esp_timer_get_time(),
                                   .id = kTapId,
                                   .x = kFallbackTapX,
                                   .y = kFallbackTapY,
                                   .pressure_per_mille = 1000U,
                                   .phase = device::TouchPhase::kDown});
        return;
    }
    const uint32_t held_ms = now_ms - confirm_down_ms_;
    if (held_ms < kFallbackTapHoldMs) {
        vTaskDelay(pdMS_TO_TICKS(kFallbackTapHoldMs - held_ms));
    }
    (void)input_->InjectTouch({.timestamp_us = esp_timer_get_time(),
                               .id = kTapId,
                               .x = kFallbackTapX,
                               .y = kFallbackTapY,
                               .pressure_per_mille = 1000U,
                               .phase = device::TouchPhase::kUp});
}

void MatrixKeyInput::EmulateHallSwipe(bool forward) {
    // The App Hall carousel is a horizontally dragged LVGL list. A synthetic
    // drag of one card step pages the selection. The drag ends with two slow
    // 2-pixel moves so the release velocity stays near zero and LVGL does not
    // keep scrolling after the finger lifts.
    constexpr uint32_t kSwipeId = 2U;
    constexpr int32_t kSwipeY = 148;  // vertical center of a Hall card
    constexpr int32_t kAnchorX = 170;
    constexpr int32_t kFastSteps = 6;
    constexpr int32_t kTrailingPx = 2;
    const int32_t step = kFallbackSwipeStepPx;
    const int32_t direction = forward ? -1 : 1;
    const int32_t start_x = kAnchorX - direction * step / 2;
    const int32_t fast_total = step - 2 * kTrailingPx;
    int32_t x = start_x;
    (void)input_->InjectTouch({.timestamp_us = esp_timer_get_time(),
                               .id = kSwipeId,
                               .x = x,
                               .y = kSwipeY,
                               .pressure_per_mille = 1000U,
                               .phase = device::TouchPhase::kDown});
    for (int32_t i = 1; i <= kFastSteps; ++i) {
        vTaskDelay(pdMS_TO_TICKS(12U));
        x = start_x + direction * fast_total * i / kFastSteps;
        (void)input_->InjectTouch({.timestamp_us = esp_timer_get_time(),
                                   .id = kSwipeId,
                                   .x = x,
                                   .y = kSwipeY,
                                   .pressure_per_mille = 1000U,
                                   .phase = device::TouchPhase::kMove});
    }
    for (int32_t i = 0; i < 2; ++i) {
        vTaskDelay(pdMS_TO_TICKS(35U));
        x += direction * kTrailingPx;
        (void)input_->InjectTouch({.timestamp_us = esp_timer_get_time(),
                                   .id = kSwipeId,
                                   .x = x,
                                   .y = kSwipeY,
                                   .pressure_per_mille = 1000U,
                                   .phase = device::TouchPhase::kMove});
    }
    vTaskDelay(pdMS_TO_TICKS(35U));
    (void)input_->InjectTouch({.timestamp_us = esp_timer_get_time(),
                               .id = kSwipeId,
                               .x = x,
                               .y = kSwipeY,
                               .pressure_per_mille = 1000U,
                               .phase = device::TouchPhase::kUp});
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
            if (debounced == emitted_[key_index]) {
                continue;
            }
            emitted_[key_index] = debounced;
            if (debounced && AnyOtherKeyEmitted(key_index)) {
                // Both buttons are down: this is the Host's two-button exit
                // shortcut, not app input.
                chord_held_ = true;
            }
            const bool delivered = input_ != nullptr && input_->InjectKey({
                                                            .timestamp_us = static_cast<uint64_t>(esp_timer_get_time()),
                                                            .code = entry.code,
                                                            .phase = debounced ? device::KeyPhase::kDown
                                                                               : device::KeyPhase::kUp,
                                                            .repeat_count = 0U,
                                                    });
            ESP_LOGD(kTag, "%s %s", entry.name, debounced ? "down" : "up");
            if (delivered || chord_held_) {
                // Consumed: either a key-driven guest handled it, or the
                // gesture router used it for the Hall navigation / the
                // two-button exit (a Hall launch must not be followed by a
                // stray synthetic tap). Releasing either button of the exit
                // chord must not also page the Hall or slide its carousel.
                continue;
            }
            // No key consumer is bound: a touch-driven host page is showing
            // and no guest app runs. Confirm and Back fall back to Hall
            // navigation or synthetic touches.
            HandleFallback(key_index, entry.code, debounced);
        }
    }
    bool any_emitted = false;
    for (const bool emitted : emitted_) {
        any_emitted = any_emitted || emitted;
    }
    if (!any_emitted) {
        chord_held_ = false;
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
