#pragma once

#include <atomic>
#include <cstdint>

#include "device/contracts/input.hpp"
#include "driver/i2c_master.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

namespace micropixel::platform::buses {
class I2cExecutor;
}

namespace micropixel::platform::xiaocheng_esp32s3 {

class BoardHardware;

// Scans the 2-row x 3-column key matrix. Rows are native GPIO outputs driven
// low one at a time; columns read back through the XL9535 input port. Emits
// debounced key events through the shared device::Input router.
//
// Button-only-board integration (ported from the community H1 kit): keys go
// through InjectKey first; whatever no consumer picked up falls back to Hall
// navigation entry points or synthetic touches, so the touch-driven App Hall
// and touch-only store games stay usable without a touch panel.
class MatrixKeyInput final {
   public:
    MatrixKeyInput(BoardHardware& hardware, buses::I2cExecutor& executor) : hardware_(hardware), executor_(executor) {}
    MatrixKeyInput(const MatrixKeyInput&) = delete;
    MatrixKeyInput& operator=(const MatrixKeyInput&) = delete;
    ~MatrixKeyInput();

    [[nodiscard]] esp_err_t Initialize(device::Input& input);

   private:
    struct ScanContext final {
        BoardHardware* hardware;
        uint16_t ports;
        esp_err_t status;
    };

    static esp_err_t ReadPorts(void* context);
    static esp_err_t DischargeColumns(void* context);
    static esp_err_t ReleaseColumns(void* context);
    void Scan();
    static void WorkerEntry(void* context);
    void Stop();
    [[nodiscard]] bool AnyOtherKeyEmitted(size_t index) const;
    void HandleFallback(size_t index, device::KeyCode code, bool pressed);
    void MirrorStickTouch(device::KeyCode code, bool pressed);
    void EmulateHallPress(bool pressed);
    void EmulateHallSwipe(bool forward);

    BoardHardware& hardware_;
    buses::I2cExecutor& executor_;
    device::Input* input_{};
    TaskHandle_t worker_{};
    SemaphoreHandle_t worker_stopped_{};
    std::atomic<bool> stopping_{};
    bool raw_[6U]{};              // last raw sample of this key
    uint32_t raw_change_ms_[6U]{};  // when the raw sample last changed
    bool emitted_[6U]{};          // current stable state already reported
    uint32_t confirm_down_ms_{};
    bool confirm_fallback_active_{};
    bool back_pending_[6U]{};
    uint32_t back_down_ms_[6U]{};
    // Set while both buttons are held at once. A simultaneous A+B press is the
    // Host shortcut that leaves the running Guest, so the matching releases
    // must not also drive Hall navigation or an emulated slide.
    bool chord_held_{};
};

}  // namespace micropixel::platform::xiaocheng_esp32s3
