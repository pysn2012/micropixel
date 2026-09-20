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
    void Scan();
    static void WorkerEntry(void* context);
    void Stop();

    BoardHardware& hardware_;
    buses::I2cExecutor& executor_;
    device::Input* input_{};
    TaskHandle_t worker_{};
    SemaphoreHandle_t worker_stopped_{};
    std::atomic<bool> stopping_{};
    bool last_raw_[6U]{};   // previous raw sample, for two-sample debounce
    bool emitted_[6U]{};    // current stable state already reported
};

}  // namespace micropixel::platform::xiaocheng_esp32s3
