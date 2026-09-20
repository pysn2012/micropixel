#pragma once

#include <cstdint>

#include "driver/i2c_master.h"
#include "esp_err.h"

namespace micropixel::platform::buses {
class I2cExecutor;
}

namespace micropixel::platform::xiaocheng_esp32s3 {

// Owns the shared I2C master bus, the XL9535 IO expander (power rails, LCD
// reset, amplifier enable, key-matrix column inputs) and the PWM backlight.
class BoardHardware final {
   public:
    [[nodiscard]] esp_err_t Initialize();
    [[nodiscard]] esp_err_t ReadInputPorts(uint16_t& ports);
    [[nodiscard]] esp_err_t PulseLcdReset();
    [[nodiscard]] esp_err_t SetBrightness(int percent);
    [[nodiscard]] esp_err_t SetAmplifier(bool enabled, buses::I2cExecutor& executor);
    [[nodiscard]] i2c_master_bus_handle_t I2cBus() const { return i2c_bus_; }

   private:
    [[nodiscard]] esp_err_t WriteRegister(uint8_t address, uint8_t value);

    i2c_master_bus_handle_t i2c_bus_{};
    i2c_master_dev_handle_t expander_{};
    uint8_t port0_output_{0x00U};  // XL9535 port 0 output latch mirror
    bool brightness_initialized_{};
};

}  // namespace micropixel::platform::xiaocheng_esp32s3
