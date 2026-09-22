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
    // Charge-transfer key scanning: discharge the key columns (drive them as
    // outputs low), then release them back to inputs before each row read.
    // Both go through port 0 mirrors so the audio rails (P0.3/P0.7) keep
    // their direction and level.
    [[nodiscard]] esp_err_t SetKeyColumnsDischarged();
    [[nodiscard]] esp_err_t SetKeyColumnsInput();
    [[nodiscard]] esp_err_t PulseLcdReset();
    [[nodiscard]] esp_err_t SetBrightness(int percent);
    [[nodiscard]] esp_err_t SetAmplifier(bool enabled, buses::I2cExecutor& executor);
    [[nodiscard]] i2c_master_bus_handle_t I2cBus() const { return i2c_bus_; }

   private:
    [[nodiscard]] esp_err_t WriteExpanderRegister(uint8_t address, uint8_t value);

    i2c_master_bus_handle_t i2c_bus_{};
    i2c_master_dev_handle_t expander_{};
    uint8_t port0_output_{0x00U};  // XL9535 port 0 output latch mirror
    uint8_t port0_config_{0xFFU};  // XL9535 port 0 direction mirror (1 = input)
    bool brightness_initialized_{};
};

}  // namespace micropixel::platform::xiaocheng_esp32s3
