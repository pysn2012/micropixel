#include "platform/boards/xiaocheng-esp32s3/board_hardware.hpp"

#include <algorithm>

#include "driver/ledc.h"
#include "esp_check.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "platform/buses/i2c_executor.hpp"
#include "platform/boards/xiaocheng-esp32s3/board_config.hpp"

namespace micropixel::platform::xiaocheng_esp32s3 {
namespace {

constexpr char kTag[] = "xiaocheng_hw";

esp_err_t WriteRegister(i2c_master_dev_handle_t device, uint8_t address, uint8_t value) {
    const uint8_t bytes[]{address, value};
    return i2c_master_transmit(device, bytes, sizeof(bytes), 100);
}

}  // namespace

esp_err_t BoardHardware::Initialize() {
    if (i2c_bus_ != nullptr || expander_ != nullptr) {
        return ESP_ERR_INVALID_STATE;
    }
    i2c_master_bus_config_t bus_config{};
    bus_config.i2c_port = kI2cPort;
    bus_config.sda_io_num = kI2cSda;
    bus_config.scl_io_num = kI2cScl;
    bus_config.clk_source = I2C_CLK_SRC_DEFAULT;
    bus_config.glitch_ignore_cnt = 7U;
    bus_config.flags.enable_internal_pullup = true;
    ESP_RETURN_ON_ERROR(i2c_new_master_bus(&bus_config, &i2c_bus_), kTag, "create shared I2C bus failed");

    i2c_device_config_t expander_config{};
    expander_config.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    expander_config.device_address = kXl9535Address;
    expander_config.scl_speed_hz = kI2cClockHz;
    ESP_RETURN_ON_ERROR(i2c_master_bus_add_device(i2c_bus_, &expander_config, &expander_), kTag,
                        "attach XL9535 failed");

    // Match the verified power sequence: preset output latches first (LCD reset
    // high, audio power on, LDO on, amplifier on), then enable those output
    // directions. Audio needs all three XL9535 rails: P1.4, P0.7 and P0.3.
    port0_output_ = kXl9535OutputPort0Preset;
    ESP_RETURN_ON_ERROR(WriteRegister(kXl9535OutputPort0, port0_output_), kTag,
                        "preset XL9535 port 0 outputs failed");
    ESP_RETURN_ON_ERROR(WriteRegister(kXl9535OutputPort1, kXl9535OutputPort1Preset), kTag,
                        "preset XL9535 port 1 outputs failed");
    ESP_RETURN_ON_ERROR(WriteRegister(kXl9535ConfigPort0, kXl9535ConfigPort0Value), kTag,
                        "configure XL9535 port 0 directions failed");
    ESP_RETURN_ON_ERROR(WriteRegister(kXl9535ConfigPort1, kXl9535ConfigPort1Value), kTag,
                        "configure XL9535 port 1 directions failed");

    ledc_timer_config_t timer_config{};
    timer_config.speed_mode = LEDC_LOW_SPEED_MODE;
    timer_config.duty_resolution = LEDC_TIMER_10_BIT;
    timer_config.timer_num = kBacklightTimer;
    timer_config.freq_hz = 5000U;
    timer_config.clk_cfg = LEDC_AUTO_CLK;
    ESP_RETURN_ON_ERROR(ledc_timer_config(&timer_config), kTag, "configure backlight timer failed");
    ledc_channel_config_t channel_config{};
    channel_config.gpio_num = kBacklight;
    channel_config.speed_mode = LEDC_LOW_SPEED_MODE;
    channel_config.channel = kBacklightChannel;
    channel_config.timer_sel = kBacklightTimer;
    channel_config.duty = 0U;
    channel_config.hpoint = 0;
    ESP_RETURN_ON_ERROR(ledc_channel_config(&channel_config), kTag, "configure backlight channel failed");
    brightness_initialized_ = true;
    return ESP_OK;
}

esp_err_t BoardHardware::ReadInputPorts(uint16_t& ports) {
    if (expander_ == nullptr) {
        return ESP_ERR_INVALID_STATE;
    }
    const uint8_t start = kXl9535InputPort0;
    uint8_t data[2]{};
    ESP_RETURN_ON_ERROR(i2c_master_transmit_receive(expander_, &start, sizeof(start), data, sizeof(data), 100), kTag,
                        "read XL9535 inputs failed");
    ports = static_cast<uint16_t>(data[0]) | static_cast<uint16_t>(data[1]) << 8U;
    return ESP_OK;
}

esp_err_t BoardHardware::PulseLcdReset() {
    if (expander_ == nullptr) {
        return ESP_ERR_INVALID_STATE;
    }
    ESP_RETURN_ON_ERROR(WriteRegister(kXl9535OutputPort1, static_cast<uint8_t>(kXl9535OutputPort1Preset)), kTag,
                        "hold LCD reset failed");
    ESP_RETURN_ON_ERROR(WriteRegister(kXl9535OutputPort1, static_cast<uint8_t>(kXl9535OutputPort1Preset &
                                                                                    ~kXl9535LcdResetBit)),
                        kTag, "assert LCD reset failed");
    vTaskDelay(pdMS_TO_TICKS(10U));
    ESP_RETURN_ON_ERROR(WriteRegister(kXl9535OutputPort1, static_cast<uint8_t>(kXl9535OutputPort1Preset)), kTag,
                        "release LCD reset failed");
    vTaskDelay(pdMS_TO_TICKS(120U));
    return ESP_OK;
}

esp_err_t BoardHardware::SetBrightness(int percent) {
    if (!brightness_initialized_) {
        return ESP_ERR_INVALID_STATE;
    }
    const uint32_t bounded = static_cast<uint32_t>(std::clamp(percent, 0, 100));
    const uint32_t duty = (kBacklightMaximumDuty * bounded + 50U) / 100U;
    ESP_RETURN_ON_ERROR(ledc_set_duty(LEDC_LOW_SPEED_MODE, kBacklightChannel, duty), kTag, "set backlight duty failed");
    return ledc_update_duty(LEDC_LOW_SPEED_MODE, kBacklightChannel);
}

esp_err_t BoardHardware::SetAmplifier(bool enabled, buses::I2cExecutor& executor) {
    struct Request final {
        BoardHardware* hardware;
        bool enabled;
    } request{this, enabled};
    return executor.Invoke(
        buses::I2cExecutor::Priority::kHigh,
        [](void* context) {
            auto& requested = *static_cast<Request*>(context);
            const uint8_t next = requested.enabled
                                     ? static_cast<uint8_t>(requested.hardware->port0_output_ |
                                                            kXl9535AmplifierEnableBit)
                                     : static_cast<uint8_t>(requested.hardware->port0_output_ &
                                                            ~kXl9535AmplifierEnableBit);
            const esp_err_t status = requested.hardware->WriteRegister(kXl9535OutputPort0, next);
            if (status == ESP_OK) {
                requested.hardware->port0_output_ = next;
            }
            return status;
        },
        &request);
}

esp_err_t BoardHardware::WriteRegister(uint8_t address, uint8_t value) {
    if (expander_ == nullptr) {
        return ESP_ERR_INVALID_STATE;
    }
    return WriteRegister(expander_, address, value);
}

}  // namespace micropixel::platform::xiaocheng_esp32s3
