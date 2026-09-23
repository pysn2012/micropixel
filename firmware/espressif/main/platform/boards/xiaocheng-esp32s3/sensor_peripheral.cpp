#include "platform/boards/xiaocheng-esp32s3/sensor_peripheral.hpp"

#include <cstdio>

#include "esp_log.h"

namespace micropixel::platform::xiaocheng_esp32s3 {
namespace {

constexpr char kTag[] = "xiaocheng_imu";

// Axis remap for the on-board LSM6DSL mounting. The chip sits rotated a
// quarter turn against the landscape panel, so the screen's horizontal axis
// follows sensor Y (inverted) and the screen's vertical axis follows sensor X.
// Raw axes land gravity on the screen's horizontal axis, which is why Gravity
// Balls piles every ball against one wall instead of letting them fall.
// Flip a sign here if a live test shows the balls drifting the wrong way.
constexpr int kDisplayXSource = 1;  // sensor Y drives screen right/left
constexpr int kDisplayYSource = 0;  // sensor X drives screen up/down
constexpr float kDisplayXSign = -1.0F;
constexpr float kDisplayYSign = 1.0F;

// One-shot bring-up aid: list every address that ACKs on the shared bus, so a
// wrong IMU address or a dead power rail is visible in a single boot log.
void ScanI2cBus(i2c_master_bus_handle_t bus) {
    char line[128];
    int offset = std::snprintf(line, sizeof(line), "shared I2C devices:");
    for (uint8_t address = 0x08U; address <= 0x77U; ++address) {
        if (i2c_master_probe(bus, address, 20U) != ESP_OK) {
            continue;
        }
        if (offset + 6 > static_cast<int>(sizeof(line))) {
            break;
        }
        offset += std::snprintf(line + offset, sizeof(line) - offset, " 0x%02x", address);
    }
    ESP_LOGI(kTag, "%s", line);
}

}  // namespace

void SensorPeripheral::Initialize(i2c_master_bus_handle_t bus, buses::I2cExecutor& i2c_executor) {
    ScanI2cBus(bus);
    inertial_.Initialize(bus, i2c_executor);
}

int32_t SensorPeripheral::GetInfo(device::PeripheralChannelId channel, micropixel_sensor_info_t& info_out) const {
    return inertial_.GetInfo(channel, info_out);
}

int32_t SensorPeripheral::Start(device::PeripheralChannelId channel, uint32_t interval_us) {
    return inertial_.Start(channel, interval_us);
}

int32_t SensorPeripheral::Read(device::PeripheralChannelId channel, device::SensorValues& values_out) {
    const int32_t status = inertial_.Read(channel, values_out);
    if (status != MICROPIXEL_STATUS_OK) {
        return status;
    }
    // Rotate both the acceleration and the angular velocity into the display
    // frame; the gyroscope is mounted with the accelerometer, so it shares the
    // mapping.
    const float display_x = kDisplayXSign * values_out.values[kDisplayXSource];
    const float display_y = kDisplayYSign * values_out.values[kDisplayYSource];
    values_out.values[0] = display_x;
    values_out.values[1] = display_y;
    return MICROPIXEL_STATUS_OK;
}

void SensorPeripheral::Stop(device::PeripheralChannelId channel) { inertial_.Stop(channel); }

}  // namespace micropixel::platform::xiaocheng_esp32s3
