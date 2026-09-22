#include "platform/drivers/sensors/lsm6dsl.hpp"

#include <array>

#include "esp_check.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace micropixel::platform::drivers {
namespace {

constexpr char kTag[] = "lsm6dsl";
constexpr uint8_t kAddress = 0x6aU;
constexpr uint8_t kWhoAmIRegister = 0x0fU;
constexpr uint8_t kWhoAmI = 0x6aU;
constexpr uint8_t kControl3Register = 0x12U;
constexpr uint8_t kAccelerationControlRegister = 0x10U;        // CTRL1_XL: ODR[7:4] + FS_XL[3:2]
constexpr uint8_t kAngularVelocityControlRegister = 0x11U;     // CTRL2_G:  ODR[7:4] + FS_G[3:1]
constexpr uint8_t kAccelerationXLowRegister = 0x28U;           // OUTX_L_XL, 6 bytes
constexpr uint8_t kAngularVelocityXLowRegister = 0x22U;        // OUTX_L_G, 6 bytes
constexpr uint8_t kSoftReset = 0x01U;
constexpr uint8_t kControl3Value = 0x44U;   // BDU=1 (bit6) + IF_INC=1 (bit2)
// Acceleration full scale 4g: FS_XL = 0b10, sensitivity 0.122 mg/LSB.
constexpr uint8_t kAccelerationFullScale4G = 0b10 << 2U;
constexpr float kAccelerationMilliGPerCount = 0.122F;
// Angular velocity full scale 245 dps: FS_G = 0b000, sensitivity 8.75 mdps/LSB.
constexpr uint8_t kAngularVelocityFullScale245Dps = 0b000 << 1U;
constexpr float kAngularVelocityMilliDpsPerCount = 8.75F;
constexpr uint32_t kI2cSpeedHz = 400000U;
constexpr int kTimeoutMs = 100;
constexpr float kGravityMetersPerSecondSquared = 9.80665F;
constexpr float kRadiansPerDegree = 0.017453292519943295F;

}  // namespace

Lsm6dsl::~Lsm6dsl() {
    if (device_ != nullptr) {
        (void)i2c_master_bus_rm_device(device_);
    }
}

esp_err_t Lsm6dsl::Initialize(i2c_master_bus_handle_t bus) {
    if (bus == nullptr || device_ != nullptr) {
        return ESP_ERR_INVALID_ARG;
    }
    if (i2c_master_probe(bus, kAddress, kTimeoutMs) != ESP_OK) {
        return ESP_ERR_NOT_FOUND;
    }
    i2c_device_config_t config{};
    config.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    config.device_address = kAddress;
    config.scl_speed_hz = kI2cSpeedHz;
    ESP_RETURN_ON_ERROR(i2c_master_bus_add_device(bus, &config, &device_), kTag, "add I2C device failed");

    uint8_t who_am_i = 0U;
    esp_err_t status = ReadRegisters(kWhoAmIRegister, &who_am_i, sizeof(who_am_i));
    if (status == ESP_OK && who_am_i != kWhoAmI) {
        status = ESP_ERR_INVALID_RESPONSE;
    }
    if (status == ESP_OK) {
        status = WriteRegister(kControl3Register, kSoftReset);
    }
    if (status == ESP_OK) {
        vTaskDelay(pdMS_TO_TICKS(50));
        status = WriteRegister(kControl3Register, kControl3Value);
    }
    if (status != ESP_OK) {
        (void)i2c_master_bus_rm_device(device_);
        device_ = nullptr;
        return status;
    }
    ESP_LOGI(kTag, "ready: WHO_AM_I=0x%02x address=0x%02x", who_am_i, kAddress);
    return ESP_OK;
}

esp_err_t Lsm6dsl::Configure(Kind kind, uint32_t interval_us) {
    if (!available()) {
        return ESP_ERR_INVALID_STATE;
    }
    // ODR<<4 | full scale; both channels keep their own ODR field, so a single
    // register write per channel is the whole configuration.
    const uint8_t control = static_cast<uint8_t>(
        (kind == Kind::kAcceleration ? kAccelerationFullScale4G : kAngularVelocityFullScale245Dps) |
        (SelectOdr(interval_us) << 4U));
    const uint8_t register_address =
        kind == Kind::kAcceleration ? kAccelerationControlRegister : kAngularVelocityControlRegister;
    return WriteRegister(register_address, control);
}

esp_err_t Lsm6dsl::Suspend(Kind kind) {
    if (!available()) {
        return ESP_ERR_INVALID_STATE;
    }
    // Power down the channel: ODR = 0000, full scale bits unchanged.
    const uint8_t control =
        kind == Kind::kAcceleration ? kAccelerationFullScale4G : kAngularVelocityFullScale245Dps;
    const uint8_t register_address =
        kind == Kind::kAcceleration ? kAccelerationControlRegister : kAngularVelocityControlRegister;
    return WriteRegister(register_address, control);
}

esp_err_t Lsm6dsl::Read(Kind kind, float (&values)[3]) {
    if (!available()) {
        return ESP_ERR_INVALID_STATE;
    }
    std::array<uint8_t, 6U> bytes{};
    const uint8_t address = kind == Kind::kAcceleration ? kAccelerationXLowRegister : kAngularVelocityXLowRegister;
    ESP_RETURN_ON_ERROR(ReadRegisters(address, bytes.data(), bytes.size()), kTag, "read vector failed");
    const float scale = kind == Kind::kAcceleration
                            ? kAccelerationMilliGPerCount * kGravityMetersPerSecondSquared / 1000.0F
                            : kAngularVelocityMilliDpsPerCount * kRadiansPerDegree / 1000.0F;
    for (uint32_t axis = 0U; axis < 3U; ++axis) {
        const int16_t raw =
            static_cast<int16_t>((static_cast<uint16_t>(bytes[axis * 2U + 1U]) << 8U) | bytes[axis * 2U]);
        values[axis] = static_cast<float>(raw) * scale;
    }
    return ESP_OK;
}

esp_err_t Lsm6dsl::WriteRegister(uint8_t address, uint8_t value) {
    const uint8_t transaction[]{address, value};
    return device_ == nullptr ? ESP_ERR_INVALID_STATE
                              : i2c_master_transmit(device_, transaction, sizeof(transaction), kTimeoutMs);
}

esp_err_t Lsm6dsl::ReadRegisters(uint8_t address, uint8_t* data, size_t length) {
    if (device_ == nullptr || data == nullptr || length == 0U) {
        return ESP_ERR_INVALID_ARG;
    }
    return i2c_master_transmit_receive(device_, &address, sizeof(address), data, length, kTimeoutMs);
}

uint8_t Lsm6dsl::SelectOdr(uint32_t interval_us) {
    if (interval_us <= 1250U) {
        return 0x8U;  // 1.66 kHz
    }
    if (interval_us <= 2500U) {
        return 0x7U;  // 833 Hz
    }
    if (interval_us <= 5000U) {
        return 0x6U;  // 416 Hz
    }
    if (interval_us <= 10000U) {
        return 0x5U;  // 208 Hz
    }
    return 0x4U;  // 104 Hz
}

esp_err_t Lsm6dsl::Vector::Initialize(i2c_master_bus_handle_t bus) { return sensor_.Initialize(bus); }

esp_err_t Lsm6dsl::Vector::Configure(uint32_t interval_us) { return sensor_.Configure(kind_, interval_us); }

esp_err_t Lsm6dsl::Vector::Suspend() { return sensor_.Suspend(kind_); }

esp_err_t Lsm6dsl::Vector::Read(float (&values)[3]) { return sensor_.Read(kind_, values); }

}  // namespace micropixel::platform::drivers
