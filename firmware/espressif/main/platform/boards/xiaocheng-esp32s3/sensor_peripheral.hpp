#pragma once

#include "device/contracts/sensors.hpp"
#include "driver/i2c_master.h"
#include "platform/buses/i2c_executor.hpp"
#include "platform/drivers/sensors/lsm6dsl.hpp"
#include "platform/sensors/polled_inertial_sensor_peripheral.hpp"

namespace micropixel::platform::xiaocheng_esp32s3 {

// LSM6DSL accelerometer + gyroscope exposed through the polled inertial
// peripheral, with the chip axes rotated into the landscape display frame that
// Guest apps expect: screen right is -sensorY, screen down is +sensorX. Guest
// sources state that convention explicitly (gravity-balls relies on it), and
// the same rotation is used by the vendor's own H1 IDF port for this board, so
// it is fixed by the hardware mounting rather than tunable per app. The
// rotation itself lives in sensor_peripheral.cpp next to the axis constants.
class SensorPeripheral final : public device::SensorPeripheral {
   public:
    static constexpr device::PeripheralChannelId kAcceleration =
        sensors::PolledInertialSensorPeripheral::kAcceleration;
    static constexpr device::PeripheralChannelId kAngularVelocity =
        sensors::PolledInertialSensorPeripheral::kAngularVelocity;

    void Initialize(i2c_master_bus_handle_t bus, buses::I2cExecutor& i2c_executor);

    [[nodiscard]] int32_t GetInfo(device::PeripheralChannelId channel, micropixel_sensor_info_t& info_out) const override;
    [[nodiscard]] int32_t Start(device::PeripheralChannelId channel, uint32_t interval_us) override;
    [[nodiscard]] int32_t Read(device::PeripheralChannelId channel, device::SensorValues& values_out) override;
    void Stop(device::PeripheralChannelId channel) override;

    [[nodiscard]] bool acceleration_available() const { return inertial_.acceleration_available(); }
    [[nodiscard]] bool angular_velocity_available() const { return inertial_.angular_velocity_available(); }

   private:
    drivers::Lsm6dsl chip_{};
    drivers::Lsm6dsl::Vector acceleration_{chip_, drivers::Lsm6dsl::Kind::kAcceleration};
    drivers::Lsm6dsl::Vector angular_velocity_{chip_, drivers::Lsm6dsl::Kind::kAngularVelocity};
    sensors::PolledInertialSensorPeripheral inertial_{
        acceleration_, angular_velocity_,
        {.log_tag = "xiaocheng_imu",
         .model = "LSM6DSL",
         .acceleration_timer_name = "xiaocheng_accel",
         .angular_velocity_timer_name = "xiaocheng_gyro"}};
};

}  // namespace micropixel::platform::xiaocheng_esp32s3
