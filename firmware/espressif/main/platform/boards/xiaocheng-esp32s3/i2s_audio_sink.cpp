#include "platform/boards/xiaocheng-esp32s3/i2s_audio_sink.hpp"

#include "platform/buses/i2c_executor.hpp"
#include "platform/boards/xiaocheng-esp32s3/board_hardware.hpp"

namespace micropixel::platform::xiaocheng_esp32s3 {

esp_err_t I2sAudioSink::Configure(i2c_master_bus_handle_t bus, buses::I2cExecutor& executor) {
    executor_ = &executor;
    return sink_.Configure(bus, executor);
}

esp_err_t I2sAudioSink::SetAmplifier(void* context, bool enabled) {
    auto* sink = static_cast<I2sAudioSink*>(context);
    if (sink == nullptr || sink->hardware_ == nullptr || sink->executor_ == nullptr) {
        return ESP_ERR_INVALID_STATE;
    }
    return sink->hardware_->SetAmplifier(enabled, *sink->executor_);
}

}  // namespace micropixel::platform::xiaocheng_esp32s3
