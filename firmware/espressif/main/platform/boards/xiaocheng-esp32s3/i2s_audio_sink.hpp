#pragma once

#include "driver/i2c_master.h"
#include "esp_err.h"
#include "platform/audio/es8311_i2s_audio_sink.hpp"

namespace micropixel::platform::buses {
class I2cExecutor;
}

namespace micropixel::platform::xiaocheng_esp32s3 {

class BoardHardware;

// ES8311 over I2S0. The NS4152 amplifier enable lives on XL9535 P0.3 and is
// routed through the shared I2cExecutor; audio power (P1.4) and LDO (P0.7)
// are switched on by BoardHardware::Initialize() before the codec probe.
class I2sAudioSink final {
   public:
    explicit I2sAudioSink(BoardHardware& hardware) : hardware_(&hardware) {}

    [[nodiscard]] esp_err_t Configure(i2c_master_bus_handle_t bus, buses::I2cExecutor& executor);
    [[nodiscard]] uint32_t SampleRate() const { return sink_.SampleRate(); }

   private:
    static esp_err_t SetAmplifier(void* context, bool enabled);

    BoardHardware* hardware_{};
    buses::I2cExecutor* executor_{};
    audio::Es8311I2sAudioSink sink_{
        {
            .name = "Xiaocheng ESP32-S3 ES8311/I2S",
            .log_tag = "xiaocheng_audio",
            .i2c_port = I2C_NUM_0,
            .i2s_port = I2S_NUM_0,
            .master_clock = GPIO_NUM_15,
            .bit_clock = GPIO_NUM_16,
            .word_select = GPIO_NUM_17,
            .data_out = GPIO_NUM_13,
            .amplifier_enable = GPIO_NUM_NC,
            .amplifier_setter = SetAmplifier,
            .amplifier_context = this,
            .codec_i2c_address = 0x18U,
            .sample_rate = 16000U,
            .i2c_clock_hz = 400000U,
            .amplifier_preroll_ms = 24U,
            .dma_descriptor_count = 6U,
            .dma_frame_count = 240U,
            .amplifier_active_low = false,
            .probe_before_attach = true,
        },
        {.amplifier_voltage = 3.3F, .codec_dac_voltage = 3.3F}};
};

}  // namespace micropixel::platform::xiaocheng_esp32s3
