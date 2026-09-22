#include "platform/platform.hpp"

#include <algorithm>

#include "esp_check.h"
#include "esp_log.h"
#include "esp_lv_adapter.h"
#include "host/ui/lvgl/square_common/square_system_ui.hpp"
#include "platform/adapters/graphics_adapter.hpp"
#include "platform/boards/esp32-s3-common/landscape_320_state.hpp"
#include "platform/boards/esp32-s3-common/lvgl_display.hpp"
#include "platform/boards/esp32-s3-common/presentation.hpp"
#include "platform/boards/xiaocheng-esp32s3/board_config.hpp"
#include "platform/boards/xiaocheng-esp32s3/board_hardware.hpp"
#include "platform/boards/xiaocheng-esp32s3/display_hardware.hpp"
#include "platform/boards/xiaocheng-esp32s3/i2s_audio_sink.hpp"
#include "platform/boards/xiaocheng-esp32s3/matrix_key_input.hpp"
#include "platform/controllers/brightness_curve.hpp"
#include "platform/drivers/sensors/lsm6dsl.hpp"
#include "platform/lvgl/guest_graphics_operations.hpp"
#include "platform/memory/ext_ram_bss.hpp"
#include "platform/memory/internal_ram.hpp"
#include "platform/sensors/polled_inertial_sensor_peripheral.hpp"
#include "platform/wifi/native_wifi_radio.hpp"
#include "platform/wifi/wifi_manager.hpp"

namespace micropixel::platform {
namespace {

namespace common = esp32_s3_common;
namespace board_detail = xiaocheng_esp32s3;
constexpr char kTag[] = "xiaocheng_esp32s3";

// Xiaocheng ESP32-S3 handheld: ST7789 320x240 (no touch), 2x3 key matrix on
// GPIO0/GPIO46 + XL9535, ES8311 audio output, native Wi-Fi.
class XiaochengEsp32S3Board final : public Board {
   public:
    XiaochengEsp32S3Board()
        : state_(TaskState(input_state_)),
          graphics_context_{.engine = &state_.guest_graphics, .hooks = state_.ui.GraphicsHooks()},
          graphics_(lvgl::MakeGuestGraphicsOperations(graphics_context_)),
          audio_output_(hardware_),
          presentation_(
              state_, kTag,
              [](void* context, int percent) {
                  const auto safe_percent = static_cast<uint8_t>(std::clamp(percent, 0, 100));
                  return static_cast<board_detail::BoardHardware*>(context)->SetBrightness(
                      controllers::VisibleBrightnessPercent(safe_percent));
              },
              &hardware_),
          system_ui_(state_.ui, presentation_),
          acceleration_(inertial_, drivers::Lsm6dsl::Kind::kAcceleration),
          angular_velocity_(inertial_, drivers::Lsm6dsl::Kind::kAngularVelocity),
          inertial_sensors_(acceleration_, angular_velocity_,
                            {.log_tag = "xiaocheng_imu",
                             .model = "LSM6DSL",
                             .acceleration_timer_name = "xiaocheng_accel",
                             .angular_velocity_timer_name = "xiaocheng_gyro"}),
          matrix_keys_(hardware_, state_.i2c_executor) {
        state_.guest_graphics.SetPresentationHooks(state_.ui.GuestFrameHooks());
    }

    [[nodiscard]] esp_err_t Initialize(BoardContext& context) override {
        ESP_RETURN_ON_FALSE(memory::IsInternalObject(*this), ESP_ERR_INVALID_STATE, kTag,
                            "Board control objects must reside in internal RAM");
        presentation_.BindAudioEngine(context.AudioEngine());
        ESP_LOGI(kTag, "initializing Xiaocheng ESP32-S3 handheld");
        ESP_RETURN_ON_ERROR(board_detail::InitializeDisplayHardware(hardware_, state_), kTag,
                            "initialize ST7789 display failed");
        ESP_RETURN_ON_ERROR(common::RegisterLvglDisplay(state_, kDoubleBuffer), kTag, "register LVGL display failed");
        ESP_RETURN_ON_ERROR(state_.display_shadow.Initialize(state_.display), kTag,
                            "initialize PSRAM displayed shadow failed");
        ESP_RETURN_ON_ERROR(state_.i2c_executor.Initialize(), kTag, "start shared I2C executor failed");
        // The LSM6DSL hangs off the shared bus next to the XL9535 and the
        // codecs; a missing or unreachable chip just skips sensor registration.
        inertial_sensors_.Initialize(hardware_.I2cBus(), state_.i2c_executor);
        ESP_RETURN_ON_ERROR(state_.guest_graphics.Initialize(state_.display, nullptr), kTag,
                            "initialize RGB565 Guest graphics failed");

        if (esp_lv_adapter_lock(-1) != ESP_OK) {
            return ESP_FAIL;
        }
        const esp_err_t ui_status = state_.ui.InitializeLocked(state_.display);
        esp_lv_adapter_unlock();
        ESP_RETURN_ON_ERROR(ui_status, kTag, "initialize 320x240 Host UI failed");
        ESP_RETURN_ON_ERROR(esp_lv_adapter_start(), kTag, "start LVGL adapter failed");

        const esp_err_t audio_status = audio_output_.Configure(hardware_.I2cBus(), state_.i2c_executor);
        if (audio_status != ESP_OK) {
            ESP_LOGW(kTag, "audio unavailable for this boot: %s", esp_err_to_name(audio_status));
        }
        ESP_RETURN_ON_ERROR(matrix_keys_.Initialize(state_.ui.Input()), kTag, "initialize matrix keys failed");
        // USB development screenshots take the same path as Remote Control.
        ESP_RETURN_ON_ERROR(
            state_.development_display.Start(state_.touch_input, state_.local_control, common::kWidth, common::kHeight,
                                             transports::DevelopmentCaptureHook::For(presentation_)),
            kTag, "start USB development control failed");
        ESP_RETURN_ON_ERROR(hardware_.SetBrightness(80), kTag, "set startup brightness failed");

        BoardRegistration registration{{
            .board = "Xiaocheng ESP32-S3",
            .host_chip = "ESP32-S3",
            .firmware_target = "xiaocheng-esp32s3",
            .wifi_coprocessor = "Native ESP32-S3",
            .display =
                {
                    .driver = state_.panel_name,
                    .interface = "SPI 40 MHz",
                    .pixel_format = "RGB565 (big-endian wire)",
                    .width_pixels = common::kWidth,
                    .height_pixels = common::kHeight,
                },
            .graphics_acceleration = "CPU only; SPI DMA transport",
        }};
        registration.SetInput(state_.ui.Input());
        registration.SetGraphics(graphics_);
        if (audio_status == ESP_OK) {
            registration.SetAudioOutput(audio_output_, audio_output_.SampleRate());
        }
        registration.SetWifi(wifi_);
        registration.SetLocalControl(state_.local_control);
        registration.SetSystemUi(system_ui_);
        bool registered = true;
        if (inertial_sensors_.acceleration_available()) {
            registered =
                registration.AddSensor(inertial_sensors_, sensors::PolledInertialSensorPeripheral::kAcceleration,
                                       "Built-in LSM6DSL accelerometer") &&
                registered;
        }
        if (inertial_sensors_.angular_velocity_available()) {
            registered =
                registration.AddSensor(inertial_sensors_, sensors::PolledInertialSensorPeripheral::kAngularVelocity,
                                       "Built-in LSM6DSL gyroscope") &&
                registered;
        }
        ESP_LOGI(kTag, "ready: ST7789 (keys only, no touch), native Wi-Fi, audio=%s, imu=%s",
                 audio_status == ESP_OK ? "ES8311" : "off",
                 inertial_sensors_.acceleration_available() ? "LSM6DSL" : "off");
        return (context.Publish(registration) && registered) ? ESP_OK : ESP_ERR_INVALID_STATE;
    }

    void BindBackgroundExecutor(work::BackgroundExecutor& executor) override {
        state_.ui.BindBackgroundExecutor(executor);
        state_.guest_graphics.BindBackgroundExecutor(executor);
        wifi_.BindBackgroundExecutor(executor);
    }

   private:
    static common::Landscape320State& TaskState(common::Landscape320InputState& input) {
        static MICROPIXEL_EXT_RAM_BSS common::Landscape320State state(input);
        return state;
    }

    static constexpr bool kDoubleBuffer = false;

    common::Landscape320InputState input_state_{};
    common::Landscape320State& state_;
    lvgl::GuestGraphicsOperationsContext graphics_context_{};
    adapters::GraphicsAdapter graphics_;
    board_detail::BoardHardware hardware_{};
    board_detail::I2sAudioSink audio_output_;
    common::Landscape320Presentation presentation_;
    host_ui::lvgl::square_common::SquareSystemUi system_ui_;
    drivers::Lsm6dsl inertial_{};
    drivers::Lsm6dsl::Vector acceleration_;
    drivers::Lsm6dsl::Vector angular_velocity_;
    sensors::PolledInertialSensorPeripheral inertial_sensors_;
    board_detail::MatrixKeyInput matrix_keys_;
    wifi::NativeWifiRadio wifi_radio_{};
    wifi::WifiManager wifi_{wifi_radio_};
};

}  // namespace

Board& ConfiguredBoard() {
    static XiaochengEsp32S3Board board;
    return board;
}

}  // namespace micropixel::platform
