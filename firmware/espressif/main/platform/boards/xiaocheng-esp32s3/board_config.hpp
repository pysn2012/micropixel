#pragma once

#include <cstddef>
#include <cstdint>

#include "device/contracts/input.hpp"
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "driver/i2s_std.h"
#include "driver/ledc.h"
#include "driver/spi_master.h"

namespace micropixel::platform::xiaocheng_esp32s3 {

// ---------------------------------------------------------------------------
// Xiaocheng ESP32-S3 handheld (N16R8: 16 MiB flash, 8 MiB octal PSRAM).
// Pin map mirrors the verified ESPHome configuration (xiaocheng-va).
// ---------------------------------------------------------------------------

// Shared I2C bus (XL9535 expander + ES8311 codec control).
inline constexpr i2c_port_num_t kI2cPort = I2C_NUM_0;
inline constexpr gpio_num_t kI2cSda = GPIO_NUM_12;
inline constexpr gpio_num_t kI2cScl = GPIO_NUM_7;
inline constexpr uint32_t kI2cClockHz = 400000U;  // 50 kHz is too slow for matrix polling.
inline constexpr uint16_t kXl9535Address = 0x20U;  // XL9535, PCA9555 register-compatible.

// XL9535 register addresses (same layout as PCA9555).
inline constexpr uint8_t kXl9535InputPort0 = 0x00U;
inline constexpr uint8_t kXl9535InputPort1 = 0x01U;
inline constexpr uint8_t kXl9535OutputPort0 = 0x02U;
inline constexpr uint8_t kXl9535OutputPort1 = 0x03U;
inline constexpr uint8_t kXl9535ConfigPort0 = 0x06U;
inline constexpr uint8_t kXl9535ConfigPort1 = 0x07U;

// XL9535 pin assignment. Config bits: 1 = input, 0 = output.
// Port 0: P0.7 = LDO enable (output), P0.3 = amplifier enable (output),
//         P0.0/P0.4/P0.6 = key columns (input).
// Port 1: P1.2 = LCD reset (output), P1.4 = audio power (output).
// Audio needs all three rails switched on: P1.4 audio power, P0.7 LDO and
// P0.3 NS4152 amplifier enable.
inline constexpr uint8_t kXl9535OutputPort0Preset =
    0x80U | (1U << 3U);                                        // LDO + amplifier high
inline constexpr uint8_t kXl9535OutputPort1Preset = 0x14U;     // LCD RST + audio power high
inline constexpr uint8_t kXl9535ConfigPort0Value = 0x77U;      // only P0.7 / P0.3 are outputs
inline constexpr uint8_t kXl9535ConfigPort1Value = 0xEBU;      // P1.2 / P1.4 are outputs
inline constexpr uint8_t kXl9535LcdResetBit = 1U << 2U;        // P1.2, active high
inline constexpr uint8_t kXl9535AudioPowerBit = 1U << 4U;      // P1.4, high = on
inline constexpr uint8_t kXl9535AmplifierEnableBit = 1U << 3U; // P0.3, high = on

// ST7789 2.0" panel, 240x320 portrait wire, used as 320x240 landscape.
inline constexpr spi_host_device_t kLcdSpiHost = SPI3_HOST;
inline constexpr gpio_num_t kLcdMosi = GPIO_NUM_45;
inline constexpr gpio_num_t kLcdClock = GPIO_NUM_47;
inline constexpr gpio_num_t kLcdDataCommand = GPIO_NUM_48;
inline constexpr gpio_num_t kLcdChipSelect = GPIO_NUM_21;
inline constexpr uint32_t kLcdPixelClockHz = 40U * 1000U * 1000U;  // verified on this panel; drop to 20 MHz on artifacts
inline constexpr size_t kPanelIoQueueDepth = 10U;
// Panel orientation (equivalent of the verified ESPHome 90 degree rotation).
// Adjust these three switches if the first boot shows a wrong orientation.
inline constexpr bool kPanelSwapXy = true;
inline constexpr bool kPanelMirrorX = true;
inline constexpr bool kPanelMirrorY = false;

// Backlight (active high on this board; add ledc output_invert if it measures opposite).
inline constexpr gpio_num_t kBacklight = GPIO_NUM_14;
inline constexpr ledc_channel_t kBacklightChannel = LEDC_CHANNEL_0;
inline constexpr ledc_timer_t kBacklightTimer = LEDC_TIMER_1;
inline constexpr uint32_t kBacklightMaximumDuty = (1U << 10U) - 1U;

// ES8311 DAC output path (ES7210 recording is not part of the board profile).
inline constexpr int kI2sPort = I2S_NUM_0;
inline constexpr gpio_num_t kAudioMasterClock = GPIO_NUM_15;
inline constexpr gpio_num_t kAudioBitClock = GPIO_NUM_16;
inline constexpr gpio_num_t kAudioWordSelect = GPIO_NUM_17;
inline constexpr gpio_num_t kAudioDataOut = GPIO_NUM_13;
inline constexpr uint8_t kEs8311Address = 0x18U;  // seven-bit, matches the verified board

// Key matrix: two native-GPIO rows x three XL9535 columns. Vendor-verified
// wiring (xiaozhi-zyb / xiaocheng-h1): the key columns have NO pull-ups and
// float, so the scanner resolves polarity by measurement instead of assuming
// it - each row is driven low, read, driven high and read again, and a closed
// key is the only state where the row level can reach the column. `low != high`
// therefore detects a press with pull-ups, pull-downs or a floating column
// alike. Rows idle as high-impedance inputs and are only driven while scanned.
// Physical layout: A/RIGHT/UP on row GPIO0 and DOWN/B/LEFT on row GPIO46.
// Reorder the codes here after on-board testing.
struct MatrixKeyEntry final {
    gpio_num_t row;
    uint8_t column_bit;  // bit inside XL9535 input port 0
    device::KeyCode code;
    const char* name;
};

inline constexpr MatrixKeyEntry kMatrixKeys[6U]{
    {GPIO_NUM_0, 1U << 0U, device::KeyCode::kConfirm, "A"},
    {GPIO_NUM_0, 1U << 4U, device::KeyCode::kRight, "RIGHT"},
    {GPIO_NUM_0, 1U << 6U, device::KeyCode::kUp, "UP"},
    {GPIO_NUM_46, 1U << 0U, device::KeyCode::kDown, "DOWN"},
    {GPIO_NUM_46, 1U << 4U, device::KeyCode::kBack, "B"},
    {GPIO_NUM_46, 1U << 6U, device::KeyCode::kLeft, "LEFT"},
};

inline constexpr gpio_num_t kMatrixRows[2U]{GPIO_NUM_0, GPIO_NUM_46};
inline constexpr uint8_t kKeyColumnMask = static_cast<uint8_t>((1U << 0U) | (1U << 4U) | (1U << 6U));  // P0.0/4/6
// Differential scan timing: two rows x two I2C reads per row, 150 us per level.
// One full sweep costs well under a millisecond, so the 20 ms debounce still
// sees three to four samples and presses reach a game without the tens of
// milliseconds of lag a discharge-based scan would add.
inline constexpr uint32_t kMatrixScanPeriodMs = 4U;
inline constexpr uint32_t kKeyDebounceMs = 20U;
inline constexpr uint32_t kRowSettleUs = 150U;

}  // namespace micropixel::platform::xiaocheng_esp32s3
