#include "platform/boards/xiaocheng-esp32s3/display_hardware.hpp"

#include <cstddef>

#include "driver/spi_master.h"
#include "esp_check.h"
#include "esp_lcd_io_spi.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_log.h"
#include "platform/boards/esp32-s3-common/landscape_320_state.hpp"
#include "platform/boards/xiaocheng-esp32s3/board_config.hpp"
#include "platform/boards/xiaocheng-esp32s3/board_hardware.hpp"

namespace micropixel::platform::xiaocheng_esp32s3 {
namespace {

constexpr char kTag[] = "xiaocheng_display";

void ReleaseDisplayHardware(esp32_s3_common::Landscape320State& state, bool bus_initialized) {
    if (state.panel != nullptr) {
        (void)esp_lcd_panel_del(state.panel);
        state.panel = nullptr;
    }
    if (state.panel_io != nullptr) {
        (void)esp_lcd_panel_io_del(state.panel_io);
        state.panel_io = nullptr;
    }
    if (bus_initialized) {
        (void)spi_bus_free(kLcdSpiHost);
    }
}

}  // namespace

esp_err_t InitializeDisplayHardware(BoardHardware& hardware, esp32_s3_common::Landscape320State& state) {
    ESP_RETURN_ON_ERROR(hardware.Initialize(), kTag, "initialize shared board hardware failed");
    const int transfer_bytes = esp32_s3_common::kWidth * CONFIG_MICROPIXEL_LVGL_PARTIAL_BUFFER_HEIGHT * 2;
    spi_bus_config_t bus_config{};
    bus_config.mosi_io_num = kLcdMosi;
    bus_config.miso_io_num = GPIO_NUM_NC;
    bus_config.sclk_io_num = kLcdClock;
    bus_config.quadwp_io_num = GPIO_NUM_NC;
    bus_config.quadhd_io_num = GPIO_NUM_NC;
    bus_config.data4_io_num = GPIO_NUM_NC;
    bus_config.data5_io_num = GPIO_NUM_NC;
    bus_config.data6_io_num = GPIO_NUM_NC;
    bus_config.data7_io_num = GPIO_NUM_NC;
    bus_config.max_transfer_sz = transfer_bytes;
    esp_err_t status = spi_bus_initialize(kLcdSpiHost, &bus_config, SPI_DMA_CH_AUTO);
    if (status != ESP_OK) {
        return status;
    }

    esp_lcd_panel_io_spi_config_t io_config{};
    io_config.cs_gpio_num = kLcdChipSelect;
    io_config.dc_gpio_num = kLcdDataCommand;
    io_config.spi_mode = 0;
    io_config.pclk_hz = kLcdPixelClockHz;
    io_config.trans_queue_depth = kPanelIoQueueDepth;
    io_config.lcd_cmd_bits = 8;
    io_config.lcd_param_bits = 8;
    io_config.flags.psram_dma_direct = 0;
    status = esp_lcd_new_panel_io_spi(static_cast<esp_lcd_spi_bus_handle_t>(kLcdSpiHost), &io_config, &state.panel_io);
    if (status != ESP_OK) {
        ReleaseDisplayHardware(state, true);
        return status;
    }

    esp_lcd_panel_dev_config_t panel_config{};
    panel_config.rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB;
    panel_config.bits_per_pixel = 16;
    panel_config.reset_gpio_num = GPIO_NUM_NC;  // reset runs through the XL9535 expander
    status = esp_lcd_new_panel_st7789(state.panel_io, &panel_config, &state.panel);
    if (status == ESP_OK) {
        status = hardware.PulseLcdReset();
    }
    if (status == ESP_OK) {
        status = esp_lcd_panel_init(state.panel);
    }
    if (status == ESP_OK) {
        status = esp_lcd_panel_invert_color(state.panel, true);
    }
    if (status == ESP_OK) {
        status = esp_lcd_panel_swap_xy(state.panel, kPanelSwapXy);
    }
    if (status == ESP_OK) {
        status = esp_lcd_panel_mirror(state.panel, kPanelMirrorX, kPanelMirrorY);
    }
    if (status == ESP_OK) {
        status = esp_lcd_panel_disp_on_off(state.panel, true);
    }
    if (status != ESP_OK) {
        ReleaseDisplayHardware(state, true);
        return status;
    }
    state.panel_name = "ST7789";
    ESP_LOGI(kTag, "ST7789 panel uses internal SRAM SPI DMA: block=%d bytes clock=%lu MHz", transfer_bytes,
             static_cast<unsigned long>(kLcdPixelClockHz / 1000000U));
    return ESP_OK;
}

}  // namespace micropixel::platform::xiaocheng_esp32s3
