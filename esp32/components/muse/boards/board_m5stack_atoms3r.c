/*
 * Copyright (c) Meta Platforms, Inc. and affiliates.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/*
 * M5Stack AtomS3R + Atomic Voice (Echo) Base: ESP32-S3-PICO-1-N8R8, 0.85"
 * 128x128 panel that is also the one push button (USER_BUT, G41), no touch,
 * USB powered only.
 *
 * Display driver: ST7735S, verified on this unit with the M5GFX autodetect
 * probe ("[Autodetect] board_M5AtomS3R (ST7735)"). M5 changed the panel from
 * GC9107 to ST7735 in 2026-05; a GC9107 unit needs a different init.
 * Verified by I2C scan on this unit: internal bus (SDA G45, SCL G0) has the
 * BMI270 at 0x68 (chip id 0x24) and the LP5562 at 0x30; the base bus (SDA G38,
 * SCL G39) has the ES8311 at 0x18 (id 0x8311) and a PI4IOE5V6408 at 0x43.
 * Pins and register values follow M5GFX (panel, LP5562 backlight) and
 * M5Unified (Atomic Echo Base: BCLK G8, WS G6, ESP DOUT G5, ESP DIN G7, ES8311
 * clocked from BCLK, amp enabled through the PI4IOE outputs).
 *
 * The ST7735S init sequence and panel/backlight/codec values are adapted from
 * M5GFX and M5Unified (https://github.com/m5stack), MIT License,
 * Copyright (c) 2021 M5Stack:
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 * One button: hold = voice note (or Select in the menu), tap = start/end a live
 * call, double tap = menu (see poll_buttons).
 */
#include "driver/i2c_master.h"
#include "driver/i2s_std.h"
#include "driver/spi_master.h"
#include "esp_check.h"
#include "esp_codec_dev_defaults.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_log.h"
#include "esp_lv_adapter.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "muse_audio.h"
#include "muse_board.h"
#include "muse_menu.h"
#include "muse_voice.h"
#include "muse_mem.h"
#include "muse_ui.h"

static const char *TAG = "board";

#define LCD_W 128
#define LCD_H 128
#define LCD_GAP_X 2                /* the 128x128 window in the controller's 132x132 */
#define LCD_GAP_Y 1
#define LCD_HOST SPI3_HOST
#define LCD_SCLK GPIO_NUM_15
#define LCD_MOSI GPIO_NUM_21
#define LCD_CS GPIO_NUM_14
#define LCD_DC GPIO_NUM_42
#define LCD_RST GPIO_NUM_48
#define DRAW_BUF_LINES 16

#define SYS_SDA GPIO_NUM_45
#define SYS_SCL GPIO_NUM_0
#define BASE_SDA GPIO_NUM_38
#define BASE_SCL GPIO_NUM_39
#define I2S_BCLK GPIO_NUM_8
#define I2S_WS GPIO_NUM_6
#define I2S_DOUT GPIO_NUM_5
#define I2S_DIN GPIO_NUM_7

#define BUTTON_GPIO GPIO_NUM_41    /* USER_BUT: pressing the screen */
#define PTT_HOLD_MS 250            /* shorter than this is a tap, not push-to-talk */
#define DOUBLE_TAP_MS 350          /* a second tap within this opens the menu */
#define TAP_PULSE_MS 120           /* tap-to-call press length: under the SDK's 300 ms note minimum */

#define LP5562_ADDR 0x30
#define LP5562_ENABLE 0x00
#define LP5562_CONFIG 0x08
#define LP5562_W_PWM 0x0E
#define LP5562_LED_MAP 0x70

#define PIOE_ADDR 0x43             /* PI4IOE5V6408 on the base: amp enable */
#define PIOE_DIR 0x03
#define PIOE_OUT 0x05
#define PIOE_HIZ 0x07
#define PIOE_PULL_EN 0x0B

static i2c_master_bus_handle_t s_sys_i2c, s_base_i2c;
static i2c_master_dev_handle_t s_lp5562, s_pioe;
static esp_lcd_panel_handle_t s_panel;
static muse_gpio_button_t s_btn;

/* Asks the button state machine for a tap, which ends a call. Set from the
 * motion task (a shake) and consumed in poll_buttons. */
static volatile bool s_hangup_req;
void atoms3r_board_request_hangup(void)
{
    s_hangup_req = true;
}

static esp_err_t reg_write(i2c_master_dev_handle_t dev, uint8_t reg, uint8_t val)
{
    const uint8_t buf[2] = { reg, val };
    return i2c_master_transmit(dev, buf, sizeof(buf), 50);
}

static esp_err_t add_dev(i2c_master_bus_handle_t bus, uint8_t addr, i2c_master_dev_handle_t *dev)
{
    const i2c_device_config_t cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = addr,
        .scl_speed_hz = 100000,
    };
    return i2c_master_bus_add_device(bus, &cfg, dev);
}

static esp_err_t init(void)
{
    i2c_master_bus_config_t sys_cfg = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = SYS_SDA,
        .scl_io_num = SYS_SCL,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    ESP_RETURN_ON_ERROR(i2c_new_master_bus(&sys_cfg, &s_sys_i2c), TAG, "sys i2c");
    i2c_master_bus_config_t base_cfg = sys_cfg;
    base_cfg.i2c_port = I2C_NUM_1;
    base_cfg.sda_io_num = BASE_SDA;
    base_cfg.scl_io_num = BASE_SCL;
    ESP_RETURN_ON_ERROR(i2c_new_master_bus(&base_cfg, &s_base_i2c), TAG, "base i2c");
    ESP_RETURN_ON_ERROR(add_dev(s_sys_i2c, LP5562_ADDR, &s_lp5562), TAG, "lp5562");
    ESP_RETURN_ON_ERROR(add_dev(s_base_i2c, PIOE_ADDR, &s_pioe), TAG, "pi4ioe");

    /* LP5562: chip on, internal clock, all channels driven from the PWM
     * registers. The white channel is the backlight; it starts dark. */
    ESP_RETURN_ON_ERROR(reg_write(s_lp5562, LP5562_ENABLE, 0x40), TAG, "lp5562 enable");
    vTaskDelay(pdMS_TO_TICKS(1));
    ESP_RETURN_ON_ERROR(reg_write(s_lp5562, LP5562_CONFIG, 0x01), TAG, "lp5562 config");
    ESP_RETURN_ON_ERROR(reg_write(s_lp5562, LP5562_LED_MAP, 0x00), TAG, "lp5562 map");
    ESP_RETURN_ON_ERROR(reg_write(s_lp5562, LP5562_W_PWM, 0), TAG, "lp5562 off");

    /* Amp enable lines: outputs, push-pull, no pulls, low (amp off). */
    ESP_RETURN_ON_ERROR(reg_write(s_pioe, PIOE_OUT, 0x00), TAG, "amp off");
    ESP_RETURN_ON_ERROR(reg_write(s_pioe, PIOE_DIR, 0xFF), TAG, "pioe dir");
    ESP_RETURN_ON_ERROR(reg_write(s_pioe, PIOE_HIZ, 0x00), TAG, "pioe push-pull");
    ESP_RETURN_ON_ERROR(reg_write(s_pioe, PIOE_PULL_EN, 0x00), TAG, "pioe pulls");

    ESP_RETURN_ON_ERROR(muse_gpio_button_init(&s_btn, BUTTON_GPIO), TAG, "button");
    /* A press that woke the board is still down; don't count it. */
    s_btn.pressed = gpio_get_level(BUTTON_GPIO) == 0;
    return ESP_OK;
}

/* ST7735S init (M5GFX Panel_ST7735S), sent after the hard reset and before
 * the generic MIPI-DCS setup of esp_lcd. */
static esp_err_t st7735s_init(esp_lcd_panel_io_handle_t io)
{
    static const struct {
        uint8_t cmd;
        uint8_t n;
        uint8_t arg[16];
    } seq[] = {
        { 0xB1, 3, { 0x01, 0x2C, 0x2D } },
        { 0xB2, 3, { 0x01, 0x2C, 0x2D } },
        { 0xB3, 6, { 0x01, 0x2C, 0x2D, 0x01, 0x2C, 0x2D } },
        { 0xB4, 1, { 0x07 } },
        { 0xC0, 3, { 0xA2, 0x02, 0x84 } },
        { 0xC1, 1, { 0xC5 } },
        { 0xC2, 2, { 0x0A, 0x00 } },
        { 0xC3, 2, { 0x8A, 0x2A } },
        { 0xC4, 2, { 0x8A, 0xEE } },
        { 0xC5, 1, { 0x0E } },
        { 0xE0, 16, { 0x02, 0x1C, 0x07, 0x12, 0x37, 0x32, 0x29, 0x2D, 0x29, 0x25, 0x2B, 0x39, 0x00, 0x01, 0x03, 0x10 } },
        { 0xE1, 16, { 0x03, 0x1D, 0x07, 0x06, 0x2E, 0x2C, 0x29, 0x2D, 0x2E, 0x2E, 0x37, 0x3F, 0x00, 0x00, 0x02, 0x10 } },
    };
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, 0x01, NULL, 0), TAG, "swreset");
    vTaskDelay(pdMS_TO_TICKS(150));
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, 0x11, NULL, 0), TAG, "sleep out");
    vTaskDelay(pdMS_TO_TICKS(120));
    for (size_t i = 0; i < sizeof(seq) / sizeof(seq[0]); i++) {
        ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, seq[i].cmd, seq[i].arg, seq[i].n), TAG, "init %02x", seq[i].cmd);
    }
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, 0x13, NULL, 0), TAG, "normal on");
    vTaskDelay(pdMS_TO_TICKS(10));
    return ESP_OK;
}

static lv_display_t *display_start(lv_indev_t **touch)
{
    (void)touch;
    const spi_bus_config_t bus = {
        .sclk_io_num = LCD_SCLK,
        .mosi_io_num = LCD_MOSI,
        .miso_io_num = GPIO_NUM_NC,
        .quadwp_io_num = GPIO_NUM_NC,
        .quadhd_io_num = GPIO_NUM_NC,
        .max_transfer_sz = LCD_W * DRAW_BUF_LINES * 2,
    };
    if (spi_bus_initialize(LCD_HOST, &bus, SPI_DMA_CH_AUTO) != ESP_OK) {
        return NULL;
    }
    esp_lcd_panel_io_handle_t io;
    const esp_lcd_panel_io_spi_config_t io_cfg = {
        .cs_gpio_num = LCD_CS,
        .dc_gpio_num = LCD_DC,
        .spi_mode = 0,
        .pclk_hz = 40 * 1000 * 1000,
        .trans_queue_depth = 10,
        .lcd_cmd_bits = 8,
        .lcd_param_bits = 8,
    };
    if (esp_lcd_new_panel_io_spi(LCD_HOST, &io_cfg, &io) != ESP_OK) {
        return NULL;
    }
    const esp_lcd_panel_dev_config_t panel_cfg = {
        .reset_gpio_num = LCD_RST,
        // This panel wires red and blue the other way round: a red/green/blue test card
        // showed blue/green/red until this was BGR.
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_BGR,
        .bits_per_pixel = 16,
    };
    /* The ST7789 helper is generic MIPI DCS (CASET/RASET/RAMWR/MADCTL/COLMOD),
     * which the ST7735S shares; only the init sequence above is ST7735S. */
    if (esp_lcd_new_panel_st7789(io, &panel_cfg, &s_panel) != ESP_OK) {
        return NULL;
    }
    esp_lcd_panel_reset(s_panel);
    if (st7735s_init(io) != ESP_OK) {
        return NULL;
    }
    esp_lcd_panel_init(s_panel);
    esp_lcd_panel_invert_color(s_panel, true);
    esp_lcd_panel_mirror(s_panel, true, true);   /* M5GFX offset_rotation 2 */
    esp_lcd_panel_set_gap(s_panel, LCD_GAP_X, LCD_GAP_Y);
    esp_lcd_panel_disp_on_off(s_panel, true);

    esp_lv_adapter_config_t adapter_cfg = ESP_LV_ADAPTER_DEFAULT_CONFIG();
    adapter_cfg.task_core_id = MUSE_UI_CORE;
    adapter_cfg.task_priority = MUSE_UI_PRIORITY;
    if (esp_lv_adapter_init(&adapter_cfg) != ESP_OK) {
        return NULL;
    }
    const esp_lv_adapter_display_config_t disp_cfg = {
        .panel = s_panel,
        .panel_io = io,
        .profile = {
            .interface = ESP_LV_ADAPTER_PANEL_IF_OTHER,
            .rotation = ESP_LV_ADAPTER_ROTATE_0,
            .hor_res = LCD_W,
            .ver_res = LCD_H,
            .buffer_height = DRAW_BUF_LINES,
            .use_psram = false,
            .require_double_buffer = true,
        },
        .tear_avoid_mode = ESP_LV_ADAPTER_TEAR_AVOID_MODE_NONE,
    };
    lv_display_t *disp = esp_lv_adapter_register_display(&disp_cfg);
    if (!disp || esp_lv_adapter_start() != ESP_OK) {
        return NULL;
    }
    return disp;
}

static bool display_lock(int timeout_ms)
{
    return esp_lv_adapter_lock(timeout_ms) == ESP_OK;
}

static void set_brightness(int pct)
{
    reg_write(s_lp5562, LP5562_W_PWM, pct * 255 / 100);
}

static void panel_sleep(bool sleep)
{
    esp_lcd_panel_disp_sleep(s_panel, sleep);   /* SLPIN/SLPOUT; GRAM is kept */
}

/* The amp enable is on the base's PI4IOE outputs. */
static void audio_power(bool on)
{
    reg_write(s_pioe, PIOE_OUT, on ? 0xFF : 0x00);
}

/* One ES8311 does both directions over a duplex I2S bus, clocked from BCLK. */
static esp_err_t audio_init(esp_codec_dev_handle_t *spk, esp_codec_dev_handle_t *mic)
{
    i2s_chan_handle_t tx, rx;
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
    chan_cfg.auto_clear = true;
    ESP_RETURN_ON_ERROR(i2s_new_channel(&chan_cfg, &tx, &rx), TAG, "i2s channel");
    const i2s_std_config_t std_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(MUSE_AUDIO_RATE),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO),
        .gpio_cfg = {
            .mclk = GPIO_NUM_NC,
            .bclk = I2S_BCLK,
            .ws = I2S_WS,
            .dout = I2S_DOUT,
            .din = I2S_DIN,
        },
    };
    ESP_RETURN_ON_ERROR(i2s_channel_init_std_mode(tx, &std_cfg), TAG, "i2s tx");
    ESP_RETURN_ON_ERROR(i2s_channel_init_std_mode(rx, &std_cfg), TAG, "i2s rx");
    ESP_RETURN_ON_ERROR(i2s_channel_enable(tx), TAG, "i2s tx on");
    ESP_RETURN_ON_ERROR(i2s_channel_enable(rx), TAG, "i2s rx on");

    audio_codec_i2s_cfg_t i2s_cfg = { .port = I2S_NUM_0, .rx_handle = rx, .tx_handle = tx };
    const audio_codec_data_if_t *data_if = audio_codec_new_i2s_data(&i2s_cfg);
    audio_codec_i2c_cfg_t i2c_cfg = { .port = I2C_NUM_1, .addr = ES8311_CODEC_DEFAULT_ADDR, .bus_handle = s_base_i2c };
    const audio_codec_ctrl_if_t *ctrl_if = audio_codec_new_i2c_ctrl(&i2c_cfg);
    ESP_RETURN_ON_FALSE(data_if && ctrl_if, ESP_ERR_NO_MEM, TAG, "codec interfaces");

    es8311_codec_cfg_t es_cfg = {
        .ctrl_if = ctrl_if,
        .gpio_if = audio_codec_new_gpio(),
        .codec_mode = ESP_CODEC_DEV_WORK_MODE_BOTH,
        .pa_pin = -1,
        .use_mclk = false,
        .hw_gain = { .pa_voltage = 5.0, .codec_dac_voltage = 3.3 },
    };
    const audio_codec_if_t *codec = es8311_codec_new(&es_cfg);
    ESP_RETURN_ON_FALSE(codec, ESP_FAIL, TAG, "ES8311 not responding");

    esp_codec_dev_cfg_t out_cfg = { .dev_type = ESP_CODEC_DEV_TYPE_OUT, .codec_if = codec, .data_if = data_if };
    esp_codec_dev_cfg_t in_cfg = { .dev_type = ESP_CODEC_DEV_TYPE_IN, .codec_if = codec, .data_if = data_if };
    *spk = esp_codec_dev_new(&out_cfg);
    *mic = esp_codec_dev_new(&in_cfg);
    /* muse_audio_init() leaves the codecs open without calling audio_power(),
     * so the amp (off since init) would stay off until the first rest cycle. */
    audio_power(true);
    return *spk && *mic ? ESP_OK : ESP_FAIL;
}

/*
 * The screen is the only button:
 *  - hold PTT_HOLD_MS or more: Talk, from then until release (voice note, or
 *    Select in the menu);
 *  - tap: Talk pressed for TAP_PULSE_MS, which the SDK reads as tap-to-call
 *    (start or end a live call). It is sent only once DOUBLE_TAP_MS has passed
 *    without a second tap;
 *  - double tap: one Aux press (open the menu, wake);
 *  - menu open: a tap is Aux at once (step down), a hold is Select.
 */
static unsigned poll_buttons(void)
{
    enum { IDLE, PRESSED, TALKING, SECOND, PENDING, PULSE };
    static int st = IDLE;
    static TickType_t t_edge;
    unsigned ev = muse_gpio_button_poll(&s_btn);
    TickType_t now = xTaskGetTickCount();
    unsigned out = 0;
    bool press = ev & MUSE_BTN_TALK_PRESS;
    bool release = ev & MUSE_BTN_TALK_RELEASE;

    if (press) {
        if (st == PENDING) {
            st = SECOND;
        } else {
            st = PRESSED;
            t_edge = now;
        }
    }
    if (st == PRESSED && s_btn.pressed && now - t_edge >= pdMS_TO_TICKS(PTT_HOLD_MS)) {
        st = TALKING;
        muse_ui_image_dismiss();
        out |= MUSE_BTN_TALK_PRESS;
    }
    if (release) {
        if (st == TALKING) {
            out |= MUSE_BTN_TALK_RELEASE;
            st = IDLE;
        } else if (st == SECOND) {
            out |= MUSE_BTN_AUX_PRESS | MUSE_BTN_AUX_RELEASE;
            st = IDLE;
        } else if (st == PRESSED) {
            if (muse_ui_image_shown()) {
                /* A tap on a picture only clears it, so it can't end a call. */
                muse_ui_image_dismiss();
                st = IDLE;
            } else if (muse_menu_is_open()) {
                out |= MUSE_BTN_AUX_PRESS | MUSE_BTN_AUX_RELEASE;
                st = IDLE;
            } else {
                st = PENDING;
                t_edge = now;
            }
        }
    }
    if (s_hangup_req) {
        s_hangup_req = false;
        if (st == IDLE && muse_voice_in_call()) {
            out |= MUSE_BTN_TALK_PRESS;
            st = PULSE;
            t_edge = now;
        }
    }
    if (st == PENDING && !s_btn.pressed && now - t_edge >= pdMS_TO_TICKS(DOUBLE_TAP_MS)) {
        out |= MUSE_BTN_TALK_PRESS;
        st = PULSE;
        t_edge = now;
    } else if (st == PULSE && now - t_edge >= pdMS_TO_TICKS(TAP_PULSE_MS)) {
        out |= MUSE_BTN_TALK_RELEASE;
        st = IDLE;
    }
    return out;
}

static void wait_buttons(int timeout_ms)
{
    muse_gpio_buttons_wait((muse_gpio_button_t *const[]){ &s_btn }, 1, timeout_ms);
}

static const muse_board_t s_board = {
    .name = "M5Stack AtomS3R",
    .width = LCD_W,
    .height = LCD_H,
    .round = false,
    .touch = false,
    .diagonal_in = 0.85f,
    .talk_button = "screen",
    .aux_button = "screen",
    // One button, so both hints share the bottom edge; the +-1 only tells the
    // menu to put Down on the left and Select on the right instead of stacking.
    .talk_hint = { LV_ALIGN_BOTTOM_MID, 1, -2 },
    .aux_hint = { LV_ALIGN_BOTTOM_MID, -1, -2 },
    .frame_ms = 40,
    .init = init,
    .display_start = display_start,
    .display_lock = display_lock,
    .display_unlock = esp_lv_adapter_unlock,
    .set_brightness = set_brightness,
    .panel_sleep = panel_sleep,
    .audio_init = audio_init,
    .mic_slot = 0,
    .audio_power = audio_power,
    .poll_buttons = poll_buttons,
    .wait_buttons = wait_buttons,
    /* USB powered, no battery, no PMIC. G41 is not an RTC pin, so deep sleep
     * can't wake on a screen press: no software power-off. */
};

/* Home Link's app_main starts Muse with this board (main/main.c). */
const muse_board_t *muse_board_get(void)
{
    return &s_board;
}
