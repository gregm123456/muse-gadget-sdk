/* SPDX-License-Identifier: Apache-2.0 */
#include <assert.h>
#include <stdbool.h>
#include <string.h>

#define CONFIG_MUSE_BOARD_M5STACK_ATOMS3R 1
#define CONFIG_MUSE_REALTIME_VOICE 1
#define CONFIG_MUSE_SMALL_FULL_REPLY 0
#define CONFIG_MUSE_VIDEO 0
#define ESP_LOGI(...)
#include "atoms3r_menu_types.h"

static bool ptt;
static int s_sel;
static view_t s_view;
static const int VOLUME_STEPS[] = {0, 100}, BRIGHT_STEPS[] = {10, 100};
static const int GAIN_STEPS[] = {0, 30}, SLEEP_STEPS[] = {0, 60};
#define COUNT(a) (sizeof(a) / sizeof((a)[0]))
static int value_step(const int *steps, int count, int current, int direction)
{ (void)count; (void)current; (void)direction; return steps[0]; }
static int muse_settings_volume(void) { return 0; }
static int muse_settings_brightness(void) { return 0; }
static int muse_settings_mic_gain(void) { return 0; }
static int muse_settings_sleep_s(void) { return 0; }
static bool muse_settings_speaker_on(void) { return false; }
static bool muse_settings_ble_on(void) { return false; }
static bool muse_settings_wifi_on(void) { return false; }
static bool muse_settings_voice_ptt(void) { return ptt; }
static void muse_settings_set_voice_ptt(bool v) { ptt = v; }
static void muse_settings_set_volume(int v) { (void)v; }
static void muse_settings_set_brightness(int v) { (void)v; }
static void muse_settings_set_mic_gain(int v) { (void)v; }
static void muse_settings_set_sleep_s(int v) { (void)v; }
static void muse_settings_set_speaker_on(bool v) { (void)v; }
static void muse_settings_set_ble_on(bool v) { (void)v; }
static void muse_settings_set_wifi_on(bool v) { (void)v; }
static void muse_voice_request_chirp(void) {}
static void muse_menu_close(void) { s_view = VIEW_CLOSED; }
static void muse_state_set_asleep(bool v) { (void)v; }
static void muse_state_set_caption(const char *v) { (void)v; }
static void muse_input_request_power_off(void) {}
static void muse_link_reset_setup(void) {}
static void refresh(void) {}
static void open_menu(void) { s_view = VIEW_LIST; s_sel = 0; }
static void show(view_t view) { s_view = view; }
static bool page_scroll(int direction) { (void)direction; return false; }
static void page_hint(void) {}

#include "atoms3r_menu.inc"

int main(void)
{
    assert(!strcmp(ITEM_NAMES[ITEM_PTT], "Push-to-talk"));
    handle(MUSE_MENU_DOWN);
    assert(s_view == VIEW_LIST);
    for (int i = 0; i < ITEM_PTT; ++i) handle(MUSE_MENU_DOWN);
    assert(s_sel == ITEM_PTT && !ptt);
    handle(MUSE_MENU_SELECT);
    assert(ptt);
    handle(MUSE_MENU_SELECT);
    assert(!ptt);
    return 0;
}
