/* SPDX-License-Identifier: Apache-2.0 */
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>

#include "atoms3r_button_defines.h"

typedef uint64_t TickType_t;
#define pdMS_TO_TICKS(ms) (ms)
static TickType_t now;
static unsigned pending;
static struct { bool pressed; } s_btn;
static bool s_hangup_req, in_call, ptt_call, ptt_enabled, menu, image;

static unsigned muse_gpio_button_poll(void *button) { (void)button; unsigned e = pending; pending = 0; return e; }
static TickType_t xTaskGetTickCount(void) { return now; }
static bool muse_voice_in_call(void) { return in_call; }
static bool muse_voice_ptt_call(void) { return ptt_call; }
static bool muse_voice_ptt_enabled(void) { return ptt_enabled; }
static bool muse_menu_is_open(void) { return menu; }
static bool muse_ui_image_shown(void) { return image; }
static void muse_ui_image_dismiss(void) { image = false; }

#include "atoms3r_button.inc"

static unsigned edge(bool down, TickType_t at)
{
    now = at;
    s_btn.pressed = down;
    pending = down ? MUSE_BTN_TALK_PRESS : MUSE_BTN_TALK_RELEASE;
    return poll_buttons();
}

static unsigned tick(TickType_t at) { now = at; return poll_buttons(); }

int main(void)
{
    in_call = ptt_call = true;
    /* A tap becomes an explicit hangup, not a microphone press. */
    assert(edge(true, 0) == 0);
    assert(edge(false, 100) == 0);
    assert(tick(449) == 0);
    assert(tick(450) == MUSE_BTN_CALL_END);

    /* A 300 ms physical hold is still talk even though its emitted hold is
     * only 50 ms; it must not be interpreted as a tap or hangup. */
    assert(edge(true, 1000) == 0);
    assert(tick(1250) == MUSE_BTN_TALK_PRESS);
    assert(edge(false, 1300) == MUSE_BTN_TALK_RELEASE);
    assert(tick(1700) == 0);

    /* A double tap opens the menu and does not hang up. */
    assert(edge(true, 2000) == 0);
    assert(edge(false, 2100) == 0);
    assert(edge(true, 2200) == 0);
    assert(edge(false, 2250) == (MUSE_BTN_AUX_PRESS | MUSE_BTN_AUX_RELEASE));
    assert(tick(2600) == 0);

    s_hangup_req = true;
    assert(tick(3000) == MUSE_BTN_CALL_END);

    /* Hands-free tap and shake keep the original pulse behavior. */
    ptt_call = false;
    assert(edge(true, 4000) == 0);
    assert(edge(false, 4100) == 0);
    assert(tick(4450) == MUSE_BTN_TALK_PRESS);
    assert(tick(4570) == MUSE_BTN_TALK_RELEASE);
    s_hangup_req = true;
    assert(tick(5000) == MUSE_BTN_TALK_PRESS);
    assert(tick(5120) == MUSE_BTN_TALK_RELEASE);
    /* A tap starts PTT without synthesizing a microphone hold. */
    in_call = false;
    ptt_enabled = true;
    assert(edge(true, 6000) == 0);
    assert(edge(false, 6100) == 0);
    assert(tick(6450) == MUSE_BTN_CALL_START);
    assert(tick(6570) == 0);
    /* A real hold still starts a call with the microphone held. */
    assert(edge(true, 7000) == 0);
    assert(tick(7250) == MUSE_BTN_TALK_PRESS);
    assert(edge(false, 7300) == MUSE_BTN_TALK_RELEASE);
    return 0;
}
