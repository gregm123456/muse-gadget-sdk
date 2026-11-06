// Copyright (c) Meta Platforms, Inc. and affiliates.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <assert.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#define CONFIG_HOMEHUB_ATOMS3R_GESTURES 1
#define taskENTER_CRITICAL(lock) ((void)(lock))
#define taskEXIT_CRITICAL(lock) ((void)(lock))
#define ESP_LOGI(tag, ...) ((void)(tag))
static int s_lock;
static const char *TAG = "test";
enum {
    SENSOR_ACC_X, SENSOR_ACC_Y, SENSOR_ACC_Z,
    SENSOR_GYR_X, SENSOR_GYR_Y, SENSOR_GYR_Z, SENSOR_TEMPERATURE, SENSOR_COUNT
};
static bool in_call;
static int hangups;
static bool muse_voice_in_call(void) { return in_call; }
static void atoms3r_board_request_hangup(void) { hangups++; }
static bool muse_state_asleep(void) { return false; }
static void muse_state_set_asleep(bool asleep) { (void)asleep; }

#include "atoms3r_gestures.inc"

static void sample(int64_t now, bool spike) {
    float v[SENSOR_COUNT] = {0};
    v[SENSOR_ACC_Z] = 1.0f;
    v[SENSOR_GYR_X] = spike ? 500.0f : 0.0f;
    update_gestures(v, now);
}

static void burst(int64_t start) {
    for (int i = 0; i < 4; i++) sample(start + i * 250000LL, true);
}

int main(void) {
    in_call = true;
    burst(1000000);
    burst(2000000);
    assert(hangups == 0);
    sample(3000000, true);
    sample(3250000, true);
    sample(3500000, true);
    assert(hangups == 0);
    sample(3750000, true); // exactly two seconds after the first detected shake
    assert(hangups == 1);
    burst(5000000);
    assert(hangups == 1);

    in_call = false;
    sample(10000000, false);
    burst(11000000);
    burst(12000000);
    burst(13000000);
    burst(14000000);
    assert(hangups == 1);

    in_call = true;
    burst(20000000);
    burst(21000000);
    burst(24000000); // a gap longer than the shake window resets the timer
    burst(25000000);
    assert(hangups == 1);
    burst(26000000);
    assert(hangups == 2);
    assert(s_shake_total > 0);
    return 0;
}
