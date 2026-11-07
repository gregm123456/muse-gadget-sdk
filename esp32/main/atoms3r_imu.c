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

#include "atoms3r_imu.h"

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "atoms3r_bmi270_config.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"
#if CONFIG_HOMEHUB_ATOMS3R_GESTURES
#include "muse_state.h"
#include "muse_voice.h"
#endif

static const char *TAG = "link.atoms3r_imu";

#define BMI270_ADDR      0x68
#define BMI270_CHIP_ID   0x24
#define REG_CHIP_ID      0x00
#define REG_DATA_ACC     0x0C   // 6 bytes, then gyro 6 bytes at 0x12
#define REG_INTERNAL_STATUS 0x21
#define REG_TEMPERATURE  0x22
#define REG_ACC_CONF     0x40
#define REG_ACC_RANGE    0x41
#define REG_GYR_CONF     0x42
#define REG_GYR_RANGE    0x43
#define REG_INIT_CTRL    0x59
#define REG_INIT_ADDR    0x5B   // word address: low nibble, then high byte
#define REG_INIT_DATA    0x5E
#define REG_PWR_CONF     0x7C
#define REG_PWR_CTRL     0x7D
#define REG_CMD          0x7E
#define CMD_SOFTRESET    0xB6

// The internal bus the board file opens for the BMI270 and the LP5562.
#define IMU_I2C_PORT I2C_NUM_0

#define CONFIG_CHUNK 32
#define POLL_MS 100
// Readings older than this count as missing, so a dead sensor drops out.
#define STALE_US (5LL * 1000 * 1000)

// +/-4 g and +/-1000 dps.
#define ACC_LSB_PER_G   8192.0f
#define GYR_LSB_PER_DPS 32.768f

enum {
    SENSOR_ACC_X, SENSOR_ACC_Y, SENSOR_ACC_Z,
    SENSOR_GYR_X, SENSOR_GYR_Y, SENSOR_GYR_Z,
    SENSOR_TEMPERATURE,
    SENSOR_COUNT,
};

static const struct {
    const char *name;
    const char *unit;
    int decimals;
} SENSORS[SENSOR_COUNT] = {
    [SENSOR_ACC_X] = {"accel_x", "g", 3},
    [SENSOR_ACC_Y] = {"accel_y", "g", 3},
    [SENSOR_ACC_Z] = {"accel_z", "g", 3},
    [SENSOR_GYR_X] = {"gyro_x", "dps", 1},
    [SENSOR_GYR_Y] = {"gyro_y", "dps", 1},
    [SENSOR_GYR_Z] = {"gyro_z", "dps", 1},
    [SENSOR_TEMPERATURE] = {"die_temperature", "celsius", 1},
};

typedef struct {
    bool valid;
    float value;
    int64_t at_us;
} reading_t;

static reading_t s_readings[SENSOR_COUNT];
static portMUX_TYPE s_lock = portMUX_INITIALIZER_UNLOCKED;
static i2c_master_dev_handle_t s_dev;

// Orientation and shake, derived from the samples. Read-only: nothing here
// triggers an action, so a stray bump can't do anything but update this.
typedef enum { ORIENT_UNKNOWN, ORIENT_FACE_UP, ORIENT_FACE_DOWN, ORIENT_ON_EDGE } orient_t;
static const char *const ORIENT_NAMES[] = {"unknown", "face_up", "face_down", "on_edge"};
#define ORIENT_HOLD_SAMPLES 5          // a posture must hold 0.5 s
#define SHAKE_SPIKE_G 1.0f             // |accel| away from 1 g
#define SHAKE_SPIKE_DPS 400.0f
#define SHAKE_SPIKES 4                 // spikes inside the window
#define SHAKE_WINDOW_US (1000LL * 1000)
#define SHAKE_HANGUP_US (2LL * 1000 * 1000)
#define MOVING_G 0.15f
#define MOVING_DPS 30.0f
#define MOVING_US (1000LL * 1000)
#define FACE_DOWN_SLEEP_US (5LL * 1000 * 1000)   // face-down and still this long sleeps the screen
// A short RAM-only history (lost on reboot) so Muse can answer "what has it
// been doing?" and not only "what is it doing now?".
typedef enum { EV_ORIENT, EV_SHAKE, EV_MOVE_START, EV_MOVE_STOP } event_type_t;
typedef struct {
    int64_t at_us;
    uint8_t type;
    uint8_t orient;
} motion_event_t;
#define EVENT_RING 48
#define SHAKE_LOG_GAP_US (3LL * 1000 * 1000)
static motion_event_t s_events[EVENT_RING];
static int s_event_next;
static int s_event_count;
static uint32_t s_shake_total, s_move_samples;
static bool s_moving_state;
static int64_t s_boot_us;

// Callers hold s_lock.
static void log_event(event_type_t type, orient_t orient, int64_t now) {
    s_events[s_event_next] = (motion_event_t){now, (uint8_t)type, (uint8_t)orient};
    s_event_next = (s_event_next + 1) % EVENT_RING;
    if (s_event_count < EVENT_RING) s_event_count++;
}

static orient_t s_orient = ORIENT_UNKNOWN;
static int64_t s_orient_since_us;
static int64_t s_last_motion_us;       // 0 = none yet
static int64_t s_last_shake_us;        // 0 = none yet

static orient_t classify(float ax, float ay, float az) {
    if (az > 0.8f) return ORIENT_FACE_UP;
    if (az < -0.8f) return ORIENT_FACE_DOWN;
    if (fabsf(az) < 0.5f && (fabsf(ax) > 0.8f || fabsf(ay) > 0.8f)) return ORIENT_ON_EDGE;
    return ORIENT_UNKNOWN;
}

// Called once per sample from the poll task only.
static void update_gestures(const float *v, int64_t now) {
    static orient_t candidate = ORIENT_UNKNOWN;
    static int candidate_n;
    static int64_t spikes[SHAKE_SPIKES];
    static int spike_i;
    float amag = sqrtf(v[SENSOR_ACC_X] * v[SENSOR_ACC_X] + v[SENSOR_ACC_Y] * v[SENSOR_ACC_Y] +
                       v[SENSOR_ACC_Z] * v[SENSOR_ACC_Z]);
    float gmag = sqrtf(v[SENSOR_GYR_X] * v[SENSOR_GYR_X] + v[SENSOR_GYR_Y] * v[SENSOR_GYR_Y] +
                       v[SENSOR_GYR_Z] * v[SENSOR_GYR_Z]);
    bool moving = fabsf(amag - 1.0f) > MOVING_G || gmag > MOVING_DPS;
    bool spike = fabsf(amag - 1.0f) > SHAKE_SPIKE_G || gmag > SHAKE_SPIKE_DPS;
    orient_t o = classify(v[SENSOR_ACC_X], v[SENSOR_ACC_Y], v[SENSOR_ACC_Z]);
    if (o == candidate) {
        if (candidate_n < ORIENT_HOLD_SAMPLES) candidate_n++;
    } else {
        candidate = o;
        candidate_n = 1;
    }
    bool shook = false;
    if (spike) {
        spikes[spike_i] = now;
        spike_i = (spike_i + 1) % SHAKE_SPIKES;
        // The oldest of the last SHAKE_SPIKES spikes is the one about to be overwritten.
        int64_t oldest = spikes[spike_i];
        shook = oldest != 0 && now - oldest <= SHAKE_WINDOW_US;
    }
    taskENTER_CRITICAL(&s_lock);
    if (candidate_n >= ORIENT_HOLD_SAMPLES && candidate != s_orient) {
        s_orient = candidate;
        s_orient_since_us = now;
        log_event(EV_ORIENT, candidate, now);
    }
    if (moving) {
        s_last_motion_us = now;
        s_move_samples++;
        if (!s_moving_state) {
            s_moving_state = true;
            log_event(EV_MOVE_START, s_orient, now);
        }
    } else if (s_moving_state && now - s_last_motion_us >= MOVING_US) {
        s_moving_state = false;
        log_event(EV_MOVE_STOP, s_orient, s_last_motion_us);
    }
    if (shook) {
        if (!s_last_shake_us || now - s_last_shake_us >= SHAKE_LOG_GAP_US) log_event(EV_SHAKE, s_orient, now);
        s_shake_total++;
        s_last_shake_us = now;
        memset(spikes, 0, sizeof(spikes));
    }
    taskEXIT_CRITICAL(&s_lock);
#if CONFIG_HOMEHUB_ATOMS3R_GESTURES
    static bool slept_this_posture;
    static int64_t hangup_shake_since_us, hangup_last_shake_us;
    static bool hangup_requested;
    if (candidate_n >= ORIENT_HOLD_SAMPLES && candidate != ORIENT_FACE_DOWN) slept_this_posture = false;
    if (!muse_voice_in_call()) {
        hangup_shake_since_us = 0;
        hangup_last_shake_us = 0;
        hangup_requested = false;
    } else if (shook && !hangup_requested) {
        // Each detected shake must follow the previous one within the window.
        if (!hangup_shake_since_us || now - hangup_last_shake_us > SHAKE_WINDOW_US) {
            hangup_shake_since_us = now;
        }
        hangup_last_shake_us = now;
        if (now - hangup_shake_since_us >= SHAKE_HANGUP_US) {
            hangup_requested = true;
            ESP_LOGI(TAG, "sustained shake: hanging up");
            atoms3r_board_request_hangup();
        }
    }
    // Once per time it is laid down, so waking it with a button while it
    // lies face-down isn't undone five seconds later.
    if (s_orient == ORIENT_FACE_DOWN && !slept_this_posture && now - s_orient_since_us >= FACE_DOWN_SLEEP_US &&
        now - s_last_motion_us >= FACE_DOWN_SLEEP_US && !muse_voice_in_call()) {
        slept_this_posture = true;
        if (!muse_state_asleep()) {
            ESP_LOGI(TAG, "face-down and still: sleeping the screen");
            muse_state_set_asleep(true);
        }
    }
#endif
}

// sensors.read result: each reading with its value, unit and age in seconds;
// stale readings are null; with no readings at all, an error.
typedef struct {
    orient_t orient;
    int64_t orient_since_us, last_motion_us, last_shake_us;
    uint32_t shake_total, move_samples;
    int event_count;
    motion_event_t events[EVENT_RING];   // oldest first
} motion_t;

static const char *const EVENT_NAMES[] = {"orientation_changed", "shaken", "started_moving", "stopped_moving"};

static void add_motion(cJSON *payload, const motion_t *m, int64_t now_us) {
    cJSON *o = cJSON_AddObjectToObject(payload, "motion");
    if (!o) return;
    cJSON_AddStringToObject(o, "orientation", ORIENT_NAMES[m->orient]);
    if (m->orient != ORIENT_UNKNOWN) {
        cJSON_AddNumberToObject(o, "orientation_for_s", (double)((now_us - m->orient_since_us) / 1000000));
    }
    cJSON_AddBoolToObject(o, "moving", m->last_motion_us && now_us - m->last_motion_us < MOVING_US);
    if (m->last_shake_us) {
        cJSON_AddNumberToObject(o, "last_shake_age_s", (double)((now_us - m->last_shake_us) / 1000000));
    } else {
        cJSON_AddNullToObject(o, "last_shake_age_s");
    }
    cJSON_AddNumberToObject(o, "uptime_s", (double)((now_us - s_boot_us) / 1000000));
    cJSON_AddNumberToObject(o, "shakes_since_boot", m->shake_total);
    cJSON_AddNumberToObject(o, "moving_seconds_since_boot", round(m->move_samples * POLL_MS / 1000.0));
    // Newest first, each with its age, so the caller can place it in time.
    cJSON *list = cJSON_AddArrayToObject(o, "recent_events");
    for (int i = m->event_count - 1; list && i >= 0; i--) {
        const motion_event_t *e = &m->events[i];
        cJSON *item = cJSON_CreateObject();
        if (!item) break;
        cJSON_AddStringToObject(item, "event", EVENT_NAMES[e->type]);
        if (e->type == EV_ORIENT) cJSON_AddStringToObject(item, "orientation", ORIENT_NAMES[e->orient]);
        cJSON_AddNumberToObject(item, "age_s", (double)((now_us - e->at_us) / 1000000));
        cJSON_AddItemToArray(list, item);
    }
}

static cJSON *readings_result(const reading_t *readings, const motion_t *motion, int64_t now_us) {
    cJSON *result = cJSON_CreateObject();
    cJSON *payload = cJSON_CreateObject();
    if (!result || !payload) {
        cJSON_Delete(result);
        cJSON_Delete(payload);
        return NULL;
    }
    bool any = false;
    for (int i = 0; i < SENSOR_COUNT; i++) {
        const reading_t *r = &readings[i];
        if (!r->valid || now_us - r->at_us > STALE_US) {
            cJSON_AddNullToObject(payload, SENSORS[i].name);
            continue;
        }
        cJSON *sensor = cJSON_AddObjectToObject(payload, SENSORS[i].name);
        double scale = pow(10, SENSORS[i].decimals);
        cJSON_AddNumberToObject(sensor, "value", round(r->value * scale) / scale);
        cJSON_AddStringToObject(sensor, "unit", SENSORS[i].unit);
        cJSON_AddNumberToObject(sensor, "age_s", (double)((now_us - r->at_us) / 1000000));
        any = true;
    }
    if (!any) {
        cJSON_Delete(payload);
        cJSON_AddBoolToObject(result, "ok", false);
        cJSON *error = cJSON_AddObjectToObject(result, "error");
        cJSON_AddStringToObject(error, "code", "no_readings");
        cJSON_AddStringToObject(error, "message", "no motion sensor readings; the BMI270 may not be responding");
        return result;
    }
    add_motion(payload, motion, now_us);
    cJSON_AddBoolToObject(result, "ok", true);
    cJSON_AddItemToObject(result, "payload", payload);
    return result;
}

static esp_err_t reg_write(uint8_t reg, uint8_t val) {
    const uint8_t buf[2] = {reg, val};
    return i2c_master_transmit(s_dev, buf, sizeof(buf), 100);
}

static esp_err_t reg_read(uint8_t reg, uint8_t *out, size_t len) {
    return i2c_master_transmit_receive(s_dev, &reg, 1, out, len, 100);
}

// Loads the configuration blob in 32-byte bursts, each at its word address.
static esp_err_t load_config(void) {
    esp_err_t err = reg_write(REG_PWR_CONF, 0x00);
    if (err != ESP_OK) return err;
    vTaskDelay(pdMS_TO_TICKS(2));
    if ((err = reg_write(REG_INIT_CTRL, 0x00)) != ESP_OK) return err;
    uint8_t buf[1 + CONFIG_CHUNK];
    buf[0] = REG_INIT_DATA;
    for (size_t i = 0; i < sizeof(bmi270_config_file); i += CONFIG_CHUNK) {
        size_t n = sizeof(bmi270_config_file) - i;
        if (n > CONFIG_CHUNK) n = CONFIG_CHUNK;
        const uint8_t addr[3] = {REG_INIT_ADDR, (uint8_t)((i / 2) & 0x0F), (uint8_t)((i / 2) >> 4)};
        err = i2c_master_transmit(s_dev, addr, sizeof(addr), 100);
        if (err != ESP_OK) return err;
        memcpy(buf + 1, bmi270_config_file + i, n);
        err = i2c_master_transmit(s_dev, buf, 1 + n, 100);
        if (err != ESP_OK) return err;
    }
    if ((err = reg_write(REG_INIT_CTRL, 0x01)) != ESP_OK) return err;
    vTaskDelay(pdMS_TO_TICKS(30));
    uint8_t status = 0;
    if ((err = reg_read(REG_INTERNAL_STATUS, &status, 1)) != ESP_OK) return err;
    if ((status & 0x0F) != 0x01) {
        ESP_LOGW(TAG, "configuration not accepted (internal status 0x%02x)", status);
        return ESP_FAIL;
    }
    return ESP_OK;
}

static esp_err_t imu_start(void) {
    uint8_t id = 0;
    esp_err_t err = reg_read(REG_CHIP_ID, &id, 1);
    if (err != ESP_OK) return err;
    if (id != BMI270_CHIP_ID) {
        ESP_LOGW(TAG, "unexpected chip id 0x%02x (want 0x%02x)", id, BMI270_CHIP_ID);
        return ESP_ERR_NOT_FOUND;
    }
    err = reg_write(REG_CMD, CMD_SOFTRESET);
    if (err != ESP_OK) return err;
    vTaskDelay(pdMS_TO_TICKS(20));
    // The data registers stay at zero until Bosch's configuration is loaded.
    if ((err = load_config()) != ESP_OK) return err;
    // Stay out of advanced power save, so the registers answer at once.
    if ((err = reg_write(REG_PWR_CONF, 0x02)) != ESP_OK) return err;
    vTaskDelay(pdMS_TO_TICKS(2));
    // ACC_CONF: 100 Hz, normal filter, performance mode; GYR_CONF likewise.
    if ((err = reg_write(REG_ACC_CONF, 0xA8)) != ESP_OK) return err;
    if ((err = reg_write(REG_ACC_RANGE, 0x01)) != ESP_OK) return err;
    if ((err = reg_write(REG_GYR_CONF, 0xE9)) != ESP_OK) return err;
    if ((err = reg_write(REG_GYR_RANGE, 0x01)) != ESP_OK) return err;
    // Temperature, accelerometer and gyroscope on; auxiliary (magnetometer) off.
    if ((err = reg_write(REG_PWR_CTRL, 0x0E)) != ESP_OK) return err;
    vTaskDelay(pdMS_TO_TICKS(60));
    return ESP_OK;
}

static int16_t le16(const uint8_t *p) { return (int16_t)(p[0] | (p[1] << 8)); }

static esp_err_t imu_poll(void) {
    uint8_t d[12];
    esp_err_t err = reg_read(REG_DATA_ACC, d, sizeof(d));
    if (err != ESP_OK) return err;
    uint8_t t[2];
    err = reg_read(REG_TEMPERATURE, t, sizeof(t));
    if (err != ESP_OK) return err;
    int16_t raw_t = le16(t);
    float v[SENSOR_COUNT];
    for (int i = 0; i < 3; i++) {
        v[SENSOR_ACC_X + i] = le16(d + 2 * i) / ACC_LSB_PER_G;
        v[SENSOR_GYR_X + i] = le16(d + 6 + 2 * i) / GYR_LSB_PER_DPS;
    }
    // 0x8000 means no sample yet.
    bool temp_ok = raw_t != INT16_MIN;
    v[SENSOR_TEMPERATURE] = 23.0f + raw_t / 512.0f;
    int64_t now = esp_timer_get_time();
    taskENTER_CRITICAL(&s_lock);
    for (int i = 0; i < SENSOR_COUNT; i++) {
        if (i == SENSOR_TEMPERATURE && !temp_ok) continue;
        s_readings[i] = (reading_t){.valid = true, .value = v[i], .at_us = now};
    }
    taskEXIT_CRITICAL(&s_lock);
    update_gestures(v, now);
    return ESP_OK;
}

static void imu_task(void *arg) {
    (void)arg;
    i2c_master_bus_handle_t bus = NULL;
    // The board file opens the bus while Muse starts; wait for it.
    for (int i = 0; i < 60 && i2c_master_get_bus_handle(IMU_I2C_PORT, &bus) != ESP_OK; i++) {
        vTaskDelay(pdMS_TO_TICKS(500));
    }
    if (!bus) {
        ESP_LOGW(TAG, "internal I2C bus never appeared; no motion sensor");
        vTaskDelete(NULL);
        return;
    }
    const i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = BMI270_ADDR,
        .scl_speed_hz = 400000,
    };
    esp_err_t err = i2c_master_bus_add_device(bus, &dev_cfg, &s_dev);
    if (err == ESP_OK) err = imu_start();
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "BMI270 start failed: %s", esp_err_to_name(err));
        vTaskDelete(NULL);
        return;
    }
    ESP_LOGI(TAG, "BMI270 running: +/-4 g, +/-1000 dps, %d ms poll", POLL_MS);
    bool failing = false;
    for (;;) {
        if (imu_poll() == ESP_OK) {
            failing = false;
        } else if (!failing) {
            failing = true;
            ESP_LOGW(TAG, "read failed; will retry");
        }
        vTaskDelay(pdMS_TO_TICKS(POLL_MS));
    }
}

void atoms3r_imu_init(void) {
    s_boot_us = esp_timer_get_time();
    if (xTaskCreate(imu_task, "imu", 3072, NULL, 2, NULL) != pdPASS) {
        ESP_LOGE(TAG, "failed to start the motion sensor task");
    }
}

cJSON *atoms3r_imu_command(void) {
    reading_t snapshot[SENSOR_COUNT];
    taskENTER_CRITICAL(&s_lock);
    memcpy(snapshot, s_readings, sizeof(snapshot));
    motion_t motion = {.orient = s_orient, .orient_since_us = s_orient_since_us,
                       .last_motion_us = s_last_motion_us, .last_shake_us = s_last_shake_us,
                       .shake_total = s_shake_total, .move_samples = s_move_samples,
                       .event_count = s_event_count};
    int first = (s_event_next - s_event_count + EVENT_RING) % EVENT_RING;
    for (int i = 0; i < s_event_count; i++) motion.events[i] = s_events[(first + i) % EVENT_RING];
    taskEXIT_CRITICAL(&s_lock);
    return readings_result(snapshot, &motion, esp_timer_get_time());
}
