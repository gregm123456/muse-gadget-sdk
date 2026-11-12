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
#include <assert.h>
#include "muse_settings.c"

static bool saved, value, legacy_saved, legacy_value;
static const char *saved_key;
static int commits, notices;
esp_err_t nvs_flash_init(void) { return ESP_OK; }
esp_err_t nvs_flash_erase(void) { assert(!"must preserve existing NVS"); return ESP_FAIL; }
esp_err_t nvs_open(const char *ns,nvs_open_mode_t mode,nvs_handle_t *h) {
    assert(!strcmp(ns,"muse") && mode==NVS_READWRITE); *h=1; return ESP_OK;
}
esp_err_t nvs_get_u8(nvs_handle_t h,const char *key,uint8_t *v) {
    assert(h==1);
    if(!strcmp(key,"voice_ptt") && saved) { *v=value; return ESP_OK; }
    if(!strcmp(key,"voice_notes") && legacy_saved) { *v=legacy_value; return ESP_OK; }
    return ESP_FAIL;
}
esp_err_t nvs_get_u16(nvs_handle_t h,const char *key,uint16_t *v) { (void)h;(void)key;(void)v;return ESP_FAIL; }
esp_err_t nvs_get_str(nvs_handle_t h,const char *key,char *v,size_t *n) { (void)h;(void)key;(void)v;(void)n;return ESP_FAIL; }
esp_err_t nvs_set_u8(nvs_handle_t h,const char *key,uint8_t v) {
    assert(h==1 && !strcmp(key,"voice_ptt")); saved=true; saved_key=key; value=v; return ESP_OK;
}
esp_err_t nvs_commit(nvs_handle_t h) { assert(h==1); ++commits; return ESP_OK; }
SemaphoreHandle_t xSemaphoreCreateMutex(void) { return (void *)1; }
static void changed(muse_setting_t what) { assert(what==MUSE_SETTING_VOICE_PTT); ++notices; }
int main(int argc,char **argv) {
    (void)argv; assert(argc==2); assert(muse_settings_init()==ESP_OK);
    bool default_ptt=CONFIG_MUSE_BOARD_M5STACK_STOPWATCH && CONFIG_MUSE_REALTIME_VOICE;
    assert(muse_settings_voice_ptt()==default_ptt && !saved && !commits);
    muse_settings_set_listener(changed);
    muse_settings_set_voice_ptt(true);
    assert(muse_settings_voice_ptt() && saved && !strcmp(saved_key,"voice_ptt") && value && commits==1 && notices==1);
    atomic_store(&s_voice_ptt,false);
    assert(muse_settings_init()==ESP_OK);
    assert(muse_settings_voice_ptt());
    muse_settings_set_voice_ptt(false);
    assert(!muse_settings_voice_ptt() && !value && commits==2 && notices==2);
    if(CONFIG_MUSE_BOARD_M5STACK_STOPWATCH && CONFIG_MUSE_REALTIME_VOICE) {
        saved=false;
        legacy_saved=true;
        legacy_value=true;
        atomic_store(&s_voice_ptt,true);
        assert(muse_settings_init()==ESP_OK);
        assert(!muse_settings_voice_ptt());
    }
    return 0;
}
