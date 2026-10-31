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

#pragma once

#include "cJSON.h"

// AtomS3R's BMI270 (0x68 on the internal I2C bus, which the board file opens).
// A background task polls the accelerometer, gyroscope and die temperature and
// keeps the latest of each; sensors.read answers with those.
void atoms3r_imu_init(void);

// sensors.read: the latest readings, each with its value, unit and age in
// seconds. A reading with no recent value is null. Returns an owned cJSON
// result, or NULL if out of memory.
cJSON *atoms3r_imu_command(void);

// Defined by the AtomS3R board file: queues a tap, which ends a call.
void atoms3r_board_request_hangup(void);
