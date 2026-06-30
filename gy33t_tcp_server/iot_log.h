/*
 * Copyright (c) 2022 HiSilicon (Shanghai) Technologies CO., LIMITED.
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

#ifndef IOT_LOG_H
#define IOT_LOG_H

#include <stdio.h>

typedef enum {
    EN_IOT_LOG_LEVEL_TRACE = 0,
    EN_IOT_LOG_LEVEL_DEBUG,
    EN_IOT_LOG_LEVEL_INFO,
    EN_IOT_LOG_LEVEL_WARN,
    EN_IOT_LOG_LEVEL_ERROR,
    EN_IOT_LOG_LEVEL_FATAL,
    EN_IOT_LOG_LEVEL_MAX,
}EnIotLogLevelT;

EnIotLogLevelT IoTLogLevelGet(void);
const char *IoTLogLevelGetName(EnIotLogLevelT logLevel);
int IoTLogLevelSet(EnIotLogLevelT level);

#endif
