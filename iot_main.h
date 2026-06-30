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

#ifndef IOT_MAIN_H
#define IOT_MAIN_H

#define CONFIG_DEVICE_ID  "67e92e505367f573f77dee72_test2"
#define CONFIG_DEVICE_PWD "123456789"

typedef void  (*FnMsgCallBack)(int qos, const char *topic, const char *payload);

int IoTMain(void);

int IoTSetMsgCallback(FnMsgCallBack msgCallback);

int IotSendMsg(int qos, const char *topic, const char *payload);

#endif
