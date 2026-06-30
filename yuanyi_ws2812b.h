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

#ifndef YUANYI_WS2812B_H
#define YUANYI_WS2812B_H

#include "iot_errno.h"

// WS2812B SPI配置参数（需根据实际硬件修改）
#define WS2812B_SPI_ID       HI_SPI_ID_0      // 使用SPI0
#define WS2812B_SPI_GPIO     9                // SPI0 MOSI引脚（GPIO9）
#define WS2812B_SPI_FREQ     3200000          // SPI频率3.2MHz（根据需要调整）
#define WS2812B_LED_COUNT    64               // 串联的LED数量（根据实际灯珠数量修改，当前为8*8灯板）
#define WS2812B_RESET_TIME   50               // 复位信号时间(us)

// 环境光传感器相关定义
#define AMBIENT_LIGHT_MIN    0                // 环境光最小值
#define AMBIENT_LIGHT_MAX    1000             // 环境光最大值（可根据实际传感器调整）
#define WARM_TEMPERATURE     2700             // 暖色温（开尔文）
#define COOL_TEMPERATURE     6500             // 冷色温（开尔文）

// WS2812B颜色结构体定义
typedef struct {
    unsigned char g; // 绿色分量
    unsigned char r; // 红色分量
    unsigned char b; // 蓝色分量
} Ws2812bColor;

/**
 * @brief 初始化WS2812B LED控制器
 * @return 成功返回IOT_SUCCESS，失败返回错误码
 */
unsigned int YuanyiWs2812bInit(void);

/**
 * @brief 设置指定LED的颜色
 * @param index LED索引
 * @param color 颜色值
 * @return 成功返回IOT_SUCCESS，失败返回错误码
 */
unsigned int YuanyiWs2812bSetLedColor(unsigned int index, Ws2812bColor color);

/**
 * @brief 设置所有LED的颜色
 * @param color 颜色值
 * @return 成功返回IOT_SUCCESS，失败返回错误码
 */
unsigned int YuanyiWs2812bSetAllColor(Ws2812bColor color);

/**
 * @brief 刷新显示，将颜色数据发送到WS2812B
 * @return 成功返回IOT_SUCCESS，失败返回错误码
 */
unsigned int YuanyiWs2812bRefresh(void);

/**
 * @brief HSV颜色空间转RGB颜色空间
 * @param h 色相 (0-359)
 * @param s 饱和度 (0-100)
 * @param v 明度 (0-255)
 * @return 转换后的RGB颜色值
 */
Ws2812bColor YuanyiWs2812bHsvToRgb(unsigned int h, unsigned int s, unsigned int v);

/**
 * @brief 清除所有LED（设为黑色）
 * @return 成功返回IOT_SUCCESS，失败返回错误码
 */
unsigned int YuanyiWs2812bClear(void);

/**
 * @brief 根据环境光值调整LED亮度
 * @param ambientLight 环境光值 (0-1000)
 * @param baseColor 基础颜色值
 * @return 调整后的颜色值
 */
Ws2812bColor YuanyiWs2812bAdjustBrightnessByAmbient(unsigned int ambientLight, Ws2812bColor baseColor);

/**
 * @brief 根据环境光值和色温偏好调整LED颜色
 * @param ambientLight 环境光值 (0-1000)
 * @param temperaturePreference 色温偏好 (0-100，0为暖色温，100为冷色温)
 * @return 调整后的颜色值
 */
Ws2812bColor YuanyiWs2812bAdjustByAmbientAndTemperature(unsigned int ambientLight, unsigned int temperaturePreference);

/**
 * @brief 设置所有LED颜色，自动根据环境光调整亮度
 * @param baseColor 基础颜色值
 * @param ambientLight 环境光值 (0-1000)
 * @return 成功返回IOT_SUCCESS，失败返回错误码
 */
unsigned int YuanyiWs2812bSetAllColorWithAmbientAdjust(Ws2812bColor baseColor, unsigned int ambientLight);

#endif /* YUANYI_WS2812B_H */