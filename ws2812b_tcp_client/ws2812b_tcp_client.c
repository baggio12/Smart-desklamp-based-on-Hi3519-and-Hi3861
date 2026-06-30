#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <stdlib.h>

#include "ohos_init.h"
#include "cmsis_os2.h"
#include "wifi_connect.h"
#include "yuanyi_ws2812b.h"

#include "lwip/sockets.h"
#include "lwip/netif.h"
#include "lwip/netifapi.h"

#define WIFI_SSID "964"
#define WIFI_PASSWORD "xxxykjfg"

#define TCP_SERVER_IP "192.168.3.70"
#define TCP_SERVER_PORT 8888
#define BUFFER_SIZE 128

static void ParseSensorData(char *data, unsigned int *colorTemp, float *lux, 
                             int *brightness, unsigned char *r, unsigned char *g, unsigned char *b)
{
    char *token;
    char *saveptr;
    char temp[BUFFER_SIZE];
    
    strncpy(temp, data, BUFFER_SIZE - 1);
    temp[BUFFER_SIZE - 1] = '\0';
    
    token = strtok_r(temp, ",\n", &saveptr);
    while (token != NULL) {
        if (strncmp(token, "CT:", 3) == 0) {
            *colorTemp = atoi(token + 3);
        } else if (strncmp(token, "LUX:", 4) == 0) {
            *lux = atof(token + 4);
        } else if (strncmp(token, "BRIGHT:", 7) == 0) {
            *brightness = atoi(token + 7);
        } else if (strncmp(token, "R:", 2) == 0) {
            *r = atoi(token + 2);
        } else if (strncmp(token, "G:", 2) == 0) {
            *g = atoi(token + 2);
        } else if (strncmp(token, "B:", 2) == 0) {
            *b = atoi(token + 2);
        }
        token = strtok_r(NULL, ",\n", &saveptr);
    }
}

static Ws2812bColor ColorTempToRgb(unsigned int colorTemp)
{
    Ws2812bColor color = {0};
    
    if (colorTemp <= 2000) {
        color.r = 255;
        color.g = 60;
        color.b = 0;
    } else if (colorTemp <= 3000) {
        color.r = 255;
        color.g = 140;
        color.b = 0;
    } else if (colorTemp <= 4000) {
        color.r = 255;
        color.g = 200;
        color.b = 100;
    } else if (colorTemp <= 5000) {
        color.r = 255;
        color.g = 240;
        color.b = 200;
    } else if (colorTemp <= 6000) {
        color.r = 255;
        color.g = 255;
        color.b = 240;
    } else if (colorTemp <= 7000) {
        color.r = 200;
        color.g = 220;
        color.b = 255;
    } else if (colorTemp <= 8000) {
        color.r = 150;
        color.g = 180;
        color.b = 255;
    } else {
        color.r = 100;
        color.g = 150;
        color.b = 255;
    }
    
    return color;
}

static Ws2812bColor BrightnessToTestColor(int brightness)
{
    Ws2812bColor color = {0};
    
    if (brightness <= 0) {
        color.r = 0;
        color.g = 0;
        color.b = 0;
    } else if (brightness <= 20) {
        color.r = 255;
        color.g = 0;
        color.b = 0;
    } else if (brightness <= 40) {
        color.r = 255;
        color.g = 128;
        color.b = 0;
    } else if (brightness <= 60) {
        color.r = 255;
        color.g = 255;
        color.b = 0;
    } else if (brightness <= 80) {
        color.r = 0;
        color.g = 255;
        color.b = 0;
    } else {
        color.r = 0;
        color.g = 128;
        color.b = 255;
    }
    
    return color;
}

static void SetLedBrightness(int brightness, Ws2812bColor color)
{
    int ledCount;
    
    if (brightness <= 0) {
        ledCount = 0;
    } else if (brightness <= 10) {
        ledCount = 4;
    } else if (brightness <= 20) {
        ledCount = 8;
    } else if (brightness <= 30) {
        ledCount = 16;
    } else if (brightness <= 40) {
        ledCount = 24;
    } else if (brightness <= 50) {
        ledCount = 32;
    } else if (brightness <= 60) {
        ledCount = 40;
    } else if (brightness <= 70) {
        ledCount = 48;
    } else if (brightness <= 80) {
        ledCount = 56;
    } else if (brightness <= 90) {
        ledCount = 60;
    } else {
        ledCount = 64;
    }
    
    printf("[LED控制] 补光强度=%d%%, 点亮LED=%d/%d, 颜色=RGB(%d,%d,%d)\n", 
           brightness, ledCount, WS2812B_LED_COUNT, color.r, color.g, color.b);
    
    YuanyiWs2812bClear();
    
    for (int i = 0; i < ledCount; i++) {
        YuanyiWs2812bSetLedColor(i, color);
    }
    
    YuanyiWs2812bRefresh();
}

static void Ws2812bTcpClientTask(void *arg)
{
    (void)arg;
    int sock = 0;
    struct sockaddr_in serv_addr;
    char buffer[BUFFER_SIZE] = {0};
    
    printf("等待 2 秒后开始...\n");
    osDelay(200);
    
    printf("连接 WiFi...\n");
    if (WifiConnect(WIFI_SSID, WIFI_PASSWORD) != 0) {
        printf("WiFi 连接失败\n");
        return;
    }
    
    printf("等待网络栈初始化...\n");
    osDelay(500);
    
    printf("初始化 WS2812B LED...\n");
    if (YuanyiWs2812bInit() != 0) {
        printf("WS2812B 初始化失败\n");
        return;
    }
    
    YuanyiWs2812bClear();
    
    while (1) {
        printf("创建 TCP 客户端套接字...\n");
        if ((sock = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
            printf("套接字创建失败\n");
            osDelay(100);
            continue;
        }
        
        serv_addr.sin_family = AF_INET;
        serv_addr.sin_port = htons(TCP_SERVER_PORT);
        
        if (inet_pton(AF_INET, TCP_SERVER_IP, &serv_addr.sin_addr) <= 0) {
            printf("无效的服务器地址\n");
            lwip_close(sock);
            osDelay(100);
            continue;
        }
        
        printf("连接到服务器 %s:%d...\n", TCP_SERVER_IP, TCP_SERVER_PORT);
        if (connect(sock, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0) {
            printf("连接服务器失败\n");
            lwip_close(sock);
            osDelay(100);
            continue;
        }
        
        printf("已连接到服务器\n");
        
        while (1) {
            memset(buffer, 0, BUFFER_SIZE);
            ssize_t valread = recv(sock, buffer, BUFFER_SIZE - 1, 0);
            
            if (valread <= 0) {
                printf("接收数据失败或连接断开\n");
                break;
            }
            
            buffer[valread] = '\0';
            printf("接收到数据: %s", buffer);
            
            unsigned int colorTemp = 0;
            float lux = 0.0f;
            int brightness = 0;
            unsigned char r = 0, g = 0, b = 0;
            
            ParseSensorData(buffer, &colorTemp, &lux, &brightness, &r, &g, &b);
            
            printf("[解析结果] 色温CT=%dK, 光照LUX=%.1f, 补光强度BRIGHT=%d%%, RGB=(%d,%d,%d)\n", 
                   colorTemp, lux, brightness, r, g, b);
            
            Ws2812bColor ledColor;
            if (r > 0 || g > 0 || b > 0) {
                ledColor.r = r;
                ledColor.g = g;
                ledColor.b = b;
            } else {
                ledColor = ColorTempToRgb(colorTemp);
            }
            
            Ws2812bColor testColor = BrightnessToTestColor(brightness);
            printf("[测试颜色] 补光强度%d%% -> 测试颜色RGB(%d,%d,%d), 补光颜色RGB(%d,%d,%d)\n", 
                   brightness, testColor.r, testColor.g, testColor.b, ledColor.r, ledColor.g, ledColor.b);
            
            SetLedBrightness(brightness, testColor);
        }
        
        lwip_close(sock);
        printf("连接已关闭，5 秒后重试...\n");
        YuanyiWs2812bClear();
        osDelay(500);
    }
}

static void Ws2812bTcpClientEntry(void)
{
    osThreadAttr_t attr;
    
    attr.name = "Ws2812bTcpTask";
    attr.attr_bits = 0U;
    attr.cb_mem = NULL;
    attr.cb_size = 0U;
    attr.stack_mem = NULL;
    attr.stack_size = 16384;
    attr.priority = osPriorityNormal;
    
    if (osThreadNew(Ws2812bTcpClientTask, NULL, &attr) == NULL) {
        printf("创建 Ws2812bTcpTask 失败\n");
    }
}

APP_FEATURE_INIT(Ws2812bTcpClientEntry);
