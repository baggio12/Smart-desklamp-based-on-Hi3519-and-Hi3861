#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <sys/time.h>

#include "ohos_init.h"
#include "cmsis_os2.h"
#include "wifi_connect.h"
#include "yuanyi_gy33t.h"
#include "yuanyi_bh1750fvi.h"
#include "iot_main.h"
#include "iot_profile.h"

#include "lwip/sockets.h"
#include "lwip/netif.h"
#include "lwip/netifapi.h"

#define WIFI_SSID "964"
#define WIFI_PASSWORD "xxxykjfg"

#define TCP_SERVER_PORT 8888
#define BUFFER_SIZE 128
#define SENSOR_READ_INTERVAL_MS 1000

#define LUX_MIN 0
#define LUX_MAX 1000
#define BRIGHT_MIN 0
#define BRIGHT_MAX 100

static bool g_mqttInitialized = false;

static int CalculateBrightness(float luminance, unsigned int colorTemp)
{
    int brightness;
    
    if (luminance < LUX_MIN) {
        luminance = LUX_MIN;
    }
    if (luminance > LUX_MAX) {
        luminance = LUX_MAX;
    }
    
    float luxFactor = 1.0f - (luminance / (float)LUX_MAX);
    
    float tempFactor = 1.0f;
    if (colorTemp < 3000) {
        tempFactor = 1.0f;
    } else if (colorTemp < 4000) {
        tempFactor = 0.9f;
    } else if (colorTemp < 5000) {
        tempFactor = 0.8f;
    } else if (colorTemp < 6000) {
        tempFactor = 0.7f;
    } else {
        tempFactor = 0.6f;
    }
    
    brightness = (int)(luxFactor * tempFactor * BRIGHT_MAX);
    
    if (brightness < BRIGHT_MIN) {
        brightness = BRIGHT_MIN;
    }
    if (brightness > BRIGHT_MAX) {
        brightness = BRIGHT_MAX;
    }
    
    return brightness;
}

static void ReportSensorData(float luminance, unsigned int colorTemp, int brightness)
{
    if (!g_mqttInitialized) {
        return;
    }
    
    IoTProfileService service;
    IoTProfileKV kvLuminance;
    IoTProfileKV kvCTemp;
    IoTProfileKV kvBright;

    memset_s(&kvBright, sizeof(kvBright), 0, sizeof(kvBright));
    kvBright.type = EN_IOT_DATATYPE_STRING;
    kvBright.key = "Bright";
    char brightStr[16];
    snprintf(brightStr, sizeof(brightStr), "%d", brightness);
    printf("Bright:%d\r\n", brightness);
    kvBright.value = brightStr;

    memset_s(&kvCTemp, sizeof(kvCTemp), 0, sizeof(kvCTemp));
    kvCTemp.type = EN_IOT_DATATYPE_STRING;
    kvCTemp.key = "CTemp";
    char ctempStr[16];
    snprintf(ctempStr, sizeof(ctempStr), "%u", colorTemp);
    printf("CTemp:%u\r\n", colorTemp);
    kvCTemp.value = ctempStr;
    kvCTemp.nxt = &kvBright;

    memset_s(&kvLuminance, sizeof(kvLuminance), 0, sizeof(kvLuminance));
    kvLuminance.type = EN_IOT_DATATYPE_STRING;
    kvLuminance.key = "Luminance";
    char luxStr[16];
    snprintf(luxStr, sizeof(luxStr), "%.2f", luminance);
    printf("Luminance:%.2f\r\n", luminance);
    kvLuminance.value = luxStr;
    kvLuminance.nxt = &kvCTemp;

    memset_s(&service, sizeof(service), 0, sizeof(service));
    service.serviceID = "Agriculture";
    service.serviceProperty = &kvLuminance;
    IoTProfilePropertyReport(CONFIG_DEVICE_ID, &service);
}

static void ReadAndPrintSensorData(int gy33tInitOk, int bh1750InitOk,
                                    unsigned int *lastColorTemp, float *lastBh1750Lux,
                                    unsigned char *lastR, unsigned char *lastG, unsigned char *lastB,
                                    int *lastBrightness)
{
    unsigned int colorTemp = 0;
    float gy33tLux = 0.0f;
    unsigned char r = 0, g = 0, b = 0;
    float bh1750Lux = 0.0f;
    
    int gy33tOk = 0;
    int bh1750Ok = 0;
    
    if (gy33tInitOk) {
        gy33tOk = (YuanyiGy33tReadData(&colorTemp, &gy33tLux, &r, &g, &b) == 0);
    }
    
    if (bh1750InitOk) {
        bh1750Ok = (YuanyiBh1750ReadLight(&bh1750Lux) == 0);
        if (bh1750Ok) {
            printf("[BH1750] 光照强度读取成功: %.1f lux (来自BH1750FVI传感器)\n", bh1750Lux);
        } else {
            printf("[BH1750] 光照强度读取失败\n");
        }
    }
    
    if ((!gy33tInitOk || gy33tOk) && (!bh1750InitOk || bh1750Ok)) {
        if (gy33tOk) {
            *lastColorTemp = colorTemp;
            *lastR = r;
            *lastG = g;
            *lastB = b;
        }
        if (bh1750Ok) {
            *lastBh1750Lux = bh1750Lux;
        }
        
        *lastBrightness = CalculateBrightness(bh1750Lux, colorTemp);
        
        printf("[传感器数据] GY-33T: 色温CT=%dK, RGB=(%d,%d,%d) | BH1750: 光照强度=%.1f lux | 补光强度Bright=%d%%\n", 
               colorTemp, r, g, b, bh1750Lux, *lastBrightness);
        
        ReportSensorData(bh1750Lux, colorTemp, *lastBrightness);
    } else {
        printf("[传感器数据] 读取失败 (GY33T:%d, BH1750:%d)\n", gy33tOk, bh1750Ok);
    }
}

static void HandleClient(int new_socket, int gy33tInitOk, int bh1750InitOk)
{
    char buffer[BUFFER_SIZE] = {0};
    unsigned int lastColorTemp = 0;
    float lastBh1750Lux = 0.0f;
    unsigned char lastR = 0, lastG = 0, lastB = 0;
    int lastBrightness = 0;
    fd_set read_fds;
    struct timeval tv;
    
    printf("客户端已连接，开始发送传感器数据...\n");
    
    while (1) {
        FD_ZERO(&read_fds);
        FD_SET(new_socket, &read_fds);
        
        tv.tv_sec = 0;
        tv.tv_usec = SENSOR_READ_INTERVAL_MS * 1000;
        
        int sel_ret = select(new_socket + 1, &read_fds, NULL, NULL, &tv);
        
        if (sel_ret < 0) {
            printf("select 错误，断开连接\n");
            break;
        }
        
        if (sel_ret == 0) {
            unsigned int colorTemp = 0;
            float gy33tLux = 0.0f;
            unsigned char r = 0, g = 0, b = 0;
            float bh1750Lux = 0.0f;
            
            int gy33tOk = 0;
            int bh1750Ok = 0;
            
            if (gy33tInitOk) {
                gy33tOk = (YuanyiGy33tReadData(&colorTemp, &gy33tLux, &r, &g, &b) == 0);
            }
            if (bh1750InitOk) {
                bh1750Ok = (YuanyiBh1750ReadLight(&bh1750Lux) == 0);
                if (bh1750Ok) {
                    printf("[BH1750] 光照强度读取成功: %.1f lux (来自BH1750FVI传感器)\n", bh1750Lux);
                } else {
                    printf("[BH1750] 光照强度读取失败\n");
                }
            }
            
            if ((!gy33tInitOk || gy33tOk) && (!bh1750InitOk || bh1750Ok)) {
                if (gy33tOk) {
                    lastColorTemp = colorTemp;
                    lastR = r;
                    lastG = g;
                    lastB = b;
                }
                if (bh1750Ok) {
                    lastBh1750Lux = bh1750Lux;
                }
                
                lastBrightness = CalculateBrightness(bh1750Lux, colorTemp);
                
                snprintf(buffer, BUFFER_SIZE, "CT:%u,LUX:%.1f,BRIGHT:%d,R:%d,G:%d,B:%d\n", 
                         colorTemp, bh1750Lux, lastBrightness, r, g, b);
                printf("发送数据: %s", buffer);
                
                ssize_t sent = send(new_socket, buffer, strlen(buffer), 0);
                if (sent < 0) {
                    printf("发送失败，断开连接\n");
                    break;
                }
                
                ReportSensorData(bh1750Lux, colorTemp, lastBrightness);
            } else {
                printf("读取传感器数据失败\n");
            }
            continue;
        }
        
        if (FD_ISSET(new_socket, &read_fds)) {
            char recv_buf[64];
            ssize_t recv_len = recv(new_socket, recv_buf, sizeof(recv_buf) - 1, 0);
            if (recv_len <= 0) {
                printf("客户端断开连接\n");
                break;
            }
            recv_buf[recv_len] = '\0';
            printf("收到客户端数据: %s\n", recv_buf);
        }
    }
    
    lwip_close(new_socket);
}

static void Gy33tTcpServerTask(void *arg)
{
    (void)arg;
    int server_fd, new_socket;
    struct sockaddr_in address;
    int opt = 1;
    int addrlen = sizeof(address);
    
    printf("等待 2 秒后开始...\n");
    osDelay(200);
    
    printf("初始化 GY-33T 传感器...\n");
    int gy33tInitOk = (YuanyiGy33tInit() == 0);
    if (!gy33tInitOk) {
        printf("GY-33T 初始化失败，继续运行...\n");
    }
    
    printf("初始化 BH1750FVI 光照传感器...\n");
    int bh1750InitOk = (YuanyiBh1750Init() == 0);
    if (!bh1750InitOk) {
        printf("BH1750FVI 初始化失败，继续运行...\n");
    }
    
    printf("尝试连接 WiFi...\n");
    int wifiConnected = (WifiConnect(WIFI_SSID, WIFI_PASSWORD) == 0);
    if (!wifiConnected) {
        printf("WiFi 连接失败，继续运行（无 TCP 服务器）...\n");
    } else {
        printf("WiFi 连接成功\n");
        printf("等待网络栈初始化...\n");
        osDelay(500);
        
        printf("正在初始化 MQTT...\n");
        IoTMain();
        printf("MQTT 初始化成功\n");
        g_mqttInitialized = true;
    }
    
    int serverCreated = 0;
    if (wifiConnected) {
        printf("等待 1 秒后创建 TCP 服务器...\n");
        osDelay(100);
        
        printf("创建 TCP 服务器...\n");
        if ((server_fd = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
            printf("socket 创建失败，错误码: %d\n", server_fd);
        } else {
            serverCreated = 1;
        }
    }
    
    if (serverCreated) {
        if (setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt))) {
            printf("setsockopt 失败\n");
            lwip_close(server_fd);
            serverCreated = 0;
        } else {
            address.sin_family = AF_INET;
            address.sin_addr.s_addr = INADDR_ANY;
            address.sin_port = htons(TCP_SERVER_PORT);
            
            if (bind(server_fd, (struct sockaddr *)&address, sizeof(address)) < 0) {
                printf("bind 失败\n");
                lwip_close(server_fd);
                serverCreated = 0;
            } else {
                if (listen(server_fd, 3) < 0) {
                    printf("listen 失败\n");
                    lwip_close(server_fd);
                    serverCreated = 0;
                } else {
                    printf("TCP 服务器已启动，端口: %d\n", TCP_SERVER_PORT);
                }
            }
        }
    }
    
    unsigned int lastColorTemp = 0;
    float lastBh1750Lux = 0.0f;
    unsigned char lastR = 0, lastG = 0, lastB = 0;
    int lastBrightness = 0;
    
    while (1) {
        if (serverCreated) {
            printf("等待客户端连接（同时读取传感器数据）...\n");
            
            fd_set read_fds;
            struct timeval tv;
            
            FD_ZERO(&read_fds);
            FD_SET(server_fd, &read_fds);
            
            tv.tv_sec = 0;
            tv.tv_usec = SENSOR_READ_INTERVAL_MS * 1000;
            
            int sel_ret = select(server_fd + 1, &read_fds, NULL, NULL, &tv);
            
            if (sel_ret < 0) {
                printf("select 错误\n");
                osDelay(100);
                continue;
            }
            
            if (sel_ret == 0) {
                ReadAndPrintSensorData(gy33tInitOk, bh1750InitOk,
                                       &lastColorTemp, &lastBh1750Lux,
                                       &lastR, &lastG, &lastB, &lastBrightness);
                continue;
            }
            
            if (FD_ISSET(server_fd, &read_fds)) {
                new_socket = accept(server_fd, (struct sockaddr *)&address, (socklen_t*)&addrlen);
                
                if (new_socket >= 0) {
                    printf("客户端已连接\n");
                    HandleClient(new_socket, gy33tInitOk, bh1750InitOk);
                    printf("客户端已断开，继续等待新连接...\n");
                } else {
                    printf("accept 失败\n");
                }
            }
        } else {
            ReadAndPrintSensorData(gy33tInitOk, bh1750InitOk,
                                   &lastColorTemp, &lastBh1750Lux,
                                   &lastR, &lastG, &lastB, &lastBrightness);
            osDelay(SENSOR_READ_INTERVAL_MS / 10);
        }
    }
    
    if (serverCreated) {
        lwip_close(server_fd);
    }
}

static void Gy33tTcpServerEntry(void)
{
    osThreadAttr_t attr;
    
    attr.name = "Gy33tTcpTask";
    attr.attr_bits = 0U;
    attr.cb_mem = NULL;
    attr.cb_size = 0U;
    attr.stack_mem = NULL;
    attr.stack_size = 16384;
    attr.priority = osPriorityNormal;
    
    if (osThreadNew(Gy33tTcpServerTask, NULL, &attr) == NULL) {
        printf("创建 Gy33tTcpTask 失败\n");
    }
}

APP_FEATURE_INIT(Gy33tTcpServerEntry);
