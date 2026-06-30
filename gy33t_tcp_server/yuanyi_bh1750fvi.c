#include "yuanyi_bh1750fvi.h"
#include "ohos_init.h"
#include "cmsis_os2.h"
#include "iot_i2c.h"
#include "iot_gpio_ex.h"
#include "iot_gpio.h"
#include "iot_errno.h"
#include <stdio.h>

#define BH1750FVI_DEFAULT_ADDR    BH1750FVI_ADDR_L
#define BH1750FVI_I2C_ID          1
#define BH1750FVI_I2C_FREQ        400000
#define BH1750FVI_SCL_PIN         1
#define BH1750FVI_SDA_PIN         0

static bool g_bh1750Initialized = false;
static unsigned short g_bh1750Addr = BH1750FVI_DEFAULT_ADDR;

static unsigned int Bh1750SendCommand(unsigned int cmd)
{
    unsigned int ret;
    unsigned char data[1] = {cmd};
    
    ret = IoTI2cWrite(BH1750FVI_I2C_ID, g_bh1750Addr << 1, data, 1);
    if (ret != IOT_SUCCESS) {
        printf("BH1750FVI发送命令失败，错误码: %d\n", ret);
        return ret;
    }
    
    return IOT_SUCCESS;
}

static unsigned int Bh1750ReadData(unsigned char *data, unsigned int len)
{
    unsigned int ret;
    
    ret = IoTI2cRead(BH1750FVI_I2C_ID, g_bh1750Addr << 1, data, len);
    if (ret != IOT_SUCCESS) {
        printf("BH1750FVI读取数据失败，错误码: %d\n", ret);
        return ret;
    }
    
    return IOT_SUCCESS;
}

static unsigned int Bh1750DetectAddress(void)
{
    unsigned int ret;
    unsigned char data[1] = {BH1750FVI_POWER_ON};
    
    printf("正在检测 BH1750FVI 传感器地址...\n");
    
    g_bh1750Addr = BH1750FVI_ADDR_L;
    ret = IoTI2cWrite(BH1750FVI_I2C_ID, g_bh1750Addr << 1, data, 1);
    if (ret == IOT_SUCCESS) {
        printf("检测到传感器地址: 0x%02X (ADDR接GND)\n", g_bh1750Addr);
        return IOT_SUCCESS;
    }
    
    g_bh1750Addr = BH1750FVI_ADDR_H;
    ret = IoTI2cWrite(BH1750FVI_I2C_ID, g_bh1750Addr << 1, data, 1);
    if (ret == IOT_SUCCESS) {
        printf("检测到传感器地址: 0x%02X (ADDR接3.3V)\n", g_bh1750Addr);
        return IOT_SUCCESS;
    }
    
    printf("未检测到 BH1750FVI 传感器\n");
    return ret;
}

unsigned int YuanyiBh1750Init(void)
{
    unsigned int ret;
    
    if (g_bh1750Initialized) {
        return IOT_SUCCESS;
    }
    
    printf("引脚配置（小熊派开发板）:\n");
    printf("  - SCL -> GPIO1 (I2C1_SCL)\n");
    printf("  - SDA -> GPIO0 (I2C1_SDA)\n");
    
    IoTGpioInit(BH1750FVI_SCL_PIN);
    IoTGpioInit(BH1750FVI_SDA_PIN);
    IoTGpioSetFunc(BH1750FVI_SCL_PIN, IOT_GPIO_FUNC_GPIO_1_I2C1_SCL);
    IoTGpioSetFunc(BH1750FVI_SDA_PIN, IOT_GPIO_FUNC_GPIO_0_I2C1_SDA);
    IoTGpioSetDir(BH1750FVI_SCL_PIN, IOT_GPIO_DIR_OUT);
    IoTGpioSetDir(BH1750FVI_SDA_PIN, IOT_GPIO_DIR_OUT);
    IoTGpioSetPull(BH1750FVI_SCL_PIN, IOT_GPIO_PULL_UP);
    IoTGpioSetPull(BH1750FVI_SDA_PIN, IOT_GPIO_PULL_UP);
    
    ret = IoTI2cInit(BH1750FVI_I2C_ID, BH1750FVI_I2C_FREQ);
    if (ret != IOT_SUCCESS) {
        printf("BH1750FVI I2C初始化失败，错误码: %d\n", ret);
        return ret;
    }
    
    ret = Bh1750DetectAddress();
    if (ret != IOT_SUCCESS) {
        return ret;
    }
    
    ret = Bh1750SendCommand(BH1750FVI_POWER_ON);
    if (ret != IOT_SUCCESS) {
        return ret;
    }
    
    ret = Bh1750SendCommand(BH1750FVI_RESET);
    if (ret != IOT_SUCCESS) {
        return ret;
    }
    
    ret = Bh1750SendCommand(BH1750FVI_CONT_H_RES_MODE2);
    if (ret != IOT_SUCCESS) {
        return ret;
    }
    
    osDelay(120);
    
    g_bh1750Initialized = true;
    printf("BH1750FVI初始化成功\n");
    
    return IOT_SUCCESS;
}

unsigned int YuanyiBh1750ReadLight(float *light)
{
    unsigned int ret;
    unsigned char data[2] = {0};
    unsigned int lightRaw;
    
    if (!g_bh1750Initialized) {
        ret = YuanyiBh1750Init();
        if (ret != IOT_SUCCESS) {
            return ret;
        }
    }
    
    ret = Bh1750ReadData(data, 2);
    if (ret != IOT_SUCCESS) {
        return ret;
    }
    
    lightRaw = (data[0] << 8) | data[1];
    *light = (float)lightRaw / 1.2f;
    
    return IOT_SUCCESS;
}

unsigned int YuanyiBh1750SetMode(unsigned int mode)
{
    if (!g_bh1750Initialized) {
        unsigned int ret = YuanyiBh1750Init();
        if (ret != IOT_SUCCESS) {
            return ret;
        }
    }
    
    return Bh1750SendCommand(mode);
}

unsigned int YuanyiBh1750PowerDown(void)
{
    if (!g_bh1750Initialized) {
        return IOT_SUCCESS;
    }
    
    unsigned int ret = Bh1750SendCommand(BH1750FVI_POWER_DOWN);
    if (ret == IOT_SUCCESS) {
        g_bh1750Initialized = false;
    }
    
    return ret;
}

unsigned int YuanyiBh1750PowerOn(void)
{
    unsigned int ret = Bh1750SendCommand(BH1750FVI_POWER_ON);
    if (ret == IOT_SUCCESS) {
        g_bh1750Initialized = true;
    }
    
    return ret;
}
