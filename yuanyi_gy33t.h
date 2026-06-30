#ifndef __YUANYI_GY33T_H__
#define __YUANYI_GY33T_H__

#include "ohos_init.h"
#include "cmsis_os2.h"
#include "iot_uart.h"
#include "iot_gpio_ex.h"
#include "iot_gpio.h"

// GY-33T UART配置（使用GPIO11/UART2_TXD和GPIO12/UART2_RXD）
#define GY33T_UART_IDX            2       // UART2
#define GY33T_UART_BAUDRATE       9600    // 默认波特率
#define GY33T_DATA_BITS           8       // 数据位
#define GY33T_STOP_BITS           1       // 停止位
#define GY33T_PARITY              0       // 无校验

// GPIO引脚定义
#define GY33T_UART0_TX            11      // GPIO11 - UART2_TXD
#define GY33T_UART0_RX            12      // GPIO12 - UART2_RXD

// GPIO功能定义
#define GY33T_GPIO3_FUNC          IOT_GPIO_FUNC_GPIO_11_UART2_TXD  // GPIO11功能定义 - UART2_TXD
#define GY33T_GPIO4_FUNC          IOT_GPIO_FUNC_GPIO_12_UART2_RXD  // GPIO12功能定义 - UART2_RXD

// 颜色状态定义
typedef enum {
    GY33T_COLOR_WHITE = 0x01,
    GY33T_COLOR_RED   = 0x02,
    GY33T_COLOR_GREEN = 0x03,
    GY33T_COLOR_BLUE  = 0x04,
    GY33T_COLOR_BLACK = 0x05,
    GY33T_COLOR_YELLOW = 0x06,
    GY33T_COLOR_PURPLE = 0x07,
    GY33T_COLOR_CYAN  = 0x08
} Gy33tColorState;

/**
 * @brief 初始化GY-33T颜色传感器
 * @return 成功返回IOT_SUCCESS，失败返回错误码
 */
unsigned int YuanyiGy33tInit(void);

/**
 * @brief 执行白平衡校准
 * @return 成功返回IOT_SUCCESS，失败返回错误码
 */
unsigned int YuanyiGy33tWhiteBalanceCalibration(void);

/**
 * @brief 读取RGB颜色值
 * @param r 指向存储红色值的指针(0-255)
 * @param g 指向存储绿色值的指针(0-255)
 * @param b 指向存储蓝色值的指针(0-255)
 * @return 成功返回IOT_SUCCESS，失败返回错误码
 */
unsigned int YuanyiGy33tReadRgb(unsigned char *r, unsigned char *g, unsigned char *b);

/**
 * @brief 读取颜色识别状态
 * @param state 指向存储颜色状态的指针
 * @return 成功返回IOT_SUCCESS，失败返回错误码
 */
unsigned int YuanyiGy33tReadColorState(Gy33tColorState *state);

/**
 * @brief 读取色温和光照数据
 * @param colorTemp 指向存储色温值的指针(单位: K)
 * @param lux 指向存储光照值的指针(单位: lux)
 * @param r 指向存储红色值的指针(0-255)
 * @param g 指向存储绿色值的指针(0-255)
 * @param b 指向存储蓝色值的指针(0-255)
 * @return 成功返回IOT_SUCCESS，失败返回错误码
 */
unsigned int YuanyiGy33tReadData(unsigned int *colorTemp, float *lux, unsigned char *r, unsigned char *g, unsigned char *b);

#endif // __YUANYI_GY33T_H__