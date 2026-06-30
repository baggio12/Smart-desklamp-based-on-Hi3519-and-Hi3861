#include "yuanyi_gy33t.h"
#include "iot_gpio_ex.h"
#include "iot_gpio.h"
#include "iot_errno.h"
#include "iot_uart.h"
#include <stdio.h>
#include <string.h>
#include <unistd.h>

// 标记传感器是否初始化
static bool g_gy33tInitialized = false;

/**
 * @brief 配置UART2引脚复用功能
 * @return 成功返回IOT_SUCCESS，失败返回错误码
 */
static unsigned int Gy33tConfigGpio(void)
{
    unsigned int ret;
    
    IoTGpioInit(GY33T_UART0_TX);
    IoTGpioInit(GY33T_UART0_RX);

    // 配置GPIO11为UART2_TXD
    ret = IoTGpioSetFunc(GY33T_UART0_TX, GY33T_GPIO3_FUNC);
    if (ret != IOT_SUCCESS) {
        printf("配置GPIO%d为UART2_TXD失败，错误码: %u\n", GY33T_UART0_TX, ret);
        return ret;
    }
    
    // 配置GPIO12为UART2_RXD
    ret = IoTGpioSetFunc(GY33T_UART0_RX, GY33T_GPIO4_FUNC);
    if (ret != IOT_SUCCESS) {
        printf("配置GPIO%d为UART2_RXD失败，错误码: %u\n", GY33T_UART0_RX, ret);
        return ret;
    }
    
    return IOT_SUCCESS;
}

/**
 * @brief 计算校验和（根据GY-33T通信协议）
 * @param data 数据缓冲区
 * @param length 数据长度
 * @return 计算得到的校验和
 */
static unsigned char Gy33tCalculateChecksum(unsigned char *data, unsigned int length)
{
    unsigned char checksum = 0;
    for (unsigned int i = 0; i < length; i++) {
        checksum += data[i];
    }
    return checksum & 0xFF;
}

/**
 * @brief 发送命令并接收响应
 * @param cmd 命令缓冲区
 * @param cmdLen 命令长度
 * @param resp 响应缓冲区
 * @param respLen 响应长度
 * @param timeout 超时时间(ms)
 * @return 成功返回IOT_SUCCESS，失败返回错误码
 */
static unsigned int Gy33tSendCommand(unsigned char *cmd, unsigned int cmdLen, 
                                unsigned char *resp, unsigned int respLen, unsigned int timeout)
{
    int ret;
    unsigned int bytesRead = 0;
    unsigned int startTime = osKernelGetTickCount();
    unsigned char dummy;
    
    // 清空输入缓冲区，避免旧数据干扰
    while (IoTUartRead(GY33T_UART_IDX, &dummy, 1) > 0) {
        // 持续读取直到缓冲区为空
    }
    
    // 发送命令
    ret = IoTUartWrite(GY33T_UART_IDX, cmd, cmdLen);
    if (ret < 0) {
        printf("GY-33T发送命令失败，错误码: %d\n", ret);
        return IOT_FAILURE;
    } else if ((unsigned int)ret != cmdLen) {
        printf("GY-33T发送命令不完整，发送: %u 字节，期望: %u 字节\n", 
               (unsigned int)ret, cmdLen);
        return IOT_FAILURE;
    }
    
    // 先找协议头 0xA4
    unsigned char headerFound = 0;
    unsigned char tempByte;
    while ((osKernelGetTickCount() - startTime) < timeout) {
        ret = IoTUartRead(GY33T_UART_IDX, &tempByte, 1);
        if (ret == 1) {
            if (tempByte == 0xA4) {
                headerFound = 1;
                resp[0] = tempByte;
                bytesRead = 1;
                break;
            }
        } else {
            osDelay(1);
        }
    }
    
    if (!headerFound) {
        printf("GY-33T未找到协议头\n");
        return IOT_FAILURE;
    }
    
    // 继续读取剩余数据
    while ((osKernelGetTickCount() - startTime) < timeout && bytesRead < respLen) {
        unsigned int toRead = respLen - bytesRead;
        ret = IoTUartRead(GY33T_UART_IDX, &resp[bytesRead], toRead);
        
        if (ret < 0) {
            printf("GY-33T读取响应失败，错误码: %d\n", ret);
            return IOT_FAILURE;
        } else if (ret == 0) {
            osDelay(1);
            continue;
        }
        
        bytesRead += (unsigned int)ret;
    }
    
    if (bytesRead >= respLen) {
        return IOT_SUCCESS;
    }
    
    printf("GY-33T接收响应超时，已接收: %u 字节，期望: %u 字节\n", 
           bytesRead, respLen);
    return IOT_FAILURE;
}

unsigned int YuanyiGy33tInit(void)
{
    unsigned int ret;
    
    // 检查是否已初始化
    if (g_gy33tInitialized) {
        return IOT_SUCCESS;
    }
    
    // 配置UART引脚
    ret = Gy33tConfigGpio();
    if (ret != IOT_SUCCESS) {
        return ret;
    }
    
    // 配置UART参数
    IotUartAttribute uartAttr = {
        .baudRate = GY33T_UART_BAUDRATE,
        .dataBits = GY33T_DATA_BITS,
        .stopBits = GY33T_STOP_BITS,
        .parity = GY33T_PARITY
    };
    
    // 初始化UART
    printf("初始化UART2...\n");
    ret = IoTUartInit(GY33T_UART_IDX, &uartAttr);
    if (ret != IOT_SUCCESS) {
        printf("GY-33T UART2初始化失败，错误码: %u\n", ret);
        return ret;
    }
    
    printf("强制重新设置GPIO11和GPIO12为UART2功能...\n");
    ret = IoTGpioSetFunc(GY33T_UART0_TX, GY33T_GPIO3_FUNC);
    printf("GPIO11重设结果: %d\n", ret);
    ret = IoTGpioSetFunc(GY33T_UART0_RX, GY33T_GPIO4_FUNC);
    printf("GPIO12重设结果: %d\n", ret);
    
    // 清空输入缓冲区，避免旧数据干扰
    printf("清空UART输入缓冲区...\n");
    unsigned char dummy;
    while (IoTUartRead(GY33T_UART_IDX, &dummy, 1) > 0) {
        // 持续读取直到缓冲区为空
    }
    
    // 配置GY-33T为连续输出模式
    unsigned char cmd[5] = {0xA4, 0x06, 0x03, 0x00, 0x00};  // 地址+功能码+寄存器+数据+校验和
    cmd[4] = Gy33tCalculateChecksum(cmd, 4);
    
    unsigned char resp[5] = {0};
    ret = Gy33tSendCommand(cmd, sizeof(cmd), resp, sizeof(resp), 100);
    if (ret != IOT_SUCCESS) {
        printf("GY-33T配置连续输出模式失败\n");
    } else {
        printf("GY-33T配置连续输出模式成功\n");
    }
    
    g_gy33tInitialized = true;
    
    // 执行白平衡校准
    ret = YuanyiGy33tWhiteBalanceCalibration();
    if (ret != IOT_SUCCESS) {
        printf("GY-33T白平衡校准失败\n");
        // 校准失败不影响初始化，继续执行
    }
    
    printf("GY-33T初始化成功\n");
    return IOT_SUCCESS;
}

unsigned int YuanyiGy33tWhiteBalanceCalibration(void)
{
    // 在校准期间，不检查初始化标志，因为这是在初始化过程中调用的
    
    // 发送白平衡校准命令
    unsigned char cmd[5] = {0xA4, 0x06, 0x06, 0x01, 0x00};  // 地址+功能码+寄存器+数据+校验和
    cmd[4] = Gy33tCalculateChecksum(cmd, 4);
    
    unsigned char resp[5] = {0};
    unsigned int ret = Gy33tSendCommand(cmd, sizeof(cmd), resp, sizeof(resp), 100);
    if (ret != IOT_SUCCESS) {
        printf("GY-33T发送白平衡校准命令失败\n");
        return ret;
    }
    
    // 等待校准完成（约100ms）
    osDelay(100);
    
    printf("GY-33T白平衡校准完成\n");
    return IOT_SUCCESS;
}

unsigned int YuanyiGy33tReadRgb(unsigned char *r, unsigned char *g, unsigned char *b)
{
    if (!g_gy33tInitialized) {
        printf("GY-33T未初始化\n");
        return IOT_FAILURE;
    }
    
    // 发送读取RGB数据命令
    unsigned char cmd[5] = {0xA4, 0x03, 0x10, 0x03, 0x00};  // 地址+功能码+起始寄存器+数量+校验和
    cmd[4] = Gy33tCalculateChecksum(cmd, 4);
    
    unsigned char resp[8] = {0};  // 响应格式: 地址+功能码+字节数+数据(3字节)+校验和
    unsigned int ret = Gy33tSendCommand(cmd, sizeof(cmd), resp, sizeof(resp), 100);
    if (ret != IOT_SUCCESS) {
        printf("GY-33T读取RGB数据失败\n");
        return ret;
    }
    
    // 校验响应
    if (resp[0] != 0xA4 || resp[1] != 0x03 || resp[2] != 0x03) {
        printf("GY-33T读取RGB数据响应格式错误\n");
        return IOT_FAILURE;
    }
    
    // 校验和检查
    unsigned char checksum = Gy33tCalculateChecksum(resp, 7);
    if (checksum != resp[7]) {
        printf("GY-33T读取RGB数据校验和错误\n");
        return IOT_FAILURE;
    }
    
    // 提取RGB数据
    *r = resp[3];
    *g = resp[4];
    *b = resp[5];
    
    return IOT_SUCCESS;
}

unsigned int YuanyiGy33tReadColorState(Gy33tColorState *state)
{
    if (!g_gy33tInitialized) {
        printf("GY-33T未初始化\n");
        return IOT_FAILURE;
    }
    
    // 发送读取颜色状态命令
    unsigned char cmd[5] = {0xA4, 0x03, 0x15, 0x01, 0x00};  // 地址+功能码+起始寄存器+数量+校验和
    cmd[4] = Gy33tCalculateChecksum(cmd, 4);
    
    unsigned char resp[6] = {0};  // 响应格式: 地址+功能码+字节数+数据(1字节)+校验和
    unsigned int ret = Gy33tSendCommand(cmd, sizeof(cmd), resp, sizeof(resp), 100);
    if (ret != IOT_SUCCESS) {
        printf("GY-33T读取颜色状态失败\n");
        return ret;
    }
    
    // 校验响应
    if (resp[0] != 0xA4 || resp[1] != 0x03 || resp[2] != 0x01) {
        printf("GY-33T读取颜色状态响应格式错误\n");
        return IOT_FAILURE;
    }
    
    // 校验和检查
    unsigned char checksum = Gy33tCalculateChecksum(resp, 5);
    if (checksum != resp[5]) {
        printf("GY-33T读取颜色状态校验和错误\n");
        return IOT_FAILURE;
    }
    
    // 提取颜色状态
    *state = (Gy33tColorState)resp[3];
    
    return IOT_SUCCESS;
}

unsigned int YuanyiGy33tReadData(unsigned int *colorTemp, float *lux, unsigned char *r, unsigned char *g, unsigned char *b)
{
    if (!g_gy33tInitialized) {
        printf("GY-33T未初始化\n");
        return IOT_FAILURE;
    }
    
    // 连续输出模式：等待并读取完整的10字节数据包
    unsigned char packet[10] = {0};
    unsigned int bytesRead = 0;
    unsigned int startTime = osKernelGetTickCount();
    unsigned int timeout = 300; // 300ms 超时
    
    // 先找协议头 0xA4
    unsigned char headerFound = 0;
    unsigned char tempByte;
    while ((osKernelGetTickCount() - startTime) < timeout && !headerFound) {
        int ret = IoTUartRead(GY33T_UART_IDX, &tempByte, 1);
        if (ret == 1) {
            if (tempByte == 0xA4) {
                headerFound = 1;
                packet[0] = tempByte;
                bytesRead = 1;
            }
        } else {
            osDelay(1);
        }
    }
    
    if (!headerFound) {
        printf("GY-33T未找到协议头\n");
        return IOT_FAILURE;
    }
    
    // 读取剩余9字节
    while ((osKernelGetTickCount() - startTime) < timeout && bytesRead < 10) {
        int ret = IoTUartRead(GY33T_UART_IDX, &packet[bytesRead], 10 - bytesRead);
        if (ret > 0) {
            bytesRead += (unsigned int)ret;
        } else {
            osDelay(1);
        }
    }
    
    if (bytesRead < 10) {
        printf("GY-33T读取数据包不完整，已读取: %u 字节\n", bytesRead);
        return IOT_FAILURE;
    }
    
    // 打印数据包
    printf("GY-33T数据包: ");
    for (int i = 0; i < 10; i++) {
        printf("%02x ", packet[i]);
    }
    printf("\n");
    
    // 校验协议头
    if (packet[0] != 0xA4) {
        printf("GY-33T数据包协议头错误\n");
        return IOT_FAILURE;
    }
    
    // 校验和检查（前9字节的校验和应该等于第10字节）
    unsigned char checksum = Gy33tCalculateChecksum(packet, 9);
    if (checksum != packet[9]) {
        printf("GY-33T数据包校验和错误，计算: %02x，接收: %02x\n", checksum, packet[9]);
        return IOT_FAILURE;
    }
    
    // 根据连续输出模式解析数据
    // 数据包格式: A4 03 10 05 00 0b 0f 05 6f 4a (示例)
    // 根据原始代码解析：
    // 字节0: 0xA4 (包头)
    // 字节1: 0x03 (功能码)
    // 字节2: 0x10 (数据长度)
    // 字节3: R值
    // 字节4: G值
    // 字节5: B值
    // 字节6-7: 色温 (2字节)
    // 字节8: 颜色状态
    // 字节9: 校验和
    
    // 提取RGB值
    *r = packet[3];
    *g = packet[4];
    *b = packet[5];
    
    // 提取色温（字节6-7）
    *colorTemp = (packet[6] << 8) | packet[7];
    
    // 光照暂时不使用，设置为0
    *lux = 0.0f;
    
    printf("解析结果: R=%u, G=%u, B=%u, CT=%u, LUX=%.1f\n", *r, *g, *b, *colorTemp, *lux);
    
    return IOT_SUCCESS;
}