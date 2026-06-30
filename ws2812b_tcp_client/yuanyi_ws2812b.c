#include "yuanyi_ws2812b.h"
#include "ohos_init.h"
#include "cmsis_os2.h"
#include "iot_gpio.h"
#include "iot_gpio_ex.h"
#include "iot_errno.h"
#include "hi_spi.h"
#include <stdio.h>
#include <string.h>
#include <unistd.h>

// 引脚连接说明（Hi3861开发板）：
// WS2812B是串联的RGB LED灯珠，通过SPI接口模拟单总线与Hi3861连接
// - VCC：接5V电源（WS2812B工作电压3.3V-5V，推荐5V以保证亮度和驱动能力）
// - GND：接Hi3861的GND引脚（共地，确保电平信号一致）
// - DATA（数据引脚）：连接Hi3861的SPI MOSI引脚（使用GPIO9）
// 注：若LED数量较多（如超过8个），建议在DATA线与VCC之间接一个4.7K上拉电阻，增强信号稳定性

// WS2812B SPI配置参数（需根据实际硬件修改）
#define WS2812B_SPI_ID       HI_SPI_ID_0      // 使用SPI0
#define WS2812B_SPI_GPIO     9                // SPI0 MOSI引脚（GPIO9）
#define WS2812B_SPI_FREQ     3200000          // SPI频率3.2MHz（根据需要调整）
#define WS2812B_LED_COUNT    64               // 串联的LED数量（根据实际灯珠数量修改，当前为8*8灯板）
#define WS2812B_RESET_TIME   50               // 复位信号时间(us)

// WS2812B数据缓存：每个LED需要24位（3字节，GRB格式）
static unsigned char g_ws2812bBuffer[WS2812B_LED_COUNT * 24];
// 全局变量，记录当前LED是否已初始化
static bool g_ws2812bInitialized = false;

/**
 * @brief 将字节数据转换为WS2812B的SPI数据
 * @param data 输入字节数据
 * @param buffer 输出SPI数据缓冲区
 */
static void ByteToWs2812bSpiData(unsigned char data, unsigned char *buffer)
{
    // WS2812B的数据是高位在前
    for (int i = 0; i < 8; i++) {
        // SPI数据格式：
        // 1位数据使用2个SPI位来表示
        // 0码: 高电平220-380ns，低电平580-1us
        // 1码: 高电平580-1us，低电平220-380ns
        unsigned char bit = (data & (0x80 >> i)) ? 1 : 0;
        
        if (bit) {
            // 1码: 使用0xC0表示
            buffer[i] = 0xC0;
        } else {
            // 0码: 使用0x80表示
            buffer[i] = 0x80;
        }
    }
}

/**
 * @brief 将颜色数据转换为WS2812B的SPI数据
 * @param color 颜色值
 * @param buffer 输出SPI数据缓冲区
 */
static void ColorToWs2812bSpiData(Ws2812bColor color, unsigned char *buffer)
{
    // WS2812B数据顺序是GRB
    ByteToWs2812bSpiData(color.g, buffer);
    ByteToWs2812bSpiData(color.r, buffer + 8);
    ByteToWs2812bSpiData(color.b, buffer + 16);
}

unsigned int YuanyiWs2812bInit(void)
{
    unsigned int ret;
    
    // 检查是否已初始化
    if (g_ws2812bInitialized) {
        return IOT_SUCCESS;
    }
    
    // 初始化GPIO
    ret = IoTGpioInit(WS2812B_SPI_GPIO);
    if (ret != IOT_SUCCESS) {
        printf("WS2812B GPIO初始化失败，错误码: %d\n", ret);
        return ret;
    }
    
    // 设置GPIO功能为SPI MOSI
    ret = IoTGpioSetFunc(WS2812B_SPI_GPIO, IOT_GPIO_FUNC_GPIO_9_SPI0_TXD);
    if (ret != IOT_SUCCESS) {
        printf("WS2812B GPIO功能设置失败，错误码: %d\n", ret);
        return ret;
    }
    
    // 初始化SPI配置
    hi_spi_cfg_init_param initParam;
    memset(&initParam, 0, sizeof(initParam));
    initParam.is_slave = 0; // 主模式
    
    hi_spi_cfg_basic_info basicInfo;
    memset(&basicInfo, 0, sizeof(basicInfo));
    basicInfo.cpol = HI_SPI_CFG_CLOCK_CPOL_0;          // 时钟极性0
    basicInfo.cpha = HI_SPI_CFG_CLOCK_CPHA_0;          // 时钟相位0
    basicInfo.fram_mode = HI_SPI_CFG_FRAM_MODE_MOTOROLA; // 摩托罗拉协议
    basicInfo.data_width = HI_SPI_CFG_DATA_WIDTH_E_8BIT; // 8位数据宽度
    basicInfo.endian = HI_SPI_CFG_ENDIAN_LITTLE;       // 小端模式
    basicInfo.freq = WS2812B_SPI_FREQ;                 // SPI频率
    
    // 初始化SPI
    ret = hi_spi_init(WS2812B_SPI_ID, initParam, &basicInfo);
    if (ret != IOT_SUCCESS) {
        printf("WS2812B SPI初始化失败，错误码: %d\n", ret);
        return ret;
    }
    
    // 设置SPI回环模式（根据参考实现）
    hi_spi_set_loop_back_mode(WS2812B_SPI_ID, 1);
    
    // 清除缓冲区
    memset(g_ws2812bBuffer, 0, sizeof(g_ws2812bBuffer));
    
    g_ws2812bInitialized = true;
    printf("WS2812B SPI初始化成功\n");
    
    return IOT_SUCCESS;
}

unsigned int YuanyiWs2812bSetLedColor(unsigned int index, Ws2812bColor color)
{
    if (!g_ws2812bInitialized) {
        unsigned int ret = YuanyiWs2812bInit();
        if (ret != IOT_SUCCESS) {
            return ret;
        }
    }
    
    // 检查索引是否有效
    if (index >= WS2812B_LED_COUNT) {
        printf("WS2812B LED索引超出范围: %u\n", index);
        return IOT_FAILURE;
    }
    
    // 将颜色转换为WS2812B数据格式
    ColorToWs2812bSpiData(color, &g_ws2812bBuffer[index * 24]);
    
    return IOT_SUCCESS;
}

unsigned int YuanyiWs2812bSetAllColor(Ws2812bColor color)
{
    if (!g_ws2812bInitialized) {
        unsigned int ret = YuanyiWs2812bInit();
        if (ret != IOT_SUCCESS) {
            return ret;
        }
    }
    
    // 为所有LED设置相同的颜色
    for (unsigned int i = 0; i < WS2812B_LED_COUNT; i++) {
        ColorToWs2812bSpiData(color, &g_ws2812bBuffer[i * 24]);
    }
    
    return IOT_SUCCESS;
}

unsigned int YuanyiWs2812bRefresh(void)
{
    unsigned int ret;
    
    if (!g_ws2812bInitialized) {
        ret = YuanyiWs2812bInit();
        if (ret != IOT_SUCCESS) {
            return ret;
        }
    }
    
    // 发送复位信号（先发送全零数据作为break/reset）
    unsigned char resetData[50];
    memset(resetData, 0, sizeof(resetData));
    hi_spi_host_write(WS2812B_SPI_ID, resetData, sizeof(resetData));
    
    // 发送数据到WS2812B
    ret = hi_spi_host_write(WS2812B_SPI_ID, g_ws2812bBuffer, sizeof(g_ws2812bBuffer));
    if (ret != IOT_SUCCESS) {
        printf("WS2812B SPI数据发送失败，错误码: %d\n", ret);
        return ret;
    }
    
    // 发送完所有数据后，需要一个50us以上的低电平作为复位信号
    // 临时将SPI引脚设置为GPIO输出低电平
    IoTGpioSetFunc(WS2812B_SPI_GPIO, IOT_GPIO_FUNC_GPIO_9_GPIO);
    IoTGpioSetDir(WS2812B_SPI_GPIO, IOT_GPIO_DIR_OUT);
    IoTGpioSetOutputVal(WS2812B_SPI_GPIO, 0);
    
    // 延时确保复位信号足够长
    usleep(WS2812B_RESET_TIME);
    
    // 恢复SPI功能
    IoTGpioSetFunc(WS2812B_SPI_GPIO, IOT_GPIO_FUNC_GPIO_9_SPI0_TXD);
    
    return IOT_SUCCESS;
}

Ws2812bColor YuanyiWs2812bHsvToRgb(unsigned int h, unsigned int s, unsigned int v)
{
    Ws2812bColor color = {0};
    unsigned int region, remainder, p, q, t;
    
    if (s == 0) {
        // 饱和度为0，灰度颜色
        color.r = color.g = color.b = v;
        return color;
    }
    
    // 将HSV值归一化
    h %= 360;
    region = h / 60;
    remainder = (h - (region * 60)) * 6;
    
    // 计算RGB分量
    p = (v * (100 - s)) / 100;
    q = (v * (100 - (s * remainder) / 600)) / 100;
    t = (v * (100 - (s * (600 - remainder)) / 600)) / 100;
    
    switch (region) {
        case 0:
            color.r = v;
            color.g = t;
            color.b = p;
            break;
        case 1:
            color.r = q;
            color.g = v;
            color.b = p;
            break;
        case 2:
            color.r = p;
            color.g = v;
            color.b = t;
            break;
        case 3:
            color.r = p;
            color.g = q;
            color.b = v;
            break;
        case 4:
            color.r = t;
            color.g = p;
            color.b = v;
            break;
        default:
            color.r = v;
            color.g = p;
            color.b = q;
            break;
    }
    
    return color;
}

unsigned int YuanyiWs2812bClear(void)
{
    if (!g_ws2812bInitialized) {
        unsigned int ret = YuanyiWs2812bInit();
        if (ret != IOT_SUCCESS) {
            return ret;
        }
    }
    
    // 清空缓冲区(所有LED设为黑色)
    memset(g_ws2812bBuffer, 0, sizeof(g_ws2812bBuffer));
    
    // 刷新显示
    return YuanyiWs2812bRefresh();
}

/**
 * @brief 根据环境光值调整LED亮度
 * @param ambientLight 环境光值 (0-1000)
 * @param baseColor 基础颜色值
 * @return 调整后的颜色值
 */
Ws2812bColor YuanyiWs2812bAdjustBrightnessByAmbient(unsigned int ambientLight, Ws2812bColor baseColor)
{
    // 确保环境光值在有效范围内
    if (ambientLight < AMBIENT_LIGHT_MIN) {
        ambientLight = AMBIENT_LIGHT_MIN;
    }
    if (ambientLight > AMBIENT_LIGHT_MAX) {
        ambientLight = AMBIENT_LIGHT_MAX;
    }
    
    // 计算亮度调整系数
    // 环境光越亮，LED亮度越低；环境光越暗，LED亮度越高
    float brightnessFactor = 1.0f - ((float)(ambientLight - AMBIENT_LIGHT_MIN) / 
                                     (float)(AMBIENT_LIGHT_MAX - AMBIENT_LIGHT_MIN));
    
    // 添加最小亮度阈值，确保在最亮环境下仍有一定可见度
    brightnessFactor = brightnessFactor * 0.8f + 0.2f;
    
    // 调整颜色分量
    Ws2812bColor adjustedColor;
    adjustedColor.r = (unsigned char)((float)baseColor.r * brightnessFactor);
    adjustedColor.g = (unsigned char)((float)baseColor.g * brightnessFactor);
    adjustedColor.b = (unsigned char)((float)baseColor.b * brightnessFactor);
    
    return adjustedColor;
}

/**
 * @brief 根据色温值计算RGB颜色（使用查表法替代复杂数学函数）
 * @param kelvin 色温值（开尔文）
 * @return 对应的RGB颜色值
 */
static Ws2812bColor TemperatureToRgb(unsigned int kelvin)
{
    Ws2812bColor color;
    
    // 限制色温范围在2000K-10000K之间
    if (kelvin < 2000) kelvin = 2000;
    if (kelvin > 10000) kelvin = 10000;
    
    // 使用查表法计算RGB值，避免使用log和pow函数
    // 基于色温与RGB的映射关系，通过分段线性插值计算
    
    // 定义几个关键色温点的RGB值
    typedef struct {
        unsigned int temp;  // 色温值
        Ws2812bColor rgb;   // 对应的RGB值
    } TempColorPoint;
    
    TempColorPoint points[] = {
        {2000, {33, 10, 0}},    // 暖黄
        {3000, {64, 37, 13}},   // 暖白
        {4000, {98, 79, 60}},   // 中性白
        {5000, {138, 123, 113}}, // 偏冷白
        {6500, {179, 179, 193}}, // 冷白
        {8000, {210, 214, 235}}, // 非常冷白
        {10000, {221, 229, 255}} // 最冷白
    };
    
    unsigned int pointCount = sizeof(points) / sizeof(TempColorPoint);
    
    // 查找色温所在的区间
    for (unsigned int i = 0; i < pointCount - 1; i++) {
        if (kelvin >= points[i].temp && kelvin <= points[i + 1].temp) {
            // 计算插值系数
            float ratio = (float)(kelvin - points[i].temp) / 
                          (float)(points[i + 1].temp - points[i].temp);
            
            // 线性插值计算RGB值
            color.r = (unsigned char)(points[i].rgb.r + 
                                    ratio * (points[i + 1].rgb.r - points[i].rgb.r));
            color.g = (unsigned char)(points[i].rgb.g + 
                                    ratio * (points[i + 1].rgb.g - points[i].rgb.g));
            color.b = (unsigned char)(points[i].rgb.b + 
                                    ratio * (points[i + 1].rgb.b - points[i].rgb.b));
            
            return color;
        }
    }
    
    // 默认返回中性白（如果查找失败）
    color.r = 255;
    color.g = 255;
    color.b = 255;
    
    return color;
}

/**
 * @brief 根据环境光值和色温偏好调整LED颜色
 * @param ambientLight 环境光值 (0-1000)
 * @param temperaturePreference 色温偏好 (0-100，0为暖色温，100为冷色温)
 * @return 调整后的颜色值
 */
Ws2812bColor YuanyiWs2812bAdjustByAmbientAndTemperature(unsigned int ambientLight, unsigned int temperaturePreference)
{
    // 当环境光值大于100时，返回黑色（关灯）
    if (ambientLight > 100) {
        Ws2812bColor blackColor = {0, 0, 0};  // RGB都为0表示黑色
        return blackColor;
    }
    
    // 当环境光值太小或无效时，使用默认值
    if (ambientLight < 1 || ambientLight == 0) {
        ambientLight = 50;  // 设置默认环境光值为50
    }
    
    // 确保参数在有效范围内
    if (temperaturePreference > 100) {
        temperaturePreference = 100;
    }
    
    // 根据偏好计算目标色温
    // 使用线性插值计算介于暖色温和冷色温之间的值
    float tempFactor = (float)temperaturePreference / 100.0f;
    unsigned int targetTemp = (unsigned int)(WARM_TEMPERATURE + 
                                           (COOL_TEMPERATURE - WARM_TEMPERATURE) * tempFactor);
    
    // 根据目标色温获取基础颜色
    Ws2812bColor baseColor = TemperatureToRgb(targetTemp);
    
    // 应用环境光亮度调整
    return YuanyiWs2812bAdjustBrightnessByAmbient(ambientLight, baseColor);
}

/**
 * @brief 设置所有LED颜色，自动根据环境光调整亮度
 * @param baseColor 基础颜色值
 * @param ambientLight 环境光值 (0-1000)
 * @return 成功返回IOT_SUCCESS，失败返回错误码
 */
unsigned int YuanyiWs2812bSetAllColorWithAmbientAdjust(Ws2812bColor baseColor, unsigned int ambientLight)
{
    if (!g_ws2812bInitialized) {
        unsigned int ret = YuanyiWs2812bInit();
        if (ret != IOT_SUCCESS) {
            return ret;
        }
    }
    
    // 根据环境光调整颜色
    Ws2812bColor adjustedColor = YuanyiWs2812bAdjustBrightnessByAmbient(ambientLight, baseColor);
    
    // 设置所有LED为调整后的颜色
    return YuanyiWs2812bSetAllColor(adjustedColor);
}