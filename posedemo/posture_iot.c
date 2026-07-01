#include "posture_iot.h"

#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <math.h>
#include <stdarg.h>
#include <stdlib.h>
#include <time.h>
#include <stdint.h>
#include <errno.h>

#include <sys/types.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <arpa/inet.h>
#include <netdb.h>

#include "iota_init.h"
#include "iota_datatrans.h"
#include "string_util.h"
#include "log_util.h"
#include "iota_cfg.h"
#include "mqttv5_util.h"

/*
 * 有些版本的头文件链里没有把这些声明带进来，
 * 这里手动补一下，保证 posture_iot.c 能独立编译。
 */
extern int IOTA_Connect(void);
extern int IOTA_IsConnected(void);

/*
 * 不再依赖 IOTA_Init(".") 的当前工作目录。
 * 如果你的板端 SDK 实际不在这个目录，请只改这一行。
 */
#define IOT_SDK_BASE_DIR "/mnt/huaweicloud-iot-device-sdk-c-master"

/* 时间判断阈值：只要还小于 2026 年，就认为板子时间明显不对 */
#define MIN_REASONABLE_YEAR 2026

/* 等待 MQTT 真正连上的超时时间 */
#define IOT_CONNECT_WAIT_MS       10000
#define IOT_CONNECT_POLL_MS         300

/* 上报节流 */
#define REPORT_INTERVAL_TICKS        60   /* 约 2 秒，按 30fps 估算 */
#define REPORT_FAIL_COOLDOWN_TICKS   60   /* 上报失败后冷却约 2 秒 */

/* NTP */
#define NTP_PORT_STR "123"
#define NTP_TIMEOUT_MS 1500
#define NTP_UNIX_EPOCH_DELTA 2208988800UL

static int g_iot_inited = 0;
static int g_iot_connected = 0;

static char g_posture_status[32] = "good";
static char g_bad_type[32] = "normal";
static double g_confidence = 0.95;

/* 节流缓存 */
static char g_last_posture_status[32] = "";
static char g_last_bad_type[32] = "";
static double g_last_confidence = -1.0;

static int g_report_frame_tick = 0;
static int g_report_fail_cooldown = 0;

static void PostureIotSleepMs(int ms)
{
#if defined(WIN32) || defined(WIN64)
    Sleep(ms);
#else
    usleep(ms * 1000);
#endif
}

static int PostureIotIsEmpty(const char *s)
{
    return (s == NULL || s[0] == '\0');
}

static int PostureIotSystemTimeReasonable(void)
{
    time_t now;
    struct tm *tm_now;

    now = time(NULL);
    if (now <= 0) {
        return 0;
    }

    tm_now = localtime(&now);
    if (tm_now == NULL) {
        return 0;
    }

    if ((tm_now->tm_year + 1900) < MIN_REASONABLE_YEAR) {
        return 0;
    }

    return 1;
}

static void PostureIotPrintNow(const char *tag)
{
    time_t now;
    struct tm *tm_now;
    char buf[64];

    now = time(NULL);
    tm_now = localtime(&now);
    if (tm_now == NULL) {
        printf("%s: <invalid time>\n", tag);
        return;
    }

    if (strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S %Z", tm_now) == 0) {
        printf("%s: <strftime failed>\n", tag);
        return;
    }

    printf("%s: %s\n", tag, buf);
}

static int PostureIotSetUnixTime(time_t unix_sec)
{
    struct timeval tv;

    tv.tv_sec = unix_sec;
    tv.tv_usec = 0;

    if (settimeofday(&tv, NULL) != 0) {
        printf("[TIME] settimeofday failed, errno=%d\n", errno);
        return -1;
    }

    return 0;
}

static int PostureIotQueryNtpServer(const char *server, time_t *out_unix_sec)
{
    struct addrinfo hints;
    struct addrinfo *res = NULL;
    struct addrinfo *rp;
    int sock = -1;
    int ret;
    uint8_t pkt[48];
    uint8_t resp[48];
    struct timeval tv;
    ssize_t n;

    if (server == NULL || out_unix_sec == NULL) {
        return -1;
    }

    memset(&hints, 0, sizeof(hints));
    hints.ai_family = AF_INET;       /* 先用 IPv4，嵌入式板子更稳 */
    hints.ai_socktype = SOCK_DGRAM;

    ret = getaddrinfo(server, NTP_PORT_STR, &hints, &res);
    if (ret != 0 || res == NULL) {
        printf("[TIME] getaddrinfo failed for %s\n", server);
        return -1;
    }

    for (rp = res; rp != NULL; rp = rp->ai_next) {
        uint32_t ntp_sec;
        time_t unix_sec;

        sock = socket(rp->ai_family, rp->ai_socktype, rp->ai_protocol);
        if (sock < 0) {
            continue;
        }

        tv.tv_sec = NTP_TIMEOUT_MS / 1000;
        tv.tv_usec = (NTP_TIMEOUT_MS % 1000) * 1000;
        setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
        setsockopt(sock, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));

        memset(pkt, 0, sizeof(pkt));
        pkt[0] = 0x1B;  /* LI=0, VN=3, Mode=3(client) */

        ret = sendto(sock, pkt, sizeof(pkt), 0, rp->ai_addr, rp->ai_addrlen);
        if (ret < 0) {
            close(sock);
            sock = -1;
            continue;
        }

        n = recvfrom(sock, resp, sizeof(resp), 0, NULL, NULL);
        close(sock);
        sock = -1;

        if (n < 48) {
            continue;
        }

        ntp_sec = ((uint32_t)resp[40] << 24) |
                  ((uint32_t)resp[41] << 16) |
                  ((uint32_t)resp[42] << 8)  |
                  ((uint32_t)resp[43]);

        if (ntp_sec <= NTP_UNIX_EPOCH_DELTA) {
            continue;
        }

        unix_sec = (time_t)(ntp_sec - NTP_UNIX_EPOCH_DELTA);
        *out_unix_sec = unix_sec;

        freeaddrinfo(res);
        return 0;
    }

    freeaddrinfo(res);
    return -1;
}

static int PostureIotSyncTimeOnce(void)
{
    static const char *servers[] = {
        "ntp1.aliyun.com",
        "ntp.tencent.com",
        "cn.pool.ntp.org"
    };
    time_t unix_sec = 0;
    size_t i;

    PostureIotPrintNow("[TIME] before sync");

    for (i = 0; i < sizeof(servers) / sizeof(servers[0]); ++i) {
        printf("[TIME] trying NTP server: %s\n", servers[i]);

        if (PostureIotQueryNtpServer(servers[i], &unix_sec) == 0) {
            if (PostureIotSetUnixTime(unix_sec) == 0) {
                PostureIotPrintNow("[TIME] after sync");
                printf("[TIME] sync success via %s\n", servers[i]);

                /* 如果板子支持 RTC，就顺手写回；不支持也没关系 */
                system("hwclock -w >/dev/null 2>&1");

                return 0;
            } else {
                printf("[TIME] got NTP time but failed to set system clock\n");
                return -1;
            }
        }
    }

    PostureIotPrintNow("[TIME] after sync");
    printf("[TIME] sync failed or system time still unreasonable\n");
    return -1;
}

/*
 * 只保留错误相关 SDK 日志，避免刷屏。
 */
static void PostureIotLog(int level, char *format, va_list args)
{
    (void)level;

    if (format == NULL) {
        return;
    }

    if (strstr(format, "ERROR") == NULL &&
        strstr(format, "error") == NULL &&
        strstr(format, "failed") == NULL &&
        strstr(format, "FATAL") == NULL) {
        return;
    }

    vprintf(format, args);
}

static int posture_iot_do_report(void)
{
    const int service_num = 1;
    ST_IOTA_SERVICE_DATA_INFO services[service_num];
    char service[256] = {0};
    int message_id;

    if (!g_iot_connected) {
        return -1;
    }

    if (!IOTA_IsConnected()) {
        g_iot_connected = 0;
        printf("[IOT] report skipped: MQTT not connected\n");
        return -1;
    }

    memset(services, 0, sizeof(services));

    snprintf(service, sizeof(service),
        "{\"PostureStatus\":\"%s\",\"BadType\":\"%s\",\"Confidence\":%.2f}",
        g_posture_status, g_bad_type, g_confidence);

    printf("[IOT] pending report: %s\n", service);

    services[0].event_time = GetEventTimesStamp();
    services[0].service_id = "PostureService";
    services[0].properties = service;

    message_id = IOTA_PropertiesReport(services, service_num, 0, NULL);
    if (message_id < 0) {
        PrintfLog(EN_LOG_LEVEL_ERROR,
            "posture_iot: IOTA_PropertiesReport failed, messageId %d\n",
            message_id);

        if (!IOTA_IsConnected()) {
            g_iot_connected = 0;
            printf("[IOT] MQTT disconnected after report failure\n");
        }

        MemFree(&services[0].event_time);
        return -1;
    }

    PrintfLog(EN_LOG_LEVEL_INFO,
        "posture_iot: reported status=%s, badType=%s, confidence=%.2f\n",
        g_posture_status, g_bad_type, g_confidence);

    MemFree(&services[0].event_time);
    return 0;
}

int posture_iot_init(const posture_iot_config_t *cfg)
{
    int ret;
    int waited_ms = 0;

    if (cfg == NULL) {
        printf("[IOT] cfg is NULL\n");
        return -1;
    }

    if (g_iot_connected && IOTA_IsConnected()) {
        return 0;
    }

    /*
     * 如果之前 init 过但没连上，先清掉，
     * 避免 MqttBase_init() error, DO NOT init again
     */
    if (g_iot_inited) {
        IOTA_Destroy();
        g_iot_inited = 0;
        g_iot_connected = 0;
    }

    if (PostureIotIsEmpty(cfg->address)) {
        printf("[IOT] cfg->address is NULL/empty\n");
        return -1;
    }
    if (PostureIotIsEmpty(cfg->port)) {
        printf("[IOT] cfg->port is NULL/empty\n");
        return -1;
    }
    if (PostureIotIsEmpty(cfg->device_id)) {
        printf("[IOT] cfg->device_id is NULL/empty\n");
        return -1;
    }
    if (PostureIotIsEmpty(cfg->secret)) {
        printf("[IOT] cfg->secret is NULL/empty\n");
        return -1;
    }

    /* 先看时间是否明显不对，不对就自动校时 */
    if (!PostureIotSystemTimeReasonable()) {
        printf("[TIME] system time looks wrong, try NTP sync...\n");
        (void)PostureIotSyncTimeOnce();
    } else {
        printf("[TIME] system time looks reasonable\n");
        PostureIotPrintNow("[TIME] current");
    }

    printf("[IOT] address   = %s\n", cfg->address);
    printf("[IOT] port      = %s\n", cfg->port);
    printf("[IOT] device_id = %s\n", cfg->device_id);
    printf("[IOT] secret    = ***set***\n");
    printf("[IOT] base_dir  = %s\n", IOT_SDK_BASE_DIR);

    IOTA_Init(IOT_SDK_BASE_DIR);
    g_iot_inited = 1;
    g_iot_connected = 0;

    IOTA_SetPrintLogCallback(PostureIotLog);

    IOTA_ConnectConfigSet(
        (char *)cfg->address,
        (char *)cfg->port,
        (char *)cfg->device_id,
        (char *)cfg->secret
    );

    IOTA_ConfigSetUint(EN_IOTA_CFG_AUTH_MODE, EN_IOTA_CFG_AUTH_MODE_SECRET);

    ret = IOTA_Connect();
    if (ret != 0) {
        PrintfLog(EN_LOG_LEVEL_ERROR,
            "posture_iot: IOTA_Connect failed, ret=%d\n", ret);

        IOTA_Destroy();
        g_iot_inited = 0;
        g_iot_connected = 0;
        return -1;
    }

    /*
     * 不再像之前那样 sleep 1.5s 后直接认为连接成功，
     * 而是轮询 IOTA_IsConnected()。
     */
    while (!IOTA_IsConnected() && waited_ms < IOT_CONNECT_WAIT_MS) {
        PostureIotSleepMs(IOT_CONNECT_POLL_MS);
        waited_ms += IOT_CONNECT_POLL_MS;
    }

    if (!IOTA_IsConnected()) {
        printf("[IOT] connect timeout: MQTT/TLS not really established\n");
        IOTA_Destroy();
        g_iot_inited = 0;
        g_iot_connected = 0;
        return -1;
    }

    g_iot_connected = 1;
    printf("[IOT] posture_iot_init success\n");
    return 0;
}

int posture_iot_report(const posture_report_t *report)
{
    if (!g_iot_connected || report == NULL) {
        return -1;
    }

    if (!IOTA_IsConnected()) {
        g_iot_connected = 0;
        return -1;
    }

    snprintf(g_posture_status, sizeof(g_posture_status), "%s",
             report->posture_status ? report->posture_status : "good");
    snprintf(g_bad_type, sizeof(g_bad_type), "%s",
             report->bad_type ? report->bad_type : "normal");

    if (report->confidence < 0.0) {
        g_confidence = 0.0;
    } else if (report->confidence > 1.0) {
        g_confidence = 1.0;
    } else {
        g_confidence = report->confidence;
    }

    return posture_iot_do_report();
}

int posture_iot_report_if_needed(const posture_report_t *report)
{
    int changed = 0;
    int ret;
    const char *cur_posture_status;
    const char *cur_bad_type;

    if (!g_iot_connected || report == NULL) {
        return -1;
    }

    if (!IOTA_IsConnected()) {
        g_iot_connected = 0;
        return -1;
    }

    if (g_report_fail_cooldown > 0) {
        g_report_fail_cooldown--;
        return 0;
    }

    cur_posture_status = report->posture_status ? report->posture_status : "";
    cur_bad_type = report->bad_type ? report->bad_type : "";

    g_report_frame_tick++;

    if (strcmp(g_last_posture_status, cur_posture_status) != 0) {
        changed = 1;
    }
    if (strcmp(g_last_bad_type, cur_bad_type) != 0) {
        changed = 1;
    }
    if (fabs(g_last_confidence - report->confidence) > 0.08) {
        changed = 1;
    }

    if (!changed && g_report_frame_tick < REPORT_INTERVAL_TICKS) {
        return 0;
    }

    ret = posture_iot_report(report);
    if (ret == 0) {
        snprintf(g_last_posture_status, sizeof(g_last_posture_status), "%s",
                 cur_posture_status);
        snprintf(g_last_bad_type, sizeof(g_last_bad_type), "%s",
                 cur_bad_type);
        g_last_confidence = report->confidence;
        g_report_frame_tick = 0;
    } else {
        /* 失败后先冷却，避免刷屏 */
        g_report_fail_cooldown = REPORT_FAIL_COOLDOWN_TICKS;
    }

    return ret;
}

void posture_iot_deinit(void)
{
    if (g_iot_inited) {
        IOTA_Destroy();
    }

    g_iot_inited = 0;
    g_iot_connected = 0;
    g_report_fail_cooldown = 0;
}
