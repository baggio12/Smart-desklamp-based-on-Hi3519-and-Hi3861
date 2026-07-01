#ifndef POSTURE_IOT_H
#define POSTURE_IOT_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    const char *address;
    const char *port;
    const char *device_id;
    const char *secret;
} posture_iot_config_t;

typedef struct {
    const char *posture_status;   // "good" / "bad"
    const char *bad_type;         // "normal" / "head_left" / ...
    double confidence;            // 0.00 ~ 1.00
} posture_report_t;

/* 初始化并连接云端，只调用一次 */
int posture_iot_init(const posture_iot_config_t *cfg);

/* 上报一次属性 */
int posture_iot_report(const posture_report_t *report);

/* 可选：根据状态变化或周期进行节流 */
int posture_iot_report_if_needed(const posture_report_t *report);

/* 释放资源，程序退出时调用 */
void posture_iot_deinit(void);

#ifdef __cplusplus
}
#endif

#endif
