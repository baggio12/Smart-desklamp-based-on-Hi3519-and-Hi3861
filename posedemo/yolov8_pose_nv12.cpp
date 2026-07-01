#include "yolov8_pose_wrapper.h"
#include <cstring>
#include <cmath>
#include <algorithm>
#include <array>
#include <cstdio>

struct PostureBaseline {
    bool  valid = false;

    // 基线：全部来自“正确坐姿标定”
    float body_lr_mean = 0.f;         // 鼻子在肩线局部坐标中的左右偏移 / 肩宽
    float head_rel_roll_mean = 0.f;   // 头相对肩膀的倾斜角
    float shoulder_roll_mean = 0.f;   // 稳定的身体侧歪角
    float hunch_mean = 0.f;           // 鼻子到肩线法向距离 / 肩宽
    float twist_mean = 0.f;           // 鼻子到左右肩距离差 / 肩宽（侧身代理量）
};

static PostureBaseline g_baseline;

// 标定累加：只累计“正确坐姿帧”
static float g_sum_body_lr = 0.f;
static float g_sum_head_rel_roll = 0.f;
static float g_sum_shoulder_roll = 0.f;
static float g_sum_hunch = 0.f;
static float g_sum_twist = 0.f;

static int g_head_rel_roll_valid_frames = 0;
static int g_calib_good_frames = 0;
static int g_calib_seen_frames = 0;

static int g_bad_body_lr_cnt = 0;
static int g_bad_head_left_cnt = 0;
static int g_bad_head_right_cnt = 0;
static int g_bad_shoulder_roll_cnt = 0;
static int g_bad_twist_cnt = 0;
static int g_bad_hunch_cnt = 0;

static const int CALIB_TOTAL_FRAMES = 120;
static const int BAD_HOLD_FRAMES = 10;
static const int HEAD_HOLD_FRAMES = 4;     // 原来 2，太敏感
static const int HUNCH_HOLD_FRAMES = 28;   // 原来 16，右前方机位下太容易误判

static int g_pose_frame_cnt = 0;

// EMA 平滑状态
static bool  g_ema_inited = false;
static float g_ema_body_lr = 0.f;
static float g_ema_head_rel_roll = 0.f;
static float g_ema_shoulder_roll = 0.f;
static float g_ema_hunch = 0.f;
static float g_ema_twist = 0.f;

// 方向符号修正
static const float HEAD_DIR_SIGN = 1.0f;
static const float SHOULDER_DIR_SIGN = -1.0f;

extern "C" {
struct PoseHandle {
    YoloV8PoseWrapper* model;
};
}

extern "C" {
#include "ot_common_video.h"
#include "ss_mpi_sys.h"
#include "posture_iot.h"
}

extern "C" {
    void*  ss_mpi_sys_mmap(td_phys_addr_t phys_addr, td_u32 size);
    td_s32 ss_mpi_sys_munmap(void* vir_addr, td_u32 size);
}

// ========================= IoT 相关新增 =========================
static bool g_iot_ready = false;

static posture_iot_config_t g_iot_cfg = {
    "b5f57c9f69.st1.iotda-device.cn-north-4.myhuaweicloud.com",
    "8883",
    "69d4f8117f2e6c302f64b895_hi3519-posture-01",
    "Cyw-1146439919"
};

static int g_iot_report_tick = 0;
/* 每 30 秒上报一次，按 30fps 估算约 900 帧 */
static const int IOT_REPORT_INTERVAL_TICKS = 900;

static inline const char* posture_status_str(bool is_bad)
{
    return is_bad ? "bad" : "good";
}

static inline const char* bad_type_to_cloud_str(int final_type,
                                                float d_body_lr,
                                                float d_shoulder_roll,
                                                float d_twist)
{
    switch (final_type) {
    case 1: // BAD_HEAD_LEFT
        return "head_left";
    case 2: // BAD_HEAD_RIGHT
        return "head_right";
    case 3: // BAD_BODY_LR
        return (d_body_lr > 0.f) ? "body_right" : "body_left";
    case 4: // BAD_SHOULDER
        return (d_shoulder_roll > 0.f) ? "shoulder_left" : "shoulder_right";
    case 5: // BAD_TWIST
        return (d_twist > 0.f) ? "twist_right" : "twist_left";
    case 6: // BAD_HUNCH
        return "hunch";
    default:
        return "normal";
    }
}

static inline double normalize_confidence(float final_score)
{
    double c = final_score;
    if (c < 0.0) c = 0.0;
    if (c > 1.0) c = 1.0;
    return c;
}

static inline void try_init_iot_once()
{
    if (g_iot_ready) return;

    int ret = posture_iot_init(&g_iot_cfg);
    if (ret == 0) {
        g_iot_ready = true;
        printf("[IOT] posture_iot_init success\n");
    } else {
        printf("[IOT] posture_iot_init failed\n");
    }
}

static inline void report_posture_to_iot(bool is_bad,
                                         int final_type,
                                         float final_score,
                                         float d_body_lr,
                                         float d_shoulder_roll,
                                         float d_twist,
                                         bool do_log)
{
    if (!g_iot_ready) return;

    posture_report_t rpt;
    rpt.posture_status = posture_status_str(is_bad);
    rpt.bad_type = bad_type_to_cloud_str(final_type, d_body_lr, d_shoulder_roll, d_twist);
    rpt.confidence = normalize_confidence(final_score);

    // 终端预览：按 do_log 节奏打印当前“待上报内容”
    if (do_log) {
        printf("[IOT] pending report: {\"PostureStatus\":\"%s\",\"BadType\":\"%s\",\"Confidence\":%.2f}\n",
               rpt.posture_status,
               rpt.bad_type,
               rpt.confidence);
    }

    g_iot_report_tick++;

    // 真实云上报：仍然保持每 30 秒一次
    if (g_iot_report_tick < IOT_REPORT_INTERVAL_TICKS) {
        return;
    }

    if (posture_iot_report(&rpt) == 0) {
        if (do_log) {
            printf("[IOT] report sent\n");
        }
        g_iot_report_tick = 0;
    } else {
        if (do_log) {
            printf("[IOT] report send failed\n");
        }
    }
}
// ======================= IoT 相关新增结束 =======================

static inline float rad2deg(float r) {
    return r * 180.0f / 3.14159265f;
}

static inline float clampf(float x, float lo, float hi) {
    return std::max(lo, std::min(hi, x));
}

static inline int update_bad_counter(bool bad, int counter) {
    if (bad) {
        if (counter < 1000) counter += 2;
    } else {
        counter -= 3;
    }

    if (counter < 0) counter = 0;
    if (counter > 1000) counter = 1000;
    return counter;
}

static inline bool is_finite_f(float x) {
    return std::isfinite(x);
}

static inline float point_dist(const std::array<float, 2>& a, const std::array<float, 2>& b) {
    float dx = a[0] - b[0];
    float dy = a[1] - b[1];
    return std::sqrt(dx * dx + dy * dy);
}

static inline float ema_update(float prev, float cur, float alpha) {
    return alpha * cur + (1.0f - alpha) * prev;
}

static inline bool is_good_calib_posture(float body_lr_ratio,
                                         float shoulder_roll_deg,
                                         bool head_rel_roll_valid,
                                         float head_rel_roll_deg,
                                         float hunch_ratio,
                                         char* reason,
                                         size_t reason_size) {
    const float CALIB_BODY_LR_MAX         = 200.0f;
    const float CALIB_SHOULDER_ROLL_MAX   = 200.0f;
    const float CALIB_HEAD_REL_ROLL_MAX   = 200.0f;
    const float CALIB_HUNCH_MIN           = 0.10f;
    const float CALIB_HUNCH_MAX           = 200.0f;

    if (!is_finite_f(body_lr_ratio) ||
        !is_finite_f(shoulder_roll_deg) ||
        !is_finite_f(hunch_ratio)) {
        std::snprintf(reason, reason_size, "数值异常");
        return false;
    }

    if (std::fabs(body_lr_ratio) > CALIB_BODY_LR_MAX) {
        std::snprintf(reason, reason_size, "身体未居中");
        return false;
    }

    if (std::fabs(shoulder_roll_deg) > CALIB_SHOULDER_ROLL_MAX) {
        std::snprintf(reason, reason_size, "身体侧歪");
        return false;
    }

    if (head_rel_roll_valid && std::fabs(head_rel_roll_deg) > CALIB_HEAD_REL_ROLL_MAX) {
        std::snprintf(reason, reason_size, "头部倾斜");
        return false;
    }

    if (hunch_ratio < CALIB_HUNCH_MIN) {
        std::snprintf(reason, reason_size, "低头/驼背");
        return false;
    }

    if (hunch_ratio > CALIB_HUNCH_MAX) {
        std::snprintf(reason, reason_size, "关键点异常");
        return false;
    }

    std::snprintf(reason, reason_size, "OK");
    return true;
}

enum BadType {
    BAD_NONE = 0,
    BAD_HEAD_LEFT,
    BAD_HEAD_RIGHT,
    BAD_BODY_LR,
    BAD_SHOULDER,
    BAD_TWIST,
    BAD_HUNCH
};

extern "C"
int yolov8_pose_infer_nv12(void* handle, const ot_video_frame_info* frame) {
    if (!handle || !frame) return -1;
    PoseHandle* h = (PoseHandle*)handle;
    if (!h->model) return -1;

    int width  = frame->video_frame.width;
    int height = frame->video_frame.height;

    g_pose_frame_cnt++;
    const int LOG_INTERVAL = 30;
    bool do_log = (g_pose_frame_cnt % LOG_INTERVAL) == 0;

    td_phys_addr_t y_phys  = frame->video_frame.phys_addr[0];
    td_phys_addr_t uv_phys = frame->video_frame.phys_addr[1];
    td_u32 y_stride  = frame->video_frame.stride[0];
    td_u32 uv_stride = frame->video_frame.stride[1];

    td_u8* y_virt = (td_u8*)ss_mpi_sys_mmap(y_phys, y_stride * height);
    if (!y_virt) return -1;
    td_u8* uv_virt = (td_u8*)ss_mpi_sys_mmap(uv_phys, uv_stride * height / 2);
    if (!uv_virt) {
        ss_mpi_sys_munmap(y_virt, y_stride * height);
        return -1;
    }

    cv::Mat nv12(height * 3 / 2, width, CV_8UC1);
    for (int i = 0; i < height; ++i) {
        memcpy(nv12.data + i * width, y_virt + i * y_stride, width);
    }
    uint8_t* uv_dst = nv12.data + width * height;
    for (int i = 0; i < height / 2; ++i) {
        memcpy(uv_dst + i * width, uv_virt + i * uv_stride, width);
    }

    cv::Mat bgr(height, width, CV_8UC3);
    cv::cvtColor(nv12, bgr, cv::COLOR_YUV2BGR_NV12);

    stdeploy::vision::DetectionResult res;
    h->model->Predict(bgr, &res);

    if (!res.contain_kpts || res.kpts.empty()) {
        if (do_log) {
            if (!g_baseline.valid) {
                printf("[CALIB] 等待关键点输出... contain_kpts=%d boxes=%zu kpts=%zu scores=%zu\n",
                       res.contain_kpts ? 1 : 0,
                       res.boxes.size(),
                       res.kpts.size(),
                       res.scores.size());
            } else {
                printf("[POSTURE] 无关键点输出\n");
            }
        }
        ss_mpi_sys_munmap(y_virt, y_stride * height);
        ss_mpi_sys_munmap(uv_virt, uv_stride * height / 2);
        return 0;
    }

    size_t num_by_box  = res.boxes.size();
    size_t num_by_kpts = res.kpts.size();
    size_t num_by_scr  = res.scores.size();
    size_t num_person  = std::min(num_by_box, std::min(num_by_kpts, num_by_scr));

    if (num_person == 0) {
        if (do_log) {
            if (!g_baseline.valid) printf("[CALIB] 未检测到有效人员\n");
            else                   printf("[POSTURE] 未检测到有效人员\n");
        }
        ss_mpi_sys_munmap(y_virt, y_stride * height);
        ss_mpi_sys_munmap(uv_virt, uv_stride * height / 2);
        return 0;
    }

    size_t best_i = 0;
    float best_area = -1.f;
    for (size_t i = 0; i < num_person; ++i) {
        const auto& box = res.boxes[i];
        float area = std::max(0.f, box[2] - box[0]) * std::max(0.f, box[3] - box[1]);
        if (area > best_area) {
            best_area = area;
            best_i = i;
        }
    }

    auto& kp_res = res.kpts[best_i];

    // 身体关键点和眼睛关键点分开阈值
    const float BODY_KPT_SCORE_THR = 0.08f;
    const float EYE_KPT_SCORE_THR  = 0.20f;

    auto get_pt = [&](int idx, float thr, bool& ok) -> std::array<float, 2> {
        ok = false;
        if (idx < 0) return {0.f, 0.f};
        if ((size_t)idx >= kp_res.keypoints.size()) return {0.f, 0.f};
        if ((size_t)idx < kp_res.scores.size() && kp_res.scores[idx] < thr) {
            return {0.f, 0.f};
        }
        ok = true;
        return kp_res.keypoints[idx];
    };

    bool has_nose=false, has_leye=false, has_reye=false, has_lsh=false, has_rsh=false;
    std::array<float,2> nose = get_pt(0, BODY_KPT_SCORE_THR, has_nose);
    std::array<float,2> leye = get_pt(1, EYE_KPT_SCORE_THR, has_leye);
    std::array<float,2> reye = get_pt(2, EYE_KPT_SCORE_THR, has_reye);
    std::array<float,2> lsh  = get_pt(5, BODY_KPT_SCORE_THR, has_lsh);
    std::array<float,2> rsh  = get_pt(6, BODY_KPT_SCORE_THR, has_rsh);

    if (!(has_nose && has_lsh && has_rsh)) {
        if (do_log) {
            if (!g_baseline.valid) {
                printf("[CALIB] 等待上半身关键点稳定... nose=%d lsh=%d rsh=%d leye=%d reye=%d\n",
                       has_nose ? 1 : 0, has_lsh ? 1 : 0, has_rsh ? 1 : 0,
                       has_leye ? 1 : 0, has_reye ? 1 : 0);
            } else {
                printf("[POSTURE] 上半身关键点不完整\n");
            }
        }
        ss_mpi_sys_munmap(y_virt, y_stride * height);
        ss_mpi_sys_munmap(uv_virt, uv_stride * height / 2);
        return 0;
    }

    std::array<float,2> sh_l_img = lsh;
    std::array<float,2> sh_r_img = rsh;
    if (sh_l_img[0] > sh_r_img[0]) {
        std::swap(sh_l_img, sh_r_img);
    }

    float mid_sh_x = 0.5f * (sh_l_img[0] + sh_r_img[0]);
    float mid_sh_y = 0.5f * (sh_l_img[1] + sh_r_img[1]);

    float shoulder_dx = sh_r_img[0] - sh_l_img[0];
    float shoulder_dy = sh_r_img[1] - sh_l_img[1];
    float shoulder_width = std::sqrt(shoulder_dx * shoulder_dx + shoulder_dy * shoulder_dy);

    if (shoulder_width < 10.f) {
        if (do_log) {
            if (!g_baseline.valid) printf("[CALIB] 肩宽过小，等待更稳定画面...\n");
            else                   printf("[POSTURE] 肩宽过小，跳过本帧\n");
        }
        ss_mpi_sys_munmap(y_virt, y_stride * height);
        ss_mpi_sys_munmap(uv_virt, uv_stride * height / 2);
        return 0;
    }

    float ux = shoulder_dx / shoulder_width;
    float uy = shoulder_dy / shoulder_width;

    float nx = -uy;
    float ny =  ux;

    float disp_x = nose[0] - mid_sh_x;
    float disp_y = nose[1] - mid_sh_y;

    float body_lr_ratio_raw = (disp_x * ux + disp_y * uy) / shoulder_width;
    float shoulder_roll_deg_raw = rad2deg(std::atan2(shoulder_dy, shoulder_dx + 1e-6f));
    float hunch_ratio_raw = std::fabs(disp_x * nx + disp_y * ny) / shoulder_width;

    bool head_rel_roll_valid = false;
    float head_rel_roll_deg_raw = 0.f;
    float eye_roll_deg = 0.f;

    if (has_leye && has_reye) {
        std::array<float,2> eye_l_img = leye;
        std::array<float,2> eye_r_img = reye;
        if (eye_l_img[0] > eye_r_img[0]) {
            std::swap(eye_l_img, eye_r_img);
        }

        float eye_dx = eye_r_img[0] - eye_l_img[0];
        float eye_dy = eye_r_img[1] - eye_l_img[1];
        float eye_dist = point_dist(eye_l_img, eye_r_img);

        // 眼距过小或者眼睛太近，说明眼睛关键点不稳定，不参与头倾
        if (eye_dist >= 8.0f && (std::fabs(eye_dx) > 1e-3f || std::fabs(eye_dy) > 1e-3f)) {
            eye_roll_deg = rad2deg(std::atan2(eye_dy, eye_dx + 1e-6f));
            head_rel_roll_deg_raw = eye_roll_deg - shoulder_roll_deg_raw;
            head_rel_roll_deg_raw = clampf(head_rel_roll_deg_raw, -45.0f, 45.0f);
            head_rel_roll_valid = true;
        }
    }

    float dist_ln = point_dist(nose, sh_l_img);
    float dist_rn = point_dist(nose, sh_r_img);
    float twist_ratio_raw = (dist_ln - dist_rn) / shoulder_width;

    bool shoulder_geom_ok = std::fabs(shoulder_dx) > 8.0f;

    // 分别给不同特征不同平滑强度
    const float EMA_BODY_A     = 0.35f;
    const float EMA_SHOULDER_A = 0.35f;
    const float EMA_HUNCH_A    = 0.22f;
    const float EMA_TWIST_A    = 0.28f;
    const float EMA_HEAD_A     = 0.22f;

    if (!g_ema_inited) {
        g_ema_body_lr = body_lr_ratio_raw;
        g_ema_shoulder_roll = shoulder_roll_deg_raw;
        g_ema_hunch = hunch_ratio_raw;
        g_ema_twist = twist_ratio_raw;
        g_ema_head_rel_roll = head_rel_roll_deg_raw;
        g_ema_inited = true;
    } else {
        g_ema_body_lr = ema_update(g_ema_body_lr, body_lr_ratio_raw, EMA_BODY_A);

        if (shoulder_geom_ok) {
            g_ema_shoulder_roll = ema_update(g_ema_shoulder_roll, shoulder_roll_deg_raw, EMA_SHOULDER_A);
        }

        g_ema_hunch = ema_update(g_ema_hunch, hunch_ratio_raw, EMA_HUNCH_A);
        g_ema_twist = ema_update(g_ema_twist, twist_ratio_raw, EMA_TWIST_A);

        if (head_rel_roll_valid) {
            g_ema_head_rel_roll = ema_update(g_ema_head_rel_roll, head_rel_roll_deg_raw, EMA_HEAD_A);
        }
    }

    float body_lr_ratio = g_ema_body_lr;
    float shoulder_roll_deg = g_ema_shoulder_roll;
    float hunch_ratio = g_ema_hunch;
    float twist_ratio = g_ema_twist;
    float head_rel_roll_deg = head_rel_roll_valid ? g_ema_head_rel_roll : 0.f;

    if (!g_baseline.valid) {
        g_calib_seen_frames++;

        char calib_reason[64] = {0};
        bool good_calib_posture = is_good_calib_posture(body_lr_ratio,
                                                        shoulder_roll_deg,
                                                        head_rel_roll_valid,
                                                        head_rel_roll_deg,
                                                        hunch_ratio,
                                                        calib_reason,
                                                        sizeof(calib_reason));

        if (good_calib_posture) {
            g_sum_body_lr += body_lr_ratio;
            g_sum_shoulder_roll += shoulder_roll_deg;
            g_sum_hunch += hunch_ratio;
            g_sum_twist += twist_ratio;

            if (head_rel_roll_valid) {
                g_sum_head_rel_roll += head_rel_roll_deg;
                g_head_rel_roll_valid_frames++;
            }

            g_calib_good_frames++;

            if (do_log) {
                int percent = (int)(100.0f * g_calib_good_frames / CALIB_TOTAL_FRAMES);
                if (percent > 100) percent = 100;

                printf("[CALIB] 正在采集正确坐姿... (%d%%) good=%d/%d seen=%d\n",
                       percent, g_calib_good_frames, CALIB_TOTAL_FRAMES, g_calib_seen_frames);
                printf("        body_lr=%.3f  head_rel=%.1f(valid=%d)  shoulder=%.1f  hunch=%.3f  twist=%.3f\n",
                       body_lr_ratio, head_rel_roll_deg, head_rel_roll_valid ? 1 : 0,
                       shoulder_roll_deg, hunch_ratio, twist_ratio);
            }
        } else {
            if (do_log) {
                printf("[CALIB] 请调整到正确坐姿后再开始标定: %s\n", calib_reason);
                printf("        body_lr=%.3f  head_rel=%.1f(valid=%d)  shoulder=%.1f  hunch=%.3f  twist=%.3f\n",
                       body_lr_ratio, head_rel_roll_deg, head_rel_roll_valid ? 1 : 0,
                       shoulder_roll_deg, hunch_ratio, twist_ratio);
            }
        }

        if (g_calib_good_frames >= CALIB_TOTAL_FRAMES) {
            g_baseline.body_lr_mean = g_sum_body_lr / (float)g_calib_good_frames;
            g_baseline.shoulder_roll_mean = g_sum_shoulder_roll / (float)g_calib_good_frames;
            g_baseline.hunch_mean = g_sum_hunch / (float)g_calib_good_frames;
            g_baseline.twist_mean = g_sum_twist / (float)g_calib_good_frames;

            if (g_head_rel_roll_valid_frames > 0) {
                g_baseline.head_rel_roll_mean = g_sum_head_rel_roll / (float)g_head_rel_roll_valid_frames;
            } else {
                g_baseline.head_rel_roll_mean = 0.f;
            }

            g_baseline.valid = true;

            if (do_log) {
                printf("[CALIB DONE] baseline from GOOD posture only\n");
                printf("             body_lr=%.3f head_rel=%.2f shoulder=%.2f hunch=%.3f twist=%.3f\n",
                       g_baseline.body_lr_mean,
                       g_baseline.head_rel_roll_mean,
                       g_baseline.shoulder_roll_mean,
                       g_baseline.hunch_mean,
                       g_baseline.twist_mean);
            }
        }

        ss_mpi_sys_munmap(y_virt, y_stride * height);
        ss_mpi_sys_munmap(uv_virt, uv_stride * height / 2);
        return 0;
    }

    // 检测阶段第一次进入时初始化 IoT，避免在标定阶段反复连云
    try_init_iot_once();

    float d_body_lr       = body_lr_ratio - g_baseline.body_lr_mean;
    float d_shoulder_roll = SHOULDER_DIR_SIGN * (shoulder_roll_deg - g_baseline.shoulder_roll_mean);
    float d_hunch         = g_baseline.hunch_mean - hunch_ratio;
    float d_head_roll     = head_rel_roll_valid
                          ? (HEAD_DIR_SIGN * (head_rel_roll_deg - g_baseline.head_rel_roll_mean))
                          : 0.f;
    float d_twist         = twist_ratio - g_baseline.twist_mean;

    const float DELTA_BODY_LR_BAD       = 0.34f;
    const float DELTA_HEAD_ROLL_BAD     = 4.4f;   // 原来 3.2，轻微抖动太容易进头倾
    const float DELTA_SHOULDER_ROLL_BAD = 15.0f;
    const float DELTA_HUNCH_BAD         = 0.17f;  // 原来 0.11，右前方机位太敏感
    const float DELTA_TWIST_BAD         = 0.36f;

    const float HEAD_DIR_DEADZONE       = 1.6f;   // 头部接近中性时不判左右

    auto relu_score = [](float x) -> float {
        return x > 0.f ? x : 0.f;
    };

    float score_body = relu_score(std::fabs(d_body_lr) - DELTA_BODY_LR_BAD) / DELTA_BODY_LR_BAD;

    float score_head_left = head_rel_roll_valid
                          ? relu_score(d_head_roll - DELTA_HEAD_ROLL_BAD) / DELTA_HEAD_ROLL_BAD
                          : 0.f;

    float score_head_right = head_rel_roll_valid
                           ? relu_score((-d_head_roll) - DELTA_HEAD_ROLL_BAD) / DELTA_HEAD_ROLL_BAD
                           : 0.f;

    float score_head = std::max(score_head_left, score_head_right);

    if (!head_rel_roll_valid || std::fabs(d_head_roll) < HEAD_DIR_DEADZONE) {
        score_head_left = 0.f;
        score_head_right = 0.f;
        score_head = 0.f;
    }

    float score_shoulder = relu_score(std::fabs(d_shoulder_roll) - DELTA_SHOULDER_ROLL_BAD) / DELTA_SHOULDER_ROLL_BAD;

    // 右前方机位下，body/twist 偏移会把正常姿态投影成“假前倾”，这里加视角惩罚
    float hunch_view_penalty = 0.f;
    if (std::fabs(d_twist) > 0.12f) {
        hunch_view_penalty += 0.025f + 0.18f * (std::fabs(d_twist) - 0.12f);
    }
    if (std::fabs(d_body_lr) > 0.08f) {
        hunch_view_penalty += 0.020f + 0.12f * (std::fabs(d_body_lr) - 0.08f);
    }

    float score_hunch = relu_score((d_hunch - hunch_view_penalty) - DELTA_HUNCH_BAD) / DELTA_HUNCH_BAD;
    float score_twist = relu_score(std::fabs(d_twist) - DELTA_TWIST_BAD) / DELTA_TWIST_BAD;

    if (score_head > 0.25f) {
        score_body *= 0.15f;
        score_shoulder *= 0.18f;
        score_twist *= 0.65f;
    }

    if (score_head > 0.80f) {
        score_body *= 0.08f;
        score_shoulder *= 0.08f;
        score_twist *= 0.30f;
    }

    if (score_hunch > 0.70f) {
        score_head_left *= 0.25f;
        score_head_right *= 0.25f;
        score_head = std::max(score_head_left, score_head_right);

        score_body *= 0.55f;
        score_shoulder *= 0.40f;
        score_twist *= 0.65f;
    }

    if (score_hunch > 1.40f) {
        score_head_left *= 0.10f;
        score_head_right *= 0.10f;
        score_head = std::max(score_head_left, score_head_right);

        score_shoulder *= 0.30f;
        score_twist *= 0.50f;
    }

    if (score_twist > 1.40f) {
        score_body *= 0.30f;
        score_shoulder *= 0.60f;
    }

    BadType raw_type = BAD_NONE;
    float raw_score = 0.f;

    auto pick_type = [&](BadType t, float s) {
        if (s > raw_score) {
            raw_score = s;
            raw_type = t;
        }
    };

    if (score_hunch > 0.90f &&
        score_hunch >= 0.90f * score_head &&
        score_hunch >= 0.75f * score_shoulder) {
        raw_type = BAD_HUNCH;
        raw_score = score_hunch + 0.45f;
    } else {
        pick_type(BAD_HEAD_LEFT, score_head_left);
        pick_type(BAD_HEAD_RIGHT, score_head_right);
        pick_type(BAD_BODY_LR, score_body);
        pick_type(BAD_SHOULDER, score_shoulder);
        pick_type(BAD_TWIST, score_twist);
        pick_type(BAD_HUNCH, score_hunch);
    }

    if (raw_score < 0.15f) {
        raw_type = BAD_NONE;
    }

    // 头部无效或接近中性时，立即清空左右头倾计数，防止轻微左倾被残留右倾计数带偏
    if (!head_rel_roll_valid || std::fabs(d_head_roll) < HEAD_DIR_DEADZONE) {
        g_bad_head_left_cnt = 0;
        g_bad_head_right_cnt = 0;
    }

    g_bad_body_lr_cnt       = update_bad_counter(raw_type == BAD_BODY_LR, g_bad_body_lr_cnt);
    g_bad_head_left_cnt     = update_bad_counter(raw_type == BAD_HEAD_LEFT, g_bad_head_left_cnt);
    g_bad_head_right_cnt    = update_bad_counter(raw_type == BAD_HEAD_RIGHT, g_bad_head_right_cnt);
    g_bad_shoulder_roll_cnt = update_bad_counter(raw_type == BAD_SHOULDER, g_bad_shoulder_roll_cnt);
    g_bad_twist_cnt         = update_bad_counter(raw_type == BAD_TWIST, g_bad_twist_cnt);
    g_bad_hunch_cnt         = update_bad_counter(raw_type == BAD_HUNCH, g_bad_hunch_cnt);

    bool bad_body_lr       = (g_bad_body_lr_cnt >= BAD_HOLD_FRAMES) && (score_body > 0.15f);

    bool bad_head_left     = head_rel_roll_valid &&
                             (std::fabs(d_head_roll) >= HEAD_DIR_DEADZONE) &&
                             (g_bad_head_left_cnt >= HEAD_HOLD_FRAMES) &&
                             (score_head_left > 0.20f);

    bool bad_head_right    = head_rel_roll_valid &&
                             (std::fabs(d_head_roll) >= HEAD_DIR_DEADZONE) &&
                             (g_bad_head_right_cnt >= HEAD_HOLD_FRAMES) &&
                             (score_head_right > 0.20f);

    bool bad_shoulder_roll = (g_bad_shoulder_roll_cnt >= BAD_HOLD_FRAMES) && (score_shoulder > 0.30f);
    bool bad_twist         = (g_bad_twist_cnt >= BAD_HOLD_FRAMES) && (score_twist > 0.30f);
    bool bad_hunch         = (g_bad_hunch_cnt >= HUNCH_HOLD_FRAMES) && (score_hunch > 0.30f);

    BadType final_type = BAD_NONE;
    float final_score = 0.f;

    auto pick_final = [&](BadType t, bool active, float s) {
        if (active && s > final_score) {
            final_score = s;
            final_type = t;
        }
    };

    if (bad_hunch &&
        score_hunch > 0.90f &&
        score_hunch >= 0.90f * score_head &&
        score_hunch >= 0.75f * score_shoulder) {
        final_type = BAD_HUNCH;
        final_score = score_hunch + 0.35f;
    } else {
        pick_final(BAD_HEAD_LEFT, bad_head_left, score_head_left);
        pick_final(BAD_HEAD_RIGHT, bad_head_right, score_head_right);
        pick_final(BAD_BODY_LR, bad_body_lr, score_body);
        pick_final(BAD_SHOULDER, bad_shoulder_roll, score_shoulder);
        pick_final(BAD_TWIST, bad_twist, score_twist);
        pick_final(BAD_HUNCH, bad_hunch, score_hunch);
    }

    bool is_bad = (final_type != BAD_NONE);

    // ===== 云上报 =====
    report_posture_to_iot(is_bad,
                          (int)final_type,
                          final_score,
                          d_body_lr,
                          d_shoulder_roll,
                          d_twist,
                          do_log);

    if (do_log) {
        printf("[POSTURE]\n");
        printf("  body_lr=%.3f delta=%.3f score=%.2f cnt=%d\n",
               body_lr_ratio, d_body_lr, score_body, g_bad_body_lr_cnt);
        printf("  head_rel=%.1f delta=%.1f scoreL=%.2f scoreR=%.2f cntL=%d cntR=%d valid=%d\n",
               head_rel_roll_deg, d_head_roll,
               score_head_left, score_head_right,
               g_bad_head_left_cnt, g_bad_head_right_cnt,
               head_rel_roll_valid ? 1 : 0);
        printf("  shoulder=%.1f delta=%.1f score=%.2f cnt=%d\n",
               shoulder_roll_deg, d_shoulder_roll, score_shoulder, g_bad_shoulder_roll_cnt);
        printf("  hunch=%.3f delta=%.3f penalty=%.3f score=%.2f cnt=%d\n",
               hunch_ratio, d_hunch, hunch_view_penalty, score_hunch, g_bad_hunch_cnt);
        printf("  twist=%.3f delta=%.3f score=%.2f cnt=%d\n",
               twist_ratio, d_twist, score_twist, g_bad_twist_cnt);

        if (!is_bad) {
            printf("  >>> POSTURE: GOOD\n");
        } else {
            switch (final_type) {
            case BAD_HEAD_LEFT:
                printf("  >>> POSTURE: BAD : 头左倾\n");
                break;
            case BAD_HEAD_RIGHT:
                printf("  >>> POSTURE: BAD : 头右倾\n");
                break;
            case BAD_BODY_LR:
                printf("  >>> POSTURE: BAD : %s\n", d_body_lr > 0 ? "身体右偏" : "身体左偏");
                break;
            case BAD_SHOULDER:
                printf("  >>> POSTURE: BAD : %s\n", d_shoulder_roll > 0 ? "身体左侧歪" : "身体右侧歪");
                break;
            case BAD_TWIST:
                printf("  >>> POSTURE: BAD : %s\n", d_twist > 0 ? "身体右侧身" : "身体左侧身");
                break;
            case BAD_HUNCH:
                printf("  >>> POSTURE: BAD : 驼背/前倾\n");
                break;
            default:
                printf("  >>> POSTURE: GOOD\n");
                break;
            }
        }
    }

    ss_mpi_sys_munmap(y_virt, y_stride * height);
    ss_mpi_sys_munmap(uv_virt, uv_stride * height / 2);
    return 0;
}
