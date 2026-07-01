#pragma once
#include <string>
#include <memory>
#include <opencv2/opencv.hpp>
#include "vision.h"   // stdeploy/vision.h

// 简写命名空间
namespace sd = stdeploy;

class YoloV8PoseWrapper {
public:
    YoloV8PoseWrapper(const std::string& model_file,
                      const std::string& param_file,
                      const std::string& cfg_file);

    bool Predict(const cv::Mat& bgr_img, sd::vision::DetectionResult* res);

private:
    std::unique_ptr<sd::vision::detection::YOLOv8Pose> model_;
};

extern "C" {
// C 接口：sample_svp_npu_process.c 里就是用这两个
void* yolov8_pose_create(const char* om_path,
                         const char* param_path,
                         const char* cfg_path);

void  yolov8_pose_destroy(void* handle);
}



