#include "yolov8_pose_wrapper.h"

YoloV8PoseWrapper::YoloV8PoseWrapper(const std::string& model_file,
                                     const std::string& param_file,
                                     const std::string& cfg_file) {
    stdeploy::RuntimeOption runtime_option;
    runtime_option.UseAclSvp();   // 使用 ACL SVP 后端

    // 配置 ACL SVP 的输入输出节点
    stdeploy::AclSvpBackendOption& option = runtime_option.acl_svp_option;
    option.input_names  = {"images"};
    option.output_names = {
        "output0",
        "onnx::ReduceSum_344","349","394",
        "363","onnx::ReduceSum_364","368","401",
        "382","onnx::ReduceSum_383","387","408"
    };

    // 构造 YOLOv8Pose 模型
    model_.reset(new sd::vision::detection::YOLOv8Pose(
        model_file, param_file, runtime_option,
        stdeploy::ModelFormat::om, cfg_file));

    // 初始化失败的话，内部自己会打 log，这里不再访问 protected 的 initialized_
}

bool YoloV8PoseWrapper::Predict(const cv::Mat& bgr_img,
                                sd::vision::DetectionResult* res) {
    if (!model_ || !res) return false;

    // Mat 的构造函数参数是非 const，需要先 clone 一份
    cv::Mat tmp = bgr_img.clone();
    stdeploy::Mat sd_img(tmp);          // 构造 stdeploy::Mat
    return model_->Predict(sd_img, res);
}

// ================= C 接口实现 =================
extern "C" {

struct PoseHandle {
    YoloV8PoseWrapper* model;
};

void* yolov8_pose_create(const char* om_path,
                         const char* param_path,
                         const char* cfg_path) {
    try {
        PoseHandle* h = new PoseHandle;
        std::string om   = om_path   ? om_path   : "";
        std::string para = param_path? param_path: "";
        std::string cfg  = cfg_path  ? cfg_path  : "";
        h->model = new YoloV8PoseWrapper(om, para, cfg);
        return (void*)h;
    } catch (...) {
        return nullptr;
    }
}

void yolov8_pose_destroy(void* handle) {
    if (!handle) return;
    PoseHandle* h = (PoseHandle*)handle;
    delete h->model;
    delete h;
}

} // extern "C"


