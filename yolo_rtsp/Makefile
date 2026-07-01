include $(PWD)/../Makefile.param

CC  := aarch64-v01c01-linux-gnu-gcc
CXX := aarch64-v01c01-linux-gnu-g++

# =========================================================
# 路径配置
# =========================================================
POSE_DIR      := /home/hisi/workspace/new_hisi3519dv500_yolo_pose/posedemo
IOT_SDK_DIR   := /home/hisi/workspace/huaweicloud-iot-device-sdk-c-master
IOT_LIB_DIR   := /home/hisi/workspace/lib_aarch64/lib
IOT_ZLIB_DIR  := /home/hisi/workspace/smp/a55_linux/source/bsp/pub/rootfs_glibc_arm64/lib64

ACL_INC       := /home/hisi/Ascend/ascend-toolkit/6.10.t03spc011b020/x86_64-linux/acllib/include
YAML_DIR      := /home/hisi/workspace/new_hisi3519dv500_yolo_pose/3rdparty/yaml-cpp/hisi3519dv500_aarch64-linux-gnu
OPENCV_DIR    := /home/hisi/workspace/new_hisi3519dv500_yolo_pose/3rdparty/opencv/hisi3519dv500_aarch64-linux-gnu
STDEPLOY_DIR  := /home/hisi/workspace/new_hisi3519dv500_yolo_pose/stdeploy

# =========================================================
# C / C++ Flags
# =========================================================
CFLAGS   += -O3
CFLAGS   += -I$(PWD)/include
CFLAGS   += -I$(PWD)/../../common
CFLAGS   += -I$(POSE_DIR)
CFLAGS   += -I$(ACL_INC)

CXXFLAGS += -I$(PWD)/../../common
CXXFLAGS += -I$(POSE_DIR)
CXXFLAGS += -I$(ACL_INC)
CXXFLAGS += -I$(YAML_DIR)/include
CXXFLAGS += -I$(OPENCV_DIR)/include/opencv4
CXXFLAGS += -I$(STDEPLOY_DIR)
CXXFLAGS += -DENABLE_VISION -DENABLE_VISION_DETECTION -DENABLE_ACL_SVP_BACKEND

# Huawei IoT SDK 头文件
CFLAGS   += -I$(IOT_SDK_DIR)/include
CFLAGS   += -I$(IOT_SDK_DIR)/include/agentlite
CFLAGS   += -I$(IOT_SDK_DIR)/include/util
CFLAGS   += -I$(IOT_SDK_DIR)/include/base
CFLAGS   += -I$(IOT_SDK_DIR)/include/platform
CFLAGS   += -I$(IOT_SDK_DIR)/include/protocol
CFLAGS   += -I$(IOT_SDK_DIR)/include/queue
CFLAGS   += -I$(IOT_SDK_DIR)/include/third_party
CFLAGS   += -I$(IOT_SDK_DIR)/include/third_party/cjson

CXXFLAGS += -I$(IOT_SDK_DIR)/include
CXXFLAGS += -I$(IOT_SDK_DIR)/include/agentlite
CXXFLAGS += -I$(IOT_SDK_DIR)/include/util
CXXFLAGS += -I$(IOT_SDK_DIR)/include/base
CXXFLAGS += -I$(IOT_SDK_DIR)/include/platform
CXXFLAGS += -I$(IOT_SDK_DIR)/include/protocol
CXXFLAGS += -I$(IOT_SDK_DIR)/include/queue
CXXFLAGS += -I$(IOT_SDK_DIR)/include/third_party
CXXFLAGS += -I$(IOT_SDK_DIR)/include/third_party/cjson

# 原工程已有
CFLAGS   += $(COMM_INC)
MPI_LIBS += $(LIBS_LD_CFLAGS)

curdir   := $(shell pwd)
MPI_LIBS += $(curdir)/rtsp_lib/librtsp.a
CFLAGS   += -I$(curdir)/rtsp_lib/
CFLAGS   += -I$(curdir)/include/
CXXFLAGS += -I$(curdir)/rtsp_lib/
CXXFLAGS += -I$(curdir)/include/

# =========================================================
# 链接路径和库
# =========================================================
LDFLAGS += -Wl,--gc-sections
LDFLAGS += -L$(OPENCV_DIR)/lib
LDFLAGS += -L$(YAML_DIR)/lib
LDFLAGS += -L$(IOT_LIB_DIR)
LDFLAGS += -L$(IOT_ZLIB_DIR)
LDFLAGS += -L$(IOT_SDK_DIR)

LIBS += -lopencv_world
LIBS += -lyaml-cpp
LIBS += -ldl -lpthread -lm

# 先链接 SDK 主库 libHWMQTT.so
LIBS += -lHWMQTT

# 再链接其依赖的第三方库
LIBS += -lpaho-mqtt3as
LIBS += -lcurl
LIBS += -lssl
LIBS += -lcrypto
LIBS += -lz
LIBS += -lboundscheck

# =========================================================
# 源文件
# =========================================================

# stdeploy 所有源码
STDEPLOY_SRCS := \
    $(wildcard $(STDEPLOY_DIR)/*.cpp) \
    $(wildcard $(STDEPLOY_DIR)/core/*.cpp) \
    $(wildcard $(STDEPLOY_DIR)/utils/*.cpp) \
    $(wildcard $(STDEPLOY_DIR)/runtime/*.cpp) \
    $(wildcard $(STDEPLOY_DIR)/runtime/backends/*.cpp) \
    $(wildcard $(STDEPLOY_DIR)/runtime/backends/acl_svp/*.cpp) \
    $(wildcard $(STDEPLOY_DIR)/vision/*.cpp) \
    $(wildcard $(STDEPLOY_DIR)/vision/common/*.cpp) \
    $(wildcard $(STDEPLOY_DIR)/vision/common/image_processes/*.cpp) \
    $(wildcard $(STDEPLOY_DIR)/vision/image_processes/*.cpp) \
    $(wildcard $(STDEPLOY_DIR)/vision/detection/*.cpp) \
    $(wildcard $(STDEPLOY_DIR)/vision/utils/*.cpp)

# 原 sample 的 C 源文件
SMP_SRCS := $(wildcard ./*.c)
SMP_SRCS += $(wildcard ./sample_svp_npu/*.c)
SMP_SRCS += $(wildcard $(PWD)/../common/*.c)

# 你新增的 IoT 封装模块
IOT_SRCS := $(POSE_DIR)/posture_iot.c

# C++ 源文件
CXX_SRCS := \
    $(POSE_DIR)/yolov8_pose_wrapper.cpp \
    $(POSE_DIR)/yolov8_pose_nv12.cpp \
    $(STDEPLOY_SRCS)

TARGET      := sample_yolo_original_rtsp
TARGET_PATH := $(PWD)

# =========================================================
# 对象文件
# =========================================================
SMP_OBJS := $(patsubst %.c,%.o,$(SMP_SRCS))
IOT_OBJS := $(patsubst %.c,%.o,$(IOT_SRCS))
CXX_OBJS := $(patsubst %.cpp,%.o,$(CXX_SRCS))

OBJS := $(SMP_OBJS) $(IOT_OBJS) $(CXX_OBJS)

# =========================================================
# 规则
# =========================================================
.PHONY : all clean rtsp_lib cleanstream

all: rtsp_lib $(TARGET)

rtsp_lib:
	@$(MAKE) -C rtsp_lib

%.o: %.c
	@echo "[CC]  $<"
	$(CC) $(CFLAGS) -c $< -o $@

%.o: %.cpp
	@echo "[CXX] $<"
	$(CXX) $(CFLAGS) $(CXXFLAGS) -c $< -o $@

$(TARGET): $(COMM_OBJ) $(OBJS)
	@$(CXX) $(CFLAGS) $(CXXFLAGS) $(LDFLAGS) -o $(TARGET_PATH)/$@ $^ \
		-Wl,--start-group $(MPI_LIBS) $(SENSOR_LIBS) $(AUDIO_LIBA) $(LIBS) $(REL_LIB)/libsecurec.a -Wl,--end-group

clean:
	@rm -f $(TARGET_PATH)/$(TARGET)
	@rm -f $(OBJS)
	@rm -f $(COMM_OBJ)

cleanstream:
	@rm -f *.h264
	@rm -f *.h265
	@rm -f *.jpg
	@rm -f *.mjp
	@rm -f *.mp4
