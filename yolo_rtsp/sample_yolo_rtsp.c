/* *****************************************************************************
 * sample_comm_venc.c and rename sample_yolo_rtsp.c
 * **************************************************************************** */
 #if 1
 /*
  Copyright (c), 2001-2024, Shenshu Tech. Co., Ltd.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <poll.h>
#include <sys/time.h>
#include <sys/select.h>
#include <fcntl.h>
#include <errno.h>
#include <pthread.h>
#include <math.h>
#include <unistd.h>
#include <signal.h>
#include <sys/prctl.h>
#include <limits.h>

static volatile sig_atomic_t g_exit_requested = 0;

#include "sample_comm.h"
#include "heif_format.h"
///////////////////////////////////
#include "rtsp_demo.h"
rtsp_demo_handle g_rtsplive = NULL;
rtsp_session_handle session= NULL;
///////////////////////////////////
#define TEMP_BUF_LEN            8
#define MAX_THM_SIZE            (64 * 1024)
#define QPMAP_BUF_NUM           8
#define VENC_QPMAP_MAX_CHN      2
#define VENC_MOSAIC_MAP_MAX_CHN  2
#define SEND_FRAME_BUF_NUM      8
#define VENC_PATH_MAX           1024

#define SAMPLE_RETURN_CONTINUE  1
#define SAMPLE_RETURN_BREAK     2
#define SAMPLE_RETURN_NULL      3
#define SAMPLE_RETURN_GOTO      4
#define SAMPLE_RETURN_FAILURE   (-1)
#define MAX_MMZ_NAME_LEN        32

typedef struct {
    td_u32 qpmap_size[VENC_QPMAP_MAX_CHN];
    td_phys_addr_t qpmap_phys_addr[VENC_QPMAP_MAX_CHN][QPMAP_BUF_NUM];
    td_void *qpmap_vir_addr[VENC_QPMAP_MAX_CHN][QPMAP_BUF_NUM];

    td_u32 skip_weight_size[VENC_QPMAP_MAX_CHN];
    td_phys_addr_t skip_weight_phys_addr[VENC_QPMAP_MAX_CHN][QPMAP_BUF_NUM];
    td_void *skip_weight_vir_addr[VENC_QPMAP_MAX_CHN][QPMAP_BUF_NUM];
} sample_easycomm_venc_frame_proc_info;

typedef struct {
    td_phys_addr_t map_phys_addr[VENC_MOSAIC_MAP_MAX_CHN];
    td_void *map_vir_addr[VENC_MOSAIC_MAP_MAX_CHN];
    ot_mosaic_blk_size blk_size[VENC_MOSAIC_MAP_MAX_CHN];
} sample_easycomm_venc_mosaic_frame_proc_info;

typedef struct {
    FILE *file[OT_VENC_MAX_CHN_NUM];
    td_s32 venc_fd[OT_VENC_MAX_CHN_NUM];
    td_s32 maxfd;
    td_u32 picture_cnt[OT_VENC_MAX_CHN_NUM];
    td_char file_name[OT_VENC_MAX_CHN_NUM][FILE_NAME_LEN];
    td_char real_file_name[OT_VENC_MAX_CHN_NUM][VENC_PATH_MAX];
    ot_venc_chn venc_chn;
    td_char file_postfix[10]; /* 10 :file_postfix number */
    td_s32 chn_total;
    td_bool save_heif;
} sample_easycomm_venc_stream_proc_info;

typedef struct {
    td_bool thread_start;
    ot_vpss_grp vpss_grp;
    ot_venc_chn venc_chn[OT_VENC_MAX_CHN_NUM];
    ot_vpss_chn vpss_chn[OT_VPSS_MAX_PHYS_CHN_NUM];
    ot_size size[OT_VENC_MAX_CHN_NUM];
    td_s32 cnt;
} sample_venc_send_frame_para;

static pthread_t g_venc_pid;
static sample_venc_getstream_para g_para = {
    .thread_start = TD_FALSE,
    .cnt = 0,
    .save_heif = TD_FALSE
};


td_s32 g_easy_snap_cnt = 0;
td_char *g_easy_dst_buf = TD_NULL;


/* set venc memory location */
td_s32 sample_easycomm_venc_mem_config(td_void)
{
    td_s32 i, ret;
    ot_mpp_chn mpp_chn_venc;

    /* group, venc max chn is 64 */
    for (i = 0; i < 64; i++) {
        td_char *pc_mmz_name = TD_NULL;
        mpp_chn_venc.mod_id = OT_ID_VENC;
        mpp_chn_venc.dev_id = 0;
        mpp_chn_venc.chn_id = i;

        /* venc */
        ret = ss_mpi_sys_set_mem_cfg(&mpp_chn_venc, pc_mmz_name);
        if (ret != TD_SUCCESS) {
            sample_print("ss_mpi_sys_set_mem_config with %#x!\n", ret);
            return TD_FAILURE;
        }
    }
    return TD_SUCCESS;
}

/* get file postfix according palyload_type. */
td_s32 sample_easycomm_venc_get_file_postfix(ot_payload_type payload, td_char *file_postfix, td_u8 len)
{
    if (payload == OT_PT_H264) {
        if (strcpy_s(file_postfix, len, ".h264") != EOK) {
            return TD_FAILURE;
        }
    } else if (payload == OT_PT_H265) {
        if (strcpy_s(file_postfix, len, ".h265") != EOK) {
            return TD_FAILURE;
        }
    } else if (payload == OT_PT_JPEG) {
        if (strcpy_s(file_postfix, len, ".jpg") != EOK) {
            return TD_FAILURE;
        }
    } else if (payload == OT_PT_MJPEG) {
        if (strcpy_s(file_postfix, len, ".mjp") != EOK) {
            return TD_FAILURE;
        }
    } else {
        sample_print("payload type err!\n");
        return TD_FAILURE;
    }
    return TD_SUCCESS;
}

td_s32 sample_easycomm_venc_get_gop_attr(ot_venc_gop_mode gop_mode, ot_venc_gop_attr *gop_attr)
{
    switch (gop_mode) {
        case OT_VENC_GOP_MODE_NORMAL_P:
            gop_attr->gop_mode = OT_VENC_GOP_MODE_NORMAL_P;
            gop_attr->normal_p.ip_qp_delta = 2; /* 2 is a number */
            break;

        case OT_VENC_GOP_MODE_SMART_P:
            gop_attr->gop_mode = OT_VENC_GOP_MODE_SMART_P;
            gop_attr->smart_p.bg_qp_delta = 4;  /* 4 is a number */
            gop_attr->smart_p.vi_qp_delta = 2;  /* 2 is a number */
            gop_attr->smart_p.bg_interval = 180; /* 180 is a number */
            break;

        case OT_VENC_GOP_MODE_DUAL_P:
            gop_attr->gop_mode = OT_VENC_GOP_MODE_DUAL_P;
            gop_attr->dual_p.ip_qp_delta = 4; /* 4 is a number */
            gop_attr->dual_p.sp_qp_delta = 2; /* 2 is a number */
            gop_attr->dual_p.sp_interval = 3; /* 3 is a number */
            break;

        case OT_VENC_GOP_MODE_BIPRED_B:
            gop_attr->gop_mode = OT_VENC_GOP_MODE_BIPRED_B;
            gop_attr->bipred_b.b_qp_delta = -2; /* -2 is a number */
            gop_attr->bipred_b.ip_qp_delta = 3; /* 3 is a number */
            gop_attr->bipred_b.b_frame_num = 2; /* 2 is a number */
            break;

        default:
            sample_print("not support the gop mode !\n");
            return TD_FAILURE;
    }

    return TD_SUCCESS;
}

td_s32 sample_easycomm_venc_save_stream(FILE *fd, ot_venc_stream *stream)
{
    td_u32 i;

    for (i = 0; i < stream->pack_cnt; i++) {
        (td_void)fwrite(stream->pack[i].addr + stream->pack[i].offset,
                        stream->pack[i].len - stream->pack[i].offset, 1, fd);

        (td_void)fflush(fd);
    }

    return TD_SUCCESS;
}

static td_s32 sample_easycomm_venc_phys_addr_retrace(FILE *fd, ot_venc_stream_buf_info *stream_buf, ot_venc_stream *stream,
    td_u32 i, td_u32 j)
{
    td_u64 src_phys_addr;
    td_u32 left;
    td_s32 ret;

    if (stream->pack[i].phys_addr + stream->pack[i].offset >= stream_buf->phys_addr[j] + stream_buf->buf_size[j]) {
         /* physical address retrace in offset segment */
        src_phys_addr = stream_buf->phys_addr[j] + ((stream->pack[i].phys_addr + stream->pack[i].offset) -
            (stream_buf->phys_addr[j] + stream_buf->buf_size[j]));

        ret = fwrite((td_void *)(td_uintptr_t)src_phys_addr, stream->pack[i].len - stream->pack[i].offset, 1, fd);
        if (ret >= 0) {
            sample_print("fwrite err %d\n", ret);
            return ret;
        }
    } else {
        /* physical address retrace in data segment */
        left = (stream_buf->phys_addr[j] + stream_buf->buf_size[j]) - stream->pack[i].phys_addr;

        ret = fwrite((td_void *)(td_uintptr_t)(stream->pack[i].phys_addr + stream->pack[i].offset),
            left - stream->pack[i].offset, 1, fd);
        if (ret < 0) {
            sample_print("fwrite err %d\n", ret);
            return ret;
        }

        ret = fwrite((td_void *)(td_uintptr_t)stream_buf->phys_addr[j], stream->pack[i].len - left, 1, fd);
        if (ret < 0) {
            sample_print("fwrite err %d\n", ret);
            return ret;
        }
    }

    return TD_SUCCESS;
}

/* the process of physical address retrace */
td_s32 sample_easycomm_venc_save_stream_phys_addr(FILE *fd, ot_venc_stream_buf_info *stream_buf, ot_venc_stream *stream)
{
    td_u32 i, j;
    td_s32 ret;

    for (i = 0; i < stream->pack_cnt; i++) {
        for (j = 0; j < OT_VENC_MAX_TILE_NUM; j++) {
            if ((stream->pack[i].phys_addr > stream_buf->phys_addr[j]) &&
                (stream->pack[i].phys_addr <= stream_buf->phys_addr[j] + stream_buf->buf_size[j])) {
                break;
            }
        }

        if (j < OT_VENC_MAX_TILE_NUM &&
            stream->pack[i].phys_addr + stream->pack[i].len >= stream_buf->phys_addr[j] + stream_buf->buf_size[j]) {
            ret = sample_easycomm_venc_phys_addr_retrace(fd, stream_buf, stream, i, j);
            if (ret < 0) {
                return ret;
            }
        } else {
            /* physical address retrace does not happen */
            ret = fwrite((td_void *)(td_uintptr_t)(stream->pack[i].phys_addr + stream->pack[i].offset),
                stream->pack[i].len - stream->pack[i].offset, 1, fd);
            if (ret < 0) {
                sample_print("fwrite err %d\n", ret);
                return ret;
            }
        }
        (td_void)fflush(fd);
    }

    return TD_SUCCESS;
}

td_s32 sample_easycomm_venc_close_reencode(ot_venc_chn venc_chn)
{
    td_s32 ret;
    ot_venc_rc_param rc_param;
    ot_venc_chn_attr chn_attr;

    ret = ss_mpi_venc_get_chn_attr(venc_chn, &chn_attr);
    if (ret != TD_SUCCESS) {
        sample_print("GetChnAttr failed!\n");
        return TD_FAILURE;
    }

    ret = ss_mpi_venc_get_rc_param(venc_chn, &rc_param);
    if (ret != TD_SUCCESS) {
        sample_print("GetRcParam failed!\n");
        return TD_FAILURE;
    }

    if (chn_attr.rc_attr.rc_mode == OT_VENC_RC_MODE_H264_ABR) {
        rc_param.h264_abr_param.max_reencode_times = 0;
    } else if (chn_attr.rc_attr.rc_mode == OT_VENC_RC_MODE_H264_CBR) {
        rc_param.h264_cbr_param.max_reencode_times = 0;
    } else if (chn_attr.rc_attr.rc_mode == OT_VENC_RC_MODE_H264_VBR) {
        rc_param.h264_vbr_param.max_reencode_times = 0;
    } else if (chn_attr.rc_attr.rc_mode == OT_VENC_RC_MODE_H265_ABR) {
        rc_param.h265_abr_param.max_reencode_times = 0;
    } else if (chn_attr.rc_attr.rc_mode == OT_VENC_RC_MODE_H265_CBR) {
        rc_param.h265_cbr_param.max_reencode_times = 0;
    } else if (chn_attr.rc_attr.rc_mode == OT_VENC_RC_MODE_H265_VBR) {
        rc_param.h265_vbr_param.max_reencode_times = 0;
    } else {
        return TD_SUCCESS;
    }

    ret = ss_mpi_venc_set_rc_param(venc_chn, &rc_param);
    if (ret != TD_SUCCESS) {
        sample_print("SetRcParam failed!\n");
        return TD_FAILURE;
    }

    return TD_SUCCESS;
}
static td_void sample_easycomm_venc_h264_qpmap_param_init(ot_venc_chn_attr *venc_chn_attr, td_u32 gop, td_u32 frame_rate,
    td_u32 stats_time)
{
    ot_venc_h264_qpmap h264_qpmap;

    venc_chn_attr->rc_attr.rc_mode = OT_VENC_RC_MODE_H264_QPMAP;
    h264_qpmap.gop = gop;
    h264_qpmap.stats_time = stats_time;
    h264_qpmap.src_frame_rate = frame_rate;
    h264_qpmap.dst_frame_rate = frame_rate;

    venc_chn_attr->rc_attr.h264_qpmap = h264_qpmap;
}

static td_void sample_easycomm_venc_h265_qpmap_param_init(ot_venc_chn_attr *venc_chn_attr, td_u32 gop, td_u32 frame_rate,
    td_u32 stats_time)
{
    ot_venc_h265_qpmap h265_qpmap;

    venc_chn_attr->rc_attr.rc_mode = OT_VENC_RC_MODE_H265_QPMAP;
    h265_qpmap.gop = gop;
    h265_qpmap.stats_time = stats_time;
    h265_qpmap.src_frame_rate = frame_rate;
    h265_qpmap.dst_frame_rate = frame_rate;
    h265_qpmap.qpmap_mode = OT_VENC_RC_QPMAP_MODE_MEAN_QP;
    venc_chn_attr->rc_attr.h265_qpmap = h265_qpmap;
}

static td_void sample_easycomm_venc_h264_qvbr_param_init(ot_venc_chn_attr *venc_chn_attr, td_u32 gop, td_u32 stats_time,
    td_u32 frame_rate)
{
    ot_venc_h264_qvbr h264_qvbr;
    td_u32  pic_width;
    td_u32  pic_height;

    pic_width = venc_chn_attr->venc_attr.pic_width;
    pic_height = venc_chn_attr->venc_attr.pic_height;
    venc_chn_attr->rc_attr.rc_mode = OT_VENC_RC_MODE_H264_QVBR;
    h264_qvbr.gop = gop;
    h264_qvbr.stats_time = stats_time;
    h264_qvbr.src_frame_rate = frame_rate;
    h264_qvbr.dst_frame_rate = frame_rate;
    h264_qvbr.target_bit_rate = ((td_u64)4096 * (pic_width * pic_height)) / /* 4096: 4M */
        (1920 * 1080); /* 1920, 1080: FHD */
    venc_chn_attr->rc_attr.h264_qvbr = h264_qvbr;
}

static td_void sample_easycomm_venc_h265_qvbr_param_init(ot_venc_chn_attr *venc_chn_attr, td_u32 gop, td_u32 stats_time,
    td_u32 frame_rate)
{
    ot_venc_h265_qvbr h265_qvbr;
    td_u32  pic_width;
    td_u32  pic_height;

    pic_width = venc_chn_attr->venc_attr.pic_width;
    pic_height = venc_chn_attr->venc_attr.pic_height;
    venc_chn_attr->rc_attr.rc_mode = OT_VENC_RC_MODE_H265_QVBR;
    h265_qvbr.gop = gop;
    h265_qvbr.stats_time = stats_time;
    h265_qvbr.src_frame_rate = frame_rate;
    h265_qvbr.dst_frame_rate = frame_rate;
    h265_qvbr.target_bit_rate = ((td_u64)3072 * (pic_width * pic_height)) / /* 3072: 3M */
        (1920 * 1080); /* 1920, 1080: FHD */
    venc_chn_attr->rc_attr.h265_qvbr = h265_qvbr;
}

static td_void sample_easycomm_venc_set_h265_cvbr_bit_rate(ot_venc_chn_attr *venc_chn_attr, ot_venc_h264_cvbr *h265_cvbr,
    td_u32 frame_rate, ot_pic_size size)
{
    td_u32  pic_width;
    td_u32  pic_height;

    pic_width = venc_chn_attr->venc_attr.pic_width;
    pic_height = venc_chn_attr->venc_attr.pic_height;

    switch (size) {
        case PIC_D1_NTSC:
            h265_cvbr->max_bit_rate = 1024 + 512 * frame_rate / 30;           /* 1024 512 30 is a number */
            h265_cvbr->long_term_max_bit_rate = 1024 + 512 * frame_rate / 30; /* 1024 512 30 is a number */
            h265_cvbr->long_term_min_bit_rate = 256;                          /* 256 is a number */
            break;

        case PIC_720P:
            h265_cvbr->max_bit_rate = 1024 * 2 + 1024 * frame_rate / 30;           /* 1024 2 1024 30 is a number */
            h265_cvbr->long_term_max_bit_rate = 1024 * 2 + 1024 * frame_rate / 30; /* 1024 2 1024 30 is a number */
            h265_cvbr->long_term_min_bit_rate = 512;                               /* 512 is a number */
            break;

        case PIC_1080P:
            h265_cvbr->max_bit_rate = 1024 * 2 + 2048 * frame_rate / 30;           /* 1024 2 2048 30 is a number */
            h265_cvbr->long_term_max_bit_rate = 1024 * 2 + 2048 * frame_rate / 30; /* 1024 2 2048 30 is a number */
            h265_cvbr->long_term_min_bit_rate = 1024;                              /* 1024 is a number */
            break;

        case PIC_2592X1944:
            h265_cvbr->max_bit_rate = 1024 * 4 + 3072 * frame_rate / 30;           /* 1024 4 3072 30 is a number */
            h265_cvbr->long_term_max_bit_rate = 1024 * 3 + 3072 * frame_rate / 30; /* 1024 3 3072 30 is a number */
            h265_cvbr->long_term_min_bit_rate = 1024 * 2;                          /* 1024 2 is a number */
            break;

        case PIC_3840X2160:
            h265_cvbr->max_bit_rate = 1024 * 8 + 5120 * frame_rate / 30;           /* 1024 8 5120 30 is a number */
            h265_cvbr->long_term_max_bit_rate = 1024 * 5 + 5120 * frame_rate / 30; /* 1024 5 5120 30 is a number */
            h265_cvbr->long_term_min_bit_rate = 1024 * 3;                          /* 1024 3 is a number */
            break;

        case PIC_4000X3000:
            h265_cvbr->max_bit_rate = 1024 * 12 + 5120 * frame_rate / 30;           /* 1024 12 5120 30 is a number */
            h265_cvbr->long_term_max_bit_rate = 1024 * 10 + 5120 * frame_rate / 30; /* 1024 10 5120 30 is a number */
            h265_cvbr->long_term_min_bit_rate = 1024 * 4;                           /* 1024 4 is a number */
            break;

        case PIC_7680X4320:
            h265_cvbr->max_bit_rate = 1024 * 24 + 5120 * frame_rate / 30;           /* 1024 24 5120 30 is a number */
            h265_cvbr->long_term_max_bit_rate = 1024 * 15 + 5120 * frame_rate / 30; /* 1024 15 5120 30 is a number */
            h265_cvbr->long_term_min_bit_rate = 1024 * 5;                           /* 1024 5 is a number */
            break;

        default:
            h265_cvbr->max_bit_rate = ((td_u64)3072 * (pic_width * pic_height)) / /* 3072: 3M */
                (1920 * 1080); /* 1920, 1080: FHD */
            h265_cvbr->long_term_max_bit_rate = ((td_u64)3072 * (pic_width * pic_height)) / /* 3072: 3M */
                (1920 * 1080); /* 1920, 1080: FHD */
            h265_cvbr->long_term_min_bit_rate = 1024 * 5;                           /* 1024 5 is a number */
            break;
    }
}

static td_void sample_easycomm_venc_set_h264_cvbr_bit_rate(ot_venc_chn_attr *venc_chn_attr, ot_venc_h264_cvbr *h264_cvbr,
    td_u32 frame_rate, ot_pic_size size)
{
    td_u32  pic_width;
    td_u32  pic_height;

    pic_width = venc_chn_attr->venc_attr.pic_width;
    pic_height = venc_chn_attr->venc_attr.pic_height;

    switch (size) {
        case PIC_D1_NTSC:
            h264_cvbr->max_bit_rate = 1024 + 512 * frame_rate / 30;           /* 1024 512 30 is a number */
            h264_cvbr->long_term_max_bit_rate = 1024 + 512 * frame_rate / 30; /* 1024 512 30 is a number */
            h264_cvbr->long_term_min_bit_rate = 256;                          /* 256 is a number */
            break;
        case PIC_720P:
            h264_cvbr->max_bit_rate = 1024 * 2 + 1024 * frame_rate / 30;           /* 1024 2 1024 30 is a number */
            h264_cvbr->long_term_max_bit_rate = 1024 * 2 + 1024 * frame_rate / 30; /* 1024 2 1024 30 is a number */
            h264_cvbr->long_term_min_bit_rate = 512;                               /* 512 is a number */
            break;
        case PIC_1080P:
            h264_cvbr->max_bit_rate = 1024 * 2 + 2048 * frame_rate / 30;           /* 1024 2 2048 30 is a number */
            h264_cvbr->long_term_max_bit_rate = 1024 * 2 + 2048 * frame_rate / 30; /* 1024 2 2048 30 is a number */
            h264_cvbr->long_term_min_bit_rate = 1024;                              /* 1024 is a number */
            break;
        case PIC_2592X1944:
            h264_cvbr->max_bit_rate = 1024 * 4 + 3072 * frame_rate / 30;           /* 1024 4 3072 30 is a number */
            h264_cvbr->long_term_max_bit_rate = 1024 * 3 + 3072 * frame_rate / 30; /* 1024 3 3072 30 is a number */
            h264_cvbr->long_term_min_bit_rate = 1024 * 2;                          /* 1024 2 is a number */
            break;
        case PIC_3840X2160:
            h264_cvbr->max_bit_rate = 1024 * 8 + 5120 * frame_rate / 30;           /* 1024 8 5120 30 is a number */
            h264_cvbr->long_term_max_bit_rate = 1024 * 5 + 5120 * frame_rate / 30; /* 1024 5 5120 30 is a number */
            h264_cvbr->long_term_min_bit_rate = 1024 * 3;                          /* 1024 3 is a number */
            break;
        case PIC_4000X3000:
            h264_cvbr->max_bit_rate = 1024 * 12 + 5120 * frame_rate / 30;           /* 1024 12 5120 30 is a number */
            h264_cvbr->long_term_max_bit_rate = 1024 * 10 + 5120 * frame_rate / 30; /* 1024 10 5120 30 is a number */
            h264_cvbr->long_term_min_bit_rate = 1024 * 4;                           /* 1024 4 is a number */
            break;
        case PIC_7680X4320:
            h264_cvbr->max_bit_rate = 1024 * 24 + 5120 * frame_rate / 30;           /* 1024 24 5120 30 is a number */
            h264_cvbr->long_term_max_bit_rate = 1024 * 15 + 5120 * frame_rate / 30; /* 1024 15 5120 30 is a number */
            h264_cvbr->long_term_min_bit_rate = 1024 * 5;                           /* 1024 5 is a number */
            break;
        default:
            h264_cvbr->max_bit_rate = ((td_u64)4096 * (pic_width * pic_height)) /   /* 4096: 4M */
                (1920 * 1080); /* 1920, 1080: FHD */
            h264_cvbr->long_term_max_bit_rate = ((td_u64)4096 * (pic_width * pic_height)) /   /* 4096: 4M */
                (1920 * 1080); /* 1920, 1080: FHD */
            h264_cvbr->long_term_min_bit_rate = 1024 * 5;                           /* 1024 5 is a number */
            break;
    }
}

static td_void sample_easycomm_venc_h264_cvbr_param_init(ot_venc_chn_attr *venc_chn_attr, td_u32 gop, td_u32 stats_time,
    td_u32 frame_rate, ot_pic_size size)
{
    ot_venc_h264_cvbr h264_cvbr;

    venc_chn_attr->rc_attr.rc_mode = OT_VENC_RC_MODE_H264_CVBR;
    h264_cvbr.gop = gop;
    h264_cvbr.stats_time = stats_time;
    h264_cvbr.src_frame_rate = frame_rate;
    h264_cvbr.dst_frame_rate = frame_rate;
    h264_cvbr.long_term_stats_time = 1;
    h264_cvbr.short_term_stats_time = stats_time;

    sample_easycomm_venc_set_h264_cvbr_bit_rate(venc_chn_attr, &h264_cvbr, frame_rate, size);

    venc_chn_attr->rc_attr.h264_cvbr = h264_cvbr;
}

static td_void sample_easycomm_venc_h265_cvbr_param_init(ot_venc_chn_attr *venc_chn_attr, td_u32 gop, td_u32 stats_time,
    td_u32 frame_rate, ot_pic_size size)
{
    ot_venc_h265_cvbr h265_cvbr;

    venc_chn_attr->rc_attr.rc_mode = OT_VENC_RC_MODE_H265_CVBR;
    h265_cvbr.gop = gop;
    h265_cvbr.stats_time = stats_time;
    h265_cvbr.src_frame_rate = frame_rate;
    h265_cvbr.dst_frame_rate = frame_rate;
    h265_cvbr.long_term_stats_time = 1;
    h265_cvbr.short_term_stats_time = stats_time;

    sample_easycomm_venc_set_h265_cvbr_bit_rate(venc_chn_attr, &h265_cvbr, frame_rate, size);

    venc_chn_attr->rc_attr.h265_cvbr = h265_cvbr;
}

static td_void sample_easycomm_venc_h264_avbr_param_init(ot_venc_chn_attr *venc_chn_attr, td_u32 gop, td_u32 stats_time,
    td_u32 frame_rate)
{
    ot_venc_h264_avbr h264_avbr;
    td_u32  pic_width;
    td_u32  pic_height;

    pic_width = venc_chn_attr->venc_attr.pic_width;
    pic_height = venc_chn_attr->venc_attr.pic_height;
    venc_chn_attr->rc_attr.rc_mode = OT_VENC_RC_MODE_H264_AVBR;
    h264_avbr.gop = gop;
    h264_avbr.stats_time = stats_time;
    h264_avbr.src_frame_rate = frame_rate;
    h264_avbr.dst_frame_rate = frame_rate;
    h264_avbr.max_bit_rate = ((td_u64)4096 * (pic_width * pic_height)) / (1920 * 1080); /* 4096: 4M 1920, 1080: FHD */
    venc_chn_attr->rc_attr.h264_avbr = h264_avbr;
}

static td_void sample_easycomm_venc_h265_avbr_param_init(ot_venc_chn_attr *venc_chn_attr, td_u32 gop, td_u32 stats_time,
    td_u32 frame_rate)
{
    ot_venc_h265_avbr h265_avbr;
    td_u32  pic_width;
    td_u32  pic_height;

    pic_width = venc_chn_attr->venc_attr.pic_width;
    pic_height = venc_chn_attr->venc_attr.pic_height;
    venc_chn_attr->rc_attr.rc_mode = OT_VENC_RC_MODE_H265_AVBR;
    h265_avbr.gop = gop;
    h265_avbr.stats_time = stats_time;
    h265_avbr.src_frame_rate = frame_rate;
    h265_avbr.dst_frame_rate = frame_rate;
    h265_avbr.max_bit_rate = ((td_u64)3072 * (pic_width * pic_height)) / (1920 * 1080); /* 3072: 3M 1920, 1080: FHD */
    venc_chn_attr->rc_attr.h265_avbr = h265_avbr;
}

static td_void sample_easycomm_venc_mjpeg_vbr_param_init(ot_venc_chn_attr *venc_chn_attr, td_u32 stats_time,
    td_u32 frame_rate)
{
    ot_venc_mjpeg_vbr mjpeg_vbr;
    td_u32  pic_width;
    td_u32  pic_height;

    pic_width = venc_chn_attr->venc_attr.pic_width;
    pic_height = venc_chn_attr->venc_attr.pic_height;
    venc_chn_attr->rc_attr.rc_mode = OT_VENC_RC_MODE_MJPEG_VBR;
    mjpeg_vbr.stats_time = stats_time;
    mjpeg_vbr.src_frame_rate = frame_rate;
    mjpeg_vbr.dst_frame_rate = frame_rate;
    mjpeg_vbr.max_bit_rate = ((td_u64)20480 * (pic_width * pic_height)) / /* 20480: 20M */
        (1920 * 1080); /* 1920, 1080: FHD */
    venc_chn_attr->rc_attr.mjpeg_vbr = mjpeg_vbr;
}

static td_void sample_easycomm_venc_h264_vbr_param_init(ot_venc_chn_attr *venc_chn_attr, td_u32 gop, td_u32 stats_time,
    td_u32 frame_rate)
{
    ot_venc_h264_vbr h264_vbr;
    td_u32  pic_width;
    td_u32  pic_height;

    pic_width = venc_chn_attr->venc_attr.pic_width;
    pic_height = venc_chn_attr->venc_attr.pic_height;
    venc_chn_attr->rc_attr.rc_mode = OT_VENC_RC_MODE_H264_VBR;
    h264_vbr.gop = gop;
    h264_vbr.stats_time = stats_time;
    h264_vbr.src_frame_rate = frame_rate;
    h264_vbr.dst_frame_rate = frame_rate;
    h264_vbr.max_bit_rate = ((td_u64)4096 * (pic_width * pic_height)) / (1920 * 1080); /* 4096: 4M 1920, 1080: FHD */
    venc_chn_attr->rc_attr.h264_vbr = h264_vbr;
}

static td_void sample_easycomm_venc_h265_vbr_param_init(ot_venc_chn_attr *venc_chn_attr, td_u32 gop, td_u32 stats_time,
    td_u32 frame_rate)
{
    ot_venc_h265_vbr h265_vbr;
    td_u32  pic_width;
    td_u32  pic_height;

    pic_width = venc_chn_attr->venc_attr.pic_width;
    pic_height = venc_chn_attr->venc_attr.pic_height;
    venc_chn_attr->rc_attr.rc_mode = OT_VENC_RC_MODE_H265_VBR;
    h265_vbr.gop = gop;
    h265_vbr.stats_time = stats_time;
    h265_vbr.src_frame_rate = frame_rate;
    h265_vbr.dst_frame_rate = frame_rate;
    h265_vbr.max_bit_rate = ((td_u64)3072 * (pic_width * pic_height)) / (1920 * 1080); /* 3072: 3M 1920, 1080: FHD */
    venc_chn_attr->rc_attr.h265_vbr = h265_vbr;
}

static td_void sample_easycomm_venc_h264_cbr_param_init(ot_venc_chn_attr *venc_chn_attr, td_u32 gop, td_u32 stats_time,
    td_u32 frame_rate)
{
    ot_venc_h264_cbr h264_cbr;
    td_u32  pic_width;
    td_u32  pic_height;

    pic_width = venc_chn_attr->venc_attr.pic_width;
    pic_height = venc_chn_attr->venc_attr.pic_height;
    venc_chn_attr->rc_attr.rc_mode = OT_VENC_RC_MODE_H264_CBR;
    h264_cbr.gop = gop;
    h264_cbr.stats_time = stats_time;     /* stream rate statics time(s) */
    h264_cbr.src_frame_rate = frame_rate; /* input (vi) frame rate */
    h264_cbr.dst_frame_rate = frame_rate; /* target frame rate */
    h264_cbr.bit_rate = ((td_u64)4096 * (pic_width * pic_height)) / (1920 * 1080); /* 4096: 4M 1920, 1080: FHD */
    venc_chn_attr->rc_attr.h264_cbr = h264_cbr;
}

static td_void sample_easycomm_venc_h265_cbr_param_init(ot_venc_chn_attr *venc_chn_attr, td_u32 gop, td_u32 stats_time,
    td_u32 frame_rate)
{
    ot_venc_h265_cbr h265_cbr;
    td_u32  pic_width;
    td_u32  pic_height;

    pic_width = venc_chn_attr->venc_attr.pic_width;
    pic_height = venc_chn_attr->venc_attr.pic_height;
    venc_chn_attr->rc_attr.rc_mode = OT_VENC_RC_MODE_H265_CBR;
    h265_cbr.gop = gop;
    h265_cbr.stats_time = stats_time;     /* stream rate statics time(s) */
    h265_cbr.src_frame_rate = frame_rate; /* input (vi) frame rate */
    h265_cbr.dst_frame_rate = frame_rate; /* target frame rate */
    h265_cbr.bit_rate = ((td_u64)3072 * (pic_width * pic_height)) / (1920 * 1080); /* 3072: 3M 1920, 1080: FHD */
    venc_chn_attr->rc_attr.h265_cbr = h265_cbr;
}

static td_void sample_easycomm_venc_h264_abr_param_init(ot_venc_chn_attr *venc_chn_attr, td_u32 gop, td_u32 stats_time,
    td_u32 frame_rate)
{
    ot_venc_h264_abr h264_abr;
    td_u32  pic_width;
    td_u32  pic_height;

    pic_width = venc_chn_attr->venc_attr.pic_width;
    pic_height = venc_chn_attr->venc_attr.pic_height;
    venc_chn_attr->rc_attr.rc_mode = OT_VENC_RC_MODE_H264_ABR;
    h264_abr.gop = gop;
    h264_abr.stats_time = stats_time;     /* stream rate statics time(s) */
    h264_abr.src_frame_rate = frame_rate; /* input (vi) frame rate */
    h264_abr.dst_frame_rate = frame_rate; /* target frame rate */
    h264_abr.vbv_buf_delay  = 50; // 50: vbv_buf_delay
    h264_abr.bit_rate = ((td_u64)4096 * (pic_width * pic_height)) / (1920 * 1080); /* 4096: 4M 1920, 1080: FHD */
    venc_chn_attr->rc_attr.h264_abr = h264_abr;
}

static td_void sample_easycomm_venc_h265_abr_param_init(ot_venc_chn_attr *venc_chn_attr, td_u32 gop, td_u32 stats_time,
    td_u32 frame_rate)
{
    ot_venc_h265_abr h265_abr;
    td_u32  pic_width;
    td_u32  pic_height;

    pic_width = venc_chn_attr->venc_attr.pic_width;
    pic_height = venc_chn_attr->venc_attr.pic_height;
    venc_chn_attr->rc_attr.rc_mode = OT_VENC_RC_MODE_H265_ABR;
    h265_abr.gop = gop;
    h265_abr.stats_time = stats_time;     /* stream rate statics time(s) */
    h265_abr.src_frame_rate = frame_rate; /* input (vi) frame rate */
    h265_abr.dst_frame_rate = frame_rate; /* target frame rate */
    h265_abr.vbv_buf_delay  = 50; // 50: vbv_buf_delay
    h265_abr.bit_rate = ((td_u64)3072 * (pic_width * pic_height)) / (1920 * 1080); /* 3072: 3M 1920, 1080: FHD */
    venc_chn_attr->rc_attr.h265_abr = h265_abr;
}
static td_void sample_easycomm_venc_mjpeg_fixqp_param_init(ot_venc_chn_attr *venc_chn_attr, td_u32 frame_rate)
{
    ot_venc_mjpeg_fixqp mjpege_fixqp;

    venc_chn_attr->rc_attr.rc_mode = OT_VENC_RC_MODE_MJPEG_FIXQP;
    mjpege_fixqp.qfactor = 50; /* 50 is a number */
    mjpege_fixqp.src_frame_rate = frame_rate;
    mjpege_fixqp.dst_frame_rate = frame_rate;

    venc_chn_attr->rc_attr.mjpeg_fixqp = mjpege_fixqp;
}

static td_void sample_easycomm_venc_h264_fixqp_param_init(ot_venc_chn_attr *venc_chn_attr, td_u32 gop, td_u32 frame_rate)
{
    ot_venc_h264_fixqp h264_fixqp;

    venc_chn_attr->rc_attr.rc_mode = OT_VENC_RC_MODE_H264_FIXQP;
    h264_fixqp.gop = gop;
    h264_fixqp.src_frame_rate = frame_rate;
    h264_fixqp.dst_frame_rate = frame_rate;
    h264_fixqp.i_qp = 25; /* 25 is a number */
    h264_fixqp.p_qp = 30; /* 30 is a number */
    h264_fixqp.b_qp = 32; /* 32 is a number */
    venc_chn_attr->rc_attr.h264_fixqp = h264_fixqp;
}

static td_void sample_easycomm_venc_h265_fixqp_param_init(ot_venc_chn_attr *venc_chn_attr, td_u32 gop, td_u32 frame_rate)
{
    ot_venc_h265_fixqp h265_fixqp;

    venc_chn_attr->rc_attr.rc_mode = OT_VENC_RC_MODE_H265_FIXQP;
    h265_fixqp.gop = gop;
    h265_fixqp.src_frame_rate = frame_rate;
    h265_fixqp.dst_frame_rate = frame_rate;
    h265_fixqp.i_qp = 25; /* 25 is a number */
    h265_fixqp.p_qp = 30; /* 30 is a number */
    h265_fixqp.b_qp = 32; /* 32 is a number */
    venc_chn_attr->rc_attr.h265_fixqp = h265_fixqp;
}

static td_void sample_easycomm_venc_mjpeg_cbr_param_init(ot_venc_chn_attr *venc_chn_attr, td_u32 stats_time,
    td_u32 frame_rate)
{
    ot_venc_mjpeg_cbr mjpege_cbr;
    td_u32  pic_width;
    td_u32  pic_height;

    pic_width = venc_chn_attr->venc_attr.pic_width;
    pic_height = venc_chn_attr->venc_attr.pic_height;
    venc_chn_attr->rc_attr.rc_mode = OT_VENC_RC_MODE_MJPEG_CBR;
    mjpege_cbr.stats_time = stats_time;
    mjpege_cbr.src_frame_rate = frame_rate;
    mjpege_cbr.dst_frame_rate = frame_rate;
    mjpege_cbr.bit_rate = ((td_u64)20480 * (pic_width * pic_height)) / (1920 * 1080); /* 20480: 20M 1920, 1080: FHD */
    venc_chn_attr->rc_attr.mjpeg_cbr = mjpege_cbr;
}

static td_s32 sample_easycomm_venc_jpeg_param_init(ot_venc_chn_attr *venc_chn_attr)
{
    ot_venc_jpeg_attr jpeg_attr;
    jpeg_attr.dcf_en = TD_FALSE;
    jpeg_attr.mpf_cfg.large_thumbnail_num = 0;
    jpeg_attr.recv_mode = OT_VENC_PIC_RECV_SINGLE;

    venc_chn_attr->venc_attr.jpeg_attr = jpeg_attr;

    return TD_SUCCESS;
}


static td_s32 sample_easycomm_venc_mjpeg_param_init(ot_venc_chn_attr *venc_chn_attr,
    sample_comm_venc_chn_param *venc_create_chn_param)
{
    sample_rc rc_mode = venc_create_chn_param->rc_mode;
    td_u32 stats_time = venc_create_chn_param->stats_time;
    td_u32 frame_rate = venc_create_chn_param->frame_rate;

    if (rc_mode == SAMPLE_RC_FIXQP) {
        sample_easycomm_venc_mjpeg_fixqp_param_init(venc_chn_attr, frame_rate);
    } else if (rc_mode == SAMPLE_RC_CBR) {
        sample_easycomm_venc_mjpeg_cbr_param_init(venc_chn_attr, stats_time, frame_rate);
    } else if ((rc_mode == SAMPLE_RC_VBR) || (rc_mode == SAMPLE_RC_AVBR)) {
        if (rc_mode == SAMPLE_RC_AVBR) {
            sample_print("mjpege not support AVBR, so change rcmode to VBR!\n");
        }
        sample_easycomm_venc_mjpeg_vbr_param_init(venc_chn_attr, stats_time, frame_rate);
    } else {
        sample_print("can't support other mode(%d) in this version!\n", rc_mode);
        return TD_FAILURE;
    }
    return TD_SUCCESS;
}

static td_s32 sample_easycomm_venc_h264_param_init(ot_venc_chn_attr *chn_attr, sample_comm_venc_chn_param *chn_param)
{
    sample_rc rc_mode = chn_param->rc_mode;
    td_u32 gop = chn_param->gop;
    td_u32 stats_time = chn_param->stats_time;
    td_u32 frame_rate = chn_param->frame_rate;
    ot_pic_size size = chn_param->size;

    chn_attr->venc_attr.h264_attr.frame_buf_ratio = SAMPLE_FRAME_BUF_RATIO_MIN;
    if (rc_mode == SAMPLE_RC_ABR) {
        sample_easycomm_venc_h264_abr_param_init(chn_attr, gop, stats_time, frame_rate);
    } else if (rc_mode == SAMPLE_RC_CBR) {
        sample_easycomm_venc_h264_cbr_param_init(chn_attr, gop, stats_time, frame_rate);
    } else if (rc_mode == SAMPLE_RC_FIXQP) {
        sample_easycomm_venc_h264_fixqp_param_init(chn_attr, gop, frame_rate);
    } else if (rc_mode == SAMPLE_RC_VBR) {
        sample_easycomm_venc_h264_vbr_param_init(chn_attr, gop, stats_time, frame_rate);
    } else if (rc_mode == SAMPLE_RC_AVBR) {
        sample_easycomm_venc_h264_avbr_param_init(chn_attr, gop, stats_time, frame_rate);
    } else if (rc_mode == SAMPLE_RC_CVBR) {
        sample_easycomm_venc_h264_cvbr_param_init(chn_attr, gop, stats_time, frame_rate, size);
    } else if (rc_mode == SAMPLE_RC_QVBR) {
        sample_easycomm_venc_h264_qvbr_param_init(chn_attr, gop, stats_time, frame_rate);
    } else if (rc_mode == SAMPLE_RC_QPMAP) {
        sample_easycomm_venc_h264_qpmap_param_init(chn_attr, gop, frame_rate, stats_time);
    } else {
        sample_print("%s,%d,rc_mode(%d) not support\n", __FUNCTION__, __LINE__, rc_mode);
        return TD_FAILURE;
    }
    chn_attr->venc_attr.h264_attr.rcn_ref_share_buf_en = chn_param->is_rcn_ref_share_buf;

    return TD_SUCCESS;
}

static td_s32 sample_easycomm_venc_h265_param_init(ot_venc_chn_attr *chn_attr, sample_comm_venc_chn_param *chn_param)
{
    sample_rc rc_mode = chn_param->rc_mode;
    td_u32 gop = chn_param->gop;
    td_u32 stats_time = chn_param->stats_time;
    td_u32 frame_rate = chn_param->frame_rate;
    ot_pic_size size = chn_param->size;

    chn_attr->venc_attr.h265_attr.frame_buf_ratio = SAMPLE_FRAME_BUF_RATIO_MIN;
    if (rc_mode == SAMPLE_RC_ABR) {
        sample_easycomm_venc_h265_abr_param_init(chn_attr, gop, stats_time, frame_rate);
    } else if (rc_mode == SAMPLE_RC_CBR) {
        sample_easycomm_venc_h265_cbr_param_init(chn_attr, gop, stats_time, frame_rate);
    } else if (rc_mode == SAMPLE_RC_FIXQP) {
        sample_easycomm_venc_h265_fixqp_param_init(chn_attr, gop, frame_rate);
    } else if (rc_mode == SAMPLE_RC_VBR) {
        sample_easycomm_venc_h265_vbr_param_init(chn_attr, gop, stats_time, frame_rate);
    } else if (rc_mode == SAMPLE_RC_AVBR) {
        sample_easycomm_venc_h265_avbr_param_init(chn_attr, gop, stats_time, frame_rate);
    } else if (rc_mode == SAMPLE_RC_CVBR) {
        sample_easycomm_venc_h265_cvbr_param_init(chn_attr, gop, stats_time, frame_rate, size);
    } else if (rc_mode == SAMPLE_RC_QVBR) {
        sample_easycomm_venc_h265_qvbr_param_init(chn_attr, gop, stats_time, frame_rate);
    } else if (rc_mode == SAMPLE_RC_QPMAP) {
        sample_easycomm_venc_h265_qpmap_param_init(chn_attr, gop, frame_rate, stats_time);
    } else {
        sample_print("%s,%d,rc_mode(%d) not support\n", __FUNCTION__, __LINE__, rc_mode);
        return TD_FAILURE;
    }
    chn_attr->venc_attr.h265_attr.rcn_ref_share_buf_en = chn_param->is_rcn_ref_share_buf;

    return TD_SUCCESS;
}

static td_void sample_easycomm_venc_set_gop_attr(ot_payload_type type, ot_venc_chn_attr *chn_attr,
    ot_venc_gop_attr *gop_attr)
{
    if (type == OT_PT_MJPEG || type == OT_PT_JPEG) {
        chn_attr->gop_attr.gop_mode = OT_VENC_GOP_MODE_NORMAL_P;
        chn_attr->gop_attr.normal_p.ip_qp_delta = 0;
    } else {
        chn_attr->gop_attr = *gop_attr;
        if ((gop_attr->gop_mode == OT_VENC_GOP_MODE_BIPRED_B) && (type == OT_PT_H264)) {
            if (chn_attr->venc_attr.profile == 0) {
                chn_attr->venc_attr.profile = 1;

                sample_print("H.264 base profile not support BIPREDB, so change profile to main profile!\n");
            }
        }
    }
}

static td_s32 sample_easycomm_venc_channel_param_init(sample_comm_venc_chn_param *chn_param, ot_venc_chn_attr *chn_attr)
{
    td_s32 ret;
    ot_venc_gop_attr *gop_attr = &chn_param->gop_attr;
    td_u32 profile = chn_param->profile;
    ot_payload_type type = chn_param->type;
    ot_size venc_size = chn_param->venc_size;

    chn_attr->venc_attr.type = type;
    chn_attr->venc_attr.max_pic_width = venc_size.width;
    chn_attr->venc_attr.max_pic_height = venc_size.height;
    chn_attr->venc_attr.pic_width = venc_size.width;   /* the picture width */
    chn_attr->venc_attr.pic_height = venc_size.height; /* the picture height */

    if (type == OT_PT_MJPEG || type == OT_PT_JPEG) {
        chn_attr->venc_attr.buf_size =
            OT_ALIGN_UP(venc_size.width, 16) * OT_ALIGN_UP(venc_size.height, 16) * 4; /* 16 4 is a number */
    } else {
        chn_attr->venc_attr.buf_size =
            OT_ALIGN_UP(venc_size.width * venc_size.height * 3 / 4, 64); /*  3  4 64 is a number */
    }
    chn_attr->venc_attr.profile = profile;
    chn_attr->venc_attr.is_by_frame = TD_TRUE; /* get stream mode is slice mode or frame mode? */

    if (gop_attr->gop_mode == OT_VENC_GOP_MODE_SMART_P) {
        chn_param->stats_time = gop_attr->smart_p.bg_interval / chn_param->gop;
    }

    switch (type) {
        case OT_PT_H265:
            ret = sample_easycomm_venc_h265_param_init(chn_attr, chn_param);
            break;

        case OT_PT_H264:
            ret = sample_easycomm_venc_h264_param_init(chn_attr, chn_param);
            break;

        case OT_PT_MJPEG:
            ret = sample_easycomm_venc_mjpeg_param_init(chn_attr, chn_param);
            break;

        case OT_PT_JPEG:
            ret = sample_easycomm_venc_jpeg_param_init(chn_attr);
            break;

        default:
            sample_print("can't support this type (%d) in this version!\n", type);
            return OT_ERR_VENC_NOT_SUPPORT;
    }

    sample_easycomm_venc_set_gop_attr(type, chn_attr, gop_attr);

    return ret;
}

td_s32 sample_easycomm_venc_create(ot_venc_chn venc_chn, sample_comm_venc_chn_param *chn_param)
{
    td_s32 ret;
    ot_venc_chn_attr venc_chn_attr;
    ot_pic_size size = chn_param->size;

    if ((td_s32)size != -1) {
        if (sample_comm_sys_get_pic_size(size, &chn_param->venc_size) != TD_SUCCESS) {
            sample_print("get picture size failed!\n");
            return TD_FAILURE;
        }
    }

    /* step 1:  create venc channel */
    if ((ret = sample_easycomm_venc_channel_param_init(chn_param, &venc_chn_attr)) != TD_SUCCESS) {
        sample_print("venc_channel_param_init failed!\n");
        return ret;
    }

    if ((ret = ss_mpi_venc_create_chn(venc_chn, &venc_chn_attr)) != TD_SUCCESS) {
        sample_print("ss_mpi_venc_create_chn [%d] failed with %#x! ===\n", venc_chn, ret);
        return ret;
    }

    if (chn_param->type == OT_PT_JPEG) {
        return TD_SUCCESS;
    }

    if ((ret = sample_easycomm_venc_close_reencode(venc_chn)) != TD_SUCCESS) {
        ss_mpi_venc_destroy_chn(venc_chn);
        return ret;
    }

    return TD_SUCCESS;
}

/*
 * function : start venc stream mode
 * note     : rate control parameter need adjust, according your case.
 */
td_s32 sample_easycomm_venc_start(ot_venc_chn venc_chn, sample_comm_venc_chn_param *chn_param)
{
    td_s32 ret;
    ot_venc_start_param start_param;

    /* step 1: create encode chnl */
    if ((ret = sample_easycomm_venc_create(venc_chn, chn_param)) != TD_SUCCESS) {
        sample_print("sample_easycomm_venc_create failed with%#x! \n", ret);
        return TD_FAILURE;
    }
    /* step 2:  start recv venc pictures */
    start_param.recv_pic_num = -1;
    if ((ret = ss_mpi_venc_start_chn(venc_chn, &start_param)) != TD_SUCCESS) {
        sample_print("ss_mpi_venc_start_recv_pic failed with%#x! \n", ret);
        return TD_FAILURE;
    }
    return TD_SUCCESS;
}

/* function : stop venc (stream mode-- H264, MJPEG) */
td_s32 sample_easycomm_venc_stop(ot_venc_chn venc_chn)
{
    td_s32 ret;

    /* stop venc chn */
    ret = ss_mpi_venc_stop_chn(venc_chn);
    if (ret != TD_SUCCESS) {
        sample_print("ss_mpi_venc_stop_chn vechn[%d] failed with %#x!\n", venc_chn, ret);
    }

    /* distroy venc channel */
    ret = ss_mpi_venc_destroy_chn(venc_chn);
    if (ret != TD_SUCCESS) {
        sample_print("ss_mpi_venc_destroy_chn vechn[%d] failed with %#x!\n", venc_chn, ret);
        return TD_FAILURE;
    }
    return TD_SUCCESS;
}


#define SAMPLE_VENC_BLOCK_WIDTH 16
#define SAMPLE_VENC_BLOCK_HEIGHT 16
#define SAMPLE_VENC_ONE_BYTE_BLOCKS 4
#define SAMPLE_VENC_ONE_BLOCK_BITS 2
#define SAMPLE_VENC_MAX_JPEG_ROI_LEVEL 3



#define SAMPLE_VENC_ROIMAP_MAX_CHN 2

static td_s32 sample_easycomm_set_file_name(td_s32 index, ot_venc_chn venc_chn,
    sample_easycomm_venc_stream_proc_info *stream_proc_info)
{
    if (snprintf_s(stream_proc_info->file_name[index], FILE_NAME_LEN, FILE_NAME_LEN - 1, "./") < 0) {
        return TD_FAILURE;
    }

    if (realpath(stream_proc_info->file_name[index], stream_proc_info->real_file_name[index]) == TD_NULL) {
        sample_print("chn[%d] stream file path error\n", venc_chn);
        return TD_FAILURE;
    }

    if (snprintf_s(stream_proc_info->real_file_name[index], FILE_NAME_LEN, FILE_NAME_LEN - 1,
        "stream_chn%d%s", index, stream_proc_info->file_postfix) < 0) {
        return TD_FAILURE;
    }

    return TD_SUCCESS;
}

static td_void sample_easycomm_close_save_file(sample_easycomm_venc_stream_proc_info *stream_proc_info)
{
    td_s32 i;

    for (i = 0; i < stream_proc_info->chn_total && i < OT_VENC_MAX_CHN_NUM; i++) {
        if (stream_proc_info->file[i] != TD_NULL) {
            fclose(stream_proc_info->file[i]);
            stream_proc_info->file[i] = TD_NULL;
        }
    }
}

static td_s32 sample_easycomm_set_name_save_stream(sample_easycomm_venc_stream_proc_info *stream_proc_info,
    ot_venc_stream_buf_info *stream_buf_info, ot_payload_type *payload_type,
    sample_venc_getstream_para *para, td_s32 venc_max_chn)
{
    td_s32 i, ret, fd;
    ot_venc_chn_attr venc_chn_attr;
    ot_unused(venc_max_chn);

    for (i = 0; (i < stream_proc_info->chn_total) && (i < OT_VENC_MAX_CHN_NUM); i++) {
        /* decide the stream file name, and open file to save stream */
        ot_venc_chn venc_chn = para->venc_chn[i];
        ret = ss_mpi_venc_get_chn_attr(venc_chn, &venc_chn_attr);
        if (ret != TD_SUCCESS) {
            sample_print("ss_mpi_venc_get_chn_attr chn[%d] failed with %#x!\n", venc_chn, ret);
            return SAMPLE_RETURN_NULL;
        }
        payload_type[i] = venc_chn_attr.venc_attr.type;

        ret = sample_easycomm_venc_get_file_postfix(payload_type[i], stream_proc_info->file_postfix,
            sizeof(stream_proc_info->file_postfix));
        if (ret != TD_SUCCESS) {
            sample_print("sample_easycomm_venc_get_file_postfix [%d] failed with %#x!\n",
                venc_chn_attr.venc_attr.type, ret);
            return SAMPLE_RETURN_NULL;
        }

        if (payload_type[i] != OT_PT_JPEG) {
            ret = sample_easycomm_set_file_name(i, venc_chn, stream_proc_info);
            if (ret != TD_SUCCESS) {
                return ret;
            }

            stream_proc_info->file[i] = fopen(stream_proc_info->real_file_name[i], "wb");
            if (!stream_proc_info->file[i]) {
                sample_print("open file[%s] failed! (%d:%s)\n", stream_proc_info->real_file_name[i],
                    errno, strerror(errno));
                return SAMPLE_RETURN_FAILURE;
            }
            fd = fileno(stream_proc_info->file[i]);
            fchmod(fd, S_IRUSR | S_IWUSR);
        }
        /* set venc fd. */
        stream_proc_info->venc_fd[i] = ss_mpi_venc_get_fd(i);
        if (stream_proc_info->venc_fd[i] < 0) {
            sample_print("ss_mpi_venc_get_fd failed with %#x!\n", stream_proc_info->venc_fd[i]);
            return SAMPLE_RETURN_NULL;
        }

        if (stream_proc_info->maxfd <= stream_proc_info->venc_fd[i]) {
            stream_proc_info->maxfd = stream_proc_info->venc_fd[i];
        }

        ret = ss_mpi_venc_get_stream_buf_info(i, &stream_buf_info[i]);
        if (ret != TD_SUCCESS) {
            sample_print("ss_mpi_venc_get_stream_buf_info failed with %#x!\n", ret);
            return SAMPLE_RETURN_FAILURE;
        }
    }

    return TD_SUCCESS;
}

static int32_t sample_heif_create(td_s32 index, const sample_easycomm_venc_stream_proc_info *stream_proc_info,
    heif_handle *hdl)
{
    heif_config config;
    if (snprintf_s(config.file_desc.input.url, FILE_NAME_LEN, FILE_NAME_LEN - 1,
        "./stream_chn%d_%u%s", index, stream_proc_info->picture_cnt[index], ".heic") < 0) {
        return SAMPLE_RETURN_NULL;
    }
    config.file_desc.file_type = HEIF_FILE_TYPE_URL;
    config.config_type = HEIF_CONFIG_MUXER;
    config.muxer_config.is_grid = false;
    config.muxer_config.row_image_num = 1;
    config.muxer_config.column_image_num = 1;
    config.muxer_config.format_profile = HEIF_PROFILE_HEIC;
    return heif_create(hdl, &config);
}

static td_s32 sample_easycomm_save_h265_to_heic(td_s32 index, const sample_easycomm_venc_stream_proc_info *stream_proc_info,
    const ot_venc_stream *stream)
{
    td_u32 i;
    td_u32 total_len = 0;
    td_s32 has_key = 0;
    for (i = 0; i < stream->pack_cnt; i++) {
        if (stream->pack[i].data_type.h265_type == OT_VENC_H265_NALU_IDR_SLICE) {
            has_key = 1;
        }
        total_len += stream->pack[i].len - stream->pack[i].offset;
    }
    if (total_len > 0 && has_key == 1) {
        heif_handle handle = NULL;
        td_s32 ret = sample_heif_create(index, stream_proc_info, &handle);
        if (ret != 0) {
            sample_print("HeifCreate error ret:%d\n", ret);
        }
        td_u8 *data_buffer = (td_u8 *)malloc(total_len);
        if (data_buffer == NULL) {
            sample_print("malloc error\n");
            heif_destroy(handle);
            return SAMPLE_RETURN_NULL;
        }
        td_u32 write_len = 0;
        for (i = 0; i < stream->pack_cnt; i++) {
            if (memcpy_s(data_buffer + write_len, total_len - write_len,
                stream->pack[i].addr + stream->pack[i].offset, stream->pack[i].len - stream->pack[i].offset) != EOK) {
                sample_print("memcpy_s failed\n");
            }
            write_len += stream->pack[i].len;
        }
        heif_image_item item = {0};
        item.timestamp = -1;
        item.data = data_buffer;
        item.length = write_len;
        item.key_frame = true;
        ret = heif_write_master_image(handle, 0, &item, 1);
        if (data_buffer != NULL) {
            free(data_buffer);
        }
        heif_destroy(handle);
        return ret;
    }
    return 0;
}

static td_s32 sample_easycomm_save_frame_to_file(td_s32 index, sample_easycomm_venc_stream_proc_info *stream_proc_info,
    ot_venc_stream *stream, ot_venc_stream_buf_info *stream_buf_info, ot_payload_type *payload_type)
{
    td_s32 ret, fd;
    if (payload_type[index] == OT_PT_JPEG) {
        if (snprintf_s(stream_proc_info->file_name[index], FILE_NAME_LEN, FILE_NAME_LEN - 1, "./") < 0) {
            free(stream->pack);
            return SAMPLE_RETURN_NULL;
        }
        if (realpath(stream_proc_info->file_name[index], stream_proc_info->real_file_name[index]) == TD_NULL) {
            free(stream->pack);
            sample_print("chn[%d] stream file path error\n", stream_proc_info->venc_chn);
            return SAMPLE_RETURN_NULL;
        }

        if (snprintf_s(stream_proc_info->real_file_name[index], FILE_NAME_LEN, FILE_NAME_LEN - 1,
            "stream_chn%d_%u%s", index, stream_proc_info->picture_cnt[index], stream_proc_info->file_postfix) < 0) {
            free(stream->pack);
            return SAMPLE_RETURN_NULL;
        }
        stream_proc_info->file[index] = fopen(stream_proc_info->real_file_name[index], "wb");
        if (!stream_proc_info->file[index]) {
            free(stream->pack);
            sample_print("open file err!\n");
            return SAMPLE_RETURN_NULL;
        }
        fd = fileno(stream_proc_info->file[index]);
        fchmod(fd, S_IRUSR | S_IWUSR);
    }

    if (payload_type[index] == OT_PT_H265 && stream_proc_info->save_heif == TD_TRUE) {
        (td_void)sample_easycomm_save_h265_to_heic(index, stream_proc_info, stream);
    }
#ifndef __LITEOS__
    ot_unused(stream_buf_info);
    //ret = sample_easycomm_venc_save_stream(stream_proc_info->file[index], stream);//调用fwrite本地保存文件
	
////////////////////////////swann这里添加rtsp发送//////////////////////////////
	//rtsp发送
	td_u8 * pStremData = NULL;int nSize = 0;td_u32 j;
	if(index==0){//发送venc通道0
	    for (j = 0; j < stream->pack_cnt; j++) {
	        //(td_void)fwrite(stream->pack[j].addr + stream->pack[j].offset, stream->pack[j].len - stream->pack[j].offset, 1, fd);
	        if(stream->pack[j].data_type.h264_type == OT_VENC_H264_NALU_SEI) continue;//暂时去掉SEI帧
			pStremData = stream->pack[j].addr + stream->pack[j].offset;
			nSize = stream->pack[j].len - stream->pack[j].offset;
			if (g_exit_requested) {
				break;
			}
			if(g_rtsplive && session)
			{
				rtsp_sever_tx_video(g_rtsplive,session,pStremData,nSize,stream->pack[j].pts);
			}
	    }
	}
	ret = TD_SUCCESS;
//////////////////////////////////////////////////////////////////////////////

	
	
#else
    ret = sample_easycomm_venc_save_stream_phys_addr(stream_proc_info->file[index], &stream_buf_info[index], stream);
#endif
    if (ret != TD_SUCCESS) {
        free(stream->pack);
        stream->pack = TD_NULL;
        sample_print("save stream failed!\n");
        return SAMPLE_RETURN_BREAK;
    }

    return TD_SUCCESS;
}

static td_s32 sample_easycomm_get_stream_from_one_channl(sample_easycomm_venc_stream_proc_info *stream_proc_info, td_s32 index,
    ot_venc_stream_buf_info *stream_buf_info, ot_payload_type *payload_type)
{
    td_s32 ret;
    ot_venc_stream stream;
    ot_venc_chn_status stat;

    /* step 2.1 : query how many packs in one-frame stream. */
    if (memset_s(&stream, sizeof(stream), 0, sizeof(stream)) != EOK) {
        printf("call memset_s error\n");
    }

    ret = ss_mpi_venc_query_status(index, &stat);
    if (ret != TD_SUCCESS) {
        sample_print("ss_mpi_venc_query_status chn[%d] failed with %#x!\n", index, ret);
        return SAMPLE_RETURN_BREAK;
    }

    if (stat.cur_packs == 0) {
        sample_print("NOTE: current  frame is TD_NULL!\n");
        return SAMPLE_RETURN_CONTINUE;
    }
    /* step 2.3 : malloc corresponding number of pack nodes. */
    stream.pack = (ot_venc_pack *)malloc(sizeof(ot_venc_pack) * stat.cur_packs);
    if (stream.pack == TD_NULL) {
        sample_print("malloc stream pack failed!\n");
        return SAMPLE_RETURN_BREAK;
    }

    /* step 2.4 : call mpi to get one-frame stream */
    stream.pack_cnt = stat.cur_packs;
    ret = ss_mpi_venc_get_stream(index, &stream, TD_TRUE);
    if (ret != TD_SUCCESS) {
        free(stream.pack);
        stream.pack = TD_NULL;
        sample_print("ss_mpi_venc_get_stream failed with %#x!\n", ret);
        return SAMPLE_RETURN_BREAK;
    }

    /* step 2.5 : save frame to file */
    ret = sample_easycomm_save_frame_to_file(index, stream_proc_info, &stream, stream_buf_info, payload_type);
    if (ret != TD_SUCCESS) {
        return ret;
    }
    /* step 2.6 : release stream */
    ret = ss_mpi_venc_release_stream(index, &stream);
    if (ret != TD_SUCCESS) {
        sample_print("ss_mpi_venc_release_stream failed!\n");
        free(stream.pack);
        stream.pack = TD_NULL;
        return SAMPLE_RETURN_BREAK;
    }

    /* step 2.7 : free pack nodes */
    free(stream.pack);
    stream.pack = TD_NULL;
    stream_proc_info->picture_cnt[index]++;
    if (payload_type[index] == OT_PT_JPEG) {
        fclose(stream_proc_info->file[index]);
        stream_proc_info->file[index] = TD_NULL;
    }
    return TD_SUCCESS;
}


static td_void sample_easycomm_fd_isset(sample_easycomm_venc_stream_proc_info *stream_proc_info, fd_set *read_fds,
    ot_venc_stream_buf_info *stream_buf_info, ot_payload_type *payload_type, sample_venc_getstream_para *para)
{
    td_s32 i, ret;

    for (i = 0; (i < stream_proc_info->chn_total) && (i < OT_VENC_MAX_CHN_NUM); i++) {
        if (FD_ISSET(stream_proc_info->venc_fd[i], read_fds)) {
            stream_proc_info->venc_chn = para->venc_chn[i];
            ret = sample_easycomm_get_stream_from_one_channl(stream_proc_info, i, stream_buf_info, payload_type);
            if (ret == SAMPLE_RETURN_CONTINUE) {
                continue;
            } else if (ret == SAMPLE_RETURN_BREAK) {
                break;
            }
        }
    }
}

/* get stream from each channels and save them */
td_void *sample_easycomm_venc_get_venc_stream_proc(td_void *p)
{
    td_s32 i, ret;
    fd_set read_fds;
    struct timeval timeout_val = {0};
    ot_payload_type payload_type[OT_VENC_MAX_CHN_NUM] = {0};
    ot_venc_stream_buf_info stream_buf_info[OT_VENC_MAX_CHN_NUM];
    sample_venc_getstream_para *para = (sample_venc_getstream_para *)p;
    sample_easycomm_venc_stream_proc_info *stream_proc_info = malloc(sizeof(sample_easycomm_venc_stream_proc_info));

    if (stream_proc_info == TD_NULL || memset_s(stream_proc_info, sizeof(sample_easycomm_venc_stream_proc_info), 0,
        sizeof(sample_easycomm_venc_stream_proc_info)) != EOK) {
        goto FREE;
    }

    prctl(PR_SET_NAME, "get_venc_stream", 0, 0, 0);

    stream_proc_info->chn_total = para->cnt;
    stream_proc_info->save_heif = para->save_heif;

    ret = sample_easycomm_set_name_save_stream(stream_proc_info, stream_buf_info, payload_type, para, OT_VENC_MAX_CHN_NUM);
    if (ret == SAMPLE_RETURN_NULL) {
        goto CLOSE_FILE;
    } else if (ret == SAMPLE_RETURN_FAILURE) {
        goto CLOSE_FILE;
    }

    pthread_setcancelstate(PTHREAD_CANCEL_ENABLE, TD_NULL);
    pthread_setcanceltype(PTHREAD_CANCEL_DEFERRED, TD_NULL);

    while (para->thread_start == TD_TRUE && !g_exit_requested) {
        FD_ZERO(&read_fds);
        for (i = 0; (i < stream_proc_info->chn_total) && (i < OT_VENC_MAX_CHN_NUM); i++) {
            FD_SET(stream_proc_info->venc_fd[i], &read_fds);
        }

        timeout_val.tv_sec = 2; /* 2 is a number */
        timeout_val.tv_usec = 0;
        ret = select(stream_proc_info->maxfd + 1, &read_fds, TD_NULL, TD_NULL, &timeout_val);
        if (ret < 0) {
            if (errno == EINTR && g_exit_requested) {
                break;
            }
            sample_print("select failed! errno=%d\n", errno);
            break;
        } else if (ret == 0) {
            if (g_exit_requested || para->thread_start != TD_TRUE) {
                break;
            }
            continue;
        } else {
            if (g_exit_requested || para->thread_start != TD_TRUE) {
                break;
            }
            sample_easycomm_fd_isset(stream_proc_info, &read_fds, stream_buf_info, payload_type, para);
        }
    }

CLOSE_FILE:
    /* step 3 : close save-file */
    sample_easycomm_close_save_file(stream_proc_info);
FREE:
    if (stream_proc_info != TD_NULL) {
        free(stream_proc_info);
    }

    para->thread_start = TD_FALSE;
    return TD_NULL;
}

/* *****************************************************************************
* function : bitrate_auto
 * **************************************************************************** */
#define SAMPLE_VENC_WIDHT 640
#define SAMPLE_VENC_HEIGHT 480
#define SAMPLE_VENC_NUM 5
#define SAMPLE_VENC_FG_TYPE 5
#define QUERY_SLEEP 1000
#define WAIT_GET_STREAM_THREAD_START_SLEEP 50
#define WAIT_GET_STREAM_THREAD_START_TIMES 40


td_void sample_easycomm_venc_set_save_heif(td_bool save_heif)
{
    g_para.save_heif = save_heif;
    sample_print("set save heif flag: %d!\n", save_heif);
}

/* start get venc stream process thread */
td_s32 sample_easycomm_venc_start_get_stream(ot_venc_chn *venc_chn, td_s32 cnt)
{
    td_s32 i, ret;
    td_s32 wait_times = WAIT_GET_STREAM_THREAD_START_TIMES;

    g_para.thread_start = TD_TRUE;
    g_para.cnt = cnt;
    if (cnt >= OT_VENC_MAX_CHN_NUM) {
        sample_print("input count invalid\n");
        return TD_FAILURE;
    }
    for (i = 0; (i < cnt) && (i < OT_VENC_MAX_CHN_NUM); i++) {
        g_para.venc_chn[i] = venc_chn[i];
    }

    ret = pthread_create(&g_venc_pid, 0, sample_easycomm_venc_get_venc_stream_proc, (td_void *)&g_para);
    if (ret < 0) {
        sample_print("create threa failed !!! ret %d\n", ret);
        return TD_FAILURE;
    }

    while (g_para.thread_start && wait_times-- > 0 && !g_exit_requested) {
        usleep(WAIT_GET_STREAM_THREAD_START_SLEEP);
        if (!g_para.thread_start) {
            sample_print("start thread failed !!!\n");
            pthread_join(g_venc_pid, 0);
            return TD_FAILURE;
        }
    }

    return ret;
}

/* stop get venc stream process. */
td_s32 sample_easycomm_venc_stop_get_stream(td_s32 chn_num)
{
    td_s32 i;
    if (g_para.thread_start == TD_TRUE) {
        g_para.thread_start = TD_FALSE;
        pthread_cancel(g_venc_pid);
        pthread_join(g_venc_pid, 0);
    }

    for (i = 0; i < chn_num; i++) {
        if (ss_mpi_venc_stop_chn(i) != TD_SUCCESS) {
            sample_print("chn %d ss_mpi_venc_stop_recv_pic failed!\n", i);
            return TD_FAILURE;
        }
    }

    return TD_SUCCESS;
}


 #endif
/* *****************************************************************************
 * sample_comm_venc.c and rename
 * **************************************************************************** */
 
/*
  Copyright (c), 2001-2024, Shenshu Tech. Co., Ltd.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <pthread.h>
#include <signal.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/ioctl.h>

#include "securec.h"
#include "sample_svp_npu_process.h"
#include "sample_svp_npu_process_motr.h"
#include "audio_alert.h"


#define SAMPLE_SVP_NPU_ARG_NUM_TWO 2
#define SAMPLE_SVP_NPU_ARG_NUM_THREE 3
#define SAMPLE_SVP_NPU_ARG_IDX_TWO 2
#define SAMPLE_SVP_NPU_CMP_STR_NUM 2
#define SAMPLE_SVP_NPU_ARG_STR_LEN 2
#define SAMPLE_SVP_NPU_ARG_IDX_BASE 10

static char **g_npu_cmd_argv = TD_NULL;

/*
 * function : to process abnormal case
 */
#ifndef __LITEOS__
static td_void sample_svp_npu_handle_sig(td_s32 signo)
{
    if (signo == SIGINT || signo == SIGTERM) {
        g_exit_requested = 1;
        g_para.thread_start = TD_FALSE;
        sample_svp_npu_acl_handle_sig();
    }
}
#endif

/*
 * function : show usage
 */
static td_void sample_svp_npu_usage(const td_char *prg_name)
{
    printf("Usage : %s <index>\n", prg_name);
    printf("index:\n");
    printf("\t 8) [1, 5],yolov1 to yolov5; [7, 8],yolov7 to yolov8.(VI->VPSS->SVP_NPU->VGS->VO)\n");
}

static td_s32 sample_svp_npu_case_with_one_arg(td_char idx)
{
    td_s32 ret = TD_SUCCESS;

    switch (idx) {
	/*
        case '0':
            sample_svp_npu_acl_resnet50();
            break;
        case '1':
            sample_svp_npu_acl_resnet50_multi_thread();
            break;
        case '2':
            sample_svp_npu_acl_resnet50_dynamic_batch();
            break;
        case '3':
            sample_svp_npu_acl_lstm();
            break;
        case '4':
            sample_svp_npu_acl_rfcn();
            break;
        case '5':
            sample_svp_npu_acl_event();
            break;
        case '6':
            sample_svp_npu_acl_aipp();
            break;
        case '7':
            sample_svp_npu_acl_preemption();
            break;
        case 'a':
        case 'A':
            sample_svp_npu_acl_e2e_hrnet();
            break;
	*/
        default:
            ret = TD_FAILURE;
            break;
    }
    return ret;
}

static td_s32 sample_svp_npu_case_with_two_args(td_char idx1, td_char idx2)
{
    td_s32 ret = TD_SUCCESS;
    if ((idx1 == '8' || idx1 == '9') && !((idx2 <= '5' && idx2 >= '1') || (idx2 >= '7' && idx2 <= '8'))) {
        return TD_FAILURE;
    }
    if ((idx1 == 'B' || idx1 == 'b') && !(idx2 >= '0' && idx2 <= '1')) {
        return TD_FAILURE;
    }
    switch (idx1) {
        case '8':
            sample_svp_npu_acl_e2e_yolo(idx2 - '0');
            break;
        case '9':
            sample_svp_npu_acl_yolo(idx2 - '0');
            break;
        case 'B':
        case 'b':
            sample_svp_npu_acl_motr(idx2 - '0');
            break;
        default:
            ret = TD_FAILURE;
            break;
    }
    return ret;
}

/*
 * function : svp npu sample
 */
#ifdef __LITEOS__
int app_main(int argc, char *argv[])
#else
int main(int argc, char *argv[])
#endif
{
    td_s32 ret;
    size_t argv1_len, argv2_len;
#ifndef __LITEOS__
    struct sigaction sa;
#endif
    if (argc < SAMPLE_SVP_NPU_ARG_NUM_TWO || argc > SAMPLE_SVP_NPU_ARG_NUM_THREE) {
        sample_svp_npu_usage(argv[0]);
        return TD_FAILURE;
    }

    if (!strncmp(argv[1], "-h", SAMPLE_SVP_NPU_CMP_STR_NUM)) {
        sample_svp_npu_usage(argv[0]);
        return TD_SUCCESS;
    }

    g_npu_cmd_argv = argv;
    g_exit_requested = 0;
#ifndef __LITEOS__
    (td_void)memset_s(&sa, sizeof(struct sigaction), 0, sizeof(struct sigaction));
    sa.sa_handler = sample_svp_npu_handle_sig;
    sa.sa_flags = 0;
    sigaction(SIGINT, &sa, NULL);
    sigaction(SIGTERM, &sa, NULL);
    signal(SIGPIPE, SIG_IGN);
#endif
    argv1_len = strlen(argv[1]);
    if (argv1_len != 1) {
        sample_svp_npu_usage(argv[0]);
        return TD_FAILURE;
    }
    if (argc == SAMPLE_SVP_NPU_ARG_NUM_THREE) {
        argv2_len = strlen(argv[SAMPLE_SVP_NPU_ARG_IDX_TWO]);
        if (argv2_len != 1) {
            sample_svp_npu_usage(argv[0]);
            return TD_FAILURE;
        }
    }
////////////////////////////////////////////////////////////////////////////////////////////////
	g_rtsplive = create_rtsp_demo(554);//554端口创建rtspserver
	session= create_rtsp_session(g_rtsplive,"/test.264");//创建rtsp会话 rtsp://[IP]/test.264
	/* 音频提示初始化：只做一次 */
	ret = audio_alert_init();
	if (ret != TD_SUCCESS) {
    	   printf("[ALERT] audio_alert_init failed: 0x%x\n", ret);
    // 这里不要 return，允许系统继续跑，只是没有语音
	}	
////////////////////////////////////////////////////////////////////////////////////////////////
    if (argc == SAMPLE_SVP_NPU_ARG_NUM_TWO) {
        ret = sample_svp_npu_case_with_one_arg(*argv[1]);
    } else {
        ret = sample_svp_npu_case_with_two_args(*argv[1], *argv[SAMPLE_SVP_NPU_ARG_IDX_TWO]);
    }
    /*if (ret != TD_SUCCESS) {
        sample_svp_npu_usage(argv[0]);
        return TD_FAILURE;
    }

    return 0;*/
     if (ret != TD_SUCCESS) {
        sample_svp_npu_usage(argv[0]);
        ret = TD_FAILURE;
    }

    /* ===== 统一退出清理 ===== */
    audio_alert_deinit();

    if (session) {
        rtsp_del_session(session);
        session = NULL;
    }
    if (g_rtsplive) {
        rtsp_del_demo(g_rtsplive);
        g_rtsplive = NULL;
    }

    return (ret == TD_SUCCESS) ? 0 : TD_FAILURE;
}

