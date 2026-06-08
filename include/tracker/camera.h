/**
 * @file camera.h
 * @brief 摄像头人体检测
 *
 * 硬件: 树莓派 CSI 摄像头 (OV5647 / IMX219)
 *
 * 检测方法:
 *   方案一 (轻量): HOG + SVM 人体检测 — CPU 够用
 *   方案二 (更轻): 背景差分 + 轮廓 — 最快
 *
 * 输出: 目标在画面中的水平/垂直偏移 (-1~+1)
 *       -1 = 最左/最上, 0 = 正中, +1 = 最右/最下
 */

#ifndef PACER_CAMERA_H
#define PACER_CAMERA_H

#include <stdint.h>
#include <stdbool.h>

/* 检测结果 */
typedef struct {
    float x;            /* 水平偏移 -1(左) ~ +1(右), 0=正中 */
    float y;            /* 垂直偏移 -1(上) ~ +1(下) */
    float width;        /* 目标宽度占比 0~1 (越大越近) */
    float height;       /* 目标高度占比 0~1 */
    float area;         /* 面积占比 (判断距离) */
    bool  detected;     /* 是否检测到人体 */
    uint64_t timestamp; /* 时间戳 (us) */
} camera_target_t;

/* 检测方法 */
typedef enum {
    CAM_DETECT_HOG,         /* HOG+SVM, 准确但慢 (~5 FPS) */
    CAM_DETECT_CONTOUR,     /* 轮廓检测, 快但不准 (~15 FPS) */
} camera_detect_method_t;

/* 配置 */
typedef struct {
    int device_index;                /* /dev/videoN 的 N */
    int width;                       /* 采集分辨率宽 */
    int height;                      /* 采集分辨率高 */
    int fps;                         /* 目标帧率 */
    camera_detect_method_t method;   /* 检测方法 */
} camera_config_t;

#define CAMERA_CONFIG_DEFAULT { \
    .device_index = 0,          \
    .width        = 320,        \
    .height       = 240,        \
    .fps          = 15,         \
    .method       = CAM_DETECT_CONTOUR \
}

/* ---------- 生命周期 ---------- */

int  camera_init(const camera_config_t *cfg);
void camera_deinit(void);

/* ---------- 检测 ---------- */

/**
 * @brief 采集一帧并检测
 * @param out 输出目标位置
 * @return 0=成功, -1=采集失败, -2=无目标
 */
int  camera_detect(camera_target_t *out);

/**
 * @brief 获取最近一次检测结果
 */
camera_target_t camera_get_last(void);

#endif /* PACER_CAMERA_H */
