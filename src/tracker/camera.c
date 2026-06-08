/**
 * @file camera.c
 * @brief 摄像头人体检测实现
 *
 * 使用 OpenCV:
 *   - 背景差分 + 轮廓检测 (快速, 15+ FPS)
 *   - HOG + SVM (准确, 5 FPS)
 *
 * 输出目标在画面中的归一化位置 (-1~+1)
 */

#include "tracker/camera.h"

#ifdef USE_OPENCV
#include <opencv2/opencv.hpp>
#include <opencv2/objdetect.hpp>
#endif

#include <stdio.h>
#include <string.h>
#include <time.h>

/* ================ 内部状态 ================ */

static struct {
    camera_config_t     cfg;
    camera_target_t     last;
    bool                ready;

#ifdef USE_OPENCV
    cv::VideoCapture    cap;
    cv::HOGDescriptor   hog;
    cv::Mat             bg_model;       /* 背景模型 */
    bool                bg_initialized;
#endif
} g;

static uint64_t now_us(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000ULL + (uint64_t)ts.tv_nsec / 1000;
}

/* ================ 公开接口 ================ */

int camera_init(const camera_config_t *cfg)
{
    g.cfg = *cfg;
    g.ready = false;

#ifdef USE_OPENCV
    /* 打开摄像头 */
    g.cap.open(cfg->device_index);
    if (!g.cap.isOpened()) {
        fprintf(stderr, "[CAMERA] cannot open /dev/video%d\n", cfg->device_index);
        return -1;
    }

    g.cap.set(cv::CAP_PROP_FRAME_WIDTH,  cfg->width);
    g.cap.set(cv::CAP_PROP_FRAME_HEIGHT, cfg->height);
    g.cap.set(cv::CAP_PROP_FPS,          cfg->fps);

    /* 初始化 HOG 人体检测器 */
    if (cfg->method == CAM_DETECT_HOG) {
        g.hog.setSVMDetector(cv::HOGDescriptor::getDefaultPeopleDetector());
        printf("[CAMERA] HOG detector loaded\n");
    }

    g.bg_initialized = false;

#else
    printf("[CAMERA] compiled without OpenCV — camera disabled\n");
    printf("[CAMERA] rebuild with: cmake -DUSE_OPENCV=ON ..\n");
    return -1;
#endif

    g.ready = true;
    printf("[CAMERA] init: %dx%d @ %dfps, method=%s\n",
           cfg->width, cfg->height, cfg->fps,
           cfg->method == CAM_DETECT_HOG ? "HOG" : "CONTOUR");
    return 0;
}

void camera_deinit(void)
{
#ifdef USE_OPENCV
    if (g.cap.isOpened()) g.cap.release();
#endif
    g.ready = false;
    printf("[CAMERA] deinit\n");
}

int camera_detect(camera_target_t *out)
{
    if (!g.ready) return -1;

#ifdef USE_OPENCV

    cv::Mat frame;
    g.cap >> frame;
    if (frame.empty()) return -1;

    int w = frame.cols;
    int h = frame.rows;
    float cx = w / 2.0f;
    float cy = h / 2.0f;

    memset(out, 0, sizeof(*out));
    out->timestamp = now_us();
    out->detected  = false;

    if (g.cfg.method == CAM_DETECT_CONTOUR) {
        /* ---- 轮廓检测 (快速) ---- */

        cv::Mat gray, fg_mask, blurred;

        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
        cv::GaussianBlur(gray, blurred, cv::Size(5, 5), 0);

        /* 背景差分 */
        if (!g.bg_initialized) {
            blurred.copyTo(g.bg_model);
            g.bg_initialized = true;
            return -2;  /* 第一帧, 还没有背景 */
        }

        cv::absdiff(blurred, g.bg_model, fg_mask);
        cv::threshold(fg_mask, fg_mask, 25, 255, cv::THRESH_BINARY);

        /* 膨胀腐蚀去噪 */
        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5));
        cv::dilate(fg_mask, fg_mask, kernel, cv::Point(-1, -1), 2);
        cv::erode(fg_mask, fg_mask, kernel, cv::Point(-1, -1), 1);

        /* 找轮廓 */
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(fg_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        /* 找最大轮廓 (面积最大的 = 最近的人) */
        double max_area = 0;
        cv::Rect best_rect;
        for (auto &c : contours) {
            double area = cv::contourArea(c);
            if (area > max_area) {
                max_area = area;
                best_rect = cv::boundingRect(c);
            }
        }

        /* 面积阈值: 太小忽略 */
        if (max_area < w * h * 0.02) return -2;

        out->detected = true;
        out->x     = ((best_rect.x + best_rect.width / 2.0f) - cx) / cx;  /* -1~+1 */
        out->y     = ((best_rect.y + best_rect.height / 2.0f) - cy) / cy;
        out->width = (float)best_rect.width / w;
        out->height = (float)best_rect.height / h;
        out->area  = (float)max_area / (w * h);

    } else {
        /* ---- HOG 人体检测 (准确但慢) ---- */

        std::vector<cv::Rect> detections;
        g.hog.detectMultiScale(frame, detections, 0, cv::Size(4, 4),
                               cv::Size(8, 8), 1.05, 2);

        if (detections.empty()) return -2;

        /* 找最大检测框 */
        int max_idx = 0;
        int max_area2 = 0;
        for (size_t i = 0; i < detections.size(); i++) {
            int a = detections[i].width * detections[i].height;
            if (a > max_area2) { max_area2 = a; max_idx = (int)i; }
        }

        cv::Rect r = detections[max_idx];
        out->detected = true;
        out->x     = ((r.x + r.width / 2.0f) - cx) / cx;
        out->y     = ((r.y + r.height / 2.0f) - cy) / cy;
        out->width = (float)r.width / w;
        out->height = (float)r.height / h;
        out->area  = (float)max_area2 / (w * h);
    }

    /* 缓存 */
    g.last = *out;
    return 0;

#else
    (void)out;
    return -1;
#endif
}

camera_target_t camera_get_last(void)
{
    return g.last;
}
