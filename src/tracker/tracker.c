/**
 * @file tracker.c
 * @brief 目标追踪 — 融合蓝牙定位 + 摄像头检测
 *
 * 状态机:
 *
 *   IDLE ──start()──→ SCANNING
 *     ↑                  │ 找到蓝牙+摄像头
 *     │                  ↓
 *   LOST ←─── 丢失 ─── LOCKED/TRACKING
 *     │                  │
 *     │  超时/重新找到   ↓
 *     └──────────── TRACKING
 *
 * 融合策略:
 *   - 蓝牙确认身份 (UUID 匹配)
 *   - 蓝牙给粗方位 (±30°) 和距离
 *   - 摄像头给精确角度 (±5°)
 *   - 两者都有 → 高置信度追踪
 *   - 只有蓝牙 → 粗追踪 (转过去找)
 *   - 只有摄像头 → 不追踪 (不确定身份)
 */

#include "tracker/tracker.h"
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <time.h>

/* ================ 内部状态 ================ */

static struct {
    tracker_config_t cfg;
    tracker_state_t  state;
    tracker_output_t output;
    bool             running;

    /* 计时 */
    uint64_t last_lock_time;    /* 上次锁定时间 */
    uint64_t last_seen_time;    /* 上次看到目标 */
    float    lost_elapsed;      /* 丢失时长 */

    /* 融合数据 */
    beacon_location_t beacon_loc;
    camera_target_t   cam_target;
} g;

static uint64_t now_us(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000ULL + (uint64_t)ts.tv_nsec / 1000;
}

/* ================ 状态转换 ================ */

static void set_state(tracker_state_t s)
{
    if (g.state != s) {
        printf("[TRACKER] %s → %s\n",
               tracker_state_str(g.state), tracker_state_str(s));
        g.state = s;
    }
}

/* ================ 公开接口 ================ */

int tracker_init(const tracker_config_t *cfg)
{
    memset(&g, 0, sizeof(g));
    g.cfg = *cfg;
    g.state = TRACK_IDLE;

    /* 初始化蓝牙 */
    if (beacon_init(cfg->beacon_uuid, cfg->hci_devs) != 0) {
        fprintf(stderr, "[TRACKER] beacon init failed\n");
        return -1;
    }

    /* 初始化摄像头 (可选, 编译时可能没有 OpenCV) */
    if (camera_init(&cfg->camera) != 0) {
        fprintf(stderr, "[TRACKER] camera init failed — tracking will be beacon-only\n");
        /* 不返回错误, 蓝牙也能粗追踪 */
    }

    printf("[TRACKER] init OK (follow=%.1fm, speed=%.1fm/s)\n",
           cfg->follow_distance, cfg->follow_speed_max);
    return 0;
}

void tracker_deinit(void)
{
    beacon_deinit();
    camera_deinit();
    g.state = TRACK_IDLE;
}

void tracker_start(void)
{
    g.running = true;
    g.last_seen_time = now_us();
    set_state(TRACK_SCANNING);
    printf("[TRACKER] started\n");
}

void tracker_stop(void)
{
    g.running = false;
    g.output.target_speed = 0;
    g.output.target_yaw_rate = 0;
    g.output.should_stop = true;
    set_state(TRACK_IDLE);
}

tracker_output_t tracker_update(void)
{
    if (!g.running) {
        g.output.target_speed = 0;
        g.output.target_yaw_rate = 0;
        g.output.should_stop = true;
        return g.output;
    }

    uint64_t t = now_us();

    /* ---- 1. 采集数据 ---- */

    /* 蓝牙扫描 (每次调用约 200ms, 不阻塞主循环) */
    beacon_scan();
    g.beacon_loc = beacon_get_location();

    /* 摄像头检测 */
    int cam_ret = camera_detect(&g.cam_target);

    /* ---- 2. 融合决策 ---- */

    bool beacon_ok = g.beacon_loc.found;
    bool camera_ok = (cam_ret == 0) && g.cam_target.detected;
    bool target_ok = beacon_ok && camera_ok;   /* 双重确认 */

    switch (g.state) {

    case TRACK_SCANNING:
        /* 扫描中: 等蓝牙先找到目标 */
        if (beacon_ok) {
            g.last_seen_time = t;
            set_state(TRACK_LOCKED);
        }
        g.output.target_speed = 0;
        /* 扫描时缓慢旋转 */
        g.output.target_yaw_rate = g.cfg.scan_rotate_speed;
        g.output.should_stop = false;
        break;

    case TRACK_LOCKED:
        /* 锁定: 等摄像头也确认 */
        if (target_ok) {
            g.last_seen_time = t;
            g.last_lock_time = t;
            set_state(TRACK_TRACKING);
        } else if (!beacon_ok) {
            set_state(TRACK_LOST);
        }
        g.output.target_speed = 0;
        g.output.target_yaw_rate = 0;
        g.output.should_stop = false;
        break;

    case TRACK_TRACKING: {
        /* 追踪中: 核心逻辑 */

        if (target_ok) {
            g.last_seen_time = t;

            /* ---- 转向: 以摄像头为主 ---- */
            float yaw = 0;
            if (camera_ok) {
                /* 摄像头精确角度: x偏移 → 转向 */
                yaw = g.cam_target.x * g.cfg.turn_speed_max;
            } else if (beacon_ok) {
                /* 只有蓝牙, 粗角度 */
                yaw = (g.beacon_loc.angle / 90.0f) * g.cfg.turn_speed_max;
            }

            /* ---- 前进: 以蓝牙距离为主 ---- */
            float speed = 0;
            if (beacon_ok) {
                float dist_err = g.beacon_loc.distance - g.cfg.follow_distance;
                if (dist_err > 0.3f) {
                    /* 目标远了, 跟上 */
                    speed = dist_err * 0.3f;
                    if (speed > g.cfg.follow_speed_max)
                        speed = g.cfg.follow_speed_max;
                } else if (dist_err < -0.2f) {
                    /* 太近了, 后退 */
                    speed = dist_err * 0.2f;
                }
                /* 在跟随距离范围内, 不动 */
            } else if (camera_ok) {
                /* 没蓝牙距离, 用摄像头面积估算 */
                if (g.cam_target.area < 0.05f) {
                    speed = g.cfg.follow_speed_max * 0.5f;  /* 目标小, 可能远了 */
                }
            }

            g.output.target_speed    = speed;
            g.output.target_yaw_rate = yaw;
            g.output.should_stop     = false;

        } else if (beacon_ok && !camera_ok) {
            /* 蓝牙在但摄像头没看到 → 转过去找 */
            g.last_seen_time = t;
            float yaw = (g.beacon_loc.angle / 90.0f) * g.cfg.turn_speed_max;
            g.output.target_speed    = 0;
            g.output.target_yaw_rate = yaw;
            g.output.should_stop     = false;

        } else {
            /* 都没了 → 丢失 */
            set_state(TRACK_LOST);
        }
        break;
    }

    case TRACK_LOST: {
        /* 丢失: 原地旋转寻找 */
        float elapsed = (float)(t - g.last_seen_time) / 1000000.0f;
        g.lost_elapsed = elapsed;

        if (beacon_ok) {
            g.last_seen_time = t;
            set_state(TRACK_LOCKED);
        } else if (elapsed > g.cfg.lost_timeout) {
            /* 超时 → 停下来 */
            g.output.target_speed = 0;
            g.output.target_yaw_rate = 0;
            g.output.should_stop = true;
            set_state(TRACK_SCANNING);
        } else {
            /* 旋转搜索 */
            g.output.target_speed    = 0;
            g.output.target_yaw_rate = g.cfg.scan_rotate_speed;
            g.output.should_stop     = false;
        }
        break;
    }

    case TRACK_IDLE:
    default:
        g.output.target_speed    = 0;
        g.output.target_yaw_rate = 0;
        g.output.should_stop     = true;
        break;
    }

    return g.output;
}

tracker_state_t tracker_get_state(void)
{
    return g.state;
}

const char *tracker_state_str(tracker_state_t s)
{
    switch (s) {
        case TRACK_IDLE:     return "IDLE";
        case TRACK_SCANNING: return "SCANNING";
        case TRACK_LOCKED:   return "LOCKED";
        case TRACK_TRACKING: return "TRACKING";
        case TRACK_LOST:     return "LOST";
    }
    return "UNKNOWN";
}
