/**
 * @file beacon.c
 * @brief 蓝牙信标扫描 + RSSI 三角定位实现
 *
 * 使用 BlueZ 的 hcitool / bluetoothctl 进行扫描。
 * 3 个 USB 蓝牙适配器分别对应 hci0/hci1/hci2,
 * 位置关系: 左前 / 右前 / 正后。
 *
 * RSSI → 距离模型:
 *   d = 10 ^ ((tx_power - rssi) / (10 * n))
 *   tx_power = 发射功率 (1米处实测 RSSI)
 *   n = 环境衰减因子 (空旷=2, 室内=2.5~3)
 */

#include "tracker/beacon.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <unistd.h>

/* ================ 内部状态 ================ */

static struct {
    beacon_id_t  target;                        /* 目标 UUID */
    beacon_rx_t  rx[BEACON_RX_COUNT];           /* 3 个接收器 */
    beacon_location_t location;                  /* 定位结果 */

    /* 滤波 */
    float rssi_history[BEACON_RX_COUNT][8];     /* RSSI 滑动窗口 */
    int   history_idx;
    int   history_size;                          /* 窗口大小 */
    int   history_count;                         /* 已有样本数 */

    /* 路径损耗模型 */
    float tx_power;
    float pathloss_n;

    /* 接收器几何位置 (相对车体中心, 米)
     * [0] 左前, [1] 右前, [2] 正后 */
    float rx_pos[BEACON_RX_COUNT][2];            /* {x, y} */

    bool ready;
} g;

/* ================ 内部函数 ================ */

/**
 * RSSI 滑动平均滤波
 */
static float filter_rssi(int rx_idx, float raw)
{
    g.rssi_history[rx_idx][g.history_idx] = raw;
    g.history_count++;

    int n = (g.history_count < g.history_size) ? g.history_count : g.history_size;
    float sum = 0;
    for (int i = 0; i < n; i++) {
        int idx = (g.history_idx - i + g.history_size) % g.history_size;
        /* 指数加权, 越新权重越大 */
        float w = 1.0f / (1.0f + i * 0.3f);
        sum += g.rssi_history[rx_idx][idx] * w;
    }
    return sum / n;
}

/**
 * RSSI → 距离 (米)
 */
static float rssi_to_distance(float rssi)
{
    if (rssi == 0) return 99.0f;
    float ratio = (g.tx_power - rssi) / (10.0f * g.pathloss_n);
    return powf(10.0f, ratio);
}

/**
 * 三角定位 — 三边测量法
 *
 * 已知3个接收器坐标和各自到目标的距离,
 * 用最小二乘法求目标坐标。
 */
static beacon_location_t trilaterate(void)
{
    beacon_location_t loc = {0};
    loc.found = false;

    /* 至少需要 2 个有效信号 */
    int valid_count = 0;
    for (int i = 0; i < BEACON_RX_COUNT; i++) {
        if (g.rx[i].valid) valid_count++;
    }
    if (valid_count < 2) return loc;

    /*
     * 简化方法: 用 RSSI 差值算角度
     *
     * R0(左前) vs R1(右前) 的 RSSI 差 → 水平方向
     * R0+R1(前方) vs R2(后方) 的 RSSI 差 → 前后方向
     */
    float rssi_front_l = g.rx[0].valid ? g.rx[0].rssi : -100;
    float rssi_front_r = g.rx[1].valid ? g.rx[1].rssi : -100;
    float rssi_rear    = g.rx[2].valid ? g.rx[2].rssi : -100;

    /* 水平角度: 左右 RSSI 差值 → 角度 */
    float lr_diff = rssi_front_l - rssi_front_r;
    /* 经验值: 每dBm差异约对应 5-10 度偏移 */
    float angle_h = lr_diff * 8.0f;

    /* 限幅 */
    if (angle_h > 90.0f)  angle_h = 90.0f;
    if (angle_h < -90.0f) angle_h = -90.0f;

    /* 前后: 前方平均 RSSI vs 后方 */
    float front_avg = (rssi_front_l + rssi_front_r) / 2.0f;
    float fb_diff = front_avg - rssi_rear;

    /* 距离估算: 用前方较强信号 */
    float best_rssi = rssi_front_l > rssi_front_r ? rssi_front_l : rssi_front_r;
    best_rssi = best_rssi > rssi_rear ? best_rssi : rssi_rear;
    float distance = rssi_to_distance(best_rssi);

    /* 如果后方信号更强, 人在后面, 角度翻转 */
    if (fb_diff < -5.0f) {
        angle_h = (angle_h > 0) ? (180.0f - angle_h) : (-180.0f - angle_h);
    }

    /* 置信度: 信号越强越可信 */
    float confidence = 0;
    if (best_rssi > -40) confidence = 1.0f;
    else if (best_rssi > -60) confidence = 0.8f;
    else if (best_rssi > -75) confidence = 0.5f;
    else confidence = 0.2f;

    loc.angle      = angle_h;
    loc.distance   = distance;
    loc.confidence = confidence;
    loc.found      = true;

    return loc;
}

/**
 * 解析 hcitool lesan 输出, 提取目标 UUID 的 RSSI
 */
static int parse_scan_output(FILE *fp)
{
    char line[256];
    int found = 0;

    while (fgets(line, sizeof(line), fp)) {
        /* 查找目标 UUID */
        if (strstr(line, g.target.uuid_str) == NULL &&
            strstr(line, "RSSI") == NULL) {
            continue;
        }

        /* 解析 RSSI: "RSSI: -xx" */
        char *rssi_str = strstr(line, "RSSI:");
        if (!rssi_str) continue;

        int rssi = 0;
        if (sscanf(rssi_str, "RSSI: %d", &rssi) != 1) continue;

        /* 确定 hci 设备 — 通过 mac 地址匹配到接收器 */
        /* 简化: 轮流用每个 hci 扫描, 所以每次结果对应当前 hci */
        for (int i = 0; i < BEACON_RX_COUNT; i++) {
            g.rx[i].rssi_raw = (float)rssi;
            g.rx[i].rssi = filter_rssi(i, (float)rssi);
            g.rx[i].distance = rssi_to_distance(g.rx[i].rssi);
            g.rx[i].valid = true;
        }
        found++;
    }

    return found;
}

/* ================ 公开接口 ================ */

int beacon_init(const char *target_uuid, const int hci_devs[BEACON_RX_COUNT])
{
    memset(&g, 0, sizeof(g));

    /* 解析 UUID */
    strncpy(g.target.uuid_str, target_uuid, sizeof(g.target.uuid_str) - 1);

    /* 接收器配置 */
    for (int i = 0; i < BEACON_RX_COUNT; i++) {
        g.rx[i].hci_dev = hci_devs[i];
        g.rx[i].valid = false;
    }

    /* 接收器几何位置 (车体坐标系, 正前方为 Y+)
     * 间距约 0.15 米 */
    g.rx_pos[0][0] = -0.10f;  g.rx_pos[0][1] =  0.08f;  /* 左前 */
    g.rx_pos[1][0] =  0.10f;  g.rx_pos[1][1] =  0.08f;  /* 右前 */
    g.rx_pos[2][0] =  0.00f;  g.rx_pos[2][1] = -0.10f;  /* 正后 */

    /* 默认参数 */
    g.tx_power = -59.0f;
    g.pathloss_n = 2.5f;     /* 室内环境 */
    g.history_size = 8;

    g.ready = true;
    printf("[BEACON] init, target=%s, hci={%d,%d,%d}\n",
           target_uuid, hci_devs[0], hci_devs[1], hci_devs[2]);
    return 0;
}

void beacon_deinit(void)
{
    g.ready = false;
    printf("[BEACON] deinit\n");
}

int beacon_scan(void)
{
    if (!g.ready) return -1;

    int total_found = 0;

    /* 轮流用每个 hci 设备扫描 */
    for (int i = 0; i < BEACON_RX_COUNT; i++) {
        char cmd[128];
        snprintf(cmd, sizeof(cmd),
                 "hcitool -i hci%d lescan --duplicates 2>/dev/null"
                 " | head -20", g.rx[i].hci_dev);

        FILE *fp = popen(cmd, "r");
        if (!fp) {
            g.rx[i].valid = false;
            continue;
        }

        /* 简化: 标记为有信号 (实际应解析输出) */
        /* TODO: 完整的 iBeacon 解析 */

        pclose(fp);
    }

    g.history_idx = (g.history_idx + 1) % g.history_size;

    /* 计算三角定位 */
    g.location = trilaterate();

    return total_found;
}

beacon_location_t beacon_get_location(void)
{
    return g.location;
}

void beacon_get_rx_data(beacon_rx_t out[BEACON_RX_COUNT])
{
    memcpy(out, g.rx, sizeof(g.rx));
}

void beacon_set_filter_window(int size)
{
    if (size > 0 && size <= 8)
        g.history_size = size;
}

void beacon_set_pathloss(float tx_power, float n)
{
    g.tx_power = tx_power;
    g.pathloss_n = n;
}
