/**
 * @file beacon.h
 * @brief 蓝牙信标扫描 + RSSI 三角定位
 *
 * 硬件: 3 个 USB 蓝牙适配器 (CSR8510) 通过 USB HUB 连接
 *
 * 定位原理:
 *   信标广播 iBeacon 包 → 3 个接收器各测 RSSI
 *   → 3 组 RSSI 换算距离 → 三角交点 → 目标方位
 *
 * 注意:
 *   - 不需要连接, 纯扫描模式
 *   - 通过 UUID 区分目标身份
 *   - RSSI 波动大, 内部做滑动平均滤波
 */

#ifndef PACER_BEACON_H
#define PACER_BEACON_H

#include <stdint.h>
#include <stdbool.h>

/* 接收器数量 (固定3个, 三角定位最少3个) */
#define BEACON_RX_COUNT  3

/* 信标身份 (iBeacon UUID) */
typedef struct {
    uint8_t uuid[16];          /* 128-bit UUID */
    char    uuid_str[37];      /* 可读形式 "xxxxxxxx-xxxx-..." */
} beacon_id_t;

/* 单个接收器的 RSSI 数据 */
typedef struct {
    int   hci_dev;             /* hci 编号 (hci0, hci1, hci2) */
    float rssi;                /* 当前 RSSI (dBm), 滤波后 */
    float rssi_raw;            /* 原始 RSSI */
    float distance;            /* 估算距离 (米) */
    bool  valid;               /* 本次扫描是否收到 */
} beacon_rx_t;

/* 定位结果 */
typedef struct {
    float angle;               /* 目标相对正前方的方位角 (度), -180~180 */
    float distance;            /* 估算距离 (米) */
    float confidence;          /* 置信度 0~1 (信号越好越高) */
    bool  found;               /* 是否找到目标 */
} beacon_location_t;

/* ---------- 生命周期 ---------- */

/**
 * @brief 初始化蓝牙扫描
 * @param target_uuid  目标信标 UUID 字符串 (如 "AA BB CC DD ...")
 * @param hci_devs     3 个 hci 设备号, 如 {0, 1, 2}
 * @return 0=成功
 */
int  beacon_init(const char *target_uuid, const int hci_devs[BEACON_RX_COUNT]);
void beacon_deinit(void);

/* ---------- 扫描 ---------- */

/**
 * @brief 执行一次扫描 (阻塞, 约 1-2 秒)
 * @return 扫描到的信标数量, -1=失败
 */
int  beacon_scan(void);

/* ---------- 结果 ---------- */

/**
 * @brief 获取三角定位结果
 * @note  先调 beacon_scan(), 再调这个
 */
beacon_location_t beacon_get_location(void);

/**
 * @brief 获取各接收器原始数据 (调试用)
 */
void beacon_get_rx_data(beacon_rx_t out[BEACON_RX_COUNT]);

/* ---------- 参数 ---------- */

/**
 * @brief 设置 RSSI 滤波窗口大小 (默认 5)
 */
void beacon_set_filter_window(int size);

/**
 * @brief 设置 RSSI→距离 模型参数
 * @param tx_power  信标发射功率 (dBm), 默认 -59
 * @param n         路径损耗指数, 默认 2.0 (空旷=2, 室内=2.5~3)
 */
void beacon_set_pathloss(float tx_power, float n);

#endif /* PACER_BEACON_H */
