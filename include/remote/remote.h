/**
 * @file remote.h
 * @brief 遥控输入统一接口 — 四旋翼
 *
 * 抽象遥控输入源，上层只调 remote_poll() 获取四通道输入。
 * 支持多种输入源:
 *   - 键盘 (stdin, 调试用)
 *   - UDP (手机APP/手柄通过网络)
 *
 * 通道映射:
 *   throttle  0~1    油门
 *   roll     -1~+1   横滚
 *   pitch    -1~+1   俯仰
 *   yaw      -1~+1   偏航
 */

#ifndef PACER_REMOTE_H
#define PACER_REMOTE_H

#include <stdbool.h>

/* 遥控输入源 */
typedef enum {
    REMOTE_SRC_KEYBOARD,   /* 标准输入 (调试) */
    REMOTE_SRC_UDP,        /* UDP 网络遥控 */
} remote_src_t;

/* 遥控命令 */
typedef struct {
    float throttle;     /* 0.0 ~ 1.0 油门 */
    float roll;         /* -1.0 ~ +1.0 */
    float pitch;        /* -1.0 ~ +1.0 */
    float yaw;          /* -1.0 ~ +1.0 */
    bool  arm_switch;   /* 解锁开关 */
    bool  estop;        /* 紧急停止 */
    bool  connected;    /* 是否有遥控连接 */
} remote_cmd_t;

/* 遥控配置 */
typedef struct {
    remote_src_t source;
    /* UDP 配置 */
    int  udp_port;          /* 监听端口, 默认 8888 */
    float timeout_sec;      /* 无数据超时 (秒), 超时后自动归零 */
} remote_config_t;

#define REMOTE_CONFIG_DEFAULT { \
    .source = REMOTE_SRC_KEYBOARD, \
    .udp_port = 8888, \
    .timeout_sec = 0.5f, \
}

/**
 * @brief 初始化遥控模块
 */
int  remote_init(const remote_config_t *cfg);

/**
 * @brief 轮询遥控输入 (非阻塞)
 */
void remote_poll(remote_cmd_t *cmd);

/**
 * @brief 关闭遥控模块
 */
void remote_deinit(void);

#endif /* PACER_REMOTE_H */
