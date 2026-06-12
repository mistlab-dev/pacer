/**
 * @file remote.c
 * @brief 遥控输入 — 键盘 + UDP (四通道)
 *
 * UDP 协议 (16 字节):
 *   [0..3]  throttle  (float, 0.0~1.0, 小端)
 *   [4..7]  roll      (float, -1.0~+1.0, 小端)
 *   [8..11] pitch     (float, -1.0~+1.0, 小端)
 *   [12..15] yaw      (float, -1.0~+1.0, 小端)
 *
 * 键盘 (调试):
 *   W/S  = pitch 前/后
 *   A/D  = roll 左/右
 *   Q/E  = yaw 左旋/右旋
 *   1-9  = 油门 10%~90%
 *   0    = 油门归零
 *   空格 = 全部归零
 *   X    = 紧急停止
 *   R    = 解锁/上锁切换
 */

#include "remote/remote.h"
#include "app/config.h"

#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <sys/select.h>
#include <time.h>
#include <stdlib.h>

/* ================ 全局状态 ================ */

static remote_config_t g_cfg;
static int g_udp_fd = -1;
static bool g_initialized = false;
static bool g_armed = false;

/* 超时保护 */
static struct timespec g_last_recv;

/* 键盘保持的油门值 (按数字键调节) */
static float g_kb_throttle = 0.0f;

/* ================ 键盘遥控 ================ */

static void poll_keyboard(remote_cmd_t *cmd)
{
    fd_set fds;
    struct timeval tv = {0, 0};
    FD_ZERO(&fds);
    FD_SET(STDIN_FILENO, &fds);

    if (select(STDIN_FILENO + 1, &fds, NULL, NULL, &tv) > 0) {
        char buf[8];
        if (fgets(buf, sizeof(buf), stdin)) {
            switch (buf[0]) {
                /* pitch */
                case 'w': case 'W': cmd->pitch =  0.5f; break;
                case 's': case 'S': cmd->pitch = -0.5f; break;
                /* roll */
                case 'a': case 'A': cmd->roll = -0.5f; break;
                case 'd': case 'D': cmd->roll =  0.5f; break;
                /* yaw */
                case 'q': case 'Q': cmd->yaw = -0.5f; break;
                case 'e': case 'E': cmd->yaw =  0.5f; break;
                /* 油门 */
                case '1': g_kb_throttle = 0.1f; break;
                case '2': g_kb_throttle = 0.2f; break;
                case '3': g_kb_throttle = 0.3f; break;
                case '4': g_kb_throttle = 0.4f; break;
                case '5': g_kb_throttle = 0.5f; break;
                case '6': g_kb_throttle = 0.6f; break;
                case '7': g_kb_throttle = 0.7f; break;
                case '8': g_kb_throttle = 0.8f; break;
                case '9': g_kb_throttle = 0.9f; break;
                case '0': g_kb_throttle = 0.0f; break;
                /* 停止 */
                case ' ': g_kb_throttle = 0.0f;
                          cmd->roll = cmd->pitch = cmd->yaw = 0.0f;
                          break;
                /* 紧急停止 */
                case 'x': case 'X': cmd->estop = true; break;
                /* 解锁切换 */
                case 'r': case 'R': g_armed = !g_armed;
                                    cmd->arm_switch = g_armed;
                                    printf("[REMOTE] %s\n", g_armed ? "ARMED" : "DISARMED");
                                    break;
            }
            cmd->connected = true;
        }
    }

    cmd->throttle = g_kb_throttle;
    cmd->arm_switch = g_armed;
}

/* ================ UDP 遥控 ================ */

static int udp_init(int port)
{
    int fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (fd < 0) {
        perror("[REMOTE] socket");
        return -1;
    }

    int flags = fcntl(fd, F_GETFL, 0);
    fcntl(fd, F_SETFL, flags | O_NONBLOCK);

    int opt = 1;
    setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family      = AF_INET;
    addr.sin_port        = htons((uint16_t)port);
    addr.sin_addr.s_addr = INADDR_ANY;

    if (bind(fd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        perror("[REMOTE] bind");
        close(fd);
        return -1;
    }

    printf("[REMOTE] UDP listening on port %d\n", port);
    return fd;
}

static void poll_udp(remote_cmd_t *cmd)
{
    if (g_udp_fd < 0) return;

    uint8_t buf[16];
    struct sockaddr_in from;
    socklen_t from_len = sizeof(from);

    while (recvfrom(g_udp_fd, buf, sizeof(buf), 0,
                    (struct sockaddr *)&from, &from_len) == 16) {
        float thr, roll, pitch, yaw;
        memcpy(&thr,   buf,      4);
        memcpy(&roll,  buf + 4,  4);
        memcpy(&pitch, buf + 8,  4);
        memcpy(&yaw,   buf + 12, 4);

        /* 限幅 */
        if (thr   > 1.0f)  thr   = 1.0f;
        if (thr   < 0.0f)  thr   = 0.0f;
        if (roll  > 1.0f)  roll  = 1.0f;
        if (roll  < -1.0f) roll  = -1.0f;
        if (pitch > 1.0f)  pitch = 1.0f;
        if (pitch < -1.0f) pitch = -1.0f;
        if (yaw   > 1.0f)  yaw   = 1.0f;
        if (yaw   < -1.0f) yaw   = -1.0f;

        cmd->throttle = thr;
        cmd->roll     = roll;
        cmd->pitch    = pitch;
        cmd->yaw      = yaw;
        cmd->connected = true;

        clock_gettime(CLOCK_MONOTONIC, &g_last_recv);
    }

    /* 超时检查 */
    if (cmd->connected) {
        struct timespec now;
        clock_gettime(CLOCK_MONOTONIC, &now);
        float elapsed = (float)(now.tv_sec - g_last_recv.tv_sec)
                      + (float)(now.tv_nsec - g_last_recv.tv_nsec) / 1e9f;
        if (elapsed > g_cfg.timeout_sec) {
            cmd->throttle = 0.0f;
            cmd->roll     = 0.0f;
            cmd->pitch    = 0.0f;
            cmd->yaw      = 0.0f;
            cmd->connected = false;
        }
    }
}

/* ================ 公开接口 ================ */

int remote_init(const remote_config_t *cfg)
{
    g_cfg = *cfg;

    if (g_cfg.source == REMOTE_SRC_UDP) {
        g_udp_fd = udp_init(g_cfg.udp_port);
        if (g_udp_fd < 0) {
            fprintf(stderr, "[REMOTE] UDP init failed, fallback to keyboard\n");
            g_cfg.source = REMOTE_SRC_KEYBOARD;
        }
    }

    if (g_cfg.source == REMOTE_SRC_KEYBOARD) {
        system("stty -echo -icanon min 0 time 0");
    }

    clock_gettime(CLOCK_MONOTONIC, &g_last_recv);
    g_initialized = true;
    g_kb_throttle = 0.0f;
    g_armed = false;

    printf("[REMOTE] init source=%s\n",
           g_cfg.source == REMOTE_SRC_UDP ? "UDP" : "keyboard");
    return 0;
}

void remote_poll(remote_cmd_t *cmd)
{
    if (!g_initialized) {
        memset(cmd, 0, sizeof(*cmd));
        return;
    }

    cmd->estop     = false;
    cmd->arm_switch = g_armed;
    /* 保留上一帧的杆位 (松手后保持，不是归零 — 跟遥控器一样) */
    cmd->roll      = 0.0f;
    cmd->pitch     = 0.0f;
    cmd->yaw       = 0.0f;

    switch (g_cfg.source) {
        case REMOTE_SRC_KEYBOARD: poll_keyboard(cmd); break;
        case REMOTE_SRC_UDP:      poll_udp(cmd);      break;
    }
}

void remote_deinit(void)
{
    if (g_udp_fd >= 0) {
        close(g_udp_fd);
        g_udp_fd = -1;
    }

    if (g_cfg.source == REMOTE_SRC_KEYBOARD) {
        system("stty echo icanon");
    }

    g_initialized = false;
    printf("[REMOTE] deinit\n");
}
