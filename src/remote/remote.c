/**
 * @file remote.c
 * @brief 遥控输入 — 键盘 + UDP
 *
 * UDP 协议 (简单二进制, 8 字节):
 *   [0..3] throttle  (float, -1.0 ~ +1.0, 小端)
 *   [4..7] steering  (float, -1.0 ~ +1.0, 小端)
 *
 * 手机APP或手柄发送这 8 字节 UDP 包即可遥控。
 * 超时未收到数据自动归零 (安全)。
 */

#include "remote/remote.h"

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

/* 超时保护: 最近一次收到遥控数据的时间 */
static struct timespec g_last_recv;

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
                case 'w': case 'W': cmd->throttle =  0.5f; cmd->steering = 0.0f; break;
                case 's': case 'S': cmd->throttle = -0.5f; cmd->steering = 0.0f; break;
                case 'a': case 'A': cmd->throttle =  0.0f; cmd->steering = -0.5f; break;
                case 'd': case 'D': cmd->throttle =  0.0f; cmd->steering =  0.5f; break;
                case 'q': case 'Q': cmd->throttle =  0.5f; cmd->steering = -0.5f; break;
                case 'e': case 'E': cmd->throttle =  0.5f; cmd->steering =  0.5f; break;
                case ' ':           cmd->throttle =  0.0f; cmd->steering = 0.0f; break;
                case 'x': case 'X': cmd->estop = true; break;
            }
            cmd->connected = true;
        }
    }
}

/* ================ UDP 遥控 ================ */

static int udp_init(int port)
{
    int fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (fd < 0) {
        perror("[REMOTE] socket");
        return -1;
    }

    /* 非阻塞 */
    int flags = fcntl(fd, F_GETFL, 0);
    fcntl(fd, F_SETFL, flags | O_NONBLOCK);

    /* 允许地址重用 */
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

    uint8_t buf[8];
    struct sockaddr_in from;
    socklen_t from_len = sizeof(from);

    while (recvfrom(g_udp_fd, buf, sizeof(buf), 0,
                    (struct sockaddr *)&from, &from_len) == 8) {
        /* 解析 float (小端) */
        float thr, steer;
        memcpy(&thr,    buf,     4);
        memcpy(&steer,  buf + 4, 4);

        /* 限幅 */
        if (thr    >  1.0f) thr    =  1.0f;
        if (thr    < -1.0f) thr    = -1.0f;
        if (steer  >  1.0f) steer  =  1.0f;
        if (steer  < -1.0f) steer  = -1.0f;

        cmd->throttle  = thr;
        cmd->steering  = steer;
        cmd->connected = true;

        clock_gettime(CLOCK_MONOTONIC, &g_last_recv);
    }

    /* 超时检查: 无数据超时自动归零 */
    if (cmd->connected) {
        struct timespec now;
        clock_gettime(CLOCK_MONOTONIC, &now);
        float elapsed = (float)(now.tv_sec - g_last_recv.tv_sec)
                      + (float)(now.tv_nsec - g_last_recv.tv_nsec) / 1e9f;
        if (elapsed > g_cfg.timeout_sec) {
            cmd->throttle = 0.0f;
            cmd->steering = 0.0f;
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
        /* 设置 stdin 非阻塞 raw 模式 */
        system("stty -echo -icanon min 0 time 0");
    }

    clock_gettime(CLOCK_MONOTONIC, &g_last_recv);
    g_initialized = true;

    printf("[REMOTE] init source=%s\n",
           g_cfg.source == REMOTE_SRC_UDP ? "UDP" : "keyboard");
    return 0;
}

void remote_poll(remote_cmd_t *cmd)
{
    if (!g_initialized) {
        cmd->throttle  = 0.0f;
        cmd->steering  = 0.0f;
        cmd->estop     = false;
        cmd->connected = false;
        return;
    }

    cmd->estop = false;

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
