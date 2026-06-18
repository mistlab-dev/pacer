/**
 * @file cli.c
 * @brief 串口命令行接口 — PID 在线调参 + 状态查询
 *
 * 通过 USART2 (PA2/PA3, 115200) 接收单字符命令。
 * 使用中断接收 + 简单行缓冲，不阻塞控制任务。
 *
 * 支持命令:
 *   ? / help          — 显示帮助
 *   status            — 显示当前状态
 *   pid               — 显示 PID 参数
 *   set <axis> <type> <kp/ki/kd> <value>
 *                     — 在线修改 PID
 *       axis: roll / pitch / yaw
 *       type: rate / angle
 *       例: set roll rate kp 0.45
 *   save              — (预留) 保存到 Flash
 *   reset             — 恢复默认 PID
 *   arm               — 远程解锁
 *   disarm            — 远程上锁
 *   calib             — 重新校准陀螺仪
 *   reboot            — 系统重启
 */

#include "app/cli.h"
#include "app/config.h"
#include "ctrl/attitude.h"
#include "ctrl/pid.h"
#include "sensor/imu.h"
#include "hal/led.h"
#include "hal/battery.h"
#include "usart_printf.h"
#include "stm32h7xx_hal.h"

#include "FreeRTOS.h"
#include "task.h"
#include "message_buffer.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

/* ================ 外部引用 ================ */

extern UART_HandleTypeDef huart2;

/* app.c 中定义的全局控制对象 */
extern attitude_ctrl_t *g_att_ctrl_ptr(void);
extern bool g_is_armed(void);
extern bool g_is_flying(void);
extern bool g_is_emergency(void);
extern const attitude_t *g_get_attitude(void);

/* ================ 行缓冲 ================ */

#define CLI_BUF_SIZE  128
static char    cli_buf[CLI_BUF_SIZE];
static uint8_t cli_idx = 0;
static uint8_t cli_rx_byte;

/* 命令历史 (简单回显) */
static void cli_prompt(void)
{
    printf("\r\npacer> ");
}

/* ================ 命令处理 ================ */

static void cmd_help(void)
{
    printf(
        "\r\n"
        "PACER CLI Commands:\r\n"
        "  ? / help          Show this help\r\n"
        "  status            Show flight status\r\n"
        "  pid               Show PID parameters\r\n"
        "  set <axis> <type> <param> <val>  Set PID\r\n"
        "    axis: roll|pitch|yaw\r\n"
        "    type: rate|angle\r\n"
        "    param: kp|ki|kd\r\n"
        "    e.g: set roll rate kp 0.45\r\n"
        "  arm / disarm      Remote arm/disarm\r\n"
        "  calib             Re-calibrate gyro\r\n"
        "  reboot            System reset\r\n"
    );
}

static void cmd_status(void)
{
    const attitude_t *att = g_get_attitude();
    bool armed = g_is_armed();
    bool flying = g_is_flying();
    bool emg = g_is_emergency();

    printf(
        "\r\n"
        "=== PACER Status ===\r\n"
        "  State:   %s%s%s\r\n"
        "  Attitude: R=%.2f P=%.2f Y=%.2f\r\n"
        "  Battery:  %.2fV (%s)\r\n"
        "  IMU:      %s\r\n"
        "  Uptime:   %lu ms\r\n",
        armed ? "ARMED" : "DISARMED",
        flying ? " FLYING" : "",
        emg ? " EMERGENCY" : "",
        att->roll, att->pitch, att->yaw,
        battery_get_voltage(),
        battery_get_status() == BATTERY_OK ? "OK" :
        battery_get_status() == BATTERY_WARNING ? "WARN" : "CRIT",
        "OK",
        (unsigned long)HAL_GetTick()
    );
}

static void cmd_pid_show(void)
{
    attitude_ctrl_t *ac = g_att_ctrl_ptr();
    if (!ac) {
        printf("\r\n[ERR] controller not ready\r\n");
        return;
    }

    printf("\r\n");
    printf("=== PID Parameters ===\r\n");

    static const char *axis_names[] = {"Roll", "Pitch", "Yaw"};
    static const char *type_names[] = {"Rate ", "Angle"};

    for (int axis = 0; axis < 3; axis++) {
        for (int type = 0; type < 2; type++) {
            pid_t *pid = (type == 0) ?
                &ac->rate_pid[axis] : &ac->angle_pid[axis];
            printf("  %s %s: Kp=%.4f Ki=%.4f Kd=%.4f I=%.4f\r\n",
                   axis_names[axis], type_names[type],
                   pid->kp, pid->ki, pid->kd, pid->integral);
        }
    }
}

static void cmd_set_pid(int argc, char **argv)
{
    if (argc < 5) {
        printf("\r\n[ERR] usage: set <axis> <type> <param> <val>\r\n");
        return;
    }

    /* 解析 axis */
    int axis = -1;
    if (strcmp(argv[1], "roll") == 0)  axis = 0;
    else if (strcmp(argv[1], "pitch") == 0) axis = 1;
    else if (strcmp(argv[1], "yaw") == 0)   axis = 2;
    if (axis < 0) {
        printf("\r\n[ERR] unknown axis: %s\r\n", argv[1]);
        return;
    }

    /* 解析 type */
    int type = -1;
    if (strcmp(argv[2], "rate") == 0)   type = 0;
    else if (strcmp(argv[2], "angle") == 0) type = 1;
    if (type < 0) {
        printf("\r\n[ERR] unknown type: %s\r\n", argv[2]);
        return;
    }

    /* 解析 param */
    const char *param = argv[3];
    float value = strtof(argv[4], NULL);
    if (value < 0.0f || value > 100.0f) {
        printf("\r\n[ERR] value out of range: %s\r\n", argv[4]);
        return;
    }

    attitude_ctrl_t *ac = g_att_ctrl_ptr();
    if (!ac) {
        printf("\r\n[ERR] controller not ready\r\n");
        return;
    }

    pid_t *pid = (type == 0) ? &ac->rate_pid[axis] : &ac->angle_pid[axis];

    taskENTER_CRITICAL();
    if (strcmp(param, "kp") == 0)      pid->kp = value;
    else if (strcmp(param, "ki") == 0) pid->ki = value;
    else if (strcmp(param, "kd") == 0) pid->kd = value;
    else {
        taskEXIT_CRITICAL();
        printf("\r\n[ERR] unknown param: %s\r\n", param);
        return;
    }
    taskEXIT_CRITICAL();

    printf("\r\n[OK] %s %s %s = %.4f\r\n", argv[1], argv[2], param, value);
}

static void cmd_calib(void)
{
    printf("\r\n[CALIB] keep quad level, calibrating gyro (500 samples)...\r\n");
    imu_calibrate_gyro(500);
    printf("[CALIB] done\r\n");
}

static void cmd_reboot(void)
{
    printf("\r\n[REBOOT] system reset in 100ms...\r\n");
    HAL_Delay(100);
    NVIC_SystemReset();
}

/* ================ 命令行解析 ================ */

#define MAX_ARGS 8

static void cli_process_line(char *line)
{
    char *argv[MAX_ARGS];
    int argc = 0;

    /* 分词 */
    char *tok = strtok(line, " \t\r\n");
    while (tok && argc < MAX_ARGS) {
        argv[argc++] = tok;
        tok = strtok(NULL, " \t\r\n");
    }

    if (argc == 0) {
        cli_prompt();
        return;
    }

    if (strcmp(argv[0], "?") == 0 || strcmp(argv[0], "help") == 0) {
        cmd_help();
    } else if (strcmp(argv[0], "status") == 0) {
        cmd_status();
    } else if (strcmp(argv[0], "pid") == 0) {
        cmd_pid_show();
    } else if (strcmp(argv[0], "set") == 0) {
        cmd_set_pid(argc, argv);
    } else if (strcmp(argv[0], "calib") == 0) {
        cmd_calib();
    } else if (strcmp(argv[0], "reboot") == 0) {
        cmd_reboot();
    } else {
        printf("\r\n[ERR] unknown command: %s (type 'help')\r\n", argv[0]);
    }

    cli_prompt();
}

/* ================ 中断回调 ================ */

/**
 * @brief USART2 RX 中断回调 — 由 HAL_UART_RxCpltCallback 调用
 */
void cli_rx_byte_cb(uint8_t b)
{
    /* 回车/换行 → 处理命令 */
    if (b == '\r' || b == '\n') {
        if (cli_idx > 0) {
            cli_buf[cli_idx] = '\0';
            cli_process_line(cli_buf);
            cli_idx = 0;
        } else {
            cli_prompt();
        }
        return;
    }

    /* Backspace */
    if (b == 0x7F || b == '\b') {
        if (cli_idx > 0) cli_idx--;
        printf("\b \b");
        return;
    }

    /* 普通字符 */
    if (cli_idx < CLI_BUF_SIZE - 1) {
        cli_buf[cli_idx++] = (char)b;
        /* 回显 */
        HAL_UART_Transmit(&huart2, &b, 1, 10);
    }
}

/* ================ 初始化 ================ */

void cli_init(void)
{
    cli_idx = 0;
    printf("\r\n=== PACER CLI ready (type 'help') ===\r\n");
    cli_prompt();

    /* 启动 USART2 中断接收 */
    HAL_UART_Receive_IT(&huart2, &cli_rx_byte, 1);
}

/**
 * @brief HAL UART RX 完成回调 — CLI 接收
 *        usart_printf.c 中的弱版本已被覆盖
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2) {
        cli_rx_byte_cb(cli_rx_byte);
        HAL_UART_Receive_IT(&huart2, &cli_rx_byte, 1);
    }
    /* USART3 (遥控) 由 remote.c 的专用回调处理 */
}
