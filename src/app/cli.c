/**
 * @file cli.c
 * @brief 串口命令行接口 — PID 在线调参 + 状态查询
 *
 * 通过 USART2 (PA2/PA3, 115200) 接收命令行。
 * 中断接收 + 行缓冲，不阻塞控制任务。
 */

#include "app/cli.h"
#include "app/app.h"
#include "app/config.h"
#include "ctrl/attitude.h"
#include "ctrl/pid.h"
#include "sensor/imu.h"
#include "hal/led.h"
#include "hal/battery.h"
#include "remote/remote.h"
#include "usart_printf.h"
#include "stm32h7xx_hal.h"

#include "FreeRTOS.h"
#include "task.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

extern UART_HandleTypeDef huart2;

#define CLI_BUF_SIZE  128
static char    cli_buf[CLI_BUF_SIZE];
static uint8_t cli_idx = 0;
uint8_t        cli_rx_byte;

static void cli_prompt(void)
{
    printf("\r\npacer> ");
}

static pacer_pid_t *pid_for_axis(attitude_ctrl_t *ac, int axis, int type)
{
    if (!ac) return NULL;

    if (type == 0) {
        switch (axis) {
        case 0: return &ac->pid_roll_rate;
        case 1: return &ac->pid_pitch_rate;
        case 2: return &ac->pid_yaw_rate;
        default: return NULL;
        }
    }

    switch (axis) {
    case 0: return &ac->pid_roll_angle;
    case 1: return &ac->pid_pitch_angle;
    default: return NULL;
    }
}

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
        "  reset             Restore default PID\r\n"
        "  calib             Re-calibrate gyro\r\n"
        "  reboot            System reset\r\n"
    );
}

static void cmd_status(void)
{
    const attitude_t *att = app_get_attitude();
    remote_stats_t rcst;
    remote_cmd_t rc;
    remote_get_stats(&rcst);
    remote_get_cmd(&rc);

    printf(
        "\r\n"
        "=== PACER Status ===\r\n"
        "  State:   %s%s%s\r\n"
        "  Attitude: R=%.2f P=%.2f Y=%.2f\r\n"
        "  Remote:  %s T=%.2f R=%.2f P=%.2f Y=%.2f\r\n"
        "  RC stat: frames=%lu drops=%lu\r\n"
        "  Battery:  %.2fV (%s)\r\n"
        "  Uptime:   %lu ms\r\n",
        app_is_armed() ? "ARMED" : "DISARMED",
        app_is_flying() ? " FLYING" : "",
        app_is_emergency() ? " EMERGENCY" : "",
        att->roll, att->pitch, att->yaw,
        rcst.connected ? "LINK" : "LOST",
        rc.throttle, rc.roll, rc.pitch, rc.yaw,
        (unsigned long)rcst.frame_count,
        (unsigned long)rcst.drop_count,
        battery_get_voltage(),
        battery_get_status() == BATTERY_OK ? "OK" :
        battery_get_status() == BATTERY_WARNING ? "WARN" : "CRIT",
        (unsigned long)HAL_GetTick()
    );
}

static void cmd_pid_show(void)
{
    attitude_ctrl_t *ac = app_get_att_ctrl();
    if (!ac) {
        printf("\r\n[ERR] controller not ready\r\n");
        return;
    }

    static const char *axis_names[] = {"Roll", "Pitch", "Yaw"};
    static const char *type_names[] = {"Rate ", "Angle"};

    printf("\r\n=== PID Parameters ===\r\n");
    for (int axis = 0; axis < 3; axis++) {
        for (int type = 0; type < 2; type++) {
            pacer_pid_t *pid = pid_for_axis(ac, axis, type);
            if (!pid) continue;
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

    int axis = -1;
    if (strcmp(argv[1], "roll") == 0)  axis = 0;
    else if (strcmp(argv[1], "pitch") == 0) axis = 1;
    else if (strcmp(argv[1], "yaw") == 0)   axis = 2;
    if (axis < 0) {
        printf("\r\n[ERR] unknown axis: %s\r\n", argv[1]);
        return;
    }

    int type = -1;
    if (strcmp(argv[2], "rate") == 0)   type = 0;
    else if (strcmp(argv[2], "angle") == 0) type = 1;
    if (type < 0) {
        printf("\r\n[ERR] unknown type: %s\r\n", argv[2]);
        return;
    }

    const char *param = argv[3];
    float value = strtof(argv[4], NULL);
    if (value < 0.0f || value > 100.0f) {
        printf("\r\n[ERR] value out of range: %s\r\n", argv[4]);
        return;
    }

    attitude_ctrl_t *ac = app_get_att_ctrl();
    pacer_pid_t *pid = pid_for_axis(ac, axis, type);
    if (!pid) {
        printf("\r\n[ERR] no PID for %s %s\r\n", argv[1], argv[2]);
        return;
    }

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

static void cmd_reset_pid(void)
{
    attitude_ctrl_t *ac = app_get_att_ctrl();
    if (!ac) {
        printf("\r\n[ERR] controller not ready\r\n");
        return;
    }

    taskENTER_CRITICAL();
    attitude_init(ac);
    attitude_enable(ac, app_is_armed());
    taskEXIT_CRITICAL();
    printf("\r\n[OK] PID reset to defaults\r\n");
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

#define MAX_ARGS 8

static void cli_process_line(char *line)
{
    char *argv[MAX_ARGS];
    int argc = 0;

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
    } else if (strcmp(argv[0], "reset") == 0) {
        cmd_reset_pid();
    } else if (strcmp(argv[0], "calib") == 0) {
        cmd_calib();
    } else if (strcmp(argv[0], "reboot") == 0) {
        cmd_reboot();
    } else {
        printf("\r\n[ERR] unknown command: %s (type 'help')\r\n", argv[0]);
    }

    cli_prompt();
}

void cli_rx_byte_cb(uint8_t b)
{
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

    if (b == 0x7F || b == '\b') {
        if (cli_idx > 0) cli_idx--;
        printf("\b \b");
        return;
    }

    if (cli_idx < CLI_BUF_SIZE - 1) {
        cli_buf[cli_idx++] = (char)b;
        HAL_UART_Transmit(&huart2, &b, 1, 10);
    }
}

void cli_init(void)
{
    cli_idx = 0;
    printf("\r\n=== PACER CLI ready (type 'help') ===\r\n");
    cli_prompt();

    usart_debug_nvic_enable();
    HAL_UART_Receive_IT(&huart2, &cli_rx_byte, 1);
}
