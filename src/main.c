/**
 * @file main.c
 * @brief 入口 — 解析参数, 调用 app_run
 */

#include "app/app.h"
#include <stdio.h>
#include <string.h>

static void usage(const char *prog)
{
    printf("Pacer v3.0 — 四旋翼无人机 (RPi Zero 2W + ICM20948)\n\n");
    printf("用法: sudo %s [选项]\n\n", prog);
    printf("选项:\n");
    printf("  --calibrate   校准 IMU 陀螺仪零偏\n");
    printf("  --esc-cal     校准 ESC 油门行程\n");
    printf("  --debug       调试模式 (打印姿态, 不驱动电机)\n");
    printf("  -h, --help    显示帮助\n\n");
    printf("默认: 启动飞控\n");
}

int main(int argc, char *argv[])
{
    app_mode_t mode = APP_MODE_RUN;

    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "--calibrate") == 0) {
            mode = APP_MODE_CALIBRATE;
        } else if (strcmp(argv[i], "--esc-cal") == 0) {
            mode = APP_MODE_ESC_CAL;
        } else if (strcmp(argv[i], "--debug") == 0) {
            mode = APP_MODE_DEBUG;
        } else if (strcmp(argv[i], "-h") == 0 || strcmp(argv[i], "--help") == 0) {
            usage(argv[0]);
            return 0;
        } else {
            fprintf(stderr, "未知参数: %s\n", argv[i]);
            usage(argv[0]);
            return 1;
        }
    }

    return app_run(mode);
}
