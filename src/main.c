/**
 * @file main.c
 * @brief 入口 — 解析参数, 调用 app_run 或 imu_debug_run
 */

#include "app/app.h"
#include "app/imu_debug.h"
#include <stdio.h>
#include <string.h>

static void usage(const char *prog)
{
    printf("Pacer v3.0 — 四旋翼无人机 (RPi Zero 2W + ICM20948)\n\n");
    printf("用法: sudo %s [选项] [参数]\n\n", prog);
    printf("飞行模式:\n");
    printf("  (无参数)        启动飞控\n");
    printf("  --calibrate     校准 IMU 陀螺仪零偏 (简易)\n");
    printf("  --esc-cal       校准 ESC 油门行程\n");
    printf("  --debug         调试模式 (打印姿态, 不驱动电机)\n\n");
    printf("IMU 调试工具:\n");
    printf("  --imu-stream          实时数据流 (6轴 + 姿态角)\n");
    printf("  --imu-calibrate       完整校准向导 (陀螺+加速度+磁力计)\n");
    printf("  --imu-diagnose        自检诊断 (7项检查)\n");
    printf("  --imu-spectrum        频谱分析 (FFT, 找共振点)\n");
    printf("  --imu-record [file]   CSV 数据记录 (默认 imu_log.csv)\n");
    printf("  --imu-replay  <file>  CSV 回放 (Madgwick vs 互补滤波对比)\n");
    printf("  --imu-noise           静态噪声分析 (RMS + 噪声密度)\n\n");
    printf("  -h, --help            显示帮助\n");
}

int main(int argc, char *argv[])
{
    /* IMU 调试子命令 */
    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "--imu-stream") == 0) {
            return imu_debug_run(IMU_DBG_STREAM, NULL);
        }
        if (strcmp(argv[i], "--imu-calibrate") == 0) {
            return imu_debug_run(IMU_DBG_CALIBRATE, NULL);
        }
        if (strcmp(argv[i], "--imu-diagnose") == 0) {
            return imu_debug_run(IMU_DBG_DIAGNOSE, NULL);
        }
        if (strcmp(argv[i], "--imu-spectrum") == 0) {
            return imu_debug_run(IMU_DBG_SPECTRUM, NULL);
        }
        if (strcmp(argv[i], "--imu-record") == 0) {
            const char *file = (i + 1 < argc && argv[i+1][0] != '-') ? argv[i+1] : NULL;
            return imu_debug_run(IMU_DBG_RECORD, file);
        }
        if (strcmp(argv[i], "--imu-replay") == 0 || strcmp(argv[i], "--imu-compare") == 0) {
            const char *file = (i + 1 < argc) ? argv[i+1] : NULL;
            return imu_debug_run(IMU_DBG_REPLAY, file);
        }
        if (strcmp(argv[i], "--imu-noise") == 0) {
            return imu_debug_run(IMU_DBG_NOISE, NULL);
        }
    }

    /* 飞行模式 */
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
        } else if (argv[i][0] == '-' && argv[i][1] != '\0') {
            /* 跳过 --imu-record 的文件参数等 */
            if (strncmp(argv[i], "--imu", 5) == 0) {
                /* 已在上面处理, 跳过 */
                continue;
            }
            fprintf(stderr, "未知参数: %s\n", argv[i]);
            usage(argv[0]);
            return 1;
        }
    }

    return app_run(mode);
}
