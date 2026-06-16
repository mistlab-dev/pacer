/**
 * @file imu_debug.c
 * @brief IMU 调试工具集实现
 *
 * 离线可跑 (回放模式), 在线需要硬件 ICM20948。
 * 所有函数独立于飞控主循环，不操作电机。
 */

#include "app/imu_debug.h"
#include "app/config.h"
#include "sensor/imu.h"
#include "sensor/imu_icm20948.h"
#include "filter/filter.h"
#include "hal/hal_gpio.h"
#include "hal/hal_i2c.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <signal.h>
#include <unistd.h>
#include <time.h>

/* ================ 公共 ================ */

static volatile int g_running = 1;
static void on_sig(int s) { (void)s; g_running = 0; }

static float now_sec(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (float)ts.tv_sec + (float)ts.tv_nsec / 1e9f;
}

static uint64_t now_us(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000ULL + (uint64_t)ts.tv_nsec / 1000;
}

static int imu_startup(void)
{
    if (hal_gpio_init() < 0) {
        fprintf(stderr, "[IMU_DBG] gpio init failed\n");
        return -1;
    }
    imu_icm20948_register();
    imu_config_t cfg = IMU_CONFIG_DEFAULT;
    cfg.sample_rate_hz = CFG_IMU_SAMPLE_HZ;
    cfg.accel_range_g  = CFG_IMU_ACCEL_RANGE_G;
    cfg.gyro_range_dps = CFG_IMU_GYRO_RANGE_DPS;
    if (imu_init(&cfg) != 0) {
        fprintf(stderr, "[IMU_DBG] IMU init failed\n");
        hal_gpio_deinit();
        return -2;
    }
    return 0;
}

static void imu_shutdown(void)
{
    imu_deinit();
    hal_gpio_deinit();
}

/* ================ 1. 实时数据流 ================ */

static int dbg_stream(void)
{
    if (imu_startup() != 0) return -1;

    filter_config_t fcfg = FILTER_CONFIG_DEFAULT;
    filter_init(&fcfg);

    printf("\n╔══════════════════════════════════════════════════════════════╗\n");
    printf("║  IMU 实时数据流 — Ctrl+C 退出                               ║\n");
    printf("╚══════════════════════════════════════════════════════════════╝\n\n");

    float t0 = now_sec();
    int line = 0;

    while (g_running) {
        imu_sample_t s;
        if (imu_read(&s) != 0) {
            usleep(1000);
            continue;
        }

        attitude_t att = filter_update(&s, CFG_CONTROL_DT);
        float t = now_sec() - t0;

        if (line % 40 == 0) {
            printf("\n%-10s %-9s %-9s %-9s  %-9s %-9s %-9s  %-8s %-8s %-8s  %-6s\n",
                   "time", "ax(m/s²)", "ay", "az", "gx(°/s)", "gy", "gz",
                   "Roll°", "Pitch°", "Yaw°", "T°C");
            printf("──────────────────────────────────────────────────────────────────────────\n");
        }

        printf("%-10.3f %-9.3f %-9.3f %-9.3f  %-9.2f %-9.2f %-9.2f  %-8.2f %-8.2f %-8.2f  %-6.1f\n",
               t,
               s.accel.x, s.accel.y, s.accel.z,
               s.gyro.x,  s.gyro.y,  s.gyro.z,
               att.roll, att.pitch, att.yaw,
               s.temperature);

        line++;
        usleep(CFG_CONTROL_INTERVAL_US);
    }

    imu_shutdown();
    return 0;
}

/* ================ 2. 校准向导 ================ */

/* 简单统计 */
typedef struct {
    vec3_t mean;
    vec3_t std;
    vec3_t min;
    vec3_t max;
    int    n;
} vec3_stats_t;

static void stats_compute(vec3_stats_t *st, vec3_t *data, int n)
{
    st->n = n;
    vec3_t sum = {0}, sum2 = {0};
    st->min = data[0];
    st->max = data[0];

    for (int i = 0; i < n; i++) {
        sum.x += data[i].x; sum.y += data[i].y; sum.z += data[i].z;
        sum2.x += data[i].x * data[i].x;
        sum2.y += data[i].y * data[i].y;
        sum2.z += data[i].z * data[i].z;
        if (data[i].x < st->min.x) st->min.x = data[i].x;
        if (data[i].y < st->min.y) st->min.y = data[i].y;
        if (data[i].z < st->min.z) st->min.z = data[i].z;
        if (data[i].x > st->max.x) st->max.x = data[i].x;
        if (data[i].y > st->max.y) st->max.y = data[i].y;
        if (data[i].z > st->max.z) st->max.z = data[i].z;
    }

    st->mean.x = sum.x / n; st->mean.y = sum.y / n; st->mean.z = sum.z / n;
    st->std.x = sqrtf(sum2.x / n - st->mean.x * st->mean.x);
    st->std.y = sqrtf(sum2.y / n - st->mean.y * st->mean.y);
    st->std.z = sqrtf(sum2.z / n - st->mean.z * st->mean.z);
}

static void stats_print(const char *label, const vec3_stats_t *st)
{
    printf("  %s:\n", label);
    printf("    mean = [%+.4f, %+.4f, %+.4f]\n", st->mean.x, st->mean.y, st->mean.z);
    printf("    std  = [ %.4f,  %.4f,  %.4f]\n", st->std.x, st->std.y, st->std.z);
    printf("    min  = [%+.4f, %+.4f, %+.4f]\n", st->min.x, st->min.y, st->min.z);
    printf("    max  = [%+.4f, %+.4f, %+.4f]\n", st->max.x, st->max.y, st->max.z);
}

#define MAX_CAL_SAMPLES 2000

static int dbg_calibrate(void)
{
    if (imu_startup() != 0) return -1;

    printf("\n╔════════════════════════════════════════════════════╗\n");
    printf("║  IMU 校准向导                                      ║\n");
    printf("╚════════════════════════════════════════════════════╝\n");

    vec3_t *gyro_data  = malloc(sizeof(vec3_t) * MAX_CAL_SAMPLES);
    vec3_t *accel_data = malloc(sizeof(vec3_t) * MAX_CAL_SAMPLES);
    vec3_t *mag_data   = malloc(sizeof(vec3_t) * MAX_CAL_SAMPLES);

    if (!gyro_data || !accel_data || !mag_data) {
        fprintf(stderr, "malloc failed\n");
        free(gyro_data); free(accel_data); free(mag_data);
        imu_shutdown();
        return -1;
    }

    /* ---- Step 1: 陀螺仪零偏 ---- */
    printf("\n━━ Step 1/3: 陀螺仪零偏 ━━\n");
    printf("请保持传感器完全静止, 按 Enter 开始采样...\n");
    getchar();

    printf("采样中 (约 5 秒)...\n");
    int gyro_n = 0;
    float t0 = now_sec();
    while (gyro_n < MAX_CAL_SAMPLES && (now_sec() - t0) < 5.0f && g_running) {
        imu_sample_t s;
        if (imu_read(&s) == 0) {
            gyro_data[gyro_n++] = s.gyro;
        }
        usleep(2500);
    }

    vec3_stats_t gyro_st;
    stats_compute(&gyro_st, gyro_data, gyro_n);
    stats_print("陀螺仪 (°/s)", &gyro_st);

    vec3_t gyro_bias = gyro_st.mean;
    printf("\n  → 陀螺零偏: [%.4f, %.4f, %.4f] °/s\n", gyro_bias.x, gyro_bias.y, gyro_bias.z);
    imu_set_gyro_bias(&gyro_bias);

    if (gyro_st.std.x > 1.0f || gyro_st.std.y > 1.0f || gyro_st.std.z > 1.0f) {
        printf("  ⚠️  陀螺噪声偏大 (std > 1°/s), 可能有振动\n");
    } else {
        printf("  ✅ 陀螺噪声正常\n");
    }

    /* ---- Step 2: 加速度计零偏 ---- */
    printf("\n━━ Step 2/3: 加速度计零偏 ━━\n");
    printf("请将传感器水平放置 (Z轴朝上), 按 Enter 开始...\n");
    getchar();

    printf("采样中 (约 3 秒)...\n");
    int accel_n = 0;
    t0 = now_sec();
    while (accel_n < MAX_CAL_SAMPLES && (now_sec() - t0) < 3.0f && g_running) {
        imu_sample_t s;
        if (imu_read(&s) == 0) {
            accel_data[accel_n++] = s.accel;
        }
        usleep(2500);
    }

    vec3_stats_t accel_st;
    stats_compute(&accel_st, accel_data, accel_n);
    stats_print("加速度计 (m/s²)", &accel_st);

    float z_error = fabsf(accel_st.mean.z - 9.80665f);
    printf("\n  Z 轴重力误差: %.4f m/s² (%.2f%%)\n", z_error, z_error / 9.80665f * 100.0f);
    if (z_error < 0.1f) {
        printf("  ✅ 加速度计水平校准良好\n");
    } else if (z_error < 0.5f) {
        printf("  ⚠️  加速度计有轻微偏差\n");
    } else {
        printf("  ❌ 加速度计偏差较大, 检查安装角度\n");
    }

    float tilt_roll  = atan2f(accel_st.mean.y, accel_st.mean.z) * 180.0f / (float)M_PI;
    float tilt_pitch = atan2f(-accel_st.mean.x,
                              sqrtf(accel_st.mean.y * accel_st.mean.y + accel_st.mean.z * accel_st.mean.z))
                       * 180.0f / (float)M_PI;
    printf("  当前倾斜: Roll=%.2f° Pitch=%.2f°\n", tilt_roll, tilt_pitch);

    /* ---- Step 3: 磁力计 ---- */
    printf("\n━━ Step 3/3: 磁力计 ━━\n");
    printf("可选: 水平缓慢旋转传感器 360°。按 Enter 开始, 's' 跳过...\n");

    int ch = getchar();
    if (ch != 's' && ch != 'S') {
        printf("采样中 — 请缓慢旋转 (约 15 秒)...\n");
        int mag_n = 0;
        t0 = now_sec();
        while (mag_n < MAX_CAL_SAMPLES && (now_sec() - t0) < 15.0f && g_running) {
            imu_sample_t s;
            if (imu_read(&s) == 0) {
                mag_data[mag_n++] = s.mag;
            }
            usleep(2500);
        }

        if (mag_n > 100) {
            vec3_stats_t mag_st;
            stats_compute(&mag_st, mag_data, mag_n);
            stats_print("磁力计 (µT)", &mag_st);

            float range_x = mag_st.max.x - mag_st.min.x;
            float range_y = mag_st.max.y - mag_st.min.y;
            float range_z = mag_st.max.z - mag_st.min.z;
            printf("  轴范围: X=%.1f  Y=%.1f  Z=%.1f µT\n", range_x, range_y, range_z);

            if (range_x < 10.0f || range_y < 10.0f) {
                printf("  ⚠️  磁力计范围偏小, 可能有磁干扰\n");
            } else {
                printf("  ✅ 磁力计范围正常\n");
            }
        }
    } else {
        printf("  已跳过磁力计校准\n");
    }

    /* ---- 总结 ---- */
    printf("\n════════════════════════════════════════\n");
    printf("  校准结果摘要:\n");
    printf("  陀螺零偏: [%.4f, %.4f, %.4f] °/s\n", gyro_bias.x, gyro_bias.y, gyro_bias.z);
    printf("  加速度计 Z: %.4f m/s² (误差 %.2f%%)\n", accel_st.mean.z, z_error / 9.80665f * 100.0f);
    printf("\n  建议将陀螺零偏写入 config:\n");
    printf("    gyro_bias = {%.4ff, %.4ff, %.4ff}\n", gyro_bias.x, gyro_bias.y, gyro_bias.z);
    printf("════════════════════════════════════════\n");

    free(gyro_data);
    free(accel_data);
    free(mag_data);
    imu_shutdown();
    return 0;
}

/* ================ 3. 自检诊断 ================ */

static int dbg_diagnose(void)
{
    printf("\n╔══════════════════════════════════════════╗\n");
    printf("║  IMU 自检诊断                            ║\n");
    printf("╚══════════════════════════════════════════╝\n\n");

    int pass = 0, fail = 0;

    /* 1. GPIO */
    printf("[1/7] GPIO 初始化... ");
    if (hal_gpio_init() == 0) { printf("✅\n"); pass++; }
    else { printf("❌ (root?)\n"); fail++; return -1; }

    /* 2. I2C 探测 */
    printf("[2/7] I2C 总线扫描... ");
    {
        i2c_dev_t probe;
        bool found = false;
        for (uint8_t addr = 0x68; addr <= 0x69; addr++) {
            if (i2c_open(&probe, 1, addr) == 0) {
                uint8_t reg = 0;
                if (i2c_read_reg(&probe, 0x00, &reg) == 0) {
                    printf("0x%02X@0x%02X ", reg, addr);
                    found = true;
                }
                i2c_close(&probe);
            }
        }
        if (found) { printf("✅\n"); pass++; }
        else       { printf("❌ 无设备\n"); fail++; }
    }

    /* 3. IMU 初始化 */
    printf("[3/7] ICM20948 初始化... ");
    imu_icm20948_register();
    imu_config_t cfg = IMU_CONFIG_DEFAULT;
    int rc = imu_init(&cfg);
    if (rc == 0) { printf("✅\n"); pass++; }
    else { printf("❌ (rc=%d)\n", rc); fail++; hal_gpio_deinit(); return -1; }

    /* 4. WHO_AM_I */
    printf("[4/7] WHO_AM_I... ");
    rc = imu_self_test();
    if (rc == 0) { printf("✅ (0xEA)\n"); pass++; }
    else         { printf("❌\n"); fail++; }

    /* 5. 数据读取 */
    printf("[5/7] 数据读取测试 (10 帧)... ");
    int ok = 0;
    imu_sample_t s;
    for (int i = 0; i < 10; i++) {
        if (imu_read(&s) == 0) ok++;
        usleep(3000);
    }
    if (ok == 10) { printf("✅ (10/10)\n"); pass++; }
    else          { printf("❌ (%d/10)\n", ok); fail++; }

    /* 6. 数据合理性 */
    printf("[6/7] 数据合理性... ");
    {
        float accel_mag = sqrtf(s.accel.x * s.accel.x +
                                s.accel.y * s.accel.y +
                                s.accel.z * s.accel.z);
        bool accel_ok = (accel_mag > 8.0f && accel_mag < 12.0f);
        bool gyro_ok  = (fabsf(s.gyro.x) < 50.0f &&
                         fabsf(s.gyro.y) < 50.0f &&
                         fabsf(s.gyro.z) < 50.0f);
        bool temp_ok  = (s.temperature > -10.0f && s.temperature < 60.0f);

        printf("accel=%.2f%s gyro=[%.1f,%.1f,%.1f]%s temp=%.1f°C%s\n",
               accel_mag, accel_ok ? "✅" : "⚠️",
               s.gyro.x, s.gyro.y, s.gyro.z, gyro_ok ? "✅" : "⚠️",
               s.temperature, temp_ok ? "✅" : "⚠️");
        if (accel_ok && gyro_ok && temp_ok) pass++; else fail++;
    }

    /* 7. 采样率 */
    printf("[7/7] 实际采样率... ");
    {
        #define RATE_TEST_N 200
        uint64_t t_start = now_us();
        int rate_ok = 0;
        for (int i = 0; i < RATE_TEST_N; i++) {
            imu_sample_t tmp;
            if (imu_read(&tmp) == 0) rate_ok++;
            usleep(2400);
        }
        uint64_t t_end = now_us();
        float actual_hz = (float)rate_ok / ((float)(t_end - t_start) / 1e6f);
        float err_pct = fabsf(actual_hz - (float)cfg.sample_rate_hz) / (float)cfg.sample_rate_hz * 100.0f;
        printf("%.1f Hz (目标 %d Hz, 误差 %.1f%%)%s\n",
               actual_hz, cfg.sample_rate_hz, err_pct,
               err_pct < 10.0f ? "✅" : "⚠️");
        if (err_pct < 10.0f) pass++; else fail++;
    }

    printf("\n════════════════════════════════════════\n");
    printf("  诊断结果: %d 通过, %d 失败 / 7\n", pass, fail);
    printf("%s\n", (fail == 0) ? "  ✅ 全部通过, IMU 正常" : "  ❌ 存在问题, 请检查硬件连接");
    printf("════════════════════════════════════════\n");

    imu_deinit();
    hal_gpio_deinit();
    return (fail > 0) ? 1 : 0;
}

/* ================ 4. 频谱分析 ================ */

/* 简易 FFT (Cooley-Tukey radix-2), N 必须是 2 的幂 */
static void fft_radix2(float *real, float *imag, int n)
{
    /* 位反转置换 */
    for (int i = 1, j = 0; i < n; i++) {
        int bit = n >> 1;
        while (j & bit) { j ^= bit; bit >>= 1; }
        j ^= bit;
        if (i < j) {
            float tr = real[i]; real[i] = real[j]; real[j] = tr;
            float ti = imag[i]; imag[i] = imag[j]; imag[j] = ti;
        }
    }

    /* 蝶形运算 */
    for (int len = 2; len <= n; len <<= 1) {
        float ang = -2.0f * (float)M_PI / (float)len;
        float w_re = cosf(ang), w_im = sinf(ang);
        for (int i = 0; i < n; i += len) {
            float cur_re = 1.0f, cur_im = 0.0f;
            for (int j = 0; j < len / 2; j++) {
                float u_re = real[i + j];
                float u_im = imag[i + j];
                float v_re = cur_re * real[i + j + len/2] - cur_im * imag[i + j + len/2];
                float v_im = cur_re * imag[i + j + len/2] + cur_im * real[i + j + len/2];
                real[i + j]         = u_re + v_re;
                imag[i + j]         = u_im + v_im;
                real[i + j + len/2] = u_re - v_re;
                imag[i + j + len/2] = u_im - v_im;
                float new_re = cur_re * w_re - cur_im * w_im;
                float new_im = cur_re * w_im + cur_im * w_re;
                cur_re = new_re; cur_im = new_im;
            }
        }
    }
}

static void print_spectrum(const float *mag, int n, float sample_hz, const char *label)
{
    printf("\n  %s 频谱 (采样率 %.0f Hz):\n", label, sample_hz);

    float max_mag = 0;
    for (int i = 1; i < n / 2; i++)
        if (mag[i] > max_mag) max_mag = mag[i];
    if (max_mag < 1e-6f) max_mag = 1.0f;

    /* Top 5 峰值 */
    int top_idx[5] = {0};
    float top_val[5] = {0};
    for (int i = 1; i < n / 2; i++) {
        for (int k = 0; k < 5; k++) {
            if (mag[i] > top_val[k]) {
                for (int m = 4; m > k; m--) {
                    top_idx[m] = top_idx[m-1];
                    top_val[m] = top_val[m-1];
                }
                top_idx[k] = i;
                top_val[k] = mag[i];
                break;
            }
        }
    }

    printf("  Top 峰值:\n");
    for (int k = 0; k < 5; k++) {
        if (top_val[k] > 0) {
            float freq = (float)top_idx[k] * sample_hz / (float)n;
            int bar_len = (int)(top_val[k] / max_mag * 40);
            printf("    %7.1f Hz |", freq);
            for (int b = 0; b < bar_len; b++) printf("█");
            printf(" %.2f\n", top_val[k]);
        }
    }

    /* 0~200Hz 概览 */
    printf("\n  0~200Hz 概览:\n");
    float hz_per_bin = sample_hz / (float)n;
    for (int band = 0; band < 20; band++) {
        int lo_bin = (int)((float)(band * 10) / hz_per_bin);
        int hi_bin = (int)((float)((band + 1) * 10) / hz_per_bin);
        if (hi_bin >= n / 2) hi_bin = n / 2 - 1;
        float band_max = 0;
        for (int i = lo_bin; i <= hi_bin; i++)
            if (mag[i] > band_max) band_max = mag[i];
        int bar_len = (int)(band_max / max_mag * 30);
        printf("    %3d-%3d Hz |", band * 10, (band + 1) * 10);
        for (int b = 0; b < bar_len; b++) printf("▓");
        printf(" %.2f\n", band_max);
    }
}

#define FFT_N 1024

static int dbg_spectrum(void)
{
    if (imu_startup() != 0) return -1;

    printf("\n╔══════════════════════════════════════════╗\n");
    printf("║  IMU 频谱分析 — Ctrl+C 提前结束          ║\n");
    printf("╚══════════════════════════════════════════╝\n\n");

    float gx[FFT_N], gy[FFT_N], gz[FFT_N];
    float ax[FFT_N], ay[FFT_N], az[FFT_N];
    float imag_buf[FFT_N];
    int collected = 0;

    printf("采集 %d 个样本 (约 %.1f 秒)...\n", FFT_N, (float)FFT_N / CFG_IMU_SAMPLE_HZ);

    while (collected < FFT_N && g_running) {
        imu_sample_t s;
        if (imu_read(&s) == 0) {
            gx[collected] = s.gyro.x; gy[collected] = s.gyro.y; gz[collected] = s.gyro.z;
            ax[collected] = s.accel.x; ay[collected] = s.accel.y; az[collected] = s.accel.z;
            collected++;
        }
        usleep(2400);
    }

    if (collected < FFT_N) {
        printf("采集中断, 仅 %d 样本\n", collected);
        imu_shutdown();
        return -1;
    }

    printf("采集完成, 计算 FFT...\n");

    /* Hann 窗 */
    for (int i = 0; i < FFT_N; i++) {
        float w = 0.5f * (1.0f - cosf(2.0f * (float)M_PI * i / (float)(FFT_N - 1)));
        gx[i] *= w; gy[i] *= w; gz[i] *= w;
        ax[i] *= w; ay[i] *= w; az[i] *= w;
    }

    float mag_gx[FFT_N/2], mag_gy[FFT_N/2], mag_gz[FFT_N/2];
    float mag_ax[FFT_N/2], mag_ay[FFT_N/2], mag_az[FFT_N/2];

    const float *channels[] = { gx, gy, gz, ax, ay, az };
    float *mags[] = { mag_gx, mag_gy, mag_gz, mag_ax, mag_ay, mag_az };
    const char *names[] = { "Gyro X", "Gyro Y", "Gyro Z", "Accel X", "Accel Y", "Accel Z" };

    for (int c = 0; c < 6; c++) {
        memset(imag_buf, 0, sizeof(float) * FFT_N);
        fft_radix2((float *)channels[c], imag_buf, FFT_N);
        for (int i = 0; i < FFT_N / 2; i++) {
            float re = channels[c][i];
            float im = imag_buf[i];
            mags[c][i] = sqrtf(re * re + im * im) / (float)FFT_N * 2.0f;
        }
        print_spectrum(mags[c], FFT_N, (float)CFG_IMU_SAMPLE_HZ, names[c]);
    }

    imu_shutdown();
    return 0;
}

/* ================ 5. CSV 数据记录 ================ */

static int dbg_record(const char *filepath)
{
    if (imu_startup() != 0) return -1;

    const char *path = (filepath && filepath[0]) ? filepath : "imu_log.csv";

    printf("\n╔══════════════════════════════════════════╗\n");
    printf("║  IMU 数据记录 → %s\n", path);
    printf("║  Ctrl+C 停止记录                         ║\n");
    printf("╚══════════════════════════════════════════╝\n\n");

    FILE *fp = fopen(path, "w");
    if (!fp) {
        fprintf(stderr, "无法打开 %s\n", path);
        imu_shutdown();
        return -1;
    }

    fprintf(fp, "time_us,ax,ay,az,gx,gy,gz,mx,my,mz,temp\n");

    int samples = 0;
    uint64_t t0 = now_us();

    while (g_running) {
        imu_sample_t s;
        if (imu_read(&s) == 0) {
            fprintf(fp, "%llu,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.2f,%.2f,%.2f,%.2f\n",
                    (unsigned long long)(s.timestamp_us - t0),
                    s.accel.x, s.accel.y, s.accel.z,
                    s.gyro.x,  s.gyro.y,  s.gyro.z,
                    s.mag.x,   s.mag.y,   s.mag.z,
                    s.temperature);
            samples++;
            if (samples % 400 == 0) {
                printf("  已记录 %d 帧 (%.1f 秒)\n", samples, (float)samples / CFG_IMU_SAMPLE_HZ);
                fflush(fp);
            }
        }
        usleep(2400);
    }

    fclose(fp);
    printf("\n  共记录 %d 帧 → %s\n", samples, path);

    imu_shutdown();
    return 0;
}

/* ================ 6. CSV 回放 + 滤波器对比 ================ */

#define DEG2RAD_F(x) ((x) * (float)M_PI / 180.0f)
#define RAD2DEG_F(x) ((x) * 180.0f / (float)M_PI)

static void madgwick_step(float q[4], const imu_sample_t *s, float dt, float beta)
{
    float w = q[0], x = q[1], y = q[2], z = q[3];
    float gx = DEG2RAD_F(s->gyro.x), gy = DEG2RAD_F(s->gyro.y), gz = DEG2RAD_F(s->gyro.z);
    float ax = s->accel.x, ay = s->accel.y, az = s->accel.z;

    float norm = sqrtf(ax*ax + ay*ay + az*az);
    if (norm < 0.001f) return;
    ax /= norm; ay /= norm; az /= norm;

    float hx = 2*(x*z - w*y), hy = 2*(w*x + y*z), hz = w*w - x*x - y*y + z*z;
    float ex = ay*hz - az*hy, ey = az*hx - ax*hz, ez = ax*hy - ay*hx;
    float e_norm = sqrtf(ex*ex + ey*ey + ez*ez);
    if (e_norm > 0.001f) { ex /= e_norm; ey /= e_norm; ez /= e_norm; }

    gx += beta*ex; gy += beta*ey; gz += beta*ez;

    float h_dt = 0.5f * dt;
    w += (-x*gx - y*gy - z*gz) * h_dt;
    x += ( w*gx + y*gz - z*gy) * h_dt;
    y += ( w*gy - x*gz + z*gx) * h_dt;
    z += ( w*gz + x*gy - y*gx) * h_dt;

    norm = sqrtf(w*w + x*x + y*y + z*z);
    q[0]=w/norm; q[1]=x/norm; q[2]=y/norm; q[3]=z/norm;
}

static void madgwick_to_euler(const float q[4], float *roll, float *pitch, float *yaw)
{
    *roll  = RAD2DEG_F(atan2f(2*(q[0]*q[1]+q[2]*q[3]), 1-2*(q[1]*q[1]+q[2]*q[2])));
    float sp = 2*(q[0]*q[2]-q[3]*q[1]);
    if (sp > 1) sp = 1;
    if (sp < -1) sp = -1;
    *pitch = RAD2DEG_F(asinf(sp));
    *yaw   = RAD2DEG_F(atan2f(2*(q[0]*q[3]+q[1]*q[2]), 1-2*(q[2]*q[2]+q[3]*q[3])));
}

static int dbg_replay(const char *filepath)
{
    const char *path = (filepath && filepath[0]) ? filepath : "imu_log.csv";

    printf("\n╔══════════════════════════════════════════╗\n");
    printf("║  IMU CSV 回放+滤波对比 — %s\n", path);
    printf("╚══════════════════════════════════════════╝\n\n");

    FILE *fp = fopen(path, "r");
    if (!fp) {
        fprintf(stderr, "无法打开 %s\n", path);
        return -1;
    }

    char line[512];
    if (!fgets(line, sizeof(line), fp)) { fclose(fp); return -1; }

    float dt = 1.0f / (float)CFG_IMU_SAMPLE_HZ;

    /* 两套滤波器状态 */
    float mq[4] = {1, 0, 0, 0};    /* Madgwick */
    float cr = 0, cp = 0, cy = 0;  /* 互补 */
    float alpha = 0.98f;
    float beta = 0.04f;

    printf("%-10s  %-8s %-8s %-8s  %-8s %-8s %-8s  %-9s %-9s %-9s\n",
           "time(ms)", "R_mad", "P_mad", "Y_mad", "R_cmp", "P_cmp", "Y_cmp", "ax", "ay", "az");
    printf("──────────────────────────────────────────────────────────────────────────────\n");

    int count = 0;
    float mr = 0, mp = 0, myaw = 0;  /* Madgwick 结果 (循环外可见) */

    while (fgets(line, sizeof(line), fp) && g_running) {
        imu_sample_t s = {0};
        unsigned long long tus = 0;
        int n = sscanf(line, "%llu,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f",
                       &tus,
                       &s.accel.x, &s.accel.y, &s.accel.z,
                       &s.gyro.x,  &s.gyro.y,  &s.gyro.z,
                       &s.mag.x,   &s.mag.y,   &s.mag.z,
                       &s.temperature);
        if (n < 7) continue;

        /* Madgwick */
        madgwick_step(mq, &s, dt, beta);
        madgwick_to_euler(mq, &mr, &mp, &myaw);

        /* 互补滤波 */
        float acc_roll  = RAD2DEG_F(atan2f(s.accel.y, s.accel.z));
        float acc_pitch = RAD2DEG_F(atan2f(-s.accel.x,
                              sqrtf(s.accel.y*s.accel.y + s.accel.z*s.accel.z)));
        cr = alpha * (cr + s.gyro.x * dt) + (1-alpha) * acc_roll;
        cp = alpha * (cp + s.gyro.y * dt) + (1-alpha) * acc_pitch;
        cy += s.gyro.z * dt;

        if (count % 80 == 0) {
            printf("%-10llu  %-8.2f %-8.2f %-8.2f  %-8.2f %-8.2f %-8.2f  %-9.3f %-9.3f %-9.3f\n",
                   tus / 1000,
                   mr, mp, myaw,
                   cr, cp, cy,
                   s.accel.x, s.accel.y, s.accel.z);
        }
        count++;
    }

    fclose(fp);
    printf("\n  回放完成: %d 帧\n", count);
    printf("  最终姿态 (Madgwick): R=%.2f° P=%.2f° Y=%.2f°\n", mr, mp, myaw);
    printf("  最终姿态 (互补):    R=%.2f° P=%.2f° Y=%.2f°\n", cr, cp, cy);
    return 0;
}

/* ================ 7. 静态噪声分析 ================ */

static int dbg_noise(void)
{
    if (imu_startup() != 0) return -1;

    printf("\n╔══════════════════════════════════════════╗\n");
    printf("║  IMU 静态噪声分析 — 请保持静止           ║\n");
    printf("╚══════════════════════════════════════════╝\n\n");

    #define NOISE_N 2000
    vec3_t gyro[NOISE_N], accel[NOISE_N];
    int n = 0;

    printf("采集 %d 帧 (约 5 秒)...\n", NOISE_N);

    while (n < NOISE_N && g_running) {
        imu_sample_t s;
        if (imu_read(&s) == 0) {
            gyro[n]  = s.gyro;
            accel[n] = s.accel;
            n++;
        }
        usleep(2400);
    }

    if (n < 100) {
        printf("样本不足\n");
        imu_shutdown();
        return -1;
    }

    vec3_stats_t gs, as;
    stats_compute(&gs, gyro, n);
    stats_compute(&as, accel, n);

    printf("\n━━ 陀螺仪噪声 (°/s) ━━\n");
    stats_print("统计", &gs);
    printf("\n  RMS (std):  X=%.4f  Y=%.4f  Z=%.4f\n", gs.std.x, gs.std.y, gs.std.z);
    printf("  Peak-peak: X=%.3f  Y=%.3f  Z=%.3f\n",
           gs.max.x - gs.min.x, gs.max.y - gs.min.y, gs.max.z - gs.min.z);

    /* 噪声密度 (°/s/√Hz) */
    float bw = (float)CFG_IMU_SAMPLE_HZ / 2.0f;  /* Nyquist */
    printf("  噪声密度:  X=%.5f  Y=%.5f  Z=%.5f  °/s/√Hz\n",
           gs.std.x / sqrtf(bw), gs.std.y / sqrtf(bw), gs.std.z / sqrtf(bw));

    printf("\n━━ 加速度计噪声 (m/s²) ━━\n");
    stats_print("统计", &as);
    printf("\n  RMS (std):  X=%.4f  Y=%.4f  Z=%.4f\n", as.std.x, as.std.y, as.std.z);
    printf("  Peak-peak: X=%.3f  Y=%.3f  Z=%.3f\n",
           as.max.x - as.min.x, as.max.y - as.min.y, as.max.z - as.min.z);
    printf("  噪声密度:  X=%.5f  Y=%.5f  Z=%.5f  m/s²/√Hz\n",
           as.std.x / sqrtf(bw), as.std.y / sqrtf(bw), as.std.z / sqrtf(bw));

    /* 质量评定 */
    printf("\n━━ 质量评定 ━━\n");
    float gyro_rms_max = sqrtf(gs.std.x*gs.std.x + gs.std.y*gs.std.y + gs.std.z*gs.std.z);
    if (gyro_rms_max < 0.1f) {
        printf("  ✅ 陀螺噪声优秀 (RMS %.3f°/s < 0.1)\n", gyro_rms_max);
    } else if (gyro_rms_max < 0.5f) {
        printf("  ⚠️  陀螺噪声可接受 (RMS %.3f°/s)\n", gyro_rms_max);
    } else {
        printf("  ❌ 陀螺噪声偏大 (RMS %.3f°/s > 0.5), 检查振动\n", gyro_rms_max);
    }

    float accel_rms = sqrtf(as.std.x*as.std.x + as.std.y*as.std.y + as.std.z*as.std.z);
    if (accel_rms < 0.02f) {
        printf("  ✅ 加速度计噪声优秀 (RMS %.4f m/s²)\n", accel_rms);
    } else if (accel_rms < 0.1f) {
        printf("  ⚠️  加速度计噪声可接受 (RMS %.4f m/s²)\n", accel_rms);
    } else {
        printf("  ❌ 加速度计噪声偏大 (RMS %.4f m/s²)\n", accel_rms);
    }

    printf("\n");
    imu_shutdown();
    return 0;
}

/* ================ 入口 ================ */

int imu_debug_run(imu_debug_mode_t mode, const char *arg)
{
    signal(SIGINT,  on_sig);
    signal(SIGTERM, on_sig);

    switch (mode) {
        case IMU_DBG_STREAM:    return dbg_stream();
        case IMU_DBG_CALIBRATE: return dbg_calibrate();
        case IMU_DBG_DIAGNOSE:  return dbg_diagnose();
        case IMU_DBG_SPECTRUM:  return dbg_spectrum();
        case IMU_DBG_RECORD:    return dbg_record(arg);
        case IMU_DBG_REPLAY:    return dbg_replay(arg);
        case IMU_DBG_COMPARE:   return dbg_replay(arg);  /* 回放自带对比 */
        case IMU_DBG_NOISE:     return dbg_noise();
    }
    return 1;
}
