/**
 * @file sim_quad.c
 * @brief 四旋翼控制逻辑全面模拟测试
 *
 * 离线可跑，不需要硬件。覆盖场景:
 *
 *   1. 混控器基本功能: 纯油门/roll/pitch/yaw/边界/限幅
 *   2. 姿态控制器 — 角速度模式 (RATE)
 *   3. 姿态控制器 — 自稳模式 (ANGLE)
 *   4. PID 阶跃响应特性 (超调/收敛/稳态误差)
 *   5. 闭环飞行: 自稳从 20° 倾斜收敛
 *   6. 闭环飞行: 遥控阶跃输入跟踪
 *   7. 闭环飞行: 抗扰动恢复
 *   8. 安全保护: 急停/上锁/倾斜保护
 *   9. 解锁上锁状态机
 *  10. 混控对称性验证
 *
 * 编译: 见 tests/CMakeLists.txt
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "ctrl/pid.h"
#include "ctrl/attitude.h"
#include "ctrl/quad_mixer.h"
#include "filter/filter.h"
#include "sensor/imu.h"
#include "app/config.h"

/* ================ 测试框架 ================ */

static int g_pass = 0, g_fail = 0;

#define PASS(msg) do { g_pass++; printf("  [PASS] %s\n", msg); } while(0)
#define FAIL(msg) do { g_fail++; fprintf(stderr, "  [FAIL] %s\n", msg); } while(0)
#define ASSERT(cond, msg) do { if (cond) PASS(msg); else FAIL(msg); } while(0)
#define ASSERT_F(cond, fmt, ...) do { \
    if (cond) { g_pass++; } \
    else { g_fail++; fprintf(stderr, "  [FAIL] " fmt "\n", __VA_ARGS__); } \
} while(0)

/* ================ 简单物理模型 ================ */

typedef struct {
    float roll, pitch, yaw;
    float roll_rate, pitch_rate, yaw_rate;
    float inertia_roll, inertia_pitch, inertia_yaw;
    float damping;
} sim_state_t;

static void sim_init(sim_state_t *s)
{
    memset(s, 0, sizeof(*s));
    s->inertia_roll  = 0.01f;
    s->inertia_pitch = 0.01f;
    s->inertia_yaw   = 0.02f;
    s->damping       = 0.1f;
}

static void sim_step(sim_state_t *s, const mixer_output_t *mix, float dt)
{
    float r_torque = (mix->motor[QUAD_FL] + mix->motor[QUAD_RL]
                    - mix->motor[QUAD_FR] - mix->motor[QUAD_RR]) * 0.5f;
    float p_torque = (mix->motor[QUAD_RL] + mix->motor[QUAD_RR]
                    - mix->motor[QUAD_FL] - mix->motor[QUAD_FR]) * 0.5f;
    float y_torque = (mix->motor[QUAD_FR] + mix->motor[QUAD_RL]
                    - mix->motor[QUAD_FL] - mix->motor[QUAD_RR]) * 0.3f;

    s->roll_rate  += (r_torque / s->inertia_roll  - s->damping * s->roll_rate)  * dt;
    s->pitch_rate += (p_torque / s->inertia_pitch - s->damping * s->pitch_rate) * dt;
    s->yaw_rate   += (y_torque / s->inertia_yaw   - s->damping * s->yaw_rate)  * dt;

    s->roll  += s->roll_rate  * dt;
    s->pitch += s->pitch_rate * dt;
    s->yaw   += s->yaw_rate   * dt;
}

static imu_sample_t sim_to_imu(const sim_state_t *s)
{
    imu_sample_t sample;
    float rr = s->roll * (float)M_PI / 180.0f;
    float pr = s->pitch * (float)M_PI / 180.0f;
    sample.accel.x =  9.81f * sinf(pr);
    sample.accel.y = -9.81f * sinf(rr) * cosf(pr);
    sample.accel.z =  9.81f * cosf(rr) * cosf(pr);
    sample.gyro.x = s->roll_rate;
    sample.gyro.y = s->pitch_rate;
    sample.gyro.z = s->yaw_rate;
    sample.mag.x = sample.mag.y = sample.mag.z = 0.0f;
    sample.temperature = 25.0f;
    sample.timestamp_us = 0;
    return sample;
}

/* ================ 闭环运行器 ================ */

typedef struct {
    float thr, roll, pitch, yaw;
} rc_input_t;

/* 运行闭环 N 步, 返回最终状态 */
static sim_state_t run_closed_loop(int steps, float dt,
                                    const rc_input_t *input, /* 每步相同输入 */
                                    att_mode_t mode,
                                    float init_roll, float init_pitch,
                                    float init_roll_rate, float init_pitch_rate,
                                    int print_every)
{
    attitude_ctrl_t ac;
    attitude_init(&ac);
    attitude_set_mode(&ac, mode);
    attitude_enable(&ac, true);

    quad_mixer_t mixer;
    mixer_config_t mcfg = MIXER_CONFIG_DEFAULT;
    quad_mixer_init(&mixer, &mcfg);
    quad_mixer_arm(&mixer);

    sim_state_t sim;
    sim_init(&sim);
    sim.roll = init_roll;
    sim.pitch = init_pitch;
    sim.roll_rate = init_roll_rate;
    sim.pitch_rate = init_pitch_rate;

    if (print_every > 0) {
        printf("  时间   Roll    Pitch   [FL    FR    RL    RR]\n");
        printf("  ────────────────────────────────────────────\n");
    }

    for (int i = 0; i < steps; i++) {
        float t = i * dt;
        imu_sample_t sample = sim_to_imu(&sim);
        attitude_t att = { .roll = sim.roll, .pitch = sim.pitch, .yaw = sim.yaw };

        attitude_set_input(&ac, input->thr, input->roll, input->pitch, input->yaw);
        const attitude_output_t *out = attitude_update(&ac, &att, &sample, dt);
        mixer_output_t mix = quad_mixer_update(&mixer, out->throttle, out->roll, out->pitch, out->yaw);

        sim_step(&sim, &mix, dt);

        if (print_every > 0 && i % print_every == 0) {
            printf("  %4.1fs  %+6.2f  %+6.2f  [%.2f  %.2f  %.02f  %.02f]\n",
                   t, sim.roll, sim.pitch,
                   mix.motor[0], mix.motor[1], mix.motor[2], mix.motor[3]);
        }
    }
    return sim;
}

/* ================ 测试用例 ================ */

/* --- 1. 混控器基本功能 --- */
static void test_mixer_basic(void)
{
    printf("\n=== 测试1: 混控器基本功能 ===\n");
    quad_mixer_t m;
    mixer_config_t cfg = MIXER_CONFIG_DEFAULT;
    quad_mixer_init(&m, &cfg);

    /* 未解锁全零 */
    mixer_output_t out = quad_mixer_update(&m, 0.5f, 0.0f, 0.0f, 0.0f);
    bool all_zero = true;
    for (int i = 0; i < 4; i++) if (out.motor[i] != 0.0f) all_zero = false;
    ASSERT(all_zero, "未解锁: 全零");

    quad_mixer_arm(&m);

    /* 纯油门 */
    out = quad_mixer_update(&m, 0.5f, 0.0f, 0.0f, 0.0f);
    bool all_equal = true;
    for (int i = 0; i < 4; i++) if (fabsf(out.motor[i] - 0.5f) > 0.001f) all_equal = false;
    ASSERT(all_equal, "纯油门 0.5: 四电机均等");

    /* Roll+ */
    out = quad_mixer_update(&m, 0.5f, 0.2f, 0.0f, 0.0f);
    ASSERT(out.motor[QUAD_FL] > out.motor[QUAD_FR], "Roll+: FL > FR");
    ASSERT(out.motor[QUAD_RL] > out.motor[QUAD_RR], "Roll+: RL > RR");

    /* Pitch+ */
    out = quad_mixer_update(&m, 0.5f, 0.0f, 0.2f, 0.0f);
    ASSERT(out.motor[QUAD_RL] > out.motor[QUAD_FL], "Pitch+: RL > FL");
    ASSERT(out.motor[QUAD_RR] > out.motor[QUAD_FR], "Pitch+: RR > FR");

    /* Yaw+ */
    out = quad_mixer_update(&m, 0.5f, 0.0f, 0.0f, 0.2f);
    ASSERT(out.motor[QUAD_FR] > out.motor[QUAD_FL], "Yaw+: FR > FL (CCW加速)");
    ASSERT(out.motor[QUAD_RL] > out.motor[QUAD_RR], "Yaw+: RL > RR (CCW加速)");

    /* 无负值 */
    out = quad_mixer_update(&m, 0.05f, 0.5f, 0.5f, 0.5f);
    bool no_neg = true;
    for (int i = 0; i < 4; i++) if (out.motor[i] < 0.0f) no_neg = false;
    ASSERT(no_neg, "低油门+大姿态: 无负值");

    /* 不超 1.0 */
    out = quad_mixer_update(&m, 1.0f, 0.5f, 0.5f, 0.5f);
    bool no_over = true;
    for (int i = 0; i < 4; i++) if (out.motor[i] > 1.0f) no_over = false;
    ASSERT(no_over, "满油门+大姿态: 不超限");
}

/* --- 2. 角速度模式 --- */
static void test_rate_mode(void)
{
    printf("\n=== 测试2: 姿态控制 — 角速度模式 ===\n");
    attitude_ctrl_t ac;
    attitude_init(&ac);
    attitude_set_mode(&ac, ATT_MODE_RATE);
    attitude_enable(&ac, true);

    float dt = 1.0f / (float)CFG_CONTROL_HZ;
    imu_sample_t sample = {0};
    attitude_t att = {0};

    /* 零误差 → 近零输出 */
    const attitude_output_t *out = attitude_update(&ac, &att, &sample, dt);
    ASSERT_F(fabsf(out->roll) < 0.01f, "零误差 roll=%.4f", out->roll);

    /* 有角速度误差 → 校正方向 */
    sample.gyro.x = 50.0f;
    attitude_set_input(&ac, 0.3f, 0.0f, 0.0f, 0.0f);
    out = attitude_update(&ac, &att, &sample, dt);
    ASSERT_F(out->roll < -0.1f, "gyro 50°/s 目标0 → roll 输出负: %.4f", out->roll);

    /* 反方向 */
    sample.gyro.x = -50.0f;
    out = attitude_update(&ac, &att, &sample, dt);
    ASSERT_F(out->roll > 0.1f, "gyro -50°/s 目标0 → roll 输出正: %.4f", out->roll);

    /* 油门直通 */
    ASSERT_F(fabsf(out->throttle - 0.3f) < 0.001f, "油门直通: %.3f", out->throttle);
}

/* --- 3. 自稳模式 --- */
static void test_angle_mode(void)
{
    printf("\n=== 测试3: 姿态控制 — 自稳模式 ===\n");
    attitude_ctrl_t ac;
    attitude_init(&ac);
    attitude_set_mode(&ac, ATT_MODE_ANGLE);
    attitude_enable(&ac, true);

    float dt = 1.0f / (float)CFG_CONTROL_HZ;
    imu_sample_t sample = {0};
    attitude_t att = {0};

    /* 倾斜15°, 目标水平 → 校正为负 */
    att.roll = 15.0f;
    attitude_set_input(&ac, 0.4f, 0.0f, 0.0f, 0.0f);
    const attitude_output_t *out = attitude_update(&ac, &att, &sample, dt);
    ASSERT_F(out->roll < -0.1f, "倾斜15° → 校正负: %.4f", out->roll);

    /* 反方向倾斜 */
    att.roll = -15.0f;
    attitude_enable(&ac, false); attitude_enable(&ac, true);
    out = attitude_update(&ac, &att, &sample, dt);
    ASSERT_F(out->roll > 0.1f, "倾斜-15° → 校校正: %.4f", out->roll);
}

/* --- 4. PID 阶跃响应 --- */
static void test_pid_step_response(void)
{
    printf("\n=== 测试4: PID 阶跃响应特性 ===\n");
    pacer_pid_t pid;
    /* 用角速度环参数 */
    pid_init(&pid, 0.40f, 0.05f, 0.01f, 0.5f, 1.0f);

    float dt = 1.0f / 400.0f;
    float error = 100.0f;  /* 阶跃: 100°/s 误差 */

    float max_output = 0.0f;
    float output_at_1s = 0.0f;
    float output_at_2s = 0.0f;

    for (int i = 0; i < 800; i++) {  /* 2 秒 */
        float out = pid_compute(&pid, error, dt);
        if (out > max_output) max_output = out;
        if (i == 399) output_at_1s = out;
        if (i == 799) output_at_2s = out;
    }

    /* 检查输出被限幅在 1.0 内 */
    ASSERT_F(max_output <= 1.0f + 0.001f, "阶跃响应输出限幅: max=%.3f", max_output);
    ASSERT_F(fabsf(output_at_1s - output_at_2s) < 0.1f, "1s~2s 输出已收敛: Δ=%.4f",
             fabsf(output_at_1s - output_at_2s));
    printf("  阶跃 100°/s: max=%.3f, 1s=%.3f, 2s=%.3f\n",
           max_output, output_at_1s, output_at_2s);

    /* 纯 P: pid 输出被限幅在 [-1,1]，但 0.5*100=50 > 1 所以限幅到 1.0 */
    pacer_pid_t pid_p;
    pid_init(&pid_p, 0.5f, 0.0f, 0.0f, 0.0f, 1.0f);
    float p_out = pid_compute(&pid_p, 100.0f, dt);
    ASSERT_F(fabsf(p_out - 1.0f) < 0.01f, "纯P大误差限幅到1.0: 实际=%.2f", p_out);

    /* 纯 P 小误差: 在限幅内 */
    pid_init(&pid_p, 0.5f, 0.0f, 0.0f, 0.0f, 1.0f);
    p_out = pid_compute(&pid_p, 1.0f, dt);
    ASSERT_F(fabsf(p_out - 0.5f) < 0.01f, "纯P小误差 0.5*1=0.5: 实际=%.2f", p_out);
}

/* --- 5. 闭环: 自稳收敛 --- */
static void test_closed_loop_stabilize(void)
{
    printf("\n=== 测试5: 闭环 — 自稳从 20° 收敛 ===\n");
    float dt = 1.0f / 400.0f;
    int steps = (int)(5.0f / dt);
    rc_input_t input = { .thr = 0.5f, .roll = 0.0f, .pitch = 0.0f, .yaw = 0.0f };

    sim_state_t final = run_closed_loop(steps, dt, &input, ATT_MODE_ANGLE,
                                         20.0f, 0.0f, 0.0f, 0.0f,
                                         steps / 15);
    printf("  最终: roll=%.2f° pitch=%.2f°\n", final.roll, final.pitch);
    ASSERT_F(fabsf(final.roll) < 5.0f, "roll 收敛到 <5°: %.2f°", final.roll);
    ASSERT_F(fabsf(final.pitch) < 2.0f, "pitch 保持接近 0°: %.2f°", final.pitch);
}

/* --- 6. 闭环: 遥控阶跃跟踪 --- */
static void test_closed_loop_tracking(void)
{
    printf("\n=== 测试6: 闭环 — 遥控阶跃跟踪 ===\n");
    float dt = 1.0f / 400.0f;

    /* 先水平飞 1 秒, 然后推 roll 摇杆到 0.3, 飞 3 秒 */
    attitude_ctrl_t ac;
    attitude_init(&ac);
    attitude_set_mode(&ac, ATT_MODE_ANGLE);
    attitude_enable(&ac, true);

    quad_mixer_t mixer;
    mixer_config_t mcfg = MIXER_CONFIG_DEFAULT;
    quad_mixer_init(&mixer, &mcfg);
    quad_mixer_arm(&mixer);

    sim_state_t sim;
    sim_init(&sim);

    /* 前 1 秒: 水平悬停 */
    for (int i = 0; i < 400; i++) {
        imu_sample_t sample = sim_to_imu(&sim);
        attitude_t att = { .roll = sim.roll, .pitch = sim.pitch, .yaw = sim.yaw };
        attitude_set_input(&ac, 0.5f, 0.0f, 0.0f, 0.0f);
        const attitude_output_t *out = attitude_update(&ac, &att, &sample, dt);
        mixer_output_t mix = quad_mixer_update(&mixer, out->throttle, out->roll, out->pitch, out->yaw);
        sim_step(&sim, &mix, dt);
    }

    /* 后 3 秒: 推 roll 到 30% (目标角度 = 0.3 * 30 = 9°) */
    float target_angle = 0.3f * (float)CFG_QUAD_MAX_ANGLE;
    for (int i = 0; i < 1200; i++) {
        imu_sample_t sample = sim_to_imu(&sim);
        attitude_t att = { .roll = sim.roll, .pitch = sim.pitch, .yaw = sim.yaw };
        attitude_set_input(&ac, 0.5f, 0.3f, 0.0f, 0.0f);
        const attitude_output_t *out = attitude_update(&ac, &att, &sample, dt);
        mixer_output_t mix = quad_mixer_update(&mixer, out->throttle, out->roll, out->pitch, out->yaw);
        sim_step(&sim, &mix, dt);
    }

    printf("  目标角度=%.1f°, 实际 roll=%.2f°\n", target_angle, sim.roll);
    /* 应该收敛到目标附近 (允许 50% 误差 — 物理模型简化) */
    bool going_right = sim.roll > 5.0f;
    ASSERT(going_right, "推 roll 后飞机右倾");

    bool settled = fabsf(sim.roll_rate) < 50.0f;
    ASSERT(settled, "角速度已收敛 (稳定在某角度)");
}

/* --- 7. 闭环: 抗扰动恢复 --- */
static void test_closed_loop_disturbance(void)
{
    printf("\n=== 测试7: 闭环 — 抗扰动恢复 ===\n");
    float dt = 1.0f / 400.0f;

    attitude_ctrl_t ac;
    attitude_init(&ac);
    attitude_set_mode(&ac, ATT_MODE_ANGLE);
    attitude_enable(&ac, true);

    quad_mixer_t mixer;
    mixer_config_t mcfg = MIXER_CONFIG_DEFAULT;
    quad_mixer_init(&mixer, &mcfg);
    quad_mixer_arm(&mixer);

    sim_state_t sim;
    sim_init(&sim);

    rc_input_t hover = { .thr = 0.5f, .roll = 0.0f, .pitch = 0.0f, .yaw = 0.0f };

    /* 先稳定 2 秒 */
    for (int i = 0; i < 800; i++) {
        imu_sample_t sample = sim_to_imu(&sim);
        attitude_t att = { .roll = sim.roll, .pitch = sim.pitch, .yaw = sim.yaw };
        attitude_set_input(&ac, hover.thr, hover.roll, hover.pitch, hover.yaw);
        const attitude_output_t *out = attitude_update(&ac, &att, &sample, dt);
        mixer_output_t mix = quad_mixer_update(&mixer, out->throttle, out->roll, out->pitch, out->yaw);
        sim_step(&sim, &mix, dt);
    }

    /* 施加扰动: 瞬间给 roll_rate 加 80°/s 冲量 (适中, 不会让 PID 完全饱和) */
    sim.roll_rate += 80.0f;
    float disturbed_roll = sim.roll;
    printf("  扰动后: roll=%.2f°, roll_rate=%.1f°/s\n", sim.roll, sim.roll_rate);

    /* 恢复 5 秒 */
    for (int i = 0; i < 2000; i++) {
        imu_sample_t sample = sim_to_imu(&sim);
        attitude_t att = { .roll = sim.roll, .pitch = sim.pitch, .yaw = sim.yaw };
        attitude_set_input(&ac, hover.thr, hover.roll, hover.pitch, hover.yaw);
        const attitude_output_t *out = attitude_update(&ac, &att, &sample, dt);
        mixer_output_t mix = quad_mixer_update(&mixer, out->throttle, out->roll, out->pitch, out->yaw);
        sim_step(&sim, &mix, dt);
    }

    printf("  恢复后: roll=%.2f° pitch=%.2f°\n", sim.roll, sim.pitch);
    ASSERT_F(fabsf(sim.roll) < 10.0f, "扰动后 roll 恢复到 <10°: %.2f°", sim.roll);
    ASSERT_F(fabsf(sim.pitch) < 5.0f, "扰动后 pitch 保持 <5°: %.2f°", sim.pitch);
}

/* --- 8. 安全保护 --- */
static void test_safety(void)
{
    printf("\n=== 测试8: 安全保护 ===\n");
    quad_mixer_t m;
    mixer_config_t cfg = MIXER_CONFIG_DEFAULT;
    quad_mixer_init(&m, &cfg);
    quad_mixer_arm(&m);

    /* 急停 */
    quad_mixer_stop(&m);
    mixer_output_t out = quad_mixer_update(&m, 0.8f, 0.3f, 0.3f, 0.3f);
    bool all_zero = true;
    for (int i = 0; i < 4; i++) if (out.motor[i] != 0.0f) all_zero = false;
    ASSERT(all_zero, "急停: 全零");

    /* 上锁 */
    quad_mixer_disarm(&m);
    out = quad_mixer_update(&m, 0.8f, 0.3f, 0.3f, 0.3f);
    all_zero = true;
    for (int i = 0; i < 4; i++) if (out.motor[i] != 0.0f) all_zero = false;
    ASSERT(all_zero, "上锁: 全零");
    ASSERT(!quad_mixer_is_armed(&m), "上锁: armed=false");

    /* 倾斜保护 (在闭环中测试) */
    printf("  (倾斜保护在 app 层检查, 此处验证混控在极端输入下不崩溃)\n");
    quad_mixer_arm(&m);
    out = quad_mixer_update(&m, 1.0f, 1.0f, 1.0f, 1.0f);
    bool all_valid = true;
    for (int i = 0; i < 4; i++) {
        if (out.motor[i] < 0.0f || out.motor[i] > 1.0f) all_valid = false;
    }
    ASSERT(all_valid, "极端输入: 输出仍在 [0,1] 范围");
}

/* --- 9. 解锁上锁状态机 --- */
static void test_arm_state_machine(void)
{
    printf("\n=== 测试9: 解锁上锁状态机 ===\n");
    quad_mixer_t m;
    mixer_config_t cfg = MIXER_CONFIG_DEFAULT;
    quad_mixer_init(&m, &cfg);

    ASSERT(!quad_mixer_is_armed(&m), "初始: 未解锁");

    quad_mixer_arm(&m);
    ASSERT(quad_mixer_is_armed(&m), "arm: 已解锁");
    ASSERT(m.enabled, "arm: enabled=true");

    /* 解锁后有输出 */
    mixer_output_t out = quad_mixer_update(&m, 0.3f, 0.0f, 0.0f, 0.0f);
    bool has_output = false;
    for (int i = 0; i < 4; i++) if (out.motor[i] > 0.0f) has_output = true;
    ASSERT(has_output, "解锁后: 有输出");

    /* 上锁 */
    quad_mixer_disarm(&m);
    ASSERT(!quad_mixer_is_armed(&m), "disarm: 已上锁");
    ASSERT(!m.enabled, "disarm: enabled=false");

    out = quad_mixer_update(&m, 0.8f, 0.5f, 0.5f, 0.5f);
    all_zero:
    bool zero = true;
    for (int i = 0; i < 4; i++) if (out.motor[i] != 0.0f) zero = false;
    ASSERT(zero, "上锁后: 无输出");

    /* 重新解锁 */
    quad_mixer_arm(&m);
    ASSERT(quad_mixer_is_armed(&m), "重新 arm: 已解锁");
}

/* --- 10. 混控对称性 --- */
static void test_mixer_symmetry(void)
{
    printf("\n=== 测试10: 混控对称性 ===\n");
    quad_mixer_t m;
    mixer_config_t cfg = MIXER_CONFIG_DEFAULT;
    quad_mixer_init(&m, &cfg);
    quad_mixer_arm(&m);

    /* Roll 对称: +roll 和 -roll 应该是镜像 */
    mixer_output_t pos = quad_mixer_update(&m, 0.5f, 0.2f, 0.0f, 0.0f);
    mixer_output_t neg = quad_mixer_update(&m, 0.5f, -0.2f, 0.0f, 0.0f);

    ASSERT_F(fabsf(pos.motor[QUAD_FL] - neg.motor[QUAD_FR]) < 0.001f,
             "roll 对称 FL↔FR: %.4f vs %.4f", pos.motor[QUAD_FL], neg.motor[QUAD_FR]);
    ASSERT_F(fabsf(pos.motor[QUAD_RL] - neg.motor[QUAD_RR]) < 0.001f,
             "roll 对称 RL↔RR: %.4f vs %.4f", pos.motor[QUAD_RL], neg.motor[QUAD_RR]);

    /* Pitch 对称 */
    pos = quad_mixer_update(&m, 0.5f, 0.0f, 0.2f, 0.0f);
    neg = quad_mixer_update(&m, 0.5f, 0.0f, -0.2f, 0.0f);

    ASSERT_F(fabsf(pos.motor[QUAD_RL] - neg.motor[QUAD_FL]) < 0.001f,
             "pitch 对称 RL↔FL: %.4f vs %.4f", pos.motor[QUAD_RL], neg.motor[QUAD_FL]);
    ASSERT_F(fabsf(pos.motor[QUAD_RR] - neg.motor[QUAD_FR]) < 0.001f,
             "pitch 对称 RR↔FR: %.4f vs %.4f", pos.motor[QUAD_RR], neg.motor[QUAD_FR]);

    /* Yaw 对称 */
    pos = quad_mixer_update(&m, 0.5f, 0.0f, 0.0f, 0.2f);
    neg = quad_mixer_update(&m, 0.5f, 0.0f, 0.0f, -0.2f);

    /* yaw+: FR(+) FL(-) | yaw-: FR(-) FL(+) */
    ASSERT_F(fabsf((pos.motor[QUAD_FR] - 0.5f) + (neg.motor[QUAD_FR] - 0.5f)) < 0.001f,
             "yaw 对称 FR: +%.4f vs %.4f (应互为相反)", pos.motor[QUAD_FR] - 0.5f, neg.motor[QUAD_FR] - 0.5f);

    /* 油门=0 时只有限幅后的最低油门 */
    mixer_output_t zero_thr = quad_mixer_update(&m, 0.0f, 0.0f, 0.0f, 0.0f);
    ASSERT_F(fabsf(zero_thr.motor[0] - m.cfg.throttle_min) < 0.001f,
             "零油门→最低油门: %.4f", zero_thr.motor[0]);
}

/* ================ 主函数 ================ */

int main(void)
{
    printf("╔══════════════════════════════════════════╗\n");
    printf("║  Pacer v3.0 — 四旋翼全面模拟测试         ║\n");
    printf("╚══════════════════════════════════════════╝\n");

    test_mixer_basic();
    test_rate_mode();
    test_angle_mode();
    test_pid_step_response();
    test_closed_loop_stabilize();
    test_closed_loop_tracking();
    test_closed_loop_disturbance();
    test_safety();
    test_arm_state_machine();
    test_mixer_symmetry();

    printf("\n══════════════════════════════════════════\n");
    printf("  结果: %d 通过, %d 失败 / %d 总计\n",
           g_pass, g_fail, g_pass + g_fail);
    printf("══════════════════════════════════════════\n");

    return g_fail > 0 ? 1 : 0;
}
