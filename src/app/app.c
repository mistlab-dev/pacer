/**
 * @file app.c
 * @brief 四旋翼主应用 — FreeRTOS 多任务实现
 *
 * STM32H743 版本，替代 RPi 的单线程主循环。
 *
 * 任务结构:
 *   1. ctrl_task   (最高优先级, 400Hz)  — IMU 读取 + 姿态控制 + 电机输出
 *   2. rx_task     (中优先级, 200Hz)    — 遥控接收解析
 *   3. telem_task  (低优先级, 10Hz)     — 调试遥测输出
 *
 * 安全链:
 *   ctrl_task: 倾斜保护 → 紧急停机
 *   rx_task:   遥控超时 → 自动降落
 */

#include "app/app.h"
#include "app/config.h"
#include "app/cli.h"
#include "sensor/imu.h"
#include "sensor/imu_icm20948.h"
#include "filter/filter.h"
#include "ctrl/attitude.h"
#include "ctrl/quad_mixer.h"
#include "motor/motor.h"
#include "remote/remote.h"
#include "hal/hal_gpio.h"
#include "hal/hal_i2c.h"
#include "hal/hal_tim.h"
#include "hal/led.h"
#include "hal/watchdog.h"
#include "hal/battery.h"
#include "usart_printf.h"

#include "stm32h7xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"

#include <stdio.h>
#include <math.h>
#include <string.h>

/* ================ 全局状态 ================ */

static struct {
    /* IMU + 滤波 */
    imu_sample_t  imu_sample;
    attitude_t    attitude;
    bool          imu_ok;

    /* 控制 */
    attitude_ctrl_t  att_ctrl;
    quad_mixer_t     mixer;
    bool             armed;
    bool             flying;
    bool             emergency;
    bool             landing;        /* 自动降落中 */
    float            land_throttle;  /* 降落油门 (递减) */
    float            takeoff_throttle; /* 起飞油门 (递增) */

    /* 遥控 */
    remote_cmd_t  rc;
    uint32_t      last_rc_ms;

    /* 运行 */
    bool          running;
} g;

/* ================ 安全 ================ */

static void check_safety(void)
{
    /* 1. 倾斜保护 */
    float tilt = sqrtf(g.attitude.roll * g.attitude.roll +
                       g.attitude.pitch * g.attitude.pitch);
    if (tilt > CFG_QUAD_MAX_TILT_EMERGENCY && g.flying) {
        printf("[SAFETY] tilt %.1f > %d → EMERGENCY\r\n",
               tilt, CFG_QUAD_MAX_TILT_EMERGENCY);
        quad_mixer_stop(&g.mixer);
        motor_stop_all();
        g.emergency = true;
        g.flying    = false;
        led_set_state(LED_BLINK_DOUBLE);
        return;
    }

    /* 2. 电池低压保护 */
    battery_status_t bat = battery_get_status();
    if (bat == BATTERY_CRITICAL && g.flying) {
        printf("[SAFETY] battery %.1fV CRITICAL → auto-land\r\n",
               battery_get_voltage());
        g.landing = true;  /* 进入自动降落模式 */
    } else if (bat == BATTERY_WARNING) {
        /* 低压警告: 快闪 LED */
        if (g.flying) led_set_state(LED_BLINK_FAST);
    }
}

static void handle_lost_signal(void)
{
    if (!remote_is_connected()) {
        if (g.flying && !g.landing) {
            printf("[SAFETY] signal lost → auto-land\r\n");
            g.landing = true;
        }
    }
}

/* ---- 自动降落: 缓慢降低油门 ---- */
static void update_landing(float dt)
{
    if (!g.landing) return;

    /* 油门按速率下降 */
    g.land_throttle -= CFG_LANDING_DESCENT_RATE * dt;
    if (g.land_throttle < 0.0f) g.land_throttle = 0.0f;

    /* 用降落油门驱动，保持水平 */
    attitude_set_input(&g.att_ctrl, g.land_throttle, 0.0f, 0.0f, 0.0f);
    const attitude_output_t *out =
        attitude_update(&g.att_ctrl, &g.attitude, &g.imu_sample, CFG_CONTROL_DT);

    mixer_output_t motors = quad_mixer_update(&g.mixer,
                                               out->throttle,
                                               out->roll,
                                               out->pitch,
                                               out->yaw);
    motor_set(MOTOR_FL, motors.motor[QUAD_FL]);
    motor_set(MOTOR_FR, motors.motor[QUAD_FR]);
    motor_set(MOTOR_RL, motors.motor[QUAD_RL]);
    motor_set(MOTOR_RR, motors.motor[QUAD_RR]);

    /* 触地: 油门低于阈值 → 停机 */
    if (g.land_throttle <= CFG_LANDING_THROTTLE_MIN) {
        printf("[APP] landing complete\r\n");
        quad_mixer_stop(&g.mixer);
        motor_stop_all();
        g.flying   = false;
        g.landing  = false;
        g.armed    = false;
        led_set_state(LED_BLINK_SLOW);
    }
}

/* ================ 控制任务 (400Hz) ================ */

static void ctrl_task(void *arg)
{
    (void)arg;

    TickType_t last = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(1000 / CFG_CONTROL_HZ);

    while (g.running) {
        vTaskDelayUntil(&last, period);

        /* 0. 喂狗 */
        watchdog_kick();

        /* 1. IMU */
        if (imu_read(&g.imu_sample) != 0) {
            g.imu_ok = false;
            continue;
        }
        g.imu_ok = true;

        /* 2. 姿态估计 */
        g.attitude = filter_update(&g.imu_sample, CFG_CONTROL_DT);

        /* 3. 安全检查 */
        check_safety();

        /* 4. 自动降落模式 */
        if (g.landing && !g.emergency) {
            update_landing(CFG_CONTROL_DT);
            continue;
        }

        /* 5. 姿态控制 + 混控 + 电机输出 */
        if (g.flying && !g.emergency) {
            /* 起飞油门渐变: 从 hover 的 50% 平滑爬升 */
            float effective_thr = g.rc.throttle;
            if (g.takeoff_throttle < CFG_TAKEOFF_THROTTLE) {
                /* 起飞阶段: 油门从 0 渐增至起飞阈值 */
                g.takeoff_throttle += CFG_TAKEOFF_RAMP_RATE * CFG_CONTROL_DT;
                if (g.takeoff_throttle > CFG_TAKEOFF_THROTTLE)
                    g.takeoff_throttle = CFG_TAKEOFF_THROTTLE;
                /* 取用户油门和渐变油门的最大值 */
                if (effective_thr < g.takeoff_throttle)
                    effective_thr = g.takeoff_throttle;
            }

            /* 设置遥控输入到姿态控制器 */
            attitude_set_input(&g.att_ctrl,
                               effective_thr,
                               g.rc.roll,
                               g.rc.pitch,
                               g.rc.yaw);

            /* PID 计算 */
            const attitude_output_t *out =
                attitude_update(&g.att_ctrl, &g.attitude,
                                &g.imu_sample, CFG_CONTROL_DT);

            /* 混控 → 4 电机油门 */
            mixer_output_t motors = quad_mixer_update(&g.mixer,
                                                      out->throttle,
                                                      out->roll,
                                                      out->pitch,
                                                      out->yaw);

            /* 输出到硬件 */
            motor_set(MOTOR_FL, motors.motor[QUAD_FL]);
            motor_set(MOTOR_FR, motors.motor[QUAD_FR]);
            motor_set(MOTOR_RL, motors.motor[QUAD_RL]);
            motor_set(MOTOR_RR, motors.motor[QUAD_RR]);
        } else if (!g.flying && !g.emergency) {
            /* 地面 idle: 电机停止 */
            motor_stop_all();
        }
        /* emergency: 已停机, 等待重启 */
    }

    motor_stop_all();
    motor_disarm();
    vTaskDelete(NULL);
}

/* ================ 遥控接收任务 (200Hz) ================ */

static void rx_task(void *arg)
{
    (void)arg;
    remote_cmd_t rc_new;

    while (g.running) {
        vTaskDelay(pdMS_TO_TICKS(5));

        if (remote_poll(&rc_new) == 0) {
            taskENTER_CRITICAL();
            g.rc = rc_new;
            g.last_rc_ms = HAL_GetTick();
            taskEXIT_CRITICAL();
        }

        /* ---- 解锁/上锁/起飞/降落 逻辑 ---- */

        /* 紧急停止恢复: 油门<5% + yaw 左满 + pitch 前满 保持 2s */
        static uint32_t emg_clear_start = 0;
        if (g.emergency &&
            rc_new.throttle < 0.05f &&
            rc_new.yaw < -0.9f &&
            rc_new.pitch > 0.9f) {
            if (emg_clear_start == 0) {
                emg_clear_start = HAL_GetTick();
            } else if (HAL_GetTick() - emg_clear_start > 2000) {
                printf("[APP] emergency cleared\r\n");
                g.emergency = false;
                g.armed = false;
                emg_clear_start = 0;
                led_set_state(LED_BLINK_SLOW);
            }
        } else {
            emg_clear_start = 0;
        }

        /* 解锁: 油门<5% + yaw 左满 + 持续 3s + IMU正常 + 遥控已连接 */
        static uint32_t arm_start = 0;
        if (!g.armed &&
            rc_new.throttle < 0.05f &&
            rc_new.yaw < -0.9f &&
            g.imu_ok &&
            remote_is_connected()) {
            battery_status_t bat = battery_get_status();
            if (bat == BATTERY_CRITICAL) {
                arm_start = 0;
                printf("[APP] cannot arm: battery critical\r\n");
            } else {
                if (arm_start == 0) {
                    arm_start = HAL_GetTick();
                } else if (HAL_GetTick() - arm_start > (uint32_t)(CFG_ARM_STICK_TIMEOUT_SEC * 1000)) {
                    /* 飞前检查 */
                    if (imu_self_test() != 0) {
                        printf("[APP] cannot arm: IMU self-test failed\r\n");
                        arm_start = 0;
                    } else {
                        printf("[APP] arming...\r\n");
                        motor_arm();
                        quad_mixer_arm(&g.mixer);
                        attitude_init(&g.att_ctrl);
                        attitude_enable(&g.att_ctrl, true);
                        g.armed = true;
                        arm_start = 0;
                    }
                }
            }
        } else if (!g.armed) {
            arm_start = 0;  /* 摇杆松开重置计时 */
        }

        /* 上锁: 油门<5% + yaw 右满 */
        if (g.armed && !g.flying &&
            rc_new.throttle < 0.05f &&
            rc_new.yaw > 0.9f) {
            printf("[APP] disarming\r\n");
            quad_mixer_disarm(&g.mixer);
            motor_disarm();
            g.armed = false;
        }

        /* 起飞: 油门>30% 且已解锁 */
        if (g.armed && !g.flying && !g.landing &&
            rc_new.throttle > 0.3f) {
            printf("[APP] takeoff!\r\n");
            g.flying          = true;
            g.takeoff_throttle = 0.0f;  /* 从 0 开始渐变 */
        }

        /* 降落: 油门<10% 且在飞 → 进入自动降落 */
        if (g.flying && !g.landing &&
            rc_new.throttle < 0.1f) {
            printf("[APP] landing...\r\n");
            g.landing       = true;
            g.land_throttle = CFG_HOVER_THROTTLE;  /* 从悬停开始降 */
        }

        /* 信号丢失 */
        handle_lost_signal();
    }

    vTaskDelete(NULL);
}

/* ================ 遥测任务 (10Hz) ================ */

static void telem_task(void *arg)
{
    (void)arg;

    while (g.running) {
        vTaskDelay(pdMS_TO_TICKS(100));

        /* LED + 电池更新 (10Hz) */
        led_tick();
        battery_update();

        /* LED 状态切换 */
        if (g.emergency) {
            /* 保持 LED_BLINK_DOUBLE, 由 check_safety 设置 */
        } else if (g.flying) {
            led_set_state(LED_BLINK_FAST);
        } else if (g.armed) {
            led_set_state(LED_ON);
        } else {
            led_set_state(LED_BLINK_SLOW);
        }

        printf("[T] %s%s%s | R:%.1f P:%.1f Y:%.1f | thr:%.2f rc:%d | bat:%.1fV\r\n",
               g.armed ? "ARM" : "DIS",
               g.flying ? "+FLY" : "",
               g.emergency ? "+EMG" : "",
               g.attitude.roll, g.attitude.pitch, g.attitude.yaw,
               g.rc.throttle,
               remote_is_connected(),
               battery_get_voltage());
    }

    vTaskDelete(NULL);
}

/* ================ 公开接口 ================ */

int app_init(void)
{
    memset(&g, 0, sizeof(g));

    /* HAL 基础 */
    hal_gpio_init();
    usart_printf_init();
    printf("\r\n=== PACER v3.0 (STM32H743) ===\r\n");

    /* I2C1 总线 (IMU 用) */
    hal_i2c1_init();

    /* LED */
    led_init();
    led_set_state(LED_BREATH);  /* 校准中 */

    /* Timer PWM (电机) */
    hal_tim_pwm_init();
    motor_init();

    /* 电池 ADC */
    battery_init();

    /* IMU */
    imu_icm20948_register();
    imu_config_t imu_cfg = {
        .i2c_bus        = 0,
        .i2c_addr       = CFG_IMU_I2C_ADDR,
        .sample_rate_hz = CFG_IMU_SAMPLE_HZ,
        .accel_range_g  = CFG_IMU_ACCEL_RANGE_G,
        .gyro_range_dps = CFG_IMU_GYRO_RANGE_DPS,
    };
    g.imu_ok = false;
    if (imu_init(&imu_cfg) != 0) {
        printf("[APP] IMU init FAILED (no sensor?) — running without IMU\r\n");
    } else {
        printf("[APP] calibrating gyro...\r\n");
        imu_calibrate_gyro(500);
        g.imu_ok = true;
    }

    /* 滤波器 */
    filter_config_t fcfg = FILTER_CONFIG_DEFAULT;
#if CFG_USE_MADGWICK
    fcfg.type  = FILTER_MADGWICK;
    fcfg.beta  = 0.1f;
#else
    fcfg.type  = FILTER_COMPLEMENTARY;
    fcfg.alpha = 0.98f;
#endif
    filter_init(&fcfg);

    /* 姿态控制器 */
    attitude_init(&g.att_ctrl);
    attitude_enable(&g.att_ctrl, false);  /* 解锁后才使能 */

    /* 混控器 */
    mixer_config_t mcfg = MIXER_CONFIG_DEFAULT;
    quad_mixer_init(&g.mixer, &mcfg);

    /* 遥控 */
    remote_init();

    /* CLI (USART2 在线调参) */
    cli_init();

    g.running   = true;
    g.armed     = false;
    g.flying    = false;
    g.emergency = false;

    printf("[APP] init OK, ready to arm (throttle 0 + yaw left)\r\n");
    led_set_state(LED_BLINK_SLOW);  /* idle */

    /* 初始化完成后再启动看门狗，避免校准阶段超时复位 */
    watchdog_init(CFG_WATCHDOG_TIMEOUT_MS);

    return 0;
}

void app_deinit(void)
{
    g.running = false;
    motor_stop_all();
    motor_disarm();
    imu_deinit();
    remote_deinit();
    led_set_state(LED_OFF);
    printf("[APP] shutdown\r\n");
}

int app_run(void)
{
    BaseType_t ret;

    ret = xTaskCreate(ctrl_task, "ctrl",
                      CFG_TASK_CTRL_STACK, NULL,
                      CFG_TASK_CTRL_PRIORITY, NULL);
    if (ret != pdPASS) goto fail;

    ret = xTaskCreate(rx_task, "rx",
                      CFG_TASK_RX_STACK, NULL,
                      CFG_TASK_RX_PRIORITY, NULL);
    if (ret != pdPASS) goto fail;

    ret = xTaskCreate(telem_task, "telem",
                      CFG_TASK_TELEM_STACK, NULL,
                      CFG_TASK_TELEM_PRIORITY, NULL);
    if (ret != pdPASS) goto fail;

    printf("[APP] tasks created, starting scheduler\r\n");
    vTaskStartScheduler();

    printf("[APP] scheduler failed\r\n");
    return -1;

fail:
    printf("[APP] task creation failed\r\n");
    return -1;
}

const attitude_t *app_get_attitude(void) { return &g.attitude; }
const imu_sample_t *app_get_imu(void)    { return &g.imu_sample; }
bool app_is_emergency(void)              { return g.emergency; }
bool app_is_armed(void)                  { return g.armed; }
bool app_is_flying(void)                 { return g.flying; }
attitude_ctrl_t *app_get_att_ctrl(void)  { return &g.att_ctrl; }
