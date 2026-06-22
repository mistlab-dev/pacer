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
#if CFG_UART_PLAIN_DEBUG
        uart_puts("SAFETY EMERGENCY\r\n");
#else
        printf("[SAFETY] tilt %.1f > %d → EMERGENCY\r\n",
               tilt, CFG_QUAD_MAX_TILT_EMERGENCY);
#endif
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
#if CFG_UART_PLAIN_DEBUG
        uart_puts("SAFETY BATTERY CRITICAL\r\n");
#else
        printf("[SAFETY] battery %.1fV CRITICAL → auto-land\r\n",
               battery_get_voltage());
#endif
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
#if CFG_UART_PLAIN_DEBUG
            uart_puts("SAFETY SIGNAL LOST\r\n");
#else
            printf("[SAFETY] signal lost → auto-land\r\n");
#endif
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
#if CFG_UART_PLAIN_DEBUG
        uart_puts("APP LANDING COMPLETE\r\n");
#else
        printf("[APP] landing complete\r\n");
#endif
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
#if !CFG_UART_PLAIN_DEBUG
        watchdog_kick();
#endif

        vTaskDelayUntil(&last, period);

        /* 1. IMU */
        if (!g.imu_ok) {
            continue;
        }
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
#if CFG_UART_PLAIN_DEBUG
                uart_puts("APP EMERGENCY CLEARED\r\n");
#else
                printf("[APP] emergency cleared\r\n");
#endif
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
#if CFG_UART_PLAIN_DEBUG
                uart_puts("APP CANNOT ARM BATTERY\r\n");
#else
                printf("[APP] cannot arm: battery critical\r\n");
#endif
            } else {
                if (arm_start == 0) {
                    arm_start = HAL_GetTick();
                } else if (HAL_GetTick() - arm_start > (uint32_t)(CFG_ARM_STICK_TIMEOUT_SEC * 1000)) {
                    /* 飞前检查 */
                    if (imu_self_test() != 0) {
#if CFG_UART_PLAIN_DEBUG
                        uart_puts("APP CANNOT ARM IMU\r\n");
#else
                        printf("[APP] cannot arm: IMU self-test failed\r\n");
#endif
                        arm_start = 0;
                    } else {
                        float tilt = sqrtf(g.attitude.roll * g.attitude.roll +
                                           g.attitude.pitch * g.attitude.pitch);
                        if (tilt > CFG_PREFLIGHT_LEVEL_DEG) {
#if CFG_UART_PLAIN_DEBUG
                            uart_puts("APP CANNOT ARM TILT\r\n");
#else
                            printf("[APP] cannot arm: tilt %.1f > %.1f deg\r\n",
                                   tilt, CFG_PREFLIGHT_LEVEL_DEG);
#endif
                            arm_start = 0;
                        } else {
#if CFG_UART_PLAIN_DEBUG
                            uart_puts("APP ARMING\r\n");
#else
                            printf("[APP] arming...\r\n");
#endif
                            motor_arm();
                            quad_mixer_arm(&g.mixer);
                            attitude_init(&g.att_ctrl);
                            attitude_enable(&g.att_ctrl, true);
                            g.armed = true;
                            arm_start = 0;
                        }
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
#if CFG_UART_PLAIN_DEBUG
            uart_puts("APP DISARMING\r\n");
#else
            printf("[APP] disarming\r\n");
#endif
            quad_mixer_disarm(&g.mixer);
            motor_disarm();
            g.armed = false;
        }

        /* 起飞: 油门>30% 且已解锁 */
        if (g.armed && !g.flying && !g.landing &&
            rc_new.throttle > 0.3f) {
#if CFG_UART_PLAIN_DEBUG
            uart_puts("APP TAKEOFF\r\n");
#else
            printf("[APP] takeoff!\r\n");
#endif
            g.flying          = true;
            g.takeoff_throttle = 0.0f;  /* 从 0 开始渐变 */
        }

        /* 降落: 油门<10% 且在飞 → 进入自动降落 */
        if (g.flying && !g.landing &&
            rc_new.throttle < 0.1f) {
#if CFG_UART_PLAIN_DEBUG
            uart_puts("APP LANDING\r\n");
#else
            printf("[APP] landing...\r\n");
#endif
            g.landing       = true;
            g.land_throttle = CFG_HOVER_THROTTLE;  /* 从悬停开始降 */
        }

        /* 信号丢失 */
        handle_lost_signal();
    }

    vTaskDelete(NULL);
}

/* ================ 遥测任务 (1Hz 固定字符串) ================ */

static void telem_task(void *arg)
{
    (void)arg;

#if CFG_UART_PLAIN_DEBUG
    uart_puts("PACER TELEM START\r\n");
#endif

    while (g.running) {
#if CFG_UART_PLAIN_DEBUG
        uart_puts("PACER ALIVE\r\n");
#endif
#if !CFG_UART_PLAIN_DEBUG
        watchdog_kick();
#endif

        vTaskDelay(pdMS_TO_TICKS(1000));

        led_tick();
#if !CFG_UART_PLAIN_DEBUG
        battery_update();
#endif

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

#if CFG_UART_PLAIN_DEBUG
        uart_puts("PACER ALIVE\r\n");
#else
        printf("[T] %s%s%s | R:%.1f P:%.1f Y:%.1f | thr:%.2f rc:%d | bat:%.1fV\r\n",
               g.armed ? "ARM" : "DIS",
               g.flying ? "+FLY" : "",
               g.emergency ? "+EMG" : "",
               g.attitude.roll, g.attitude.pitch, g.attitude.yaw,
               g.rc.throttle,
               remote_is_connected(),
               battery_get_voltage());
#endif
    }

    vTaskDelete(NULL);
}

#if CFG_UART_PLAIN_DEBUG && (CFG_IMU_DEBUG || CFG_REMOTE_DEBUG)

static void imu_debug_print_bias(void)
{
    vec3_t bias;
    char buf[96];

    imu_get_gyro_bias(&bias);
    snprintf(buf, sizeof(buf),
             "PACER IMU BIAS gx=%.3f gy=%.3f gz=%.3f\r\n",
             bias.x, bias.y, bias.z);
    uart_puts(buf);
}

static void imu_debug_print_sample(const imu_sample_t *s, const attitude_t *att)
{
    char buf[160];

    snprintf(buf, sizeof(buf),
             "PACER IMU ax=%.2f ay=%.2f az=%.2f "
             "gx=%.3f gy=%.3f gz=%.3f "
             "roll=%.1f pitch=%.1f yaw=%.1f T=%.1f\r\n",
             s->accel.x, s->accel.y, s->accel.z,
             s->gyro.x, s->gyro.y, s->gyro.z,
             att->roll, att->pitch, att->yaw,
             s->temperature);
    uart_puts(buf);
}

static void remote_debug_print_status(void)
{
    remote_cmd_t cmd;
    remote_stats_t st;
    char buf[128];

    remote_get_cmd(&cmd);
    remote_get_stats(&st);
    snprintf(buf, sizeof(buf),
             "PACER REMOTE T=%.2f R=%.2f P=%.2f Y=%.2f "
             "conn=%d frames=%lu drops=%lu\r\n",
             cmd.throttle, cmd.roll, cmd.pitch, cmd.yaw,
             st.connected ? 1 : 0,
             (unsigned long)st.frame_count,
             (unsigned long)st.drop_count);
    uart_puts(buf);
}

static void link_debug_run_loop(void)
{
    const uint32_t imu_hz = CFG_IMU_DEBUG ? (uint32_t)CFG_IMU_DEBUG_HZ : 0U;
    const uint32_t rc_hz  = CFG_REMOTE_DEBUG ? (uint32_t)CFG_REMOTE_DEBUG_HZ : 0U;
    const uint32_t loop_hz = (imu_hz > rc_hz) ? imu_hz : rc_hz;
    const uint32_t delay_ms = (loop_hz > 0U) ? (1000U / loop_hz) : 100U;
    const float dt = 1.0f / (float)loop_hz;
    uint32_t imu_tick = 0;
    uint32_t rc_tick  = 0;
    uint32_t err_count = 0;

    uart_puts("PACER LINK DEBUG ON\r\n");
    while (g.running) {
        imu_tick++;
        rc_tick++;

#if CFG_IMU_DEBUG
        if (g.imu_ok && (imu_hz == 0U || (imu_tick % (loop_hz / imu_hz)) == 0U)) {
            if (imu_read(&g.imu_sample) != 0) {
                err_count++;
                if ((err_count % loop_hz) == 1U) {
                    uart_puts("PACER IMU READ ERR\r\n");
                }
            } else {
                g.attitude = filter_update(&g.imu_sample, dt);
                imu_debug_print_sample(&g.imu_sample, &g.attitude);
            }
        }
#endif

#if CFG_REMOTE_DEBUG
        if (rc_hz == 0U || (rc_tick % (loop_hz / rc_hz)) == 0U) {
            remote_cmd_t rc_new;
            if (remote_poll(&rc_new) == 0) {
                g.rc = rc_new;
                g.last_rc_ms = HAL_GetTick();
            }
            remote_debug_print_status();
        }
#endif

        HAL_Delay(delay_ms);
    }
}

#endif /* CFG_UART_PLAIN_DEBUG && (CFG_IMU_DEBUG || CFG_REMOTE_DEBUG) */

/* ================ 公开接口 ================ */

int app_init(void)
{
    memset(&g, 0, sizeof(g));

    /* HAL 基础 */
    hal_gpio_init();
    usart_printf_init();
#if CFG_UART_PLAIN_DEBUG
    uart_puts("PACER INIT START\r\n");
#else
    printf("\r\n=== PACER v3.0 (STM32H743) ===\r\n");
#endif

    /* I2C1 总线 (IMU 用) */
    hal_i2c1_init();

    /* LED */
    led_init();
    led_set_state(LED_BREATH);  /* 校准中 */

#if !CFG_UART_PLAIN_DEBUG
    hal_tim_pwm_init();
    motor_init();
    battery_init();
#endif

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
#if CFG_UART_PLAIN_DEBUG
        uart_puts("PACER NO IMU\r\n");
#else
        printf("[APP] IMU init FAILED (no sensor?) — running without IMU\r\n");
#endif
    } else {
#if CFG_UART_PLAIN_DEBUG
        uart_puts("PACER IMU CALIBRATE\r\n");
#else
        printf("[APP] calibrating gyro...\r\n");
#endif
        imu_calibrate_gyro(500);
        g.imu_ok = true;
#if CFG_UART_PLAIN_DEBUG && CFG_IMU_DEBUG
        if (imu_self_test() == 0) {
            uart_puts("PACER IMU ID OK\r\n");
        }
        uart_puts("PACER IMU CAL DONE\r\n");
        imu_debug_print_bias();
#endif
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

    /* 遥控 — 飞控模式或联调遥控模式启用 USART3 */
#if !CFG_UART_PLAIN_DEBUG || CFG_REMOTE_DEBUG
    remote_init();
#if CFG_UART_PLAIN_DEBUG && CFG_REMOTE_DEBUG
    uart_puts("PACER REMOTE READY\r\n");
#endif
#endif

    /* CLI (排查串口阶段先关闭，避免 printf 干扰) */
#if !CFG_UART_PLAIN_DEBUG
    cli_init();
#endif

    g.running   = true;
    g.armed     = false;
    g.flying    = false;
    g.emergency = false;

#if CFG_UART_PLAIN_DEBUG
    uart_puts("PACER INIT OK\r\n");
#else
    printf("[APP] init OK, ready to arm (throttle 0 + yaw left)\r\n");
#endif
    led_set_state(LED_BLINK_SLOW);  /* idle */

    /* 空板联调阶段暂不启 IWDG，避免阻塞外设导致反复复位 */
#if !CFG_UART_PLAIN_DEBUG
    watchdog_init(CFG_WATCHDOG_TIMEOUT_MS);
#endif

    return 0;
}

void app_deinit(void)
{
    g.running = false;
    motor_stop_all();
    motor_disarm();
    imu_deinit();
#if !CFG_UART_PLAIN_DEBUG || CFG_REMOTE_DEBUG
    remote_deinit();
#endif
    led_set_state(LED_OFF);
#if CFG_UART_PLAIN_DEBUG
    uart_puts("PACER SHUTDOWN\r\n");
#else
    printf("[APP] shutdown\r\n");
#endif
}

int app_run(void)
{
#if CFG_UART_PLAIN_DEBUG
    uart_puts("PACER RUN\r\n");
#if CFG_IMU_DEBUG || CFG_REMOTE_DEBUG
    link_debug_run_loop();
#else
    while (g.running) {
        uart_puts("PACER ALIVE\r\n");
        for (volatile uint32_t d = 0; d < 25600000U; d++) {
            __NOP();
        }
    }
#endif
    return 0;
#else
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
#endif
}

const attitude_t *app_get_attitude(void) { return &g.attitude; }
const imu_sample_t *app_get_imu(void)    { return &g.imu_sample; }
bool app_is_emergency(void)              { return g.emergency; }
bool app_is_armed(void)                  { return g.armed; }
bool app_is_flying(void)                 { return g.flying; }
attitude_ctrl_t *app_get_att_ctrl(void)  { return &g.att_ctrl; }
