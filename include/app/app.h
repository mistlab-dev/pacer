/**
 * @file app.h
 * @brief 四旋翼主应用接口 — FreeRTOS 多任务
 *
 * app_init() → 硬件/传感器/控制初始化
 * app_run()  → 创建任务 + 启动调度器 (不返回)
 */

#ifndef PACER_APP_H
#define PACER_APP_H

#include "filter/filter.h"
#include "sensor/imu.h"

/**
 * @brief 初始化所有子系统
 * @return 0=成功, -1=失败
 */
int  app_init(void);
void app_deinit(void);

/**
 * @brief 启动 FreeRTOS 任务 + 调度器
 * @return 正常不返回, -1=失败
 */
int  app_run(void);

/* 调试接口 */
const attitude_t   *app_get_attitude(void);
const imu_sample_t *app_get_imu(void);
bool app_is_emergency(void);

#endif /* PACER_APP_H */
