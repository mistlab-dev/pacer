/**
 * @file app.h
 * @brief 应用层 — 四旋翼无人机系统初始化、主循环、模式管理
 */

#ifndef PACER_APP_H
#define PACER_APP_H

/**
 * @brief 运行模式
 */
typedef enum {
    APP_MODE_RUN,        /* 正常飞行 */
    APP_MODE_CALIBRATE,  /* 校准 IMU */
    APP_MODE_DEBUG,      /* 打印姿态数据 (不驱动电机) */
    APP_MODE_ESC_CAL,    /* 校准 ESC */
} app_mode_t;

/**
 * @brief 应用入口
 * @param mode 运行模式
 * @return 退出码
 */
int app_run(app_mode_t mode);

#endif /* PACER_APP_H */
