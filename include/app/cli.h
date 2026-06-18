/**
 * @file cli.h
 * @brief 串口命令行接口 — PID 在线调参 + 状态查询
 */

#ifndef PACER_CLI_H
#define PACER_CLI_H

/**
 * @brief 初始化 CLI 任务
 *        创建 FreeRTOS 任务处理 USART2 接收的命令。
 */
void cli_init(void);

/**
 * @brief CLI 任务入口 (FreeRTOS)
 *        由 cli_init 创建，不应直接调用。
 */
void cli_task(void *arg);

#endif /* PACER_CLI_H */