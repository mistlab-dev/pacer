/**
 * @file cli.h
 * @brief 串口命令行接口 — PID 在线调参 + 状态查询
 */

#ifndef PACER_CLI_H
#define PACER_CLI_H

#include <stdint.h>

void cli_init(void);
void cli_rx_byte_cb(uint8_t b);
extern uint8_t cli_rx_byte;

#endif /* PACER_CLI_H */
