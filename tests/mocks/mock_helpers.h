/**
 * @file mock_helpers.h
 * @brief Mock 重置和查询接口
 */

#ifndef MOCK_HELPERS_H
#define MOCK_HELPERS_H

void mock_reset(void);
int  mock_get_pwm_log(int index, int *pin, int *duty);
int  mock_get_pwm_log_count(void);
int  mock_gpio_was_init(void);

void mock_i2c_reset(void);
uint8_t mock_i2c_get_reg(uint8_t reg);
int  mock_i2c_was_opened(void);

#endif
