/**
 * @file logger.h
 * @brief 轻量级日志工具
 */

#ifndef PACER_LOGGER_H
#define PACER_LOGGER_H

#include <stdio.h>
#include <time.h>

#define LOG_LEVEL_DEBUG 0
#define LOG_LEVEL_INFO  1
#define LOG_LEVEL_WARN  2
#define LOG_LEVEL_ERROR 3

extern int g_log_level;

#define LOG_D(fmt, ...) do { if (g_log_level <= LOG_LEVEL_DEBUG) \
    fprintf(stderr, "[D][%s:%d] " fmt "\n", __FILE__, __LINE__, ##__VA_ARGS__); } while(0)
#define LOG_I(fmt, ...) do { if (g_log_level <= LOG_LEVEL_INFO) \
    fprintf(stderr, "[I] " fmt "\n", ##__VA_ARGS__); } while(0)
#define LOG_W(fmt, ...) do { if (g_log_level <= LOG_LEVEL_WARN) \
    fprintf(stderr, "[W] " fmt "\n", ##__VA_ARGS__); } while(0)
#define LOG_E(fmt, ...) do { if (g_log_level <= LOG_LEVEL_ERROR) \
    fprintf(stderr, "[E][%s:%d] " fmt "\n", __FILE__, __LINE__, ##__VA_ARGS__); } while(0)

/**
 * @brief 初始化日志文件
 */
int logger_init(const char *path);

/**
 * @brief 关闭日志
 */
void logger_close(void);

#endif /* PACER_LOGGER_H */
