/**
 * @file logger.c
 * @brief 日志工具实现
 */

#include "utils/logger.h"
#include "config.h"
#include <stdio.h>
#include <stdarg.h>
#include <string.h>

int g_log_level = LOG_LEVEL_INFO;
static FILE *g_log_file = NULL;

int logger_init(const char *path)
{
    if (path && strlen(path) > 0) {
        g_log_file = fopen(path, "a");
        if (!g_log_file) {
            fprintf(stderr, "[LOG] Cannot open log file: %s\n", path);
            return -1;
        }
        /* 设置行缓冲 */
        setvbuf(g_log_file, NULL, _IOLBF, 0);
        printf("[LOG] Logging to %s\n", path);
    }
    return 0;
}

void logger_close(void)
{
    if (g_log_file) {
        fclose(g_log_file);
        g_log_file = NULL;
    }
}
