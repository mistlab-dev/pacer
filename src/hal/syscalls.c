/**
 * @file syscalls.c
 * @brief Newlib 系统调用桩 — 裸机 STM32
 */

#include <errno.h>
#include <sys/stat.h>
#include <stdint.h>
#include "usart_printf.h"

int errno;

int _close(int fd)
{
    (void)fd;
    errno = EBADF;
    return -1;
}

int _fstat(int fd, struct stat *st)
{
    (void)fd;
    if (st) {
        st->st_mode = S_IFCHR;
    }
    return 0;
}

int _isatty(int fd)
{
    (void)fd;
    return 1;
}

int _lseek(int fd, int ptr, int dir)
{
    (void)fd;
    (void)ptr;
    (void)dir;
    return 0;
}

int _read(int fd, char *ptr, int len)
{
    (void)fd;
    (void)ptr;
    (void)len;
    errno = EBADF;
    return -1;
}

int _write(int fd, const char *ptr, int len)
{
    (void)fd;
    if (ptr && len > 0) {
        uart_write_bytes((const uint8_t *)ptr, (uint16_t)len);
    }
    return len;
}

void *_sbrk(int incr)
{
    extern char _end;
    extern char _heap_limit;   /* 链接脚本定义, AXI SRAM 末尾 */
    static char *heap_end = &_end;
    char *prev = heap_end;

    if (heap_end + incr > (char *)&_heap_limit) {
        errno = ENOMEM;
        return (void *)-1;
    }

    heap_end += incr;
    return (void *)prev;
}
