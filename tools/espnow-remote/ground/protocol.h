/**
 * @file protocol.h
 * @brief PACER 遥控帧 — 与 src/remote/remote.c / pacer-remote 一致
 */

#ifndef PACER_ESPNOW_PROTOCOL_H
#define PACER_ESPNOW_PROTOCOL_H

#include <stdint.h>
#include <stddef.h>

#define PACER_FRAME_HDR1   0xAA
#define PACER_FRAME_HDR2   0x55
#define PACER_FRAME_SIZE   19

static inline uint8_t pacer_frame_xor(const uint8_t *buf)
{
    uint8_t crc = 0;
    for (int i = 0; i < PACER_FRAME_SIZE - 1; i++)
        crc ^= buf[i];
    return crc;
}

static inline int pacer_frame_valid(const uint8_t *buf, size_t len)
{
    if (len != PACER_FRAME_SIZE)
        return 0;
    if (buf[0] != PACER_FRAME_HDR1 || buf[1] != PACER_FRAME_HDR2)
        return 0;
    return buf[18] == pacer_frame_xor(buf);
}

#endif /* PACER_ESPNOW_PROTOCOL_H */
