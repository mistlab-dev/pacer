/**
 * PACER ESP-NOW 机载端（飞机上的 ESP32）
 *
 * ESP-NOW 收 19 字节遥控帧 → USART 转发给 STM32 USART3
 *
 * 接线（ESP32 ↔ STM32 H743）:
 *   ESP TX  → PD9 (USART3_RX)
 *   ESP RX  → PD8 (USART3_TX)
 *   GND     → GND
 *
 * Arduino: 开发板选 ESP32 Dev Module，库 ESP32 by Espressif 2.x+
 */

#include <WiFi.h>
#include <esp_now.h>
#include "protocol.h"

/* Serial2 → STM32 USART3 @ 115200 */
#ifndef UART_TX_PIN
#define UART_TX_PIN  17   /* 接 STM32 PD9 (RX) */
#endif
#ifndef UART_RX_PIN
#define UART_RX_PIN  16   /* 接 STM32 PD8 (TX) */
#endif

/* 地面端 ESP32 的 MAC，烧录前改成 Serial Monitor 里看到的地址 */
uint8_t groundMac[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

static uint32_t rxCount = 0;
static uint32_t badCount = 0;

static void handleFrame(const uint8_t *data, int len)
{
    if (!pacer_frame_valid(data, (size_t)len)) {
        badCount++;
        return;
    }
    Serial2.write(data, PACER_FRAME_SIZE);
    rxCount++;
}

#if ESP_ARDUINO_VERSION_MAJOR >= 3
void onEspNowRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len)
{
    (void)info;
    handleFrame(data, len);
}
#else
void onEspNowRecv(const uint8_t *mac, const uint8_t *data, int len)
{
    (void)mac;
    handleFrame(data, len);
}
#endif

void setup()
{
    Serial.begin(115200);
    delay(500);
    Serial2.begin(115200, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);

    WiFi.mode(WIFI_STA);
    WiFi.disconnect();

    Serial.println();
    Serial.println("=== PACER ESP-NOW Air ===");
    Serial.print("Air MAC (copy to ground sketch): ");
    Serial.println(WiFi.macAddress());

    if (esp_now_init() != ESP_OK) {
        Serial.println("ESP-NOW init failed");
        while (1) delay(1000);
    }

#if ESP_ARDUINO_VERSION_MAJOR >= 3
    esp_now_register_recv_cb(onEspNowRecv);
#else
    esp_now_register_recv_cb(onEspNowRecv);
#endif

    esp_now_peer_info_t peer = {};
    memcpy(peer.peer_addr, groundMac, 6);
    peer.channel = 0;
    peer.encrypt = false;
    if (esp_now_add_peer(&peer) != ESP_OK) {
        Serial.println("Warning: add peer failed (check groundMac)");
    }

    Serial.println("Waiting ESP-NOW frames from ground...");
}

void loop()
{
    static uint32_t last = 0;
    if (millis() - last > 2000) {
        last = millis();
        Serial.printf("fwd=%lu bad=%lu\r\n", (unsigned long)rxCount, (unsigned long)badCount);
    }
}
