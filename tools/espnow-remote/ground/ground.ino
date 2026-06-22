/**
 * PACER ESP-NOW 地面端（手柄 / USB 串口桥）
 *
 * 模式 A — 串口桥（默认）:
 *   电脑 USB → 本机 Serial，收到 19 字节帧 → ESP-NOW 发往飞机
 *   用法: pacer-remote -port COMx  （COMx 为本 ESP 的 USB 口）
 *
 * 模式 B — 演示摇杆（无电脑）:
 *   编译时 -D DEMO_JOYSTICK=1，周期性发测试波形
 *
 * 模式 C — ADC 摇杆（无电脑）:
 *   编译时 -D ADC_JOYSTICK=1，读 GPIO34/35/32/33 电位器
 *
 * Arduino: ESP32 Dev Module / ESP32 Mini
 */

#include <WiFi.h>
#include <esp_now.h>
#include "protocol.h"

#if __has_include("mac_config.h")
#include "mac_config.h"
#endif

#ifndef DEMO_JOYSTICK
#define DEMO_JOYSTICK 0
#endif

#ifndef ADC_JOYSTICK
#define ADC_JOYSTICK 0
#endif

#ifndef JOY_THR_PIN
#define JOY_THR_PIN   34
#endif
#ifndef JOY_ROLL_PIN
#define JOY_ROLL_PIN  35
#endif
#ifndef JOY_PITCH_PIN
#define JOY_PITCH_PIN 32
#endif
#ifndef JOY_YAW_PIN
#define JOY_YAW_PIN   33
#endif

/* 机载 ESP32 的 MAC */
#ifdef PACER_AIR_MAC
uint8_t airMac[] = PACER_AIR_MAC;
#else
uint8_t airMac[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
#endif

static uint8_t  rxBuf[PACER_FRAME_SIZE];
static int      rxIdx = 0;
static uint32_t txCount = 0;
static uint32_t txFail = 0;

static void sendFrame(const uint8_t *frame)
{
    esp_err_t r = esp_now_send(airMac, frame, PACER_FRAME_SIZE);
    if (r == ESP_OK)
        txCount++;
    else
        txFail++;
}

#if ESP_ARDUINO_VERSION_MAJOR >= 3
void onEspNowSent(const esp_now_send_info_t *info, esp_now_send_status_t status)
#else
void onEspNowSent(const uint8_t *mac, esp_now_send_status_t status)
#endif
{
    (void)info;
    if (status != ESP_NOW_SEND_SUCCESS)
        txFail++;
}

static void feedByte(uint8_t b)
{
    if (rxIdx == 0) {
        if (b != PACER_FRAME_HDR1) return;
        rxBuf[rxIdx++] = b;
        return;
    }
    if (rxIdx == 1) {
        if (b != PACER_FRAME_HDR2) {
            rxIdx = 0;
            return;
        }
    }
    rxBuf[rxIdx++] = b;
    if (rxIdx < PACER_FRAME_SIZE)
        return;

    rxIdx = 0;
    if (pacer_frame_valid(rxBuf, PACER_FRAME_SIZE))
        sendFrame(rxBuf);
}

static float mapAdc(int pin, float outMin, float outMax)
{
    int raw = analogRead(pin);
    float norm = (float)raw / 4095.0f;
    return outMin + norm * (outMax - outMin);
}

static void buildFrame(uint8_t *f, float thr, float roll, float pitch, float yaw)
{
    f[0] = PACER_FRAME_HDR1;
    f[1] = PACER_FRAME_HDR2;
    memcpy(f + 2, &thr, 4);
    memcpy(f + 6, &roll, 4);
    memcpy(f + 10, &pitch, 4);
    memcpy(f + 14, &yaw, 4);
    f[18] = pacer_frame_xor(f);
}

#if DEMO_JOYSTICK
static void demoSend()
{
    static uint32_t t0 = 0;
    float t = (millis() - t0) / 1000.0f;
    uint8_t f[PACER_FRAME_SIZE];
    float thr = 0.0f;
    float roll = 0.3f * sinf(t);
    float pitch = 0.2f * cosf(t * 0.7f);
    float yaw = 0.1f * sinf(t * 0.5f);
    buildFrame(f, thr, roll, pitch, yaw);
    sendFrame(f);
}
#endif

#if ADC_JOYSTICK
static void adcSend()
{
    uint8_t f[PACER_FRAME_SIZE];
    float thr = mapAdc(JOY_THR_PIN, 0.0f, 1.0f);
    float roll = mapAdc(JOY_ROLL_PIN, -1.0f, 1.0f);
    float pitch = mapAdc(JOY_PITCH_PIN, -1.0f, 1.0f);
    float yaw = mapAdc(JOY_YAW_PIN, -1.0f, 1.0f);
    buildFrame(f, thr, roll, pitch, yaw);
    sendFrame(f);
}
#endif

void setup()
{
    Serial.begin(115200);
    delay(500);

    WiFi.mode(WIFI_STA);
    WiFi.disconnect();

    Serial.println();
    Serial.println("=== PACER ESP-NOW Ground ===");
    Serial.print("Ground MAC (copy to air sketch): ");
    Serial.println(WiFi.macAddress());

    if (esp_now_init() != ESP_OK) {
        Serial.println("ESP-NOW init failed");
        while (1) delay(1000);
    }

#if ESP_ARDUINO_VERSION_MAJOR >= 3
    esp_now_register_send_cb(onEspNowSent);
#else
    esp_now_register_send_cb(onEspNowSent);
#endif

    esp_now_peer_info_t peer = {};
    memcpy(peer.peer_addr, airMac, 6);
    peer.channel = 0;
    peer.encrypt = false;
    if (esp_now_add_peer(&peer) != ESP_OK) {
        Serial.println("Warning: add peer failed (set airMac first)");
    }

#if ADC_JOYSTICK
    analogReadResolution(12);
    Serial.println("ADC_JOYSTICK mode, 50Hz from GPIO34/35/32/33");
#elif DEMO_JOYSTICK
    Serial.println("DEMO_JOYSTICK mode, 50Hz test waveform");
#else
    Serial.println("Serial bridge: feed 19-byte frames from pacer-remote");
#endif
}

void loop()
{
#if ADC_JOYSTICK || DEMO_JOYSTICK
    static uint32_t last;
    if (millis() - last >= 20) {
        last = millis();
#if ADC_JOYSTICK
        adcSend();
#else
        demoSend();
#endif
    }
#else
    while (Serial.available() > 0)
        feedByte((uint8_t)Serial.read());
#endif

    static uint32_t statLast;
    if (millis() - statLast > 3000) {
        statLast = millis();
        Serial.printf("espnow_tx=%lu fail=%lu\r\n",
                      (unsigned long)txCount, (unsigned long)txFail);
    }
}
