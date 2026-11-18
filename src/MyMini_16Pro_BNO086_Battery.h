#ifndef MYMINI16PRO_BNO086_BATTERY_H
#define MYMINI16PRO_BNO086_BATTERY_H

/*
 * MyMini_16Pro_BNO086_Battery.h
 * แรงดันแบต (ADS1115 AIN0), ไฟระดับแบต 8 ดวง (PCF8574) และเสียงเตือนแบตต่ำ
 * — เขียนใหม่จาก readBatteryVoltage / updateBatteryLed / updateBatteryWarning ของ MyMINI_Pro.cpp
 *
 * สารบัญ:
 *   [1] ค่าคงที่
 *   [2] ตัวแปร (Global Variables)
 *   [3] อ่านแรงดัน / ระดับแบต
 *   [4] ไฟระดับแบต (PCF8574)
 *   [5] เสียงเตือนแบตต่ำ (non-blocking)
 */

#include <Arduino.h>
#include "MyMini_16Pro_BNO086_Buzzer.h"
#include "MyMini_16Pro_BNO086_I2C.h"

// ======================================================================
// [1] ค่าคงที่
// ======================================================================

#define BAT_DIVIDER ((20000.0f + 6800.0f) / 6800.0f)  // ตัวแบ่ง 20k / 6.8k
#define BAT_CAL (12.53f / 12.18f)                      // คาลิเบรตจุดเดียว
#define BAT_LED_MIN_MV 11000
#define BAT_LED_MAX_MV 12300
#define BAT_ZERO_LED_MV 11175   // ≤ ค่านี้ = 0 ดวง
#define BAT_LOCK_MV 10850       // 6000..10850 mV = ห้ามออกตัว
#define BAT_MIN_VALID_MV 6000   // ต่ำกว่านี้ = ไม่ได้เสียบแบต (ไฟ USB)
#define BAT_LED_UPDATE_MS 200
#define BAT_CRITICAL_STEP_MS 100
#define BAT_WARN_2LED_HZ 1800
#define BAT_WARN_1LED_HZ 3200
#define BAT_WARN_0LED_HZ 3500
#define BAT_LEVEL_CRITICAL 0xFE

// ======================================================================
// [2] ตัวแปร (Global Variables)
// ======================================================================

bool batValid = false;
int16_t batRaw = 0;
int32_t batMillivolts = 0;
uint8_t batLevel = 0;  // 0..8 ดวง

// ======================================================================
// [3] อ่านแรงดัน / ระดับแบต
// ======================================================================

uint8_t batteryLedLevel(int32_t mv) {
  if (mv <= BAT_ZERO_LED_MV) return 0;
  return (uint8_t)constrain(map(mv, BAT_LED_MIN_MV, BAT_LED_MAX_MV, 0L, 8L), 0L, 8L);
}

bool readBatteryVoltage() {
  int16_t raw = 0;
  batValid = readADS1115(ADS_BATTERY, raw);
  if (!batValid) return false;
  batRaw = raw;
  batMillivolts = (int32_t)(adsToVolts(raw) * BAT_DIVIDER * BAT_CAL * 1000.0f + 0.5f);
  batLevel = batteryLedLevel(batMillivolts);
  return true;
}

float getBatteryVoltage() {
  return adsToVolts(batRaw) * BAT_DIVIDER * BAT_CAL;
}

// true = แบตวิกฤติ ห้ามกด START ออกตัว
bool isBatteryStartBlocked() {
  return batValid && batMillivolts >= BAT_MIN_VALID_MV && batMillivolts <= BAT_LOCK_MV;
}

// ======================================================================
// [4] ไฟระดับแบต (PCF8574)
// ======================================================================

unsigned long batLedLastMs = 0;
bool batLedWritten = false;
uint8_t batLedLastMask = 0;
bool batLedCritical = false;
uint8_t batLedPhase = 0;

void BatteryLED_Reset(unsigned long nowMs) {
  batLedLastMs = nowMs;
  batLedWritten = false;
  batLedLastMask = 0;
  batLedCritical = false;
  batLedPhase = 0;
}

// ปกติ: แถบไฟตามระดับ ทุก 200 ms | วิกฤติ: ไฟวิ่ง 0..7..0 ทุก 100 ms
void BatteryLED_Update(unsigned long nowMs) {
  bool critical = isBatteryStartBlocked();
  unsigned long interval = critical ? BAT_CRITICAL_STEP_MS : BAT_LED_UPDATE_MS;
  if (critical != batLedCritical) {
    batLedCritical = critical;
    batLedPhase = 0;
    batLedWritten = false;
    batLedLastMs = nowMs - interval;
  }
  if (nowMs - batLedLastMs < interval) return;
  batLedLastMs = nowMs;

  uint8_t mask = 0;
  if (critical) {
    uint8_t pos = batLedPhase <= 7 ? batLedPhase : 14 - batLedPhase;
    mask = (uint8_t)(1U << pos);
  } else if (batValid) {
    mask = batLevel == 0 ? 0x00 : (uint8_t)(0xFFU << (8U - batLevel));
  }
  if (batLedWritten && mask == batLedLastMask) return;
  if (LED_Write(mask)) {
    batLedLastMask = mask;
    batLedWritten = true;
    if (critical) batLedPhase = (batLedPhase + 1) % 14;
  }
}

// ======================================================================
// [5] เสียงเตือนแบตต่ำ (non-blocking)
// ======================================================================

// 2 ดวง: 1800 Hz 50 ms × 2 รอบ 2 วินาที | 1 ดวง: 3200 Hz 180 ms × 2 รอบ 1 วินาที
// 0 ดวง: 3500 Hz 1 วินาทีเปิด/ปิด | วิกฤติ: 3500 Hz 100 ms เปิด/ปิด
#define BW_IDLE 0
#define BW_FIRST 1
#define BW_GAP 2
#define BW_SECOND 3
#define BW_PAUSE 4
#define BW_ZERO 5
#define BW_ZERO_GAP 6

uint8_t bwState = BW_IDLE;
uint8_t bwLevel = 0xFF;
unsigned long bwDeadline = 0, bwCycleStart = 0;
bool bwToneOn = false;

void BatteryWarning_Stop() {
  if (bwToneOn) noTone(BUZZER_PIN);
  bwState = BW_IDLE;
  bwLevel = 0xFF;
  bwToneOn = false;
}

void bwTone(uint16_t hz, unsigned long ms, unsigned long nowMs) {
  tone(BUZZER_PIN, hz);
  bwToneOn = true;
  bwDeadline = nowMs + ms;
}

void bwStartPattern(uint8_t level, unsigned long nowMs) {
  bwCycleStart = nowMs;
  if (level == BAT_LEVEL_CRITICAL) {
    bwTone(BAT_WARN_0LED_HZ, BAT_CRITICAL_STEP_MS, nowMs);
    bwState = BW_ZERO;
  } else if (level == 2) {
    bwTone(BAT_WARN_2LED_HZ, 50, nowMs);
    bwState = BW_FIRST;
  } else if (level == 1) {
    bwTone(BAT_WARN_1LED_HZ, 180, nowMs);
    bwState = BW_FIRST;
  } else {
    bwTone(BAT_WARN_0LED_HZ, 1000, nowMs);
    bwState = BW_ZERO;
  }
}

void BatteryWarning_Update(unsigned long nowMs) {
  if (!batValid || batMillivolts < BAT_MIN_VALID_MV) {
    BatteryWarning_Stop();
    return;
  }
  uint8_t level = isBatteryStartBlocked() ? BAT_LEVEL_CRITICAL : batLevel;
  if (level >= 3 && level != BAT_LEVEL_CRITICAL) {
    BatteryWarning_Stop();
    return;
  }
  if (bwLevel != level) {
    BatteryWarning_Stop();
    bwLevel = level;
    bwStartPattern(level, nowMs);
    return;
  }
  if ((long)(nowMs - bwDeadline) < 0) return;

  switch (bwState) {
    case BW_FIRST:
      noTone(BUZZER_PIN);
      bwToneOn = false;
      bwDeadline = nowMs + 100;
      bwState = BW_GAP;
      break;
    case BW_GAP:
      bwTone(level == 2 ? BAT_WARN_2LED_HZ : BAT_WARN_1LED_HZ, level == 2 ? 50 : 180, nowMs);
      bwState = BW_SECOND;
      break;
    case BW_SECOND:
      noTone(BUZZER_PIN);
      bwToneOn = false;
      bwDeadline = bwCycleStart + (level == 2 ? 2000U : 1000U);
      bwState = BW_PAUSE;
      break;
    case BW_PAUSE:
    case BW_ZERO_GAP:
      bwStartPattern(level, nowMs);
      break;
    case BW_ZERO:
      noTone(BUZZER_PIN);
      bwToneOn = false;
      bwDeadline = nowMs + (level == BAT_LEVEL_CRITICAL ? BAT_CRITICAL_STEP_MS : 1000U);
      bwState = BW_ZERO_GAP;
      break;
    default:
      break;
  }
}

// ======================================================================
// [6] BatteryMonitor แบบ MyRP_Pico2 (อ็อบเจกต์ bat) บนฮาร์ดแวร์ MyMINI
// ======================================================================
// เมธอด / เกณฑ์ / ตัวกรองตาม BatteryMonitor.cpp ของ MyRP:
//   อ่านทุก 100 ms, กรอง 0.87 / 0.13, เต็ม 12.40 V, หมด 11.0 V, ไม่มีแบต < 6.0 V, เตือน 1400/2200/3000 Hz ทุก 3 วินาที
// แรงดันอ่านด้วย readBatteryVoltage() ของ MyMINI (ADS1115 AIN0 + ตัวแบ่ง 20k / 6.8k + คาลิเบรต MyMINI)
// ไฟ LED ใช้รูปแบบ 8 ดวงของ MyMINI_16Pro บน PCF8574 (BatteryLED_Update)
class BatteryMonitor {
public:
  BatteryMonitor()
    : _voltage(11.8f), _rawVoltage(11.8f), _lastUpdate(0), _lastBuzz(0), _debug(false) {}

  void begin() {
    noTone(BUZZER_PIN);
    LED_Off();
    BatteryLED_Reset(millis());
    if (readBatteryVoltage()) {
      _rawVoltage = getBatteryVoltage();
      _voltage = _rawVoltage;
    }
    Serial.println("=== BatteryMonitor initialized (Non-blocking mode) ===");
  }

  void update() {
    unsigned long now = millis();
    if (now - _lastUpdate < 100) return;  // อ่านจริงทุก 100 ms
    _lastUpdate = now;
    if (readBatteryVoltage()) {
      _rawVoltage = getBatteryVoltage();
      _voltage = 0.87f * _voltage + 0.13f * _rawVoltage;
    }
  }

  void update_led() {
    update();
    if (_debug) {
      Serial.print("BAT: ");
      Serial.print(_voltage, 3);
      Serial.println(" V");
    }
    BatteryLED_Update(millis());
    if (isLowBattery()) buzzerAlertNonBlocking();
  }

  float getVoltage() const {
    return _voltage;
  }

  float getRawVoltage() const {
    return _rawVoltage;
  }

  bool isLowBattery() const {
    return (_voltage < V_EMPTY && _voltage >= V_MIN_VALID);
  }

  bool isNoBattery() const {
    return (_voltage < V_MIN_VALID);
  }

  bool isNormal() const {
    return (_voltage >= V_EMPTY);
  }

  void setDebug(bool enable) {
    _debug = enable;
  }

private:
  static constexpr float V_FULL = 12.40f;
  static constexpr float V_EMPTY = 11.0f;
  static constexpr float V_MIN_VALID = 6.0f;
  float _voltage;
  float _rawVoltage;
  unsigned long _lastUpdate;
  unsigned long _lastBuzz;
  bool _debug;

  void buzzerAlertNonBlocking() {
    unsigned long now = millis();
    if (now - _lastBuzz < 3000) return;
    _lastBuzz = now;
    tone(BUZZER_PIN, 1400, 120);
    delay(100);
    tone(BUZZER_PIN, 2200, 120);
    delay(100);
    tone(BUZZER_PIN, 3000, 120);
  }
};

BatteryMonitor bat;

#endif  // MYMINI16PRO_BNO086_BATTERY_H
