#ifndef MYMINI_16PRO_BATTERY_H
#define MYMINI_16PRO_BATTERY_H

/*
 * MyMini_16Pro_BNO086_Battery.h
 * อ่านแรงดันแบต (ADS1115 AIN0) แสดงระดับบนไฟ LED และเสียงเตือนแบตต่ำ
 *
 * สารบัญ:
 *   [1] ค่าคงที่แบตเตอรี่ (แบต 3S)
 *   [2] อ่านแรงดัน (updateBattery / getBatteryVoltage)
 *   [3] ระดับแบต / สถานะ
 *   [4] แสดงแบตบนไฟ LED + เสียงเตือน (ไม่บล็อก)
 */

#include "MyMini_16Pro_BNO086_Buzzer.h"
#include "MyMini_16Pro_BNO086_LED.h"

// ======================================================================
// [1] ค่าคงที่แบตเตอรี่ (แบต 3S)
// ======================================================================

// ตัวต้านทานแบ่งแรงดัน 20k / 6.8k + ค่าชดเชยจากมิเตอร์ (12.53 V วัดจริง / 12.18 V ที่อ่านได้)
#define BAT_DIVIDER ((20000.0f + 6800.0f) / 6800.0f)
#define BAT_CAL (12.53f / 12.18f)

#define BAT_LED_MIN_MV 11000   // ระดับไฟ 0 ดวง
#define BAT_LED_MAX_MV 12300   // ระดับไฟ 8 ดวง
#define BAT_ZERO_LED_MV 11175  // ต่ำกว่านี้ = 0 ดวง
#define BAT_LOCK_MV 10850      // ต่ำกว่านี้ห้ามออกตัว (แบตวิกฤติ)
#define BAT_MIN_VALID_MV 6000  // ต่ำกว่านี้ถือว่าไม่ได้เสียบแบต (ไฟจาก USB อย่างเดียว)
#define BAT_READ_INTERVAL 100  // อ่าน ADS1115 ไม่ถี่กว่านี้ (ms)

float batVoltage = 0.0f;     // แรงดันที่กรองแล้ว (V)
float batRawVoltage = 0.0f;  // แรงดันครั้งล่าสุด (V)
bool batValid = false;
unsigned long batLastRead = 0;

// ======================================================================
// [2] อ่านแรงดัน (updateBattery / getBatteryVoltage)
// ======================================================================

// อ่านทันที (ใช้ ~2 ms) — คืน false ถ้า ADS1115 ไม่ตอบ
bool readBatteryVoltage() {
  int16_t raw = 0;
  batLastRead = millis();
  if (!readADS1115(ADS_BATTERY, raw)) {
    batValid = false;
    return false;
  }
  batRawVoltage = adsToVolts(raw) * BAT_DIVIDER * BAT_CAL;
  if (!batValid) batVoltage = batRawVoltage;
  else batVoltage = batVoltage * 0.7f + batRawVoltage * 0.3f;
  batValid = true;
  return true;
}

// อ่านแบบจำกัดความถี่ — เรียกได้บ่อยใน loop / Motor()
void updateBattery() {
  if (millis() - batLastRead < BAT_READ_INTERVAL) return;
  readBatteryVoltage();
}

float getBatteryVoltage() {
  return batValid ? batVoltage : 0.0f;
}

int getBatteryMillivolts() {
  return (int)(getBatteryVoltage() * 1000.0f + 0.5f);
}

// ======================================================================
// [3] ระดับแบต / สถานะ
// ======================================================================

// ระดับแบต 0-8 (จำนวนไฟ LED)
int getBatteryLevel() {
  int mv = getBatteryMillivolts();
  if (mv <= BAT_ZERO_LED_MV) return 0;
  return constrain((int)map(mv, BAT_LED_MIN_MV, BAT_LED_MAX_MV, 0, 8), 0, 8);
}

bool isNoBattery() {
  return !batValid || getBatteryMillivolts() < BAT_MIN_VALID_MV;
}

// แบตวิกฤติ: เสียบแบตอยู่แต่ต่ำกว่า BAT_LOCK_MV → sw() จะไม่ยอมออกตัว
bool isBatteryCritical() {
  int mv = getBatteryMillivolts();
  return batValid && mv >= BAT_MIN_VALID_MV && mv <= BAT_LOCK_MV;
}

bool isLowBattery() {
  return !isNoBattery() && (isBatteryCritical() || getBatteryLevel() <= 2);
}

// ======================================================================
// [4] แสดงแบตบนไฟ LED + เสียงเตือน (ไม่บล็อก)
// ======================================================================

// ปกติ = แถบไฟตามระดับ | วิกฤติ = ไฟวิ่งไป-กลับ
void bat_update_led() {
  static unsigned long last = 0;
  if (isBatteryCritical()) {
    LED_ScannerUpdate(100);
    return;
  }
  if (millis() - last < 200) return;
  last = millis();
  uint8_t mask = batValid && !isNoBattery() ? LED_BarMask(getBatteryLevel()) : 0x00;
  if (mask != pcfLedState) LED_Write(mask);
}

// เสียงเตือนตามระดับ (เหมือน MyMINI_16Pro):
//   2 ดวง = ปี๊บสั้น 2 ครั้งทุก 2 วินาที | 1 ดวง = ปี๊บ 2 ครั้งทุก 1 วินาที
//   0 ดวง = ปี๊บยาวทุก 2 วินาที           | วิกฤติ = ปี๊บถี่ทุก 200 ms
void bat_warning() {
  static unsigned long next = 0;
  static uint8_t step = 0;
  if (isNoBattery() || !isLowBattery()) {
    step = 0;
    return;
  }
  unsigned long now = millis();
  if ((long)(now - next) < 0) return;

  if (isBatteryCritical()) {
    tone(buzzer_pin, 3500, 100);
    next = now + 200;
    step = 0;
    return;
  }

  int level = getBatteryLevel();
  if (level == 0) {
    tone(buzzer_pin, 3500, 1000);
    next = now + 2000;
    step = 0;
  } else {
    int freq = level == 2 ? 1800 : 3200;
    int dur = level == 2 ? 50 : 180;
    tone(buzzer_pin, freq, dur);
    if (step == 0) {
      next = now + dur + 100;
      step = 1;
    } else {
      next = now + (level == 2 ? 1850 : 720);
      step = 0;
    }
  }
}

#endif  // MYMINI_16PRO_BATTERY_H
