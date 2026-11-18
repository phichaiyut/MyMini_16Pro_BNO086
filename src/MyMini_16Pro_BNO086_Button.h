#ifndef MYMINI16PRO_BNO086_BUTTON_H
#define MYMINI16PRO_BNO086_BUTTON_H

/*
 * MyMini_16Pro_BNO086_Button.h
 * ปุ่ม START (GP2), ปุ่ม Calibrate หน้า (GP3) และปุ่ม ADCcal (MCP3421) — จาก CalibrationManager.cpp
 *
 * บอร์ดนี้ไม่มีปุ่ม OK แยก: กด START สั้น = ออกตัว, กด START ค้าง 3 วินาที = Calibrate กลาง
 *
 * สารบัญ:
 *   [1] ขา / ค่าคงที่
 *   [2] อ่านปุ่มแบบทันที (START_PUSH / OK_PUSH / CAL_PUSH / ADC_CAL_PUSH)
 *   [3] รอกดปุ่ม (swSTART)
 *   [4] ตัวจับเหตุการณ์ปุ่ม (debounce / กดสั้น / กดค้าง / ขอบปุ่ม Calibrate)
 */

#include <Arduino.h>
#include "MyMini_16Pro_BNO086_Buzzer.h"
#include "MyMini_16Pro_BNO086_I2C.h"

// ======================================================================
// [1] ขา / ค่าคงที่
// ======================================================================

#define START_PIN 2   // PIN_START_BUTTON (INPUT_PULLUP, active LOW)
#define CAL_F_PIN 3   // PIN_FRONT_CAL จากบอร์ดเซนเซอร์หน้า (active LOW)
#define PRESS 0

#define BTN_DEBOUNCE_MS 30
#define BTN_SHORT_MIN_MS 30
#define BTN_SHORT_MAX_MS 800
#define BTN_LONG_MS 3000
#define ADCCAL_POLL_MS 100
#define ADCCAL_THRESHOLD 512     // MCP3421 ≤ 512 = กดปุ่ม ADCcal
#define ADCCAL_CONFIRM 3         // ต้องอ่านได้ติดกัน 3 ครั้ง

#define BTN_NONE 0
#define BTN_START_SHORT 1  // กด START สั้น 30–800 ms (ปล่อยแล้ว)
#define BTN_START_LONG 2   // กด START ค้างครบ 3 วินาที (ยังไม่ปล่อย)
#define BTN_CAL_F 3        // ขอบกด GP3
#define BTN_CAL_B 4        // ปุ่ม ADCcal ยืนยันแล้ว

// ======================================================================
// [2] อ่านปุ่มแบบทันที (START_PUSH / CAL_PUSH / ADC_CAL_PUSH)
// ======================================================================

void ButtonBegin();

// คืน PRESS (0) เมื่อกด, 1 เมื่อไม่กด (OK_PUSH / START_PUSH)
int START_PUSH() {
  return digitalRead(START_PIN) == LOW ? PRESS : 1;
}

// ปุ่ม OK → MyMINI ไม่มีปุ่ม OK แยก ใช้ปุ่ม Calibrate หน้า GP3 แทน
// (GP19 ของ MyMINI เป็นขา EN มอเตอร์ซ้าย ห้ามอ่าน / ตั้งเป็น input)
#define OK_PIN CAL_F_PIN

int OK_PUSH() {
  return digitalRead(OK_PIN) == LOW ? PRESS : 1;
}

int CAL_PUSH() {
  return digitalRead(CAL_F_PIN) == LOW ? PRESS : 1;
}

// อ่าน MCP3421 ครั้งเดียว (ไม่ยืนยัน 3 ครั้ง) — อ่านไม่ได้ = ไม่กด
int ADC_CAL_PUSH() {
  int16_t raw = 0;
  if (!readMCP3421(raw)) return 1;
  return raw <= ADCCAL_THRESHOLD ? PRESS : 1;
}

// ======================================================================
// [3] รอกดปุ่ม (swSTART)
// ======================================================================

// รอกด START (debounce 30 ms) และรอปล่อย แล้วปี๊บ
void swSTART() {
  while (true) {
    if (digitalRead(START_PIN) == LOW) {
      delay(BTN_DEBOUNCE_MS);
      if (digitalRead(START_PIN) == LOW) break;
    }
  }
  while (digitalRead(START_PIN) == LOW) {}
  delay(BTN_DEBOUNCE_MS);
  Beep(200);
}

// ======================================================================
// [4] ตัวจับเหตุการณ์ปุ่ม (debounce / กดสั้น / กดค้าง / ขอบปุ่ม Calibrate)
// ======================================================================

bool btnRaw = false, btnStable = false, btnArmed = false, btnTracking = false;
unsigned long btnChangedMs = 0, btnPressedMs = 0;
bool calFLatched = false;
bool adcCalLatched = true;  // ต้องเห็นปล่อยก่อน กันค่าค้างตอนบูตลบค่า Calibrate หลัง
uint8_t adcCalCount = 0;
unsigned long adcCalNextMs = 0;

void ButtonBegin() {
  pinMode(START_PIN, INPUT_PULLUP);
  pinMode(CAL_F_PIN, INPUT_PULLUP);
  unsigned long now = millis();
  btnRaw = digitalRead(START_PIN) == LOW;
  btnStable = btnRaw;
  btnArmed = !btnStable;  // กดค้างตั้งแต่บูต → ต้องปล่อยก่อน
  btnTracking = false;
  btnChangedMs = now;
  adcCalNextMs = now;
  calFLatched = digitalRead(CAL_F_PIN) == LOW;
  adcCalLatched = true;
  adcCalCount = 0;
}

// ปุ่ม START: คืน BTN_START_SHORT / BTN_START_LONG / BTN_NONE
uint8_t ButtonStartUpdate(unsigned long now) {
  bool pressed = digitalRead(START_PIN) == LOW;
  if (pressed != btnRaw) {
    btnRaw = pressed;
    btnChangedMs = now;
  }
  if (btnRaw != btnStable && now - btnChangedMs >= BTN_DEBOUNCE_MS) {
    btnStable = btnRaw;
    if (btnStable) {
      if (btnArmed) {
        btnPressedMs = now;
        btnTracking = true;
      }
    } else {
      bool shortPress = false;
      if (btnTracking) {
        unsigned long held = now - btnPressedMs;
        shortPress = held >= BTN_SHORT_MIN_MS && held <= BTN_SHORT_MAX_MS;
      }
      btnTracking = false;
      btnArmed = true;
      if (shortPress) return BTN_START_SHORT;
    }
  }
  if (btnTracking && now - btnPressedMs >= BTN_LONG_MS) {
    btnTracking = false;
    btnArmed = false;  // ต้องปล่อยก่อนถึงจะกดใหม่ได้
    return BTN_START_LONG;
  }
  return BTN_NONE;
}

// ปุ่ม Calibrate: คืน BTN_CAL_F / BTN_CAL_B / BTN_NONE
uint8_t ButtonCalUpdate(unsigned long now) {
  if (digitalRead(CAL_F_PIN) != LOW) {
    calFLatched = false;
  } else if (!calFLatched) {
    calFLatched = true;
    return BTN_CAL_F;
  }

  if ((long)(now - adcCalNextMs) < 0) return BTN_NONE;
  adcCalNextMs = now + ADCCAL_POLL_MS;
  int16_t raw = 0;
  if (!readMCP3421(raw)) return BTN_NONE;
  if (raw > ADCCAL_THRESHOLD) {
    adcCalLatched = false;
    adcCalCount = 0;
  } else if (!adcCalLatched && ++adcCalCount >= ADCCAL_CONFIRM) {
    adcCalLatched = true;
    adcCalCount = 0;
    return BTN_CAL_B;
  }
  return BTN_NONE;
}

#endif  // MYMINI16PRO_BNO086_BUTTON_H
