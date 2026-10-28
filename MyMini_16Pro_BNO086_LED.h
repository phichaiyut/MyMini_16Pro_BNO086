#ifndef MYMINI_16PRO_LED_H
#define MYMINI_16PRO_LED_H

/*
 * MyMini_16Pro_BNO086_LED.h
 * ไฟ LED 8 ดวงบน PCF8574 และลูกเล่นไฟ
 *
 * สารบัญ:
 *   [1] สถานะไฟ / เขียนไฟ (LED_Write / LED_Set / LED_Bar)
 *   [2] ลูกเล่นไฟแบบรอจนจบ (Blink, Chase, KnightRider, Fill, Center ...)
 *   [3] ลูกเล่นไฟแบบไม่บล็อก (เรียกซ้ำใน loop)
 */

#include "MyMini_16Pro_BNO086_I2C.h"

// ======================================================================
// [1] สถานะไฟ / เขียนไฟ (LED_Write / LED_Set / LED_Bar)
// ======================================================================

#define PCF_LED_COUNT 8

// bit0 = ดวงที่ 0 ... bit7 = ดวงที่ 7 (PCF8574 ออก HIGH = ไฟติด)
uint8_t pcfLedState = 0;

bool LED_Write(uint8_t mask) {
  if (!writePCF8574(mask)) return false;
  pcfLedState = mask;
  return true;
}

void LED_Off() {
  LED_Write(0x00);
}

void LED_On() {
  LED_Write(0xFF);
}

void LED_Set(int i, bool on) {
  if (i < 0 || i >= PCF_LED_COUNT) return;
  uint8_t mask = on ? (pcfLedState | (1 << i)) : (pcfLedState & ~(1 << i));
  LED_Write(mask);
}

void LED_Toggle(int i) {
  if (i < 0 || i >= PCF_LED_COUNT) return;
  LED_Write(pcfLedState ^ (1 << i));
}

// แถบไฟตามระดับ 0-8 (เติมจาก bit7 ลงมา เหมือนแถบแบตของ MyMINI_16Pro)
uint8_t LED_BarMask(int level) {
  level = constrain(level, 0, PCF_LED_COUNT);
  if (level == 0) return 0x00;
  return (uint8_t)(0xFF << (PCF_LED_COUNT - level));
}

void LED_Bar(int level) {
  LED_Write(LED_BarMask(level));
}

// แสดงเลข 0-255 เป็นเลขฐานสอง (เช่นบอกหมายเลขภารกิจ)
void LED_Number(uint8_t n) {
  LED_Write(n);
}

// ======================================================================
// [2] ลูกเล่นไฟแบบรอจนจบ (Blink, Chase, KnightRider, Fill, Center ...)
// ======================================================================

// กระพริบทุกดวงพร้อมกัน
void LED_Blink(int times = 3, int ms = 150) {
  for (int i = 0; i < times; i++) {
    LED_On();
    delay(ms);
    LED_Off();
    delay(ms);
  }
}

// ไฟวิ่งทีละดวง 0 → 7
void LED_Chase(int times = 1, int ms = 60) {
  for (int t = 0; t < times; t++) {
    for (int i = 0; i < PCF_LED_COUNT; i++) {
      LED_Write(1 << i);
      delay(ms);
    }
  }
  LED_Off();
}

// ไฟวิ่งทีละดวง 7 → 0
void LED_ChaseBack(int times = 1, int ms = 60) {
  for (int t = 0; t < times; t++) {
    for (int i = PCF_LED_COUNT - 1; i >= 0; i--) {
      LED_Write(1 << i);
      delay(ms);
    }
  }
  LED_Off();
}

// ไฟวิ่งไป-กลับ 0..7..0 (ไม่ซ้ำดวงปลาย)
void LED_KnightRider(int times = 2, int ms = 50) {
  for (int t = 0; t < times; t++) {
    for (int i = 0; i < PCF_LED_COUNT; i++) {
      LED_Write(1 << i);
      delay(ms);
    }
    for (int i = PCF_LED_COUNT - 2; i > 0; i--) {
      LED_Write(1 << i);
      delay(ms);
    }
  }
  LED_Off();
}

// เติมไฟทีละดวงจนเต็ม แล้วดับทีละดวง
void LED_Fill(int times = 1, int ms = 60) {
  for (int t = 0; t < times; t++) {
    uint8_t mask = 0;
    for (int i = 0; i < PCF_LED_COUNT; i++) {
      mask |= (1 << i);
      LED_Write(mask);
      delay(ms);
    }
    for (int i = 0; i < PCF_LED_COUNT; i++) {
      mask &= ~(1 << i);
      LED_Write(mask);
      delay(ms);
    }
  }
  LED_Off();
}

// กระจายจากกลางออกข้าง แล้วหุบกลับ
void LED_Center(int times = 1, int ms = 80) {
  const uint8_t frames[] = { 0x18, 0x3C, 0x7E, 0xFF, 0x7E, 0x3C, 0x18, 0x00 };
  for (int t = 0; t < times; t++) {
    for (uint8_t f : frames) {
      LED_Write(f);
      delay(ms);
    }
  }
  LED_Off();
}

// สลับดวงคู่ / ดวงคี่
void LED_Alternate(int times = 4, int ms = 120) {
  for (int t = 0; t < times; t++) {
    LED_Write(0x55);
    delay(ms);
    LED_Write(0xAA);
    delay(ms);
  }
  LED_Off();
}

// สุ่มไฟ (ลูกเล่นตอนรอ)
void LED_Random(int times = 10, int ms = 80) {
  for (int t = 0; t < times; t++) {
    LED_Write((uint8_t)random(0, 256));
    delay(ms);
  }
  LED_Off();
}

// ไฟพร้อมใช้งาน: กระจายจากกลาง + ไฟวิ่งไป-กลับหนึ่งรอบ
void LED_Ready() {
  LED_Center(1, 50);
  LED_KnightRider(1, 35);
}

// ======================================================================
// [3] ลูกเล่นไฟแบบไม่บล็อก (เรียกซ้ำใน loop)
// ======================================================================

// ไฟวิ่งไป-กลับ เปลี่ยนดวงทุก ms มิลลิวินาที — ใช้ตอน Calibrate / แบตวิกฤติ
void LED_ScannerUpdate(int ms = 80) {
  static unsigned long last = 0;
  static uint8_t phase = 0;
  unsigned long now = millis();
  if (now - last < (unsigned long)ms) return;
  last = now;
  uint8_t pos = phase <= 7 ? phase : 14 - phase;
  if (LED_Write(1 << pos)) phase = (phase + 1) % 14;
}

// กระพริบทุกดวงแบบไม่บล็อก
void LED_BlinkUpdate(int ms = 250) {
  static unsigned long last = 0;
  unsigned long now = millis();
  if (now - last < (unsigned long)ms) return;
  last = now;
  LED_Write(pcfLedState ? 0x00 : 0xFF);
}

#endif  // MYMINI_16PRO_LED_H
