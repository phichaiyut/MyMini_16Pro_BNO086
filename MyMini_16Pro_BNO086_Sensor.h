#ifndef MYMINI_16PRO_SENSOR_H
#define MYMINI_16PRO_SENSOR_H

/*
 * MyMini_16Pro_BNO086_Sensor.h
 * เซนเซอร์เส้น 16+16 ตัว (MUX 74HC4067 x2), เซนเซอร์กลาง CL/CR (ADS1115),
 * EEPROM, คาลิเบรต และ Serial debug
 *
 * สารบัญ:
 *   [1] ขา MUX / ตัวแปรเซนเซอร์และค่าอ้างอิง
 *   [2] อ่านค่า ADC ดิบ (MUX หน้า / หลัง + ADS1115 กลาง)
 *   [3] รูปแบบข้อมูล Calibrate ใน EEPROM (150 ไบต์ + CRC16)
 *   [4] EEPROM ในบอร์ด (local flash) — สำรองค่า Calibrate
 *   [5] บันทึก / โหลดค่า Calibrate (EEPROM ภายนอก + สำรอง flash)
 *   [6] คาลิเบรตเซนเซอร์ (calibrateF / B / C)
 *   [7] แสดงค่าคาลิเบรต
 *   [8] อ่านเซนเซอร์ + แปลงเป็นค่าคาลิเบรต (ReadCalibrate)
 *   [9] แสดงเส้นบนไฟ LED
 *   [10] Serial debug (ทุกตัวเป็น while(1) ห้ามเรียกใน Mission)
 */

#include <Arduino.h>
#include <EEPROM.h>
#include <stddef.h>
#include <string.h>
#include "MyMini_16Pro_BNO086_I2C.h"
#include "MyMini_16Pro_BNO086_Buzzer.h"
#include "MyMini_16Pro_BNO086_LED.h"
#include "MyMini_16Pro_BNO086_Motor.h"

// ======================================================================
// [1] ขา MUX / ตัวแปรเซนเซอร์และค่าอ้างอิง
// ======================================================================

#define NUM_SENSORS 16

// มองจากด้านบน: CH0 = ซ้ายสุด, CH15 = ขวาสุด (ทั้งแถวหน้าและหลัง)
// เส้นดำ = ค่า ADC ต่ำ, พื้นขาว = ค่า ADC สูง
#define MUX_F_SIG 27
#define MUX_B_SIG 26
const uint8_t MUX_F_SEL[4] = { 17, 16, 15, 14 };  // S0..S3 แถวหน้า (J13)
const uint8_t MUX_B_SEL[4] = { 10, 11, 12, 13 };  // S0..S3 แถวหลัง (J5)

#define START_BUTTON 2       // ปุ่ม START (กดสั้น = ออกตัว, กดค้าง 3 วิ = Calibrate C)
#define CAL_F_BUTTON 3       // ปุ่ม Calibrate หน้า (active LOW)
#define CAL_B_THRESHOLD 512  // ปุ่ม ADCcal (MCP3421) กด = ค่าต่ำกว่านี้ → Calibrate หลัง

// ตั้งค่าการอ่าน MUX (ค่าเดิมของ MyMINI_16Pro): รอ 40 us, ทิ้ง 1 ครั้ง, เฉลี่ย 2 ครั้ง
int muxSettleUs = 40;
int muxDiscard = 1;
int muxAverage = 2;

int F[NUM_SENSORS], B[NUM_SENSORS], C[2];
uint16_t rawF[NUM_SENSORS], rawB[NUM_SENSORS];  // ค่าดิบตามช่อง MUX
int16_t rawC[2];                                 // ค่าดิบ ADS1115 (ดัชนี CCL / CCR)

// ลำดับช่อง MUX → F[i] / B[i] (สลับลำดับได้ถ้าต่อสายกลับด้าน)
// B[] กลับลำดับ (15 → 0) แบบ MyRP_Pico2_mini_BNO086 / STORMXII: B[0] = MUX CH15
// ถ้าถอยตามเส้นแล้วหุ่นส่ายหนีเส้น ให้กลับเป็น B_PIN = { 0, 1, ..., 15 }
uint8_t F_PIN[NUM_SENSORS] = { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15 };
uint8_t B_PIN[NUM_SENSORS] = { 15, 14, 13, 12, 11, 10, 9, 8, 7, 6, 5, 4, 3, 2, 1, 0 };


// ค่า Calibrate (ค่าดิบ min / max ของแต่ละช่อง)
int minValueF[NUM_SENSORS], maxValueF[NUM_SENSORS];
int minValueB[NUM_SENSORS], maxValueB[NUM_SENSORS];
int minValueC[2], maxValueC[2];

#define CAL_VALID_F 0x01
#define CAL_VALID_B 0x02
#define CAL_VALID_C 0x04
uint8_t calibValid = 0;  // บิตบอกว่ากลุ่มไหน Calibrate แล้ว
unsigned long calibrateTime = 5000;

// ดัชนีเซนเซอร์กลาง: C[CCL] = ซ้าย, C[CCR] = ขวา (ค่าเดียวกับ MyRP_Pico2_mini_BNO086)
#define CCL 1
#define CCR 0
int Ref = 500;
int RefC = 500;
int LineColor = 0;
// เซนเซอร์วัดระยะ: ค่าเริ่มต้น = ADS1115 AIN3 (ADC_AUX) เพราะ GP26/27 ใช้กับ MUX และ GP28 เป็นเซอร์โว
// SetAnalogDistance(28) = ใช้ analogRead(GP28) แทน (ถ้าไม่ได้ต่อเซอร์โวช่องนั้น)
#define DIST_ADS_AUX 255
int DIST = DIST_ADS_AUX;
// FRONT
int FRONT_MIN = 80;
int FRONT_MAX = 900;

// CENTER
int CENTER_MIN = 200;
int CENTER_MAX = 900;

// BACK
int BACK_MIN = 0;
int BACK_MAX = 1000;

// ======================================================================
// [2] อ่านค่า ADC ดิบ (MUX หน้า / หลัง + ADS1115 กลาง)
// ======================================================================

// MUX หน้าและหลังสลับช่องพร้อมกันเสมอ (อ่านทั้งสองแถวในรอบเดียวได้)
void muxSelect(uint8_t channel) {
  for (uint8_t bit = 0; bit < 4; bit++) {
    uint8_t level = (channel >> bit) & 0x01;
    digitalWrite(MUX_F_SEL[bit], level);
    digitalWrite(MUX_B_SEL[bit], level);
  }
}

// ทิ้งค่าแรกหลังสลับช่อง แล้วเฉลี่ย
uint16_t muxReadPin(uint8_t pin) {
  for (int i = 0; i < muxDiscard; i++) {
    (void)analogRead(pin);
  }
  uint32_t sum = 0;
  for (int i = 0; i < muxAverage; i++) {
    sum += analogRead(pin);
  }
  return (uint16_t)(sum / muxAverage);
}

void SensorBegin() {
  for (int i = 0; i < 4; i++) {
    pinMode(MUX_F_SEL[i], OUTPUT);
    pinMode(MUX_B_SEL[i], OUTPUT);
  }
  pinMode(MUX_F_SIG, INPUT);
  pinMode(MUX_B_SIG, INPUT);
  pinMode(START_BUTTON, INPUT_PULLUP);
  pinMode(CAL_F_BUTTON, INPUT_PULLUP);
  analogReadResolution(12);
  muxSelect(0);
}

// settleUs 4-200, discard 0-4, average 1-8
void SetSensorRead(int settleUs, int discard, int average) {
  muxSettleUs = constrain(settleUs, 4, 200);
  muxDiscard = constrain(discard, 0, 4);
  muxAverage = constrain(average, 1, 8);
}

uint16_t read_sensorF(int ch) {
  if (ch < 0 || ch >= NUM_SENSORS) return 0;
  muxSelect(ch);
  delayMicroseconds(muxSettleUs);
  return muxReadPin(MUX_F_SIG);
}

uint16_t read_sensorB(int ch) {
  if (ch < 0 || ch >= NUM_SENSORS) return 0;
  muxSelect(ch);
  delayMicroseconds(muxSettleUs);
  return muxReadPin(MUX_B_SIG);
}

// อ่านครบ 16 ช่อง (ราว 1 ms) เก็บลง rawF[] / rawB[]
void readMuxFrame(bool front, bool back) {
  for (uint8_t ch = 0; ch < NUM_SENSORS; ch++) {
    muxSelect(ch);
    delayMicroseconds(muxSettleUs);
    if (front) rawF[ch] = muxReadPin(MUX_F_SIG);
    if (back) rawB[ch] = muxReadPin(MUX_B_SIG);
  }
}

// i = CCL หรือ CCR — คืน false ถ้า ADS1115 ไม่ตอบ
bool read_sensorC(int i, int16_t &raw) {
  if (i < 0 || i > 1) return false;
  return readADS1115(i == CCL ? ADS_CENTER_L : ADS_CENTER_R, raw);
}

// ======================================================================
// [3] รูปแบบข้อมูล Calibrate ใน EEPROM (150 ไบต์ + CRC16)
// ======================================================================

// โครงสร้างเดียวกับ MyMINI_16Pro (ที่แอดเดรส 0x0000) → ค่าที่เคย Calibrate ไว้ใช้ต่อได้เลย
struct __attribute__((packed)) CalibrationData {
  uint32_t magic;
  uint16_t version;
  uint16_t size;
  uint8_t validMask;
  uint8_t reserved[3];
  uint16_t frontMin[16];
  uint16_t frontMax[16];
  uint16_t rearMin[16];
  uint16_t rearMax[16];
  int16_t centerMin[2];
  int16_t centerMax[2];
  uint16_t crc16;
};

static_assert(sizeof(CalibrationData) == 150, "CalibrationData EEPROM layout must remain 150 bytes");

#define CAL_EEPROM_ADDR 0x0000
#define CAL_MAGIC 0x4D4D5031UL
#define CAL_VERSION 1

uint16_t crc16(const uint8_t *bytes, uint16_t length) {
  uint16_t crc = 0xFFFF;
  for (uint16_t i = 0; i < length; i++) {
    crc ^= (uint16_t)bytes[i] << 8;
    for (uint8_t bit = 0; bit < 8; bit++) {
      crc = (crc & 0x8000U) ? (uint16_t)((crc << 1) ^ 0x1021U) : (uint16_t)(crc << 1);
    }
  }
  return crc;
}

bool calibDataValid(const CalibrationData &d) {
  return d.magic == CAL_MAGIC && d.version == CAL_VERSION && d.size == sizeof(CalibrationData)
         && (d.validMask & ~(CAL_VALID_F | CAL_VALID_B | CAL_VALID_C)) == 0
         && d.reserved[0] == 0 && d.reserved[1] == 0 && d.reserved[2] == 0
         && d.crc16 == crc16((const uint8_t *)&d, offsetof(CalibrationData, crc16));
}

// ค่าในตัวแปร → โครงสร้าง (คำนวณ CRC ให้)
void calibToData(CalibrationData &d) {
  memset(&d, 0, sizeof(d));
  d.magic = CAL_MAGIC;
  d.version = CAL_VERSION;
  d.size = sizeof(CalibrationData);
  d.validMask = calibValid;
  for (int i = 0; i < NUM_SENSORS; i++) {
    d.frontMin[i] = minValueF[i];
    d.frontMax[i] = maxValueF[i];
    d.rearMin[i] = minValueB[i];
    d.rearMax[i] = maxValueB[i];
  }
  // EEPROM: ช่อง 0 = ซ้าย, 1 = ขวา (รูปแบบ MyMINI_16Pro)
  d.centerMin[0] = minValueC[CCL];
  d.centerMax[0] = maxValueC[CCL];
  d.centerMin[1] = minValueC[CCR];
  d.centerMax[1] = maxValueC[CCR];
  d.crc16 = crc16((const uint8_t *)&d, offsetof(CalibrationData, crc16));
}

// โครงสร้าง → ตัวแปร
void dataToCalib(const CalibrationData &d) {
  calibValid = d.validMask;
  for (int i = 0; i < NUM_SENSORS; i++) {
    minValueF[i] = d.frontMin[i];
    maxValueF[i] = d.frontMax[i];
    minValueB[i] = d.rearMin[i];
    maxValueB[i] = d.rearMax[i];
  }
  minValueC[CCL] = d.centerMin[0];
  maxValueC[CCL] = d.centerMax[0];
  minValueC[CCR] = d.centerMin[1];
  maxValueC[CCR] = d.centerMax[1];
}

// ======================================================================
// [4] EEPROM ในบอร์ด (local flash) — สำรองค่า Calibrate
// ======================================================================

// EEPROM (flash) ภายใน Pico 2 ตัวที่รัน sketch นี้เอง
// Layout: CalibrationData 150 ไบต์ที่แอดเดรส 0 (รูปแบบเดียวกับ EEPROM ภายนอก)
#define LOCAL_EEPROM_SIZE 256

void ensureEEPROMBegin_LOCAL() {
  static bool started = false;
  if (!started) {
    EEPROM.begin(LOCAL_EEPROM_SIZE);
    started = true;
  }
}

void writeEEPROM_LOCAL(unsigned int eeAddress, byte *data, int dataLength) {
  ensureEEPROMBegin_LOCAL();
  for (int i = 0; i < dataLength; i++) {
    EEPROM.write(eeAddress + i, data[i]);
  }
  EEPROM.commit();
}

void readEEPROM_LOCAL(unsigned int eeAddress, byte *buffer, int dataLength) {
  ensureEEPROMBegin_LOCAL();
  for (int i = 0; i < dataLength; i++) {
    buffer[i] = EEPROM.read(eeAddress + i);
  }
}

// ======================================================================
// [5] บันทึก / โหลดค่า Calibrate (EEPROM ภายนอก + สำรอง flash)
// ======================================================================

void printCalibration();

void saveCalibration_LOCAL() {
  CalibrationData d;
  calibToData(d);
  writeEEPROM_LOCAL(0, (byte *)&d, sizeof(d));
  Serial.println("✓ บันทึก Calibrate ลง flash (LOCAL) เสร็จสิ้น");
}

// บันทึกลง EEPROM ภายนอก แล้วอ่านกลับมาตรวจ + สำรองลง flash
// คืน true เมื่อ EEPROM ภายนอกบันทึกและตรวจสอบผ่าน
bool saveCalibration() {
  CalibrationData d, verify;
  calibToData(d);
  bool ok = writeEEPROM(EEPROM_ADDRESS, CAL_EEPROM_ADDR, (byte *)&d, sizeof(d));
  if (ok) {
    ok = false;
    for (int attempt = 0; attempt < 3 && !ok; attempt++) {
      ok = readEEPROM(EEPROM_ADDRESS, CAL_EEPROM_ADDR, (byte *)&verify, sizeof(verify))
           && memcmp(&verify, &d, sizeof(d)) == 0;
      if (!ok) delay(10);
    }
  }
  Serial.println(ok ? "✓ บันทึก Calibrate ลง EEPROM เสร็จสิ้น" : "✗ บันทึก EEPROM ไม่สำเร็จ");
  saveCalibration_LOCAL();
  return ok;
}

// โหลดจาก flash อย่างเดียว
bool loadCalibration_LOCAL() {
  CalibrationData d;
  readEEPROM_LOCAL(0, (byte *)&d, sizeof(d));
  if (!calibDataValid(d)) {
    Serial.println("ไม่พบค่า Calibrate ใน flash (LOCAL)");
    return false;
  }
  dataToCalib(d);
  Serial.println("โหลด Calibrate จาก flash (LOCAL)");
  printCalibration();
  return true;
}

// โหลดจาก EEPROM ภายนอก (ลอง 3 ครั้ง) ถ้าไม่ได้ใช้ค่าสำรองจาก flash
bool loadCalibration() {
  CalibrationData d;
  for (int attempt = 0; attempt < 3; attempt++) {
    if (readEEPROM(EEPROM_ADDRESS, CAL_EEPROM_ADDR, (byte *)&d, sizeof(d)) && calibDataValid(d)) {
      dataToCalib(d);
      Serial.println("โหลด Calibrate จาก EEPROM");
      printCalibration();
      return true;
    }
    delay(10);
  }
  Serial.println("อ่าน EEPROM ไม่ได้ / ข้อมูลไม่ถูกต้อง → ลองค่าสำรองใน flash");
  if (loadCalibration_LOCAL()) return true;
  calibValid = 0;
  Serial.println("✗ ยังไม่มีค่า Calibrate — กรุณา Calibrate ในเมนู sw()");
  return false;
}

// ======================================================================
// [6] คาลิเบรตเซนเซอร์ (calibrateF / B / C)
// ======================================================================

// เวลาเก็บค่า (ms) — กวาดหุ่นผ่านเส้นดำ/พื้นขาวให้ครบทุกตัวภายในเวลานี้
void SetCalibrateTime(unsigned long ms) {
  calibrateTime = constrain(ms, 1000UL, 20000UL);
}

// เสียงติ๊กทุก 300 ms + ไฟวิ่งไป-กลับ ระหว่างเก็บค่า
void calibrateTick(unsigned long &lastTick) {
  unsigned long now = millis();
  if (now - lastTick >= 300) {
    tone(buzzer_pin, 3000, 50);
    lastTick = now;
  }
  LED_ScannerUpdate(60);
}

void calibrateFinish(bool success) {
  noTone(buzzer_pin);
  LED_Off();
  if (success) {
    LED_Blink(2, 100);
    SoundSuccess();
  } else {
    SoundError();
  }
}

// front = true → แถวหน้า, false → แถวหลัง
void calibrateMux(bool front) {
  MotorStop();
  uint16_t lo[NUM_SENSORS], hi[NUM_SENSORS];
  for (int i = 0; i < NUM_SENSORS; i++) {
    lo[i] = 0xFFFF;
    hi[i] = 0;
  }

  Serial.println(front ? "กำลัง calibrate Sensor F (Line Front)..." : "กำลัง calibrate Sensor B (Line Back)...");
  unsigned long start = millis();
  unsigned long lastTick = 0;
  while (millis() - start < calibrateTime) {
    readMuxFrame(front, !front);
    const uint16_t *raw = front ? rawF : rawB;
    for (int i = 0; i < NUM_SENSORS; i++) {
      if (raw[i] < lo[i]) lo[i] = raw[i];
      if (raw[i] > hi[i]) hi[i] = raw[i];
    }
    calibrateTick(lastTick);
  }

  // ทุกช่องต้องมี max > min ไม่อย่างนั้นไม่บันทึก (ค่าเดิมใน EEPROM ยังอยู่)
  for (int i = 0; i < NUM_SENSORS; i++) {
    if (hi[i] <= lo[i]) {
      Serial.printf("✗ %c%d ค่าไม่เปลี่ยน (min=max=%d) — ไม่บันทึก\n", front ? 'F' : 'B', i, lo[i]);
      calibrateFinish(false);
      return;
    }
  }

  for (int i = 0; i < NUM_SENSORS; i++) {
    if (front) {
      minValueF[i] = lo[i];
      maxValueF[i] = hi[i];
    } else {
      minValueB[i] = lo[i];
      maxValueB[i] = hi[i];
    }
  }
  calibValid |= front ? CAL_VALID_F : CAL_VALID_B;
  bool saved = saveCalibration();
  Serial.println(front ? "✓ Calibrate Sensor F เสร็จสิ้น" : "✓ Calibrate Sensor B เสร็จสิ้น");
  calibrateFinish(saved);
}

void calibrateF() {
  calibrateMux(true);
}

void calibrateB() {
  calibrateMux(false);
}

// ชื่อเดิมของ MyRP_Pico2_mini_BNO086 (A = แถวหน้า)
void calibrateA() {
  calibrateF();
}

void calibrateC() {
  MotorStop();
  int16_t lo[2] = { 32767, 32767 };
  int16_t hi[2] = { -32768, -32768 };
  uint8_t gotMask = 0;

  Serial.println("กำลัง calibrate Sensor C (Center CL / CR)...");
  unsigned long start = millis();
  unsigned long lastTick = 0;
  while (millis() - start < calibrateTime) {
    for (int i = 0; i < 2; i++) {
      int16_t raw = 0;
      if (read_sensorC(i, raw)) {
        if (raw < lo[i]) lo[i] = raw;
        if (raw > hi[i]) hi[i] = raw;
        gotMask |= (1 << i);
      }
    }
    calibrateTick(lastTick);
    delay(5);
  }

  if (gotMask != 0x03 || hi[0] <= lo[0] || hi[1] <= lo[1]) {
    Serial.println("✗ Calibrate Sensor C ไม่สำเร็จ (อ่าน ADS1115 ไม่ได้ หรือค่าไม่เปลี่ยน) — ไม่บันทึก");
    calibrateFinish(false);
    return;
  }

  for (int i = 0; i < 2; i++) {
    minValueC[i] = lo[i];
    maxValueC[i] = hi[i];
  }
  calibValid |= CAL_VALID_C;
  bool saved = saveCalibration();
  Serial.println("✓ Calibrate Sensor C เสร็จสิ้น");
  calibrateFinish(saved);
}

// ======================================================================
// [7] แสดงค่าคาลิเบรต
// ======================================================================

void printCalibration() {
  Serial.println("\n=== Calibration Values Loaded ===");
  Serial.printf("--- Sensor F (Front Line) %s ---\n", (calibValid & CAL_VALID_F) ? "" : "[ยังไม่ Calibrate]");
  for (int i = 0; i < NUM_SENSORS; i++) {
    Serial.printf("F%-2d → Max: %4d  Min: %4d\n", i, maxValueF[i], minValueF[i]);
  }

  Serial.printf("--- Sensor B (Back Line) %s ---\n", (calibValid & CAL_VALID_B) ? "" : "[ยังไม่ Calibrate]");
  for (int i = 0; i < NUM_SENSORS; i++) {
    Serial.printf("B%-2d → Max: %4d  Min: %4d\n", i, maxValueB[i], minValueB[i]);
  }

  Serial.printf("--- Sensor C (Center) %s ---\n", (calibValid & CAL_VALID_C) ? "" : "[ยังไม่ Calibrate]");
  Serial.printf("CL  → Max: %5d  Min: %5d\n", maxValueC[CCL], minValueC[CCL]);
  Serial.printf("CR  → Max: %5d  Min: %5d\n", maxValueC[CCR], minValueC[CCR]);
  Serial.println("=================================\n");
}

// ======================================================================
// [8] อ่านเซนเซอร์ + แปลงเป็นค่าคาลิเบรต (ReadCalibrate)
// ======================================================================

void ReadF() {
  readMuxFrame(true, false);
  for (int i = 0; i < NUM_SENSORS; i++) {
    F[i] = rawF[F_PIN[i]];
  }
}

void ReadB() {
  readMuxFrame(false, true);
  for (int i = 0; i < NUM_SENSORS; i++) {
    B[i] = rawB[B_PIN[i]];
  }
}

// อ่าน ADS1115 ไม่ได้ → ใช้ค่าครั้งก่อน
void ReadC() {
  for (int i = 0; i < 2; i++) {
    int16_t raw = 0;
    if (read_sensorC(i, raw)) rawC[i] = raw;
    C[i] = rawC[i];
  }
}

void TrackLineColor(int Col) {
  LineColor = Col;
}

void clampSensorValueF(int x, int y) {
  FRONT_MIN = x;
  FRONT_MAX = y;
}

void clampSensorValueC(int x, int y) {
  CENTER_MIN = x;
  CENTER_MAX = y;
}

void clampSensorValueB(int x, int y) {
  BACK_MIN = x;
  BACK_MAX = y;
}

void ReadCalibrateF() {
  ReadF();
  for (int i = 0; i < NUM_SENSORS; i++) {
    F[i] = constrain(F[i], minValueF[F_PIN[i]], maxValueF[F_PIN[i]]);
    int16_t x;
    if (LineColor == 0)
      x = map(F[i], minValueF[F_PIN[i]], maxValueF[F_PIN[i]], 1000, 0);
    else
      x = map(F[i], minValueF[F_PIN[i]], maxValueF[F_PIN[i]], 0, 1000);
    if (x < FRONT_MIN) x = 0;
    if (x > FRONT_MAX) x = 1000;
    F[i] = x;
  }
}

void ReadCalibrateC() {
  ReadC();

  for (int i = 0; i < 2; i++) {
    C[i] = constrain(C[i], minValueC[i], maxValueC[i]);
    int16_t x;
    if (LineColor == 0)
      x = map(C[i], minValueC[i], maxValueC[i], 1000, 0);
    else
      x = map(C[i], minValueC[i], maxValueC[i], 0, 1000);
    if (x < CENTER_MIN) x = 0;
    if (x > CENTER_MAX) x = 1000;
    C[i] = x;
  }
}

void ReadCalibrateB() {
  ReadB();

  for (int i = 0; i < NUM_SENSORS; i++) {
    B[i] = constrain(B[i], minValueB[B_PIN[i]], maxValueB[B_PIN[i]]);
    int16_t x;
    if (LineColor == 0)
      x = map(B[i], minValueB[B_PIN[i]], maxValueB[B_PIN[i]], 1000, 0);
    else
      x = map(B[i], minValueB[B_PIN[i]], maxValueB[B_PIN[i]], 0, 1000);
    if (x < BACK_MIN) x = 0;
    if (x > BACK_MAX) x = 1000;
    B[i] = x;
  }
}

void ReadSensor() {
  ReadCalibrateF();
  ReadCalibrateB();
  ReadCalibrateC();
}

void ReadSensorRaw() {
  ReadF();
  ReadB();
  ReadC();
}

void RefLineValue(int x) {
  Ref = x;
}

void RefCenterLineValue(int x) {
  RefC = x;
}

// ======================================================================
// [9] แสดงเส้นบนไฟ LED
// ======================================================================

// เซนเซอร์ 16 ตัว → ไฟ 8 ดวง (ดวงละ 2 ตัว) ติดเมื่อเจอเส้น (ค่า >= Ref)
uint8_t lineToLedMask(const int *v) {
  uint8_t mask = 0;
  for (int i = 0; i < PCF_LED_COUNT; i++) {
    if (v[i * 2] >= Ref || v[i * 2 + 1] >= Ref) mask |= (1 << i);
  }
  return mask;
}

void LED_ShowLineF() {
  ReadCalibrateF();
  LED_Write(lineToLedMask(F));
}

void LED_ShowLineB() {
  ReadCalibrateB();
  LED_Write(lineToLedMask(B));
}

// ======================================================================
// [10] Serial debug (ทุกตัวเป็น while(1) ห้ามเรียกใน Mission)
// ======================================================================

void printSensorArray(const char *label, const int *v, int n) {
  Serial.print(label);
  for (int i = 0; i < n; i++) {
    Serial.print(v[i]);
    Serial.print("\t");
  }
}

void Serial_FrontSensor() {
  while (1) {
    ReadSensorRaw();
    printSensorArray("", F, NUM_SENSORS);
    Serial.println("");
    delay(50);
  }
}

void Serial_BackSensor() {
  while (1) {
    ReadSensorRaw();
    printSensorArray("", B, NUM_SENSORS);
    Serial.println("");
    delay(50);
  }
}

void Serial_CenterSensor() {
  while (1) {
    ReadSensorRaw();
    printSensorArray("", C, 2);
    Serial.println("");
    delay(50);
  }
}

void SerialCalibrate_FrontSensor() {
  while (1) {
    ReadSensor();
    printSensorArray("", F, NUM_SENSORS);
    Serial.println("");
    delay(100);
  }
}

void SerialCalibrate_BackSensor() {
  while (1) {
    ReadSensor();
    printSensorArray("", B, NUM_SENSORS);
    Serial.println("");
    delay(100);
  }
}

void SerialCalibrate_CenterSensor() {
  while (1) {
    ReadSensor();
    printSensorArray("", C, 2);
    Serial.println("");
    delay(100);
  }
}

void Serial_AllSensor() {
  while (1) {
    ReadSensorRaw();
    printSensorArray("F: ", F, NUM_SENSORS);
    printSensorArray("B: ", B, NUM_SENSORS);
    printSensorArray("C: ", C, 2);
    Serial.println();
    delay(50);
  }
}

void SerialCalibrate_AllSensor() {
  while (1) {
    ReadSensor();
    printSensorArray("F : ", F, NUM_SENSORS);
    printSensorArray("B : ", B, NUM_SENSORS);
    printSensorArray("C : ", C, 2);
    Serial.println();
    delay(100);
  }
}

// แปลงค่าดิบเป็น 0-1000 สำหรับแสดงในตาราง (ช่องที่ยังไม่ Calibrate แสดง 0)
int tableCalib(int raw, int lo, int hi) {
  if (hi <= lo) return 0;
  raw = constrain(raw, lo, hi);
  return LineColor == 0 ? map(raw, lo, hi, 1000, 0) : map(raw, lo, hi, 0, 1000);
}

// ตาราง raw / min / max / ค่า calibrate ของทุกช่อง (พิมพ์ครั้งเดียว ไม่ค้าง)
void Serial_SensorTable() {
  ReadSensorRaw();
  int calF[NUM_SENSORS], calB[NUM_SENSORS], calC[2];
  for (int i = 0; i < NUM_SENSORS; i++) {
    calF[i] = tableCalib(rawF[i], minValueF[i], maxValueF[i]);
    calB[i] = tableCalib(rawB[i], minValueB[i], maxValueB[i]);
  }
  for (int i = 0; i < 2; i++) {
    calC[i] = tableCalib(rawC[i], minValueC[i], maxValueC[i]);
  }

  Serial.println("Sensor\traw\tmin\tmax\tcal");
  for (int i = 0; i < NUM_SENSORS; i++) {
    Serial.printf("F%d\t%d\t%d\t%d\t%d\n", i, rawF[i], minValueF[i], maxValueF[i], calF[i]);
  }
  for (int i = 0; i < NUM_SENSORS; i++) {
    Serial.printf("B%d\t%d\t%d\t%d\t%d\n", i, rawB[i], minValueB[i], maxValueB[i], calB[i]);
  }
  Serial.printf("CL\t%d\t%d\t%d\t%d\n", rawC[CCL], minValueC[CCL], maxValueC[CCL], calC[CCL]);
  Serial.printf("CR\t%d\t%d\t%d\t%d\n", rawC[CCR], minValueC[CCR], maxValueC[CCR], calC[CCR]);
}

void SetAnalogDistance(int x) {
  DIST = x;
}

// ค่าเซนเซอร์วัดระยะ 0-4095 (สเกลเดียวกับ analogRead 12 บิต ยิ่งใกล้ยิ่งมาก)
// ADS1115 อ่านไม่ได้ → คืน 0
int readDistance() {
  if (DIST != DIST_ADS_AUX) return analogRead(DIST);
  int16_t raw = 0;
  if (!readADS1115(ADS_AUX, raw)) return 0;
  return constrain((int)(adsToVolts(raw) / 3.3f * 4095.0f), 0, 4095);
}

void SerialDistance() {
  while (1) {
    Serial.print("ADC : ");
    Serial.println(readDistance());
    delay(100);
  }
}

#endif  // MYMINI_16PRO_SENSOR_H
