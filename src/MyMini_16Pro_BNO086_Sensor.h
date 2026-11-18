#ifndef MYMINI16PRO_BNO086_SENSOR_H
#define MYMINI16PRO_BNO086_SENSOR_H

/*
 * MyMini_16Pro_BNO086_Sensor.h
 * เซนเซอร์เส้น 16CH หน้า / หลัง (MUX + ADC ของ Pico), เซนเซอร์กลาง CL / CR (ADS1115),
 * Calibrate และบันทึกลง CAT24C128 — เขียนใหม่จาก DualMuxSensors.cpp, CalibrationManager.cpp
 * และ MyMINI_Pro.cpp ของ MyMINI_16Pro
 *
 * - ช่อง 0 = ซ้ายสุด, 15 = ขวาสุด ทั้งแถวหน้าและหลัง (มองจากด้านบน) — ไม่กลับลำดับแถวหลัง
 * - เส้นดำ = ค่า ADC ต่ำ → ค่า normalize 0..1000 แบบต้นฉบับ: min → 0 (ดำ), max → 1000 (ขาว)
 * - รูปแบบข้อมูลใน EEPROM ตรงกับต้นฉบับทุกไบต์ (150 ไบต์ที่ 0x0200, สำรองที่ 0x0000)
 *   หุ่นที่ Calibrate ล่าสุดด้วย MyMINI_16Pro → ใช้ค่าเดิมได้ทันที
 *   หุ่นที่ Calibrate ล่าสุดด้วย MyMini_16Pro_BNO086 (เขียนที่ 0x0000) → ถ้า 0x0200 ยังมีชุดเก่าจะโหลดชุดเก่า
 *   ให้ Calibrate ใหม่ 1 ครั้ง
 *
 * สารบัญ:
 *   [1] ขา / ค่าคงที่
 *   [2] ตัวแปร (Global Variables)
 *   [3] เริ่มต้น / ตั้งค่าการอ่าน MUX
 *   [4] อ่านค่าดิบ (MUX หน้า/หลัง + ADS1115 กลาง)
 *   [5] Normalize 0..1000 (ReadSensor / readADC_*)
 *   [6] บันทึก / โหลดค่า Calibrate (CAT24C128)
 *   [7] Calibrate (หน้า / หลัง / กลาง)
 *   [8] Serial Debug
 *   [9] API ตามต้นแบบ MyRP_Pico2 (muxReadF, LightValue_*, caribrateSensor*, calibrateA..C, ..._LOCAL, Serial_*)
 */

#include <Arduino.h>
#include <stddef.h>
#include <string.h>
#include "MyMini_16Pro_BNO086_Buzzer.h"
#include "MyMini_16Pro_BNO086_I2C.h"

// ======================================================================
// [1] ขา / ค่าคงที่
// ======================================================================

#define NUM_SENSORS 16

// MUX หน้า (J13): S0..S3 = 17, 16, 15, 14 / สัญญาณ GP27 (ADC1)
#define MUXF_S0 17
#define MUXF_S1 16
#define MUXF_S2 15
#define MUXF_S3 14
#define MUXF_SIG 27
// MUX หลัง (J5): S0..S3 = 10, 11, 12, 13 / สัญญาณ GP26 (ADC0)
#define MUXB_S0 10
#define MUXB_S1 11
#define MUXB_S2 12
#define MUXB_S3 13
#define MUXB_SIG 26

#define ADC_BITS 12
#define MUX_SETTLE_US 40       // รอหลังสลับช่อง (4–100 µs)
#define MUX_DISCARD_READS 1    // ทิ้งค่า ADC แรก (1–4)
#define MUX_AVERAGE_SAMPLES 2  // อ่านมาเฉลี่ย (1–8)
#define MUX_SMOOTHING_DIV 4    // กรองข้ามเฟรม 1 = ปิด, 2 / 4 / 8

#define CCL 0  // ดัชนี C[] เซนเซอร์กลางซ้าย (ADS AIN1)
#define CCR 1  // ดัชนี C[] เซนเซอร์กลางขวา (ADS AIN2)

#define ADC_INVALID -1  // ค่าที่คืนเมื่ออ่านไม่ได้ / ยังไม่ Calibrate (ไม่คืน 0 ปลอม)

// Calibrate / EEPROM (CalibrationManager.cpp)
#define CAL_DURATION_MS 5000  // ค่าเริ่มต้นต้นฉบับ — เปลี่ยนได้ด้วย SetCalibrateTime()
#define CAL_CENTER_INTERVAL_MS 100
#define CAL_TONE_MS 50
#define CAL_TONE_INTERVAL_MS 300
#define CAL_EEPROM_ADDR 0x0200
#define CAL_EEPROM_LEGACY_ADDR 0x0000
#define CAL_MAGIC 0x4D4D5031UL
#define CAL_VERSION 1
#define CAL_READ_ATTEMPTS 3
#define CAL_VALID_F 0x01
#define CAL_VALID_B 0x02
#define CAL_VALID_C 0x04
#define CAL_VALID_ALL (CAL_VALID_F | CAL_VALID_B | CAL_VALID_C)

// ======================================================================
// [2] ตัวแปร (Global Variables)
// ======================================================================

const uint8_t MUXF_SEL[4] = { MUXF_S0, MUXF_S1, MUXF_S2, MUXF_S3 };
const uint8_t MUXB_SEL[4] = { MUXB_S0, MUXB_S1, MUXB_S2, MUXB_S3 };

// F[] / B[] / C[] สำหรับเดินตามเส้น (แบบ MyMini_16Pro_BNO086):
//   หลัง ReadSensorRaw() = ค่าดิบ (กรองแล้ว), หลัง ReadSensor() / ReadCalibrate*() = 0..1000 โดย "เจอเส้น = 1000"
//   F[i] = ช่อง F_PIN[i], B[i] = ช่อง B_PIN[i] (แถวหลังกลับลำดับ: B[0] = ช่อง 15)
// ค่าแบบต้นฉบับ (ช่องจริง, ดำ ≈ 0) ใช้ readADC_F / readADC_B / readADC_CL / readADC_CR
int F[NUM_SENSORS], B[NUM_SENSORS], C[2];
uint8_t F_PIN[NUM_SENSORS] = { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15 };
uint8_t B_PIN[NUM_SENSORS] = { 15, 14, 13, 12, 11, 10, 9, 8, 7, 6, 5, 4, 3, 2, 1, 0 };

#define BLACK 0
#define WHITE 1
int LineColor = BLACK;  // BLACK = วิ่งเส้นดำบนพื้นขาว
int Ref = 500;          // F[] / B[] >= Ref = เจอเส้น
int RefC = 500;         // C[] >= RefC = เจอเส้น
int FRONT_MIN = 80;     // ค่าหลังแปลง < MIN → 0, > MAX → 1000
int FRONT_MAX = 900;
int CENTER_MIN = 200;
int CENTER_MAX = 900;
int BACK_MIN = 0;
int BACK_MAX = 1000;

// เซนเซอร์วัดระยะ: ค่าเริ่มต้น = ADS1115 AIN3 (GP26/27 เป็น MUX, GP28 เป็นเซอร์โว)
#define DIST_ADS_AUX 255
int DIST = DIST_ADS_AUX;
uint16_t rawF[NUM_SENSORS], rawB[NUM_SENSORS];    // ค่าดิบเฉลี่ย (ใช้ Calibrate)
uint16_t filtF[NUM_SENSORS], filtB[NUM_SENSORS];  // ค่ากรองแล้ว (ใช้ normalize)
int16_t rawC[2];
bool rawCValid[2] = { false, false };

int32_t smoothF_Q8[NUM_SENSORS], smoothB_Q8[NUM_SENSORS];
bool smoothReady = false;
uint32_t muxFrameCount = 0;

uint16_t muxSettleUs = MUX_SETTLE_US;
uint8_t muxDiscard = MUX_DISCARD_READS;
uint8_t muxAverage = MUX_AVERAGE_SAMPLES;
uint8_t muxSmooth = MUX_SMOOTHING_DIV;

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
static_assert(sizeof(CalibrationData) == 150, "CalibrationData ต้องเป็น 150 ไบต์ ตรงกับ MyMINI_16Pro");

CalibrationData calib = {};
#define CAL_LOAD_NOT_TRIED 0
#define CAL_LOAD_OK 1
#define CAL_LOAD_READ_FAIL 2
#define CAL_LOAD_INVALID 3
uint8_t calibLoadStatus = CAL_LOAD_NOT_TRIED;
void calibToArrays();  // คัดลอก calib → minValueF / maxValueF ... (ตัวจริงอยู่ใน [9])
unsigned long calDurationMs = CAL_DURATION_MS;

// เวลาเก็บค่าตอน Calibrate (ms)
void SetCalibrateTime(unsigned long ms) {
  calDurationMs = ms;
}

// ======================================================================
// [3] เริ่มต้น / ตั้งค่าการอ่าน MUX
// ======================================================================

void muxSelect(uint8_t ch) {
  for (uint8_t bit = 0; bit < 4; bit++) {
    uint8_t level = (ch >> bit) & 0x01U;
    digitalWrite(MUXF_SEL[bit], level);
    digitalWrite(MUXB_SEL[bit], level);
  }
}

void SensorBegin() {
  smoothReady = false;
  for (uint8_t i = 0; i < 4; i++) {
    pinMode(MUXF_SEL[i], OUTPUT);
    pinMode(MUXB_SEL[i], OUTPUT);
  }
  pinMode(MUXF_SIG, INPUT);
  pinMode(MUXB_SIG, INPUT);
  analogReadResolution(ADC_BITS);
  muxSelect(0);
}

// configure_sensor_reading() ต้นฉบับ — คืน false ถ้าค่าอยู่นอกช่วงที่รองรับ (ไม่เปลี่ยนค่าเดิม)
bool SetSensorRead(uint16_t settleUs, uint8_t discard, uint8_t average, uint8_t smooth) {
  if (settleUs < 4 || settleUs > 100 || discard < 1 || discard > 4 || average < 1 || average > 8 ||
      (smooth != 1 && smooth != 2 && smooth != 4 && smooth != 8)) {
    return false;
  }
  muxSettleUs = settleUs;
  muxDiscard = discard;
  muxAverage = average;
  muxSmooth = smooth;
  // เริ่มตัวกรองใหม่จากค่าดิบล่าสุด
  for (uint8_t ch = 0; ch < NUM_SENSORS; ch++) {
    smoothF_Q8[ch] = (int32_t)rawF[ch] << 8;
    smoothB_Q8[ch] = (int32_t)rawB[ch] << 8;
    filtF[ch] = rawF[ch];
    filtB[ch] = rawB[ch];
  }
  smoothReady = muxFrameCount != 0;
  return true;
}

// แบบ MyMini_16Pro_BNO086: (settle µs, ทิ้งค่า, เฉลี่ย) — คงค่ากรองข้ามเฟรมเดิม
bool SetSensorRead(uint16_t settleUs, uint8_t discard, uint8_t average) {
  return SetSensorRead(settleUs, discard, average, muxSmooth);
}

// กรองข้ามเฟรม 1 = ปิด | 2 / 4 / 8 (ต้นฉบับใช้ 4)
bool SetSensorSmooth(uint8_t divisor) {
  return SetSensorRead(muxSettleUs, muxDiscard, muxAverage, divisor);
}

// ======================================================================
// [4] อ่านค่าดิบ (MUX หน้า/หลัง + ADS1115 กลาง)
// ======================================================================

uint16_t muxReadAveraged(uint8_t pin) {
  for (uint8_t i = 0; i < muxDiscard; i++) (void)analogRead(pin);
  uint32_t sum = 0;
  for (uint8_t i = 0; i < muxAverage; i++) sum += (uint16_t)analogRead(pin);
  return (uint16_t)(sum / muxAverage);
}

// อ่าน 1 เฟรม: เลือกช่องพร้อมกันทั้งสอง MUX → รอ settle → อ่านหน้า + หลัง → กรอง Q8
void ReadMuxFrame() {
  for (uint8_t ch = 0; ch < NUM_SENSORS; ch++) {
    muxSelect(ch);
    delayMicroseconds(muxSettleUs);
    rawF[ch] = muxReadAveraged(MUXF_SIG);
    rawB[ch] = muxReadAveraged(MUXB_SIG);

    int32_t targetF = (int32_t)rawF[ch] << 8;
    int32_t targetB = (int32_t)rawB[ch] << 8;
    if (!smoothReady) {
      smoothF_Q8[ch] = targetF;
      smoothB_Q8[ch] = targetB;
    } else {
      smoothF_Q8[ch] += (targetF - smoothF_Q8[ch]) / muxSmooth;
      smoothB_Q8[ch] += (targetB - smoothB_Q8[ch]) / muxSmooth;
    }
    filtF[ch] = (uint16_t)((smoothF_Q8[ch] + 128) >> 8);
    filtB[ch] = (uint16_t)((smoothB_Q8[ch] + 128) >> 8);
  }
  smoothReady = true;
  muxFrameCount++;
}

// i = CCL / CCR — คืน false ถ้า ADS1115 ไม่ตอบ
bool readCenterRaw(uint8_t i, int16_t &raw) {
  if (i > 1) return false;
  return readADS1115(i == CCL ? ADS_CENTER_L : ADS_CENTER_R, raw);
}

void ReadC() {
  for (uint8_t i = 0; i < 2; i++) {
    int16_t raw = 0;
    rawCValid[i] = readCenterRaw(i, raw);
    if (rawCValid[i]) rawC[i] = raw;
    C[i] = rawC[i];
  }
}

// คัดลอกจากเฟรมล่าสุด (ไม่อ่านใหม่) ตามลำดับ F_PIN / B_PIN
void loadF() {
  for (int i = 0; i < NUM_SENSORS; i++) F[i] = filtF[F_PIN[i]];
}

void loadB() {
  for (int i = 0; i < NUM_SENSORS; i++) B[i] = filtB[B_PIN[i]];
}

void ReadF() {
  ReadMuxFrame();
  loadF();
}

void ReadB() {
  ReadMuxFrame();
  loadB();
}

// F / B = ค่ากรองแล้ว (ลำดับ F_PIN / B_PIN), C = ค่าดิบ ADS1115
void ReadSensorRaw() {
  ReadMuxFrame();
  loadF();
  loadB();
  ReadC();
}

// ======================================================================
// [5] Normalize 0..1000 (ReadSensor / readADC_*)
// ======================================================================

bool normalizeValue(int32_t raw, int32_t minValue, int32_t maxValue, uint16_t &out) {
  if (maxValue <= minValue) return false;
  int32_t mapped = map(raw, minValue, maxValue, 0L, 1000L);
  out = (uint16_t)constrain(mapped, 0L, 1000L);
  return true;
}

bool CalibratedF() {
  return (calib.validMask & CAL_VALID_F) != 0;
}
bool CalibratedB() {
  return (calib.validMask & CAL_VALID_B) != 0;
}
bool CalibratedC() {
  return (calib.validMask & CAL_VALID_C) != 0;
}

// ---------- ตั้งค่าการแปลงสำหรับเดินตามเส้น ----------
void TrackLineColor(int Col) {
  LineColor = Col;
}

void SwitchLine() {
  LineColor = !LineColor;
}

void RefLineValue(int x) {
  Ref = x;
}

void RefCenterLineValue(int x) {
  RefC = x;
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

// ค่าดิบ → 0..1000 แบบ "เจอเส้น = 1000" (BLACK: ค่าดิบต่ำ → 1000)
// ยังไม่ Calibrate (max <= min) → 0 (ถือว่าพื้น) กันหารศูนย์
int calibMap(int v, int lo, int hi) {
  if (hi <= lo) return 0;
  v = constrain(v, lo, hi);
  return LineColor == BLACK ? map(v, lo, hi, 1000, 0) : map(v, lo, hi, 0, 1000);
}

int clampLine(int x, int lo, int hi) {
  if (x < lo) return 0;
  if (x > hi) return 1000;
  return x;
}

// แปลงค่าที่อ่านไว้แล้วใน F[] / B[] / C[] (ไม่อ่านใหม่)
void calibrateLoadedF() {
  for (int i = 0; i < NUM_SENSORS; i++) {
    uint8_t ch = F_PIN[i];
    F[i] = clampLine(calibMap(F[i], calib.frontMin[ch], calib.frontMax[ch]), FRONT_MIN, FRONT_MAX);
  }
}

void calibrateLoadedB() {
  for (int i = 0; i < NUM_SENSORS; i++) {
    uint8_t ch = B_PIN[i];
    B[i] = clampLine(calibMap(B[i], calib.rearMin[ch], calib.rearMax[ch]), BACK_MIN, BACK_MAX);
  }
}

void calibrateLoadedC() {
  for (int i = 0; i < 2; i++) {  // ดัชนี i = CCL(0) / CCR(1) ตรงกับ calib.centerMin[0] = CL
    C[i] = clampLine(calibMap(C[i], calib.centerMin[i], calib.centerMax[i]), CENTER_MIN, CENTER_MAX);
  }
}

void ReadCalibrateF() {
  ReadF();
  calibrateLoadedF();
}

void ReadCalibrateB() {
  ReadB();
  calibrateLoadedB();
}

void ReadCalibrateC() {
  ReadC();
  calibrateLoadedC();
}

// หน้า + หลังจากเฟรมเดียวกัน + กลาง
void ReadSensor() {
  ReadMuxFrame();
  loadF();
  calibrateLoadedF();
  loadB();
  calibrateLoadedB();
  ReadCalibrateC();
}

// ---------- เซนเซอร์วัดระยะ ----------
void SetAnalogDistance(int x) {
  DIST = x;
}

// 0-4095 (ยิ่งใกล้ยิ่งมาก) — ADS1115 อ่านไม่ได้ → 0
int readDistance() {
  if (DIST != DIST_ADS_AUX) return analogRead(DIST);
  int16_t raw = 0;
  if (!readADS1115(ADS_AUX, raw)) return 0;
  return constrain((int)(adsToVolts(raw) / 3.3f * 4095.0f), 0, 4095);
}

// อ่านทีละช่อง (readADC_F / readADC_B / readADC_CL / readADC_CR ต้นฉบับ)
int readADC_F(uint8_t ch) {
  if (ch >= NUM_SENSORS || !CalibratedF()) return ADC_INVALID;
  ReadMuxFrame();
  uint16_t n = 0;
  return normalizeValue(filtF[ch], calib.frontMin[ch], calib.frontMax[ch], n) ? n : ADC_INVALID;
}

int readADC_B(uint8_t ch) {
  if (ch >= NUM_SENSORS || !CalibratedB()) return ADC_INVALID;
  ReadMuxFrame();
  uint16_t n = 0;
  return normalizeValue(filtB[ch], calib.rearMin[ch], calib.rearMax[ch], n) ? n : ADC_INVALID;
}

int readADC_C(uint8_t i) {
  if (i > 1 || !CalibratedC()) return ADC_INVALID;
  int16_t raw = 0;
  if (!readCenterRaw(i, raw)) return ADC_INVALID;
  uint16_t n = 0;
  return normalizeValue(raw, calib.centerMin[i], calib.centerMax[i], n) ? n : ADC_INVALID;
}

int readADC_CL() {
  return readADC_C(CCL);
}

int readADC_CR() {
  return readADC_C(CCR);
}

// ADS1115 AIN3 (AUX) ค่าดิบมีเครื่องหมาย — readADC_F() แบบไม่มีอาร์กิวเมนต์ของต้นฉบับ
int32_t readADC_AUX() {
  int16_t raw = 0;
  return readADS1115(ADS_AUX, raw) ? (int32_t)raw : (int32_t)INT32_MIN;
}

// ======================================================================
// [6] บันทึก / โหลดค่า Calibrate (CAT24C128)
// ======================================================================

// CRC16-CCITT (init 0xFFFF, poly 0x1021)
uint16_t calibCRC16(const uint8_t *bytes, uint16_t length) {
  uint16_t crc = 0xFFFF;
  for (uint16_t i = 0; i < length; i++) {
    crc ^= (uint16_t)bytes[i] << 8;
    for (uint8_t bit = 0; bit < 8; bit++) {
      crc = (crc & 0x8000U) ? (uint16_t)((crc << 1) ^ 0x1021U) : (uint16_t)(crc << 1);
    }
  }
  return crc;
}

bool saveCalibration() {
  calib.magic = CAL_MAGIC;
  calib.version = CAL_VERSION;
  calib.size = sizeof(CalibrationData);
  memset(calib.reserved, 0, sizeof(calib.reserved));
  calib.crc16 = calibCRC16((const uint8_t *)&calib, offsetof(CalibrationData, crc16));
  if (!writeEEPROM(CAL_EEPROM_ADDR, (const uint8_t *)&calib, sizeof(calib))) return false;

  CalibrationData verify = {};
  for (uint8_t attempt = 0; attempt < CAL_READ_ATTEMPTS; attempt++) {
    if (readEEPROM(CAL_EEPROM_ADDR, (uint8_t *)&verify, sizeof(verify)) &&
        memcmp(&verify, &calib, sizeof(calib)) == 0) {
      return true;
    }
    if (attempt + 1 < CAL_READ_ATTEMPTS) delay(10);
  }
  return false;
}

// ลอง 0x0200 ก่อน แล้วค่อยที่อยู่เก่า 0x0000 (เจอที่อยู่เก่า → ย้ายไป 0x0200)
bool loadCalibration() {
  CalibrationData loaded = {};
  bool anyRead = false;
  const uint16_t slots[2] = { CAL_EEPROM_ADDR, CAL_EEPROM_LEGACY_ADDR };

  for (uint8_t slot = 0; slot < 2; slot++) {
    bool slotRead = false;
    for (uint8_t attempt = 0; attempt < CAL_READ_ATTEMPTS; attempt++) {
      if (readEEPROM(slots[slot], (uint8_t *)&loaded, sizeof(loaded))) {
        slotRead = true;
        anyRead = true;
        if (loaded.magic == CAL_MAGIC && loaded.version == CAL_VERSION && loaded.size == sizeof(CalibrationData) &&
            (loaded.validMask & (uint8_t)~CAL_VALID_ALL) == 0 && loaded.reserved[0] == 0 &&
            loaded.reserved[1] == 0 && loaded.reserved[2] == 0 &&
            loaded.crc16 == calibCRC16((const uint8_t *)&loaded, offsetof(CalibrationData, crc16))) {
          calib = loaded;
          calibLoadStatus = CAL_LOAD_OK;
          calibToArrays();
          if (slot == 1) {
            Serial.println(saveCalibration() ? "Legacy calibration migrated to 0x0200"
                                             : "Legacy calibration migration failed; values kept in RAM");
          }
          return true;
        }
      }
      if (attempt + 1 < CAL_READ_ATTEMPTS) delay(10);
    }
    // อ่าน 0x0200 ไม่ได้เลย → ห้ามเอาค่าเก่าที่ 0x0000 มาทับ
    if (slot == 0 && !slotRead) break;
  }

  memset(&calib, 0, sizeof(calib));
  calibLoadStatus = anyRead ? CAL_LOAD_INVALID : CAL_LOAD_READ_FAIL;
  calibToArrays();
  return false;
}

void printCalibrationStatus() {
  Serial.print("Calibration EEPROM boot=");
  switch (calibLoadStatus) {
    case CAL_LOAD_OK: Serial.print("OK"); break;
    case CAL_LOAD_READ_FAIL: Serial.print("READ_FAIL"); break;
    case CAL_LOAD_INVALID: Serial.print("INVALID_DATA"); break;
    default: Serial.print("NOT_TRIED"); break;
  }
  Serial.printf(" F=%d B=%d C=%d\n", CalibratedF(), CalibratedB(), CalibratedC());
}

// ======================================================================
// [7] Calibrate (หน้า / หลัง / กลาง)
// ======================================================================

#define CAL_SRC_F 0
#define CAL_SRC_B 1
#define CAL_SRC_C 2

// เก็บ min / max ค่าดิบ 5 วินาที (ปี๊บ 3000 Hz 50 ms ทุก 300 ms) แล้วบันทึก
// สำเร็จ → ปี๊บ 3500 Hz 250 ms × 2 | ล้มเหลว → ค่าเดิมไม่ถูกแตะ
bool calibrateGroup(uint8_t source) {
  // อ่าน EEPROM ตอนบูตไม่ได้ → ลองโหลดใหม่ก่อน กันไม่ให้ค่ากลุ่มอื่นถูกทับด้วยค่าว่าง
  if (calibLoadStatus == CAL_LOAD_READ_FAIL) {
    loadCalibration();
    if (calibLoadStatus == CAL_LOAD_READ_FAIL) {
      Serial.println("Calibration EEPROM_SAVE_FAILED_OR_RANGE_INVALID");
      return false;
    }
  }

  uint16_t capMin[NUM_SENSORS], capMax[NUM_SENSORS];
  for (int i = 0; i < NUM_SENSORS; i++) {
    capMin[i] = 0xFFFF;
    capMax[i] = 0;
  }
  int16_t cMin[2] = { 32767, 32767 }, cMax[2] = { -32768, -32768 };
  uint32_t frames = 0;
  uint8_t centerMask = 0;

  noTone(BUZZER_PIN);
  unsigned long start = millis();
  unsigned long nextTone = start, toneOff = start, nextCenter = start;
  bool toneOn = false;

  while (millis() - start < calDurationMs) {
    unsigned long now = millis();
    if (toneOn && (long)(now - toneOff) >= 0) {
      noTone(BUZZER_PIN);
      toneOn = false;
    }
    if (!toneOn && (long)(now - nextTone) >= 0) {
      tone(BUZZER_PIN, TONE_CAL_TICK);
      toneOn = true;
      toneOff = now + CAL_TONE_MS;
      nextTone = now + CAL_TONE_INTERVAL_MS;
    }

    if (source == CAL_SRC_C) {
      if ((long)(now - nextCenter) >= 0) {
        nextCenter = now + CAL_CENTER_INTERVAL_MS;
        for (uint8_t i = 0; i < 2; i++) {
          int16_t raw = 0;
          if (readCenterRaw(i, raw)) {
            if (raw < cMin[i]) cMin[i] = raw;
            if (raw > cMax[i]) cMax[i] = raw;
            centerMask |= (1 << i);
          }
        }
      }
    } else {
      ReadMuxFrame();
      const uint16_t *raw = (source == CAL_SRC_F) ? rawF : rawB;
      for (int i = 0; i < NUM_SENSORS; i++) {
        if (raw[i] < capMin[i]) capMin[i] = raw[i];
        if (raw[i] > capMax[i]) capMax[i] = raw[i];
      }
      frames++;
    }
  }
  noTone(BUZZER_PIN);

  CalibrationData previous = calib;
  bool ok = true;
  if (source == CAL_SRC_C) {
    ok = centerMask == 0x03 && cMax[0] > cMin[0] && cMax[1] > cMin[1];
    if (ok) {
      for (int i = 0; i < 2; i++) {
        calib.centerMin[i] = cMin[i];
        calib.centerMax[i] = cMax[i];
      }
      calib.validMask |= CAL_VALID_C;
    }
  } else {
    ok = frames > 0;
    for (int i = 0; ok && i < NUM_SENSORS; i++) ok = capMax[i] > capMin[i];
    if (ok) {
      for (int i = 0; i < NUM_SENSORS; i++) {
        if (source == CAL_SRC_F) {
          calib.frontMin[i] = capMin[i];
          calib.frontMax[i] = capMax[i];
        } else {
          calib.rearMin[i] = capMin[i];
          calib.rearMax[i] = capMax[i];
        }
      }
      calib.validMask |= (source == CAL_SRC_F) ? CAL_VALID_F : CAL_VALID_B;
    }
  }
  if (ok && !saveCalibration()) {
    calib = previous;
    ok = false;
  }
  calibToArrays();

  Serial.print("Calibration ");
  Serial.print(source == CAL_SRC_B ? "B" : source == CAL_SRC_F ? "F" : "C");
  Serial.println(ok ? " EEPROM_SAVE_OK" : " EEPROM_SAVE_FAILED_OR_RANGE_INVALID");
  if (ok) SoundSuccess();
  return ok;
}

bool CalibrateSensorF() {
  return calibrateGroup(CAL_SRC_F);
}

bool CalibrateSensorB() {
  return calibrateGroup(CAL_SRC_B);
}

bool CalibrateSensorC() {
  return calibrateGroup(CAL_SRC_C);
}

// ======================================================================
// [8] Serial Debug
// ======================================================================

void printSensorArray(const char *label, const int *v, int n) {
  Serial.print(label);
  for (int i = 0; i < n; i++) {
    if (i > 0) Serial.print(',');
    if (v[i] == ADC_INVALID) Serial.print("NA");
    else Serial.print(v[i]);
  }
}

// หนึ่งบรรทัดค่าดิบ F: B: adcL: adcR: (read_rowSensor ต้นฉบับ) — เรียกซ้ำใน loop()
// ลำดับช่องจริง (CH0 = ซ้ายสุด ทั้งสองแถว) แบบต้นฉบับ
void read_rowSensor() {
  ReadMuxFrame();
  ReadC();
  int rf[NUM_SENSORS], rb[NUM_SENSORS];
  for (int i = 0; i < NUM_SENSORS; i++) {
    rf[i] = rawF[i];
    rb[i] = rawB[i];
  }
  printSensorArray("F:", rf, NUM_SENSORS);
  printSensorArray(" B:", rb, NUM_SENSORS);
  Serial.print(" adcL:");
  if (rawCValid[CCL]) Serial.print(rawC[CCL]);
  else Serial.print("ERR");
  Serial.print(" adcR:");
  if (rawCValid[CCR]) Serial.print(rawC[CCR]);
  else Serial.print("ERR");
  Serial.println();
}

// หนึ่งบรรทัดค่า 0..1000 (ช่องที่ยังไม่ Calibrate แสดง NA)
void read_rowSensorCalibrate() {
  ReadSensor();
  printSensorArray("F:", F, NUM_SENSORS);
  printSensorArray(" B:", B, NUM_SENSORS);
  printSensorArray(" C:", C, 2);
  Serial.println();
}

// ตาราง Sensor / now_raw / min_raw / max_raw / normalized / valid (print_line_sensor_table)
void Serial_SensorTable() {
  ReadMuxFrame();
  ReadC();
  Serial.println("Sensor\tnow_raw\tmin_raw\tmax_raw\tnormalized\tvalid");
  for (int g = 0; g < 3; g++) {
    int n = (g == 2) ? 2 : NUM_SENSORS;
    for (int i = 0; i < n; i++) {
      bool valid = g == 0 ? CalibratedF() : g == 1 ? CalibratedB() : CalibratedC();
      bool rawOk = g < 2 || rawCValid[i];
      int32_t raw = g == 0 ? rawF[i] : g == 1 ? rawB[i] : rawC[i];
      int32_t lo = g == 0 ? calib.frontMin[i] : g == 1 ? calib.rearMin[i] : calib.centerMin[i];
      int32_t hi = g == 0 ? calib.frontMax[i] : g == 1 ? calib.rearMax[i] : calib.centerMax[i];
      valid = valid && hi > lo;
      uint16_t n01 = 0;
      bool nOk = rawOk && valid && normalizeValue(raw, lo, hi, n01);

      if (g == 2) Serial.print(i == CCL ? "CL" : "CR");
      else {
        Serial.print(g == 0 ? 'F' : 'B');
        Serial.print(i);
      }
      Serial.print('\t');
      if (rawOk) Serial.print(raw);
      else Serial.print("ERR");
      Serial.print('\t');
      if (valid) Serial.print(lo);
      else Serial.print("N/A");
      Serial.print('\t');
      if (valid) Serial.print(hi);
      else Serial.print("N/A");
      Serial.print('\t');
      if (nOk) Serial.print(n01);
      else Serial.print("N/A");
      Serial.print('\t');
      Serial.println(valid ? "YES" : "NO");
    }
  }
}

// ======================================================================
// [9] API ตามไลบรารีต้นแบบ (MyRP_Pico2) บนฮาร์ดแวร์ MyMINI_16Pro
// ======================================================================
//
// ตารางค่า Calibrate (minValueF / maxValueF / ...) เก็บตาม "ช่องจริง" ของ MUX (ch 0..15)
// เหมือนกับ calib.frontMin[ch] ใน CAT24C128 และแบบ MyRP_Pico2 (minValueF[F_PIN[i]])
// — คำสั่ง (LightValue_* / CalibrateSensor) แปลงลำดับ F[i] / B[i] ↔ ช่องจริงด้วย F_PIN / B_PIN ให้เอง
//   จึงพิมพ์ค่าแล้ววางกลับใน LightValue_*() ได้ตรงกันทุกช่อง
//
// เซนเซอร์กลาง: MyRP อ่าน analogRead() 12 บิต — MyMINI อ่าน ADS1115 AIN1 / AIN2 (16 บิต, ±4.096 V)

#include <EEPROM.h>

// ---------- ตารางค่า Calibrate (MyRP) ----------
int minValueF[NUM_SENSORS], maxValueF[NUM_SENSORS];
int minValueB[NUM_SENSORS], maxValueB[NUM_SENSORS];
int minValueC[2], maxValueC[2];
// ค่าที่เก็บจาก calibrateA / B / C หรือ read_eepA / B / C (แบบ MyRP: ชุด "EEPROM ภายนอก")
int sensorMaxA[NUM_SENSORS], sensorMinA[NUM_SENSORS];
int sensorMaxB[NUM_SENSORS], sensorMinB[NUM_SENSORS];
int sensorMaxC[2], sensorMinC[2];

// MyRP: C_PIN = ขา ADC ของเซนเซอร์กลาง → MyMINI: ช่อง ADS1115 (C[CCL] = AIN1, C[CCR] = AIN2)
const uint8_t C_PIN[2] = { ADS_CENTER_L, ADS_CENTER_R };

// จำนวนช่องที่ใช้เดินเส้น (6..16) — ใช้ใน calcPositionB_none() แบบต้นฉบับ
uint8_t TrackLineCH = 0;

#define EEPROM_ADDRESS EEPROM_ADDR  // MyRP: ที่อยู่ I2C ของ EEPROM ภายนอก (MyMINI = CAT24C128 0x50)
#define EEPROM_CALIB_ADDR CAL_EEPROM_ADDR
#define EEPROM_CALIB_MAGIC CAL_MAGIC
#define LOCAL_EEPROM_SIZE 256  // flash จำลอง EEPROM ของ RP2350 (16 ช่อง × 2 แถว ต้องใช้ 136 ไบต์)

// ขอบลด/เพิ่มของ caribrateSensorC (+60 / -80 หน่วย ADC 12 บิต @3.3 V)
// แปลงเป็นแรงดันเท่ากันบน ADS1115 (125 µV/LSB): 60 × 806 µV ≈ 387, 80 × 806 µV ≈ 516
#define CARIB_C_MARGIN_MIN 387
#define CARIB_C_MARGIN_MAX 516
#define CARIB_MARGIN_MIN 60  // ช่อง MUX ใช้ ADC 12 บิตของ Pico
#define CARIB_MARGIN_MAX 80

// calib (ช่องจริง) → ตาราง
void calibToArrays() {
  for (int ch = 0; ch < NUM_SENSORS; ch++) {
    minValueF[ch] = calib.frontMin[ch];
    maxValueF[ch] = calib.frontMax[ch];
    minValueB[ch] = calib.rearMin[ch];
    maxValueB[ch] = calib.rearMax[ch];
    sensorMinA[ch] = calib.frontMin[ch];
    sensorMaxA[ch] = calib.frontMax[ch];
    sensorMinB[ch] = calib.rearMin[ch];
    sensorMaxB[ch] = calib.rearMax[ch];
  }
  for (int i = 0; i < 2; i++) {
    minValueC[i] = calib.centerMin[i];
    maxValueC[i] = calib.centerMax[i];
    sensorMinC[i] = calib.centerMin[i];
    sensorMaxC[i] = calib.centerMax[i];
  }
}

// ตาราง → calib ของกลุ่มที่เลือก (CAL_VALID_F / B / C) + ตั้งบิต valid
// คืนจำนวนช่องที่ max <= min (ช่องนั้นจะอ่านเป็น 0 = พื้น)
int arraysToCalib(uint8_t group) {
  int bad = 0;
  if (group & CAL_VALID_F) {
    for (int ch = 0; ch < NUM_SENSORS; ch++) {
      calib.frontMin[ch] = (uint16_t)constrain(minValueF[ch], 0, 65535);
      calib.frontMax[ch] = (uint16_t)constrain(maxValueF[ch], 0, 65535);
      if (maxValueF[ch] <= minValueF[ch]) bad++;
    }
    calib.validMask |= CAL_VALID_F;
  }
  if (group & CAL_VALID_B) {
    for (int ch = 0; ch < NUM_SENSORS; ch++) {
      calib.rearMin[ch] = (uint16_t)constrain(minValueB[ch], 0, 65535);
      calib.rearMax[ch] = (uint16_t)constrain(maxValueB[ch], 0, 65535);
      if (maxValueB[ch] <= minValueB[ch]) bad++;
    }
    calib.validMask |= CAL_VALID_B;
  }
  if (group & CAL_VALID_C) {
    for (int i = 0; i < 2; i++) {
      calib.centerMin[i] = (int16_t)constrain(minValueC[i], -32768, 32767);
      calib.centerMax[i] = (int16_t)constrain(maxValueC[i], -32768, 32767);
      if (maxValueC[i] <= minValueC[i]) bad++;
    }
    calib.validMask |= CAL_VALID_C;
  }
  if (bad > 0) Serial.printf("Calibration: %d channel(s) Max <= Min -> read as 0\n", bad);
  return bad;
}

// ---------- MUX แยกแถว (muxSelectF / muxReadF / muxSelectB / muxReadB) ----------
// ch = ช่องจริง 0..15 | คืนค่าดิบ 12 บิต (ทิ้งค่าแรก + เฉลี่ยตาม SetSensorRead)
inline void muxSelectF(uint8_t ch) {
  for (uint8_t bit = 0; bit < 4; bit++) digitalWrite(MUXF_SEL[bit], (ch >> bit) & 0x01U);
}

int muxReadF(uint8_t ch) {
  muxSelectF(ch);
  delayMicroseconds(muxSettleUs);
  return muxReadAveraged(MUXF_SIG);
}

inline void muxSelectB(uint8_t ch) {
  for (uint8_t bit = 0; bit < 4; bit++) digitalWrite(MUXB_SEL[bit], (ch >> bit) & 0x01U);
}

int muxReadB(uint8_t ch) {
  muxSelectB(ch);
  delayMicroseconds(muxSettleUs);
  return muxReadAveraged(MUXB_SIG);
}

// MyRP: อ่านค่าดิบทีละช่อง (ช่องนอกช่วงคืน 0 แบบ MyRP)
uint16_t read_sensorA(int sensor) {
  if (sensor < 0 || sensor >= NUM_SENSORS) return 0;
  return muxReadF(sensor);
}

uint16_t read_sensorB(int sensor) {
  if (sensor < 0 || sensor >= NUM_SENSORS) return 0;
  return muxReadB(sensor);
}

void SetSensorTrackLine_CH(int x) {
  TrackLineCH = x;
}

// ---------- ใส่ค่า Calibrate เอง (LightValue_*) — ลำดับพารามิเตอร์ = F[0..15] / B[0..15] ----------
// ใส่ใน RAM (ไม่เขียน EEPROM) แบบต้นฉบับ: เรียกใน Setting ทุกครั้งที่บูต | ต้องการบันทึก → SaveCalibrateToEEPROM()
void LightValue_FrontSensor(
  uint16_t minF0, uint16_t minF1, uint16_t minF2, uint16_t minF3,
  uint16_t minF4, uint16_t minF5, uint16_t minF6, uint16_t minF7,
  uint16_t minF8, uint16_t minF9, uint16_t minF10, uint16_t minF11,
  uint16_t minF12, uint16_t minF13, uint16_t minF14, uint16_t minF15,
  uint16_t maxF0, uint16_t maxF1, uint16_t maxF2, uint16_t maxF3,
  uint16_t maxF4, uint16_t maxF5, uint16_t maxF6, uint16_t maxF7,
  uint16_t maxF8, uint16_t maxF9, uint16_t maxF10, uint16_t maxF11,
  uint16_t maxF12, uint16_t maxF13, uint16_t maxF14, uint16_t maxF15) {
  const uint16_t mn[NUM_SENSORS] = { minF0, minF1, minF2, minF3, minF4, minF5, minF6, minF7,
                                     minF8, minF9, minF10, minF11, minF12, minF13, minF14, minF15 };
  const uint16_t mx[NUM_SENSORS] = { maxF0, maxF1, maxF2, maxF3, maxF4, maxF5, maxF6, maxF7,
                                     maxF8, maxF9, maxF10, maxF11, maxF12, maxF13, maxF14, maxF15 };
  for (int i = 0; i < NUM_SENSORS; i++) {
    minValueF[F_PIN[i]] = mn[i];
    maxValueF[F_PIN[i]] = mx[i];
  }
  arraysToCalib(CAL_VALID_F);
}

void LightValue_CenterSensor(uint16_t minC0, uint16_t minC1, uint16_t maxC0, uint16_t maxC1) {
  minValueC[0] = minC0;
  minValueC[1] = minC1;
  maxValueC[0] = maxC0;
  maxValueC[1] = maxC1;
  arraysToCalib(CAL_VALID_C);
}

void LightValue_BackSensor(
  uint16_t minB0, uint16_t minB1, uint16_t minB2, uint16_t minB3,
  uint16_t minB4, uint16_t minB5, uint16_t minB6, uint16_t minB7,
  uint16_t minB8, uint16_t minB9, uint16_t minB10, uint16_t minB11,
  uint16_t minB12, uint16_t minB13, uint16_t minB14, uint16_t minB15,
  uint16_t maxB0, uint16_t maxB1, uint16_t maxB2, uint16_t maxB3,
  uint16_t maxB4, uint16_t maxB5, uint16_t maxB6, uint16_t maxB7,
  uint16_t maxB8, uint16_t maxB9, uint16_t maxB10, uint16_t maxB11,
  uint16_t maxB12, uint16_t maxB13, uint16_t maxB14, uint16_t maxB15) {
  const uint16_t mn[NUM_SENSORS] = { minB0, minB1, minB2, minB3, minB4, minB5, minB6, minB7,
                                     minB8, minB9, minB10, minB11, minB12, minB13, minB14, minB15 };
  const uint16_t mx[NUM_SENSORS] = { maxB0, maxB1, maxB2, maxB3, maxB4, maxB5, maxB6, maxB7,
                                     maxB8, maxB9, maxB10, maxB11, maxB12, maxB13, maxB14, maxB15 };
  for (int i = 0; i < NUM_SENSORS; i++) {
    minValueB[B_PIN[i]] = mn[i];
    maxValueB[B_PIN[i]] = mx[i];
  }
  arraysToCalib(CAL_VALID_B);
}

// ---------- คาลิเบรตอัตโนมัติ (pauseTime ms ต่อรอบ, samples รอบ) ----------
// เก็บ min / max ค่าดิบ แล้วเผื่อขอบ min + 60 / max - 80 (แบบต้นฉบับ) — เก็บใน RAM (บันทึกด้วย SaveCalibrateToEEPROM)
void caribrateSensorF(int pauseTime, int samples) {
  int mn[NUM_SENSORS], mx[NUM_SENSORS];
  for (int ch = 0; ch < NUM_SENSORS; ch++) {
    mn[ch] = 4095;
    mx[ch] = 0;
  }
  for (int s = 0; s <= samples; s++) {
    ReadMuxFrame();
    for (int ch = 0; ch < NUM_SENSORS; ch++) {
      if (rawF[ch] < mn[ch]) mn[ch] = rawF[ch];
      if (rawF[ch] > mx[ch]) mx[ch] = rawF[ch];
    }
    delay(pauseTime);
  }
  for (int ch = 0; ch < NUM_SENSORS; ch++) {
    minValueF[ch] = mn[ch] + CARIB_MARGIN_MIN;
    maxValueF[ch] = mx[ch] - CARIB_MARGIN_MAX;
  }
  arraysToCalib(CAL_VALID_F);
}

void caribrateSensorC(int pauseTime, int samples) {
  int mn[2] = { 32767, 32767 }, mx[2] = { -32768, -32768 };
  for (int s = 0; s <= samples; s++) {
    for (uint8_t i = 0; i < 2; i++) {
      int16_t raw = 0;
      if (!readCenterRaw(i, raw)) continue;
      if (raw < mn[i]) mn[i] = raw;
      if (raw > mx[i]) mx[i] = raw;
    }
    delay(pauseTime);
  }
  for (int i = 0; i < 2; i++) {
    minValueC[i] = mn[i] + CARIB_C_MARGIN_MIN;
    maxValueC[i] = mx[i] - CARIB_C_MARGIN_MAX;
  }
  arraysToCalib(CAL_VALID_C);
}

void caribrateSensorB(int pauseTime, int samples) {
  int mn[NUM_SENSORS], mx[NUM_SENSORS];
  for (int ch = 0; ch < NUM_SENSORS; ch++) {
    mn[ch] = 4095;
    mx[ch] = 0;
  }
  for (int s = 0; s <= samples; s++) {
    ReadMuxFrame();
    for (int ch = 0; ch < NUM_SENSORS; ch++) {
      if (rawB[ch] < mn[ch]) mn[ch] = rawB[ch];
      if (rawB[ch] > mx[ch]) mx[ch] = rawB[ch];
    }
    delay(pauseTime);
  }
  for (int ch = 0; ch < NUM_SENSORS; ch++) {
    minValueB[ch] = mn[ch] + CARIB_MARGIN_MIN;
    maxValueB[ch] = mx[ch] - CARIB_MARGIN_MAX;
  }
  arraysToCalib(CAL_VALID_B);
}

// ---------- บันทึก / โหลด (MyMINI: CAT24C128 ที่ 0x0200 รูปแบบเดียวกับ MyMINI_16Pro) ----------
void SaveCalibrateToEEPROM() {
  if (!saveCalibration()) Serial.println("SaveCalibrateToEEPROM: CAT24C128 write/verify FAILED");
}

bool LoadCalibrateFromEEPROM() {
  return loadCalibration();  // loadCalibration() เรียก calibToArrays() ให้แล้ว
}

// ---------- EEPROM ภายนอกแบบ MyRP (deviceAddress, eeAddress, data, length) ----------
// deviceAddress = EEPROM_ADDRESS (CAT24C128) → ใช้ตัวเขียนแบบแบ่งหน้า 64 ไบต์ + รอเขียนเสร็จของ MyMINI
// อุปกรณ์อื่น → I2C ตรงแบบต้นฉบับ (ที่อยู่ 16 บิต)
void writeEEPROM(int deviceAddress, unsigned int eeAddress, byte *data, int dataLength) {
  if (dataLength <= 0) return;
  if (deviceAddress == EEPROM_ADDR) {
    if (!writeEEPROM((uint16_t)eeAddress, (const uint8_t *)data, (uint16_t)dataLength)) {
      Serial.println("writeEEPROM: CAT24C128 write failed");
    }
    return;
  }
  Wire.beginTransmission(deviceAddress);
  Wire.write((int)(eeAddress >> 8));
  Wire.write((int)(eeAddress & 0xFF));
  for (int i = 0; i < dataLength; i++) Wire.write(data[i]);
  Wire.endTransmission();
  delay(5);
}

void readEEPROM(int deviceAddress, unsigned int eeAddress, byte *buffer, int dataLength) {
  if (dataLength <= 0) return;
  if (deviceAddress == EEPROM_ADDR) {
    if (!readEEPROM((uint16_t)eeAddress, (uint8_t *)buffer, (uint16_t)dataLength)) {
      Serial.println("readEEPROM: CAT24C128 read failed");
    }
    return;
  }
  Wire.beginTransmission(deviceAddress);
  Wire.write((int)(eeAddress >> 8));
  Wire.write((int)(eeAddress & 0xFF));
  Wire.endTransmission();
  Wire.requestFrom(deviceAddress, dataLength);
  for (int i = 0; i < dataLength; i++) {
    if (Wire.available()) buffer[i] = Wire.read();
  }
}

// ---------- EEPROM ในบอร์ด (flash จำลองของ RP2350) แบบ MyRP: ..._LOCAL ----------
void ensureEEPROMBegin_LOCAL() {
  static bool started = false;
  if (!started) {
    EEPROM.begin(LOCAL_EEPROM_SIZE);
    started = true;
  }
}

void writeEEPROM_LOCAL(unsigned int eeAddress, byte *data, int dataLength, bool commit = true) {
  ensureEEPROMBegin_LOCAL();
  for (int i = 0; i < dataLength; i++) EEPROM.write(eeAddress + i, data[i]);
  if (commit) EEPROM.commit();
}

void readEEPROM_LOCAL(unsigned int eeAddress, byte *buffer, int dataLength) {
  ensureEEPROMBegin_LOCAL();
  for (int i = 0; i < dataLength; i++) buffer[i] = EEPROM.read(eeAddress + i);
}

// ผังข้อมูล LOCAL (big-endian 16 บิต แบบ MyRP, ขยายเป็น 16 ช่อง):
// 0 = maxF[16], 32 = minF[16], 64 = maxB[16], 96 = minB[16], 128 = maxC[2], 132 = minC[2]
#define LOCAL_ADDR_MAX_F 0
#define LOCAL_ADDR_MIN_F 32
#define LOCAL_ADDR_MAX_B 64
#define LOCAL_ADDR_MIN_B 96
#define LOCAL_ADDR_MAX_C 128
#define LOCAL_ADDR_MIN_C 132

static void packWords(const int *v, int n, byte *buf) {
  for (int i = 0; i < n; i++) {
    buf[i * 2] = highByte((uint16_t)v[i]);
    buf[i * 2 + 1] = lowByte((uint16_t)v[i]);
  }
}

static void unpackWords(const byte *buf, int n, int *v, bool isSigned) {
  for (int i = 0; i < n; i++) {
    uint16_t w = ((uint16_t)buf[i * 2] << 8) | buf[i * 2 + 1];
    v[i] = isSigned ? (int)(int16_t)w : (int)w;
  }
}

// อ่าน record ค่า Calibrate (0x0200) ที่ CRC ถูกต้อง — ไม่แตะ calib ที่ใช้งานอยู่
static bool readCalibRecord(CalibrationData &out) {
  for (uint8_t attempt = 0; attempt < CAL_READ_ATTEMPTS; attempt++) {
    if (readEEPROM(CAL_EEPROM_ADDR, (uint8_t *)&out, sizeof(out)) && out.magic == CAL_MAGIC &&
        out.version == CAL_VERSION && out.size == sizeof(CalibrationData) &&
        out.crc16 == calibCRC16((const uint8_t *)&out, offsetof(CalibrationData, crc16))) {
      return true;
    }
    delay(10);
  }
  return false;
}

// ---------- คาลิเบรตแบบ MyRP (calibrateA / B / C: 1000 รอบ แล้วบันทึกลง EEPROM ภายนอก) ----------
// MyMINI: บันทึกเป็น record ของ MyMINI_16Pro ที่ 0x0200 (CRC) แทนผังดิบที่ 0..71 ของ MyRP
//         เพื่อไม่ทับค่า Calibrate ของเครื่อง → ค่าใหม่ใช้เดินเส้นได้ทันที
void calibrateA() {
  const int samples = 1000;
  Serial.println("กำลัง calibrate Sensor A (Line Front)...");
  for (int ch = 0; ch < NUM_SENSORS; ch++) {
    sensorMaxA[ch] = 0;
    sensorMinA[ch] = 4095;  // ADC ของ Pico = 12 บิต
  }
  for (int s = 0; s < samples; s++) {
    ReadMuxFrame();
    for (int ch = 0; ch < NUM_SENSORS; ch++) {
      if (rawF[ch] > sensorMaxA[ch]) sensorMaxA[ch] = rawF[ch];
      if (rawF[ch] < sensorMinA[ch]) sensorMinA[ch] = rawF[ch];
    }
    delay(1);
  }
  for (int ch = 0; ch < NUM_SENSORS; ch++) {
    minValueF[ch] = sensorMinA[ch];
    maxValueF[ch] = sensorMaxA[ch];
  }
  arraysToCalib(CAL_VALID_F);
  if (!saveCalibration()) Serial.println("calibrateA: CAT24C128 save FAILED (values kept in RAM)");
  beep(3000, 100);
  delay(150);
  beep(3000, 200);
  Serial.println("✓ Calibrate Sensor A เสร็จสิ้น");
}

void calibrateB() {
  const int samples = 1000;
  Serial.println("กำลัง calibrate Sensor B (Line Back)...");
  for (int ch = 0; ch < NUM_SENSORS; ch++) {
    sensorMaxB[ch] = 0;
    sensorMinB[ch] = 4095;
  }
  for (int s = 0; s < samples; s++) {
    ReadMuxFrame();
    for (int ch = 0; ch < NUM_SENSORS; ch++) {
      if (rawB[ch] > sensorMaxB[ch]) sensorMaxB[ch] = rawB[ch];
      if (rawB[ch] < sensorMinB[ch]) sensorMinB[ch] = rawB[ch];
    }
    delay(1);
  }
  for (int ch = 0; ch < NUM_SENSORS; ch++) {
    minValueB[ch] = sensorMinB[ch];
    maxValueB[ch] = sensorMaxB[ch];
  }
  arraysToCalib(CAL_VALID_B);
  if (!saveCalibration()) Serial.println("calibrateB: CAT24C128 save FAILED (values kept in RAM)");
  beep(3000, 100);
  delay(150);
  beep(3000, 200);
  Serial.println("✓ Calibrate Sensor B เสร็จสิ้น");
}

// 1000 รอบ ห่าง 5 ms (แบบ MyRP) อ่าน ADS1115 AIN1 / AIN2
void calibrateC() {
  const int samples = 1000;
  Serial.println("กำลัง calibrate Sensor C (Side Sensors)...");
  for (int i = 0; i < 2; i++) {
    sensorMaxC[i] = -32768;
    sensorMinC[i] = 32767;
  }
  for (int s = 0; s < samples; s++) {
    for (uint8_t i = 0; i < 2; i++) {
      int16_t v = 0;
      if (!readCenterRaw(i, v)) continue;
      if (v > sensorMaxC[i]) sensorMaxC[i] = v;
      if (v < sensorMinC[i]) sensorMinC[i] = v;
    }
    delay(5);
  }
  for (int i = 0; i < 2; i++) {
    minValueC[i] = sensorMinC[i];
    maxValueC[i] = sensorMaxC[i];
  }
  arraysToCalib(CAL_VALID_C);
  if (!saveCalibration()) Serial.println("calibrateC: CAT24C128 save FAILED (values kept in RAM)");
  beep(3000, 150);
  delay(200);
  beep(3000, 200);
  Serial.println("✓ Calibrate Sensor C เสร็จสิ้น");
}

// ---------- บันทึกสำเนาลง flash ในบอร์ด (MyRP: saveCalibA_LOCAL / B / C) ----------
void saveCalibA_LOCAL() {
  for (int ch = 0; ch < NUM_SENSORS; ch++) {
    maxValueF[ch] = sensorMaxA[ch];
    minValueF[ch] = sensorMinA[ch];
  }
  byte buf[NUM_SENSORS * 2];
  packWords(maxValueF, NUM_SENSORS, buf);
  writeEEPROM_LOCAL(LOCAL_ADDR_MAX_F, buf, sizeof(buf), false);  // commit ครั้งเดียวตอนเขียนชุด min
  packWords(minValueF, NUM_SENSORS, buf);
  writeEEPROM_LOCAL(LOCAL_ADDR_MIN_F, buf, sizeof(buf));
  arraysToCalib(CAL_VALID_F);
  Serial.println("✓ บันทึก Calibrate Sensor A ลง EEPROM (LOCAL) เสร็จสิ้น");
}

void saveCalibB_LOCAL() {
  for (int ch = 0; ch < NUM_SENSORS; ch++) {
    maxValueB[ch] = sensorMaxB[ch];
    minValueB[ch] = sensorMinB[ch];
  }
  byte buf[NUM_SENSORS * 2];
  packWords(maxValueB, NUM_SENSORS, buf);
  writeEEPROM_LOCAL(LOCAL_ADDR_MAX_B, buf, sizeof(buf), false);
  packWords(minValueB, NUM_SENSORS, buf);
  writeEEPROM_LOCAL(LOCAL_ADDR_MIN_B, buf, sizeof(buf));
  arraysToCalib(CAL_VALID_B);
  Serial.println("✓ บันทึก Calibrate Sensor B ลง EEPROM (LOCAL) เสร็จสิ้น");
}

void saveCalibC_LOCAL() {
  for (int i = 0; i < 2; i++) {
    maxValueC[i] = sensorMaxC[i];
    minValueC[i] = sensorMinC[i];
  }
  byte buf[4];
  packWords(maxValueC, 2, buf);
  writeEEPROM_LOCAL(LOCAL_ADDR_MAX_C, buf, 4, false);
  packWords(minValueC, 2, buf);
  writeEEPROM_LOCAL(LOCAL_ADDR_MIN_C, buf, 4);
  arraysToCalib(CAL_VALID_C);
  Serial.println("✓ บันทึก Calibrate Sensor C ลง EEPROM (LOCAL) เสร็จสิ้น");
}

// ---------- อ่านค่า Calibrate จาก EEPROM ภายนอก → sensorMin / sensorMax (MyRP: read_eepA / B / C) ----------
void read_eepA() {
  CalibrationData rec;
  if (!readCalibRecord(rec) || !(rec.validMask & CAL_VALID_F)) {
    Serial.println("read_eepA: no valid front calibration in CAT24C128");
    return;
  }
  for (int ch = 0; ch < NUM_SENSORS; ch++) {
    sensorMaxA[ch] = rec.frontMax[ch];
    sensorMinA[ch] = rec.frontMin[ch];
  }
}

void read_eepB() {
  CalibrationData rec;
  if (!readCalibRecord(rec) || !(rec.validMask & CAL_VALID_B)) {
    Serial.println("read_eepB: no valid back calibration in CAT24C128");
    return;
  }
  for (int ch = 0; ch < NUM_SENSORS; ch++) {
    sensorMaxB[ch] = rec.rearMax[ch];
    sensorMinB[ch] = rec.rearMin[ch];
  }
}

void read_eepC() {
  CalibrationData rec;
  if (!readCalibRecord(rec) || !(rec.validMask & CAL_VALID_C)) {
    Serial.println("read_eepC: no valid center calibration in CAT24C128");
    return;
  }
  for (int i = 0; i < 2; i++) {
    sensorMaxC[i] = rec.centerMax[i];
    sensorMinC[i] = rec.centerMin[i];
  }
}

// ---------- อ่านสำเนาใน flash → ตารางที่ใช้เดินเส้น (MyRP: read_eepA_LOCAL / B / C) ----------
void read_eepA_LOCAL() {
  byte buf[NUM_SENSORS * 2];
  readEEPROM_LOCAL(LOCAL_ADDR_MAX_F, buf, sizeof(buf));
  unpackWords(buf, NUM_SENSORS, maxValueF, false);
  readEEPROM_LOCAL(LOCAL_ADDR_MIN_F, buf, sizeof(buf));
  unpackWords(buf, NUM_SENSORS, minValueF, false);
}

void read_eepB_LOCAL() {
  byte buf[NUM_SENSORS * 2];
  readEEPROM_LOCAL(LOCAL_ADDR_MAX_B, buf, sizeof(buf));
  unpackWords(buf, NUM_SENSORS, maxValueB, false);
  readEEPROM_LOCAL(LOCAL_ADDR_MIN_B, buf, sizeof(buf));
  unpackWords(buf, NUM_SENSORS, minValueB, false);
}

void read_eepC_LOCAL() {
  byte buf[4];
  readEEPROM_LOCAL(LOCAL_ADDR_MAX_C, buf, 4);
  unpackWords(buf, 2, maxValueC, true);
  readEEPROM_LOCAL(LOCAL_ADDR_MIN_C, buf, 4);
  unpackWords(buf, 2, minValueC, true);
}

void printCalibration() {
  Serial.println("\n=== Calibration Values Loaded ===");
  Serial.println("--- Sensor A (Front Line) ---");
  for (int i = 0; i < NUM_SENSORS; i++) Serial.printf("A%d → Max: %4d  Min: %4d\n", i, sensorMaxA[i], sensorMinA[i]);
  Serial.println("--- Sensor B (Back Line) ---");
  for (int i = 0; i < NUM_SENSORS; i++) Serial.printf("B%d → Max: %4d  Min: %4d\n", i, sensorMaxB[i], sensorMinB[i]);
  Serial.println("--- Sensor C (Side) ---");
  Serial.printf("C0 → Max: %4d  Min: %4d\n", sensorMaxC[0], sensorMinC[0]);
  Serial.printf("C1 → Max: %4d  Min: %4d\n", sensorMaxC[1], sensorMinC[1]);
  Serial.println("=================================\n");
}

int countBadCalibration_LOCAL() {
  int bad = 0;
  for (int i = 0; i < NUM_SENSORS; i++) {
    if (maxValueF[i] <= minValueF[i]) bad++;
    if (maxValueB[i] <= minValueB[i]) bad++;
  }
  for (int i = 0; i < 2; i++) {
    if (maxValueC[i] <= minValueC[i]) bad++;
  }
  return bad;
}

void printCalibration_LOCAL() {
  Serial.println("\n=== Calibration Values Loaded (LOCAL) ===");
  Serial.println("--- Sensor A (Front Line) ---");
  for (int i = 0; i < NUM_SENSORS; i++) Serial.printf("A%d → Max: %4d  Min: %4d\n", i, maxValueF[i], minValueF[i]);
  Serial.println("--- Sensor B (Back Line) ---");
  for (int i = 0; i < NUM_SENSORS; i++) Serial.printf("B%d → Max: %4d  Min: %4d\n", i, maxValueB[i], minValueB[i]);
  Serial.println("--- Sensor C (Side) ---");
  Serial.printf("C0 → Max: %4d  Min: %4d\n", maxValueC[0], minValueC[0]);
  Serial.printf("C1 → Max: %4d  Min: %4d\n", maxValueC[1], minValueC[1]);
  int bad = countBadCalibration_LOCAL();
  if (bad > 0) {
    Serial.printf("!! WARNING: %d sensor channel(s) not calibrated (Max <= Min) -> read as 0\n", bad);
  }
  Serial.println("=================================\n");
}

// MyRP: โหลดสำเนาใน flash เป็นค่าที่ใช้เดินเส้น (ใช้แทน CAT24C128 ได้เมื่อ EEPROM ภายนอกเสีย)
// ใช้เฉพาะกลุ่มที่ทุกช่อง max > min — กลุ่มที่ยังไม่เคยบันทึกจะคงค่าจาก CAT24C128 ไว้
void loadCalibration_LOCAL() {
  read_eepA_LOCAL();
  read_eepB_LOCAL();
  read_eepC_LOCAL();
  printCalibration_LOCAL();
  bool okF = true, okB = true, okC = true;
  for (int i = 0; i < NUM_SENSORS; i++) {
    if (maxValueF[i] <= minValueF[i]) okF = false;
    if (maxValueB[i] <= minValueB[i]) okB = false;
  }
  for (int i = 0; i < 2; i++) {
    if (maxValueC[i] <= minValueC[i]) okC = false;
  }
  uint8_t group = (okF ? CAL_VALID_F : 0) | (okB ? CAL_VALID_B : 0) | (okC ? CAL_VALID_C : 0);
  if (group) arraysToCalib(group);
  calibToArrays();  // กลุ่มที่ไม่ได้ใช้ → ตารางกลับเป็นค่าที่ใช้งานจริง
}

// ---------- Serial Debug (MyRP) — วนไม่จบ ห้ามเรียกใน Mission ----------
void Serial_FrontSensor() {
  while (1) {
    ReadSensorRaw();
    for (int i = 0; i < NUM_SENSORS; i++) {
      Serial.print(F[i]);
      Serial.print("\t");
    }
    Serial.println();
    delay(50);
  }
}

void Serial_BackSensor() {
  while (1) {
    ReadSensorRaw();
    for (int i = 0; i < NUM_SENSORS; i++) {
      Serial.print(B[i]);
      Serial.print("\t");
    }
    Serial.println();
    delay(50);
  }
}

void Serial_CenterSensor() {
  while (1) {
    ReadSensorRaw();
    for (int i = 0; i < 2; i++) {
      Serial.print(C[i]);
      Serial.print("\t");
    }
    Serial.println();
    delay(50);
  }
}

void SerialCalibrate_FrontSensor() {
  while (1) {
    ReadSensor();
    for (int i = 0; i < NUM_SENSORS; i++) {
      Serial.print(F[i]);
      Serial.print("\t");
    }
    Serial.println();
    delay(100);
  }
}

void SerialCalibrate_BackSensor() {
  while (1) {
    ReadSensor();
    for (int i = 0; i < NUM_SENSORS; i++) {
      Serial.print(B[i]);
      Serial.print("\t");
    }
    Serial.println();
    delay(100);
  }
}

void SerialCalibrate_CenterSensor() {
  while (1) {
    ReadSensor();
    for (int i = 0; i < 2; i++) {
      Serial.print(C[i]);
      Serial.print("\t");
    }
    Serial.println();
    delay(100);
  }
}

// ค่าดิบ F | B | C
void SerialAllSensor() {
  while (1) {
    ReadSensorRaw();
    Serial.print("F: ");
    for (int i = 0; i < NUM_SENSORS; i++) {
      Serial.print(F[i]);
      Serial.print("\t");
    }
    Serial.print(" | B: ");
    for (int i = 0; i < NUM_SENSORS; i++) {
      Serial.print(B[i]);
      Serial.print("\t");
    }
    Serial.print(" | C: ");
    for (int i = 0; i < 2; i++) {
      Serial.print(C[i]);
      Serial.print("\t");
    }
    Serial.println();
    delay(100);
  }
}

// MyRP: ค่าดิบ F B C
void Serial_AllSensor() {
  while (1) {
    ReadSensorRaw();
    Serial.print("F: ");
    for (int i = 0; i < NUM_SENSORS; i++) {
      Serial.print(F[i]);
      Serial.print("\t");
    }
    Serial.print("B: ");
    for (int i = 0; i < NUM_SENSORS; i++) {
      Serial.print(B[i]);
      Serial.print("\t");
    }
    Serial.print("C: ");
    for (int i = 0; i < 2; i++) {
      Serial.print(C[i]);
      Serial.print("\t");
    }
    Serial.println();
    delay(50);
  }
}

// ค่า 0..1000 F B C
void SerialCalibrate_AllSensor() {
  while (1) {
    ReadSensor();
    Serial.print("F : ");
    for (int i = 0; i < NUM_SENSORS; i++) {
      Serial.print(F[i]);
      Serial.print("\t");
    }
    Serial.print("B : ");
    for (int i = 0; i < NUM_SENSORS; i++) {
      Serial.print(B[i]);
      Serial.print("\t");
    }
    Serial.print("C : ");
    for (int i = 0; i < 2; i++) {
      Serial.print(C[i]);
      Serial.print("\t");
    }
    Serial.println();
    delay(100);
  }
}

// MyRP: ค่าเซนเซอร์วัดระยะ (ขา DIST หรือ ADS1115 AIN3 ตาม SetAnalogDistance)
void SerialDistance() {
  while (1) {
    Serial.print("ADC : ");
    Serial.println(readDistance());
    delay(100);
  }
}

#endif  // MYMINI16PRO_BNO086_SENSOR_H
