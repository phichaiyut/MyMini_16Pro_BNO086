#ifndef MYMINI16PRO_BNO086_H
#define MYMINI16PRO_BNO086_H

/*
 * MyMini_16Pro_BNO086.h
 * ไฟล์หลักของไลบรารี: รวมทุกโมดูล + RobotSetup() + เมนูรอปุ่ม sw()
 * ฮาร์ดแวร์ / ลำดับเริ่มต้นตาม MyMINI_16Pro (robot_begin / wait_button)
 * สไตล์การเขียนตาม MyMini_16Pro_BNO086 (header-only)
 *
 * สารบัญ:
 *   [1] รวมโมดูลทั้งหมด (include)
 *   [2] เริ่มต้นระบบ (RobotSetup)
 *   [3] แสดงค่าเซนเซอร์หนึ่งบรรทัด
 *   [4] เมนูรอปุ่ม sw() — Calibrate / แสดงค่า / ออกตัว
 *   [5] อ่าน ADC ภายนอก (MyRP: ADC_i2c)
 *   [6] เมนูปุ่ม (swOK / CalibrateSensor)
 */

// ======================================================================
// [1] รวมโมดูลทั้งหมด (include)
// ======================================================================

#include <Arduino.h>
#include "MyMini_16Pro_BNO086_Buzzer.h"
#include "MyMini_16Pro_BNO086_I2C.h"
#include "MyMini_16Pro_BNO086_Motor.h"
#include "MyMini_16Pro_BNO086_Sensor.h"
#include "MyMini_16Pro_BNO086_Battery.h"
#include "MyMini_16Pro_BNO086_Button.h"
#include "MyMini_16Pro_BNO086_PID.h"
#include "MyMini_16Pro_BNO086_Servo.h"
#include "MyMini_16Pro_BNO086_Gyro.h"
#include "MyMini_16Pro_BNO086_my_MCP3008s.h"     // ไดรเวอร์ MCP3008 ของ MyRP (MyMINI ไม่มีชิปนี้ — ใช้เมื่อต่อโมดูลเพิ่มเอง)

// อ็อบเจกต์ MCP3008 ชื่อ adc แบบ MyRP — ยังไม่ begin() (ไม่จองขาใดของ MyMINI จนกว่าจะเรียก adc.begin(...))
my_MCP3008s adc;

// ======================================================================
// [1.1] เริ่มต้น / ตั้งศูนย์ไจโร (งบเวลา 2000 ms แบบ gyro_reset() ต้นฉบับ)
// ======================================================================

#define GYRO_RESET_BUDGET_MS 2000

bool GyroBegin() {
  bool ok = gyro_Begin(GYRO_RESET_BUDGET_MS);
  Wire.setClock(I2C_CLOCK);
  Wire.setTimeout(I2C_TIMEOUT_MS, true);
  return ok;
}

// เปิดไจโรถ้ายังไม่พร้อม → ตั้งศูนย์ (resetAngles) → เสียงพร้อม | ผู้เรียกต้อง MotorStop() ก่อน
bool GyroReset() {
  if (!imuReady) GyroBegin();
  bool ok = imuReady && resetYaw();
  if (ok) {
    current_degree = angleRead();
    previous_errorG = 0;
    previous_errorGB = 0;
  }
  Serial.println(ok ? "BNO086: GYRO_READY heading zero" : "BNO086: GYRO_NOT_READY");
  if (ok) SoundGyroReady();
  return ok;
}

// ======================================================================
// [2] เริ่มต้นระบบ (RobotSetup)
// ======================================================================

// ลำดับเดียวกับ robot_begin() ต้นฉบับ:
// Serial → buzzer LOW → มอเตอร์ (หยุดก่อนเสมอ) → MUX → I2C → ปุ่ม + โหลด Calibrate
// → Motor(0,0) → เสียงเปิดเครื่อง → 500 ms → Motor(0,0)
// ไจโรไม่ได้เริ่มที่นี่ (ต้นฉบับเริ่มใน wait_button) — sw() จะเปิดและตั้งศูนย์ให้
void RobotSetup() {
  Serial.begin(115200);
  BuzzerBegin();
  MotorBegin();
  SensorBegin();
  I2C_Begin();
  ButtonBegin();
  loadCalibration();
  printCalibrationStatus();
  Motor(0, 0);
  SoundStartup();
  delay(500);
  Motor(0, 0);
  MotorStop();
  GyroReset();  // เปิดไจโร + ตั้งศูนย์ 0 องศา ตอนเปิดเครื่อง (หุ่นต้องอยู่นิ่ง)
  Motor(0, 0);
}

// ======================================================================
// [3] แสดงค่าเซนเซอร์หนึ่งบรรทัด
// ======================================================================

// F / B / CL / CR แบบเดินเส้น 0..1000 (เจอเส้น = 1000, ยังไม่ Calibrate = NA) + มุมไจโร + แรงดันแบต
void SerialShowAllSensorOnce() {
  ReadSensor();
  Serial.print("F:");
  if (CalibratedF()) printSensorArray("", F, NUM_SENSORS);
  else Serial.print("NA");
  Serial.print(" B:");
  if (CalibratedB()) printSensorArray("", B, NUM_SENSORS);
  else Serial.print("NA");
  Serial.print(" adcL:");
  if (!rawCValid[CCL]) Serial.print("ERR");
  else if (!CalibratedC()) Serial.print("NA");
  else Serial.print(C[CCL]);
  Serial.print(" adcR:");
  if (!rawCValid[CCR]) Serial.print("ERR");
  else if (!CalibratedC()) Serial.print("NA");
  else Serial.print(C[CCR]);
  Serial.print(" gyro:");
  if (imuReady) Serial.print(angleRead());
  else Serial.print("NA");
  Serial.print(" vbat:");
  if (readBatteryVoltage()) Serial.print(getBatteryVoltage(), 2);
  else Serial.print("ERR");
  Serial.println();
}

// ======================================================================
// [4] เมนูรอปุ่ม sw() — Calibrate / แสดงค่า / ออกตัว
// ======================================================================

// ปุ่ม GP3 (Cal หน้า)        → CalibrateSensorF()
// ปุ่ม ADCcal (MCP3421)     → CalibrateSensorB()
// START กดค้าง 3 วินาที      → CalibrateSensorC()
// START กดสั้น 30–800 ms     → ออกตัว (แบตวิกฤติ 6.0–10.85 V จะไม่ยอมออก)
void sw() {
  MotorStop();
  if (!imuReady) GyroBegin();  // เปิดไจโรอย่างเดียว — ไม่ตั้งศูนย์องศา (เรียก GyroReset() / resetAngles() เองเมื่อต้องการ)
  Motor(0, 0);
  SoundWait();

  unsigned long now = millis();
  unsigned long lastPrint = now;
  BatteryLED_Reset(now);
  BatteryWarning_Stop();

  while (true) {
    now = millis();
    uint8_t ev = ButtonStartUpdate(now);
    if (ev == BTN_NONE) ev = ButtonCalUpdate(now);

    if (ev == BTN_START_SHORT) {
      readBatteryVoltage();  // ตรวจค่าใหม่ก่อนออกตัว
      if (!isBatteryStartBlocked()) break;
      Serial.printf("Battery too low (%.2f V) — start blocked\n", getBatteryVoltage());
    } else if (ev == BTN_CAL_F || ev == BTN_CAL_B || ev == BTN_START_LONG) {
      BatteryWarning_Stop();
      if (ev == BTN_CAL_F) CalibrateSensorF();
      else if (ev == BTN_CAL_B) CalibrateSensorB();
      else CalibrateSensorC();
      continue;
    }

    if (now - lastPrint >= 100) {
      lastPrint = now;
      SerialShowAllSensorOnce();
    }
    BatteryLED_Update(millis());
    BatteryWarning_Update(millis());
  }

  // กด START: ออกตัว (ไม่ตั้งศูนย์องศาไจโร)
  Motor(0, 0);
  BatteryWarning_Stop();
  LED_Off();
  SoundStart();
}


// ======================================================================
// [5] อ่าน ADC ภายนอก (MyRP_Pico2: ADC_i2c) — MCP3421 ปุ่ม ADCcal
// ======================================================================

// สเกลเดียวกับ MyRP: ปล่อยปุ่ม ≈ 4000, กดปุ่ม ≈ 0, อ่านไม่ได้ = 9999 (MyRP: < 2500 = กด)
// MyRP อ่าน 4 ไบต์ของ MCP3421 โหมด 12 บิตเป็นค่า 24 บิต (ค่า << 8 | ไบต์ config)
// MyMINI อ่านด้วย readMCP3421() (12 บิต, 1 mV/LSB) แล้วประกอบค่าแบบเดียวกันก่อน map
int ADC_i2c() {
  int16_t raw = 0;
  if (!readMCP3421(raw)) return 9999;
  long ADC01 = ((long)raw << 8) | 0x10;  // 0x10 = ไบต์ config ตอนค่าพร้อม (continuous, 12 บิต, gain x1)
  return map(ADC01, 524048, 282, 4000, 0);
}

// ======================================================================
// [6] เมนูปุ่ม (swOK / CalibrateSensor)
// ======================================================================
// ปุ่ม OK = ปุ่ม Calibrate หน้า GP3 (OK_PIN) | ปุ่ม START = GP2

void CalibrateSensor();

// แสดงค่าเซนเซอร์ไปเรื่อย ๆ | กด OK = ออก | กด START = CalibrateSensor() (หน้า → หลัง → กลาง)
void swOK() {
  MotorStop();
  while (1) {
    SerialShowAllSensorOnce();
    if (digitalRead(OK_PIN) == PRESS) {
      break;
    }
    if (digitalRead(START_PIN) == PRESS) {
      delay(30);  // debounce
      if (digitalRead(START_PIN) == PRESS) {
        Beep(200);
        Beep(50);
        Beep(200);
        CalibrateSensor();
        while (digitalRead(START_PIN) == PRESS)
          ;
      }
    }
    delay(50);
  }
  delay(200);
  Beep(200);
}

// พิมพ์ค่าแบบ LightValue_*(min..., max...) ลำดับ F[0..15] / B[0..15] → คัดลอกไปวางใน Setting ได้ทันที
void printLightValue(const char *name, const int *mn, const int *mx, const uint8_t *pins, int n) {
  Serial.print(name);
  Serial.print(" (");
  for (int i = 0; i < n; i++) {
    Serial.print(mn[pins ? pins[i] : i]);
    Serial.print(",");
  }
  for (int i = 0; i < n; i++) {
    Serial.print(mx[pins ? pins[i] : i]);
    if (i < n - 1) Serial.print(",");
    else Serial.println(");");
  }
}

// กด START ทีละขั้น → Calibrate หน้า / หลัง / กลาง (caribrateSensor*(20, 200))
// → พิมพ์ LightValue_* → บันทึกลง CAT24C128 → รอกด START
void CalibrateSensor() {
  Serial.println("Press Cali to start caribrate Front Sensor");
  swSTART();
  Serial.println("Caribrating");
  Beep(100);
  delay(600);
  caribrateSensorF(20, 200);
  Beep(100);
  delay(600);
  Serial.println("Finish\n");

  Serial.println("Press Cali to start caribrate Back Sensor");
  swSTART();
  Serial.println("Caribrating");
  Beep(100);
  delay(600);
  caribrateSensorB(20, 200);
  Beep(100);
  delay(600);
  Serial.println("Finish\n");

  Serial.println("Press Cali to start Caribrate Center Sensor");
  swSTART();
  Serial.println("Caribrating");
  Beep(100);
  delay(600);
  caribrateSensorC(20, 200);
  Beep(100);
  delay(600);
  Serial.println("Finish\n");

  printLightValue("LightValue_FrontSensor", minValueF, maxValueF, F_PIN, NUM_SENSORS);
  printLightValue("LightValue_BackSensor", minValueB, maxValueB, B_PIN, NUM_SENSORS);
  printLightValue("LightValue_CenterSensor", minValueC, maxValueC, nullptr, 2);

  SaveCalibrateToEEPROM();
  Serial.println("Saved calibration to EEPROM");

  while (digitalRead(START_PIN) == !PRESS)
    ;
}

#endif  // MYMINI16PRO_BNO086_H
