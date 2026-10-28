#ifndef MYMINI_16PRO_H
#define MYMINI_16PRO_H

/*
 * MyMini_16Pro_BNO086.h
 * ไฟล์หลักของไลบรารี: รวมทุกโมดูล + RobotSetup() + เมนูปุ่ม sw()
 *
 * สารบัญ:
 *   [1] รวมโมดูลทั้งหมด (include)
 *   [2] เริ่มต้นระบบ (RobotSetup)
 *   [3] แสดงค่าเซนเซอร์หนึ่งบรรทัด (ใช้ในเมนู sw)
 *   [4] เมนูปุ่ม sw() — คาลิเบรต / แสดงค่าเซนเซอร์ / ออกตัว
 */

// ======================================================================
// [1] รวมโมดูลทั้งหมด (include)
// ======================================================================

#include "MyMini_16Pro_BNO086_Buzzer.h"
#include "MyMini_16Pro_BNO086_I2C.h"
#include "MyMini_16Pro_BNO086_LED.h"
#include "MyMini_16Pro_BNO086_Battery.h"
#include "MyMini_16Pro_BNO086_Motor.h"
#include "MyMini_16Pro_BNO086_Sensor.h"
#include "MyMini_16Pro_BNO086_PID.h"
#include "MyMini_16Pro_BNO086_Servo.h"
#include "MyMini_16Pro_BNO086_Gyro.h"

// ======================================================================
// [2] เริ่มต้นระบบ (RobotSetup)
// ======================================================================

void RobotSetup() {
  // หยุดมอเตอร์ก่อนอย่างอื่นเสมอ (ขาลอยตอนบูตอาจทำให้ล้อหมุน)
  MotorBegin();

  Serial.begin(115200);
  delay(100);

  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(buzzer_pin, OUTPUT);
  digitalWrite(buzzer_pin, LOW);

  I2C_Begin();
  SensorBegin();
  LED_Off();

  readBatteryVoltage();
  loadCalibration();
  if (gyro_Begin()) resetYaw();

  // ตั้งความเร็ว I2C หลังสุด เพราะ myIMU.begin() เรียก Wire.begin() ซ้ำข้างใน
  Wire.setClock(I2C_CLOCK);

  SoundStartup();
  LED_Ready();
  blink(3);
  MotorCoast();
  Serial.println("MyMini_16Pro Class Ready for Competition!");
}

// ======================================================================
// [3] แสดงค่าเซนเซอร์หนึ่งบรรทัด (ใช้ในเมนู sw)
// ======================================================================

// ค่าดิบ F0-15 / B0-15 / CL CR + มุมไจโร + แรงดันแบต
void printSensorRow() {
  ReadSensorRaw();
  Serial.print("F: ");
  for (int i = 0; i < NUM_SENSORS; i++) {
    Serial.print(F[i]);
    Serial.print(" ");
  }
  Serial.print("  B: ");
  for (int i = 0; i < NUM_SENSORS; i++) {
    Serial.print(B[i]);
    Serial.print(" ");
  }
  Serial.print("  C: ");
  Serial.print(C[CCL]);
  Serial.print(" ");
  Serial.print(C[CCR]);
  Serial.print("  Gyro: ");
  if (imuReady) Serial.print(gyroZ());  // มุม Yaw ช่วง -180..180
  else Serial.print("NA");
  Serial.print("  Bat: ");
  Serial.print(getBatteryVoltage(), 2);
  Serial.println();
}

// ======================================================================
// [4] เมนูปุ่ม sw() — คาลิเบรต / แสดงค่าเซนเซอร์ / ออกตัว
// ======================================================================

// ปุ่ม 3 (Cal หน้า)        → Calibrate F
// ปุ่ม ADCcal (MCP3421)   → Calibrate B
// ปุ่ม 2 กดค้าง 3 วินาที   → Calibrate C
// ปุ่ม 2 กดสั้น            → ออกจากเมนู (ถ้าแบตวิกฤติจะไม่ยอมออกตัว)
void sw() {
  MotorStop();
  tone(buzzer_pin, 2000, 100);  // โด
  delay(100);
  tone(buzzer_pin, 2400, 100);  // เร
  delay(100);
  tone(buzzer_pin, 3000, 160);  // มี
  delay(500);
  tone(buzzer_pin, 3000, 60);
  delay(80);
  tone(buzzer_pin, 3000, 60);
  delay(80);

  unsigned long pressStartTime = 0;
  bool isPressed = false;
  unsigned long lastPrint = 0;
  unsigned long lastCalBRead = 0;
  // ปุ่ม Cal ที่ค้างอยู่ตั้งแต่เข้าเมนูต้องปล่อยก่อน กันลบค่า Calibrate โดยไม่ตั้งใจ
  bool calFLatched = digitalRead(CAL_F_BUTTON) == LOW;
  bool calBLatched = true;
  int calBCount = 0;

  while (true) {
    updateBattery();
    bat_update_led();
    bat_warning();

    // ปุ่ม 3 → Calibrate F
    if (digitalRead(CAL_F_BUTTON) == LOW) {
      if (!calFLatched) {
        calFLatched = true;
        digitalWrite(LED_BUILTIN, HIGH);
        noTone(buzzer_pin);
        tone(buzzer_pin, 3000, 300);
        delay(300);
        calibrateF();
        digitalWrite(LED_BUILTIN, LOW);
      }
    } else {
      calFLatched = false;
    }

    // ปุ่ม ADCcal → Calibrate B (ต้องอ่านได้ค่ากด 3 ครั้งติดกัน กันสัญญาณกระตุก)
    if (millis() - lastCalBRead >= 100) {
      lastCalBRead = millis();
      int adcCal = ADC_i2c();
      if (adcCal > CAL_B_THRESHOLD) {
        calBLatched = false;
        calBCount = 0;
      } else if (!calBLatched && ++calBCount >= 3) {
        calBLatched = true;
        calBCount = 0;
        digitalWrite(LED_BUILTIN, HIGH);
        noTone(buzzer_pin);
        tone(buzzer_pin, 3000, 100);
        delay(200);
        calibrateB();
        digitalWrite(LED_BUILTIN, LOW);
      }
    }

    // แสดงค่าซีเรียลทุก 100ms
    if (millis() - lastPrint >= 100) {
      lastPrint = millis();
      printSensorRow();
    }

    // ตรวจจับกดค้างปุ่ม 2 นาน 3 วินาที → Calibrate C
    if (digitalRead(START_BUTTON) == LOW) {  // ปุ่มถูกกด (LOW เพราะใช้ PULLUP)
      if (!isPressed) {
        pressStartTime = millis();  // บันทึกเวลาที่กดปุ่มครั้งแรก
        isPressed = true;
      } else if (millis() - pressStartTime >= 3000) {
        digitalWrite(LED_BUILTIN, HIGH);
        noTone(buzzer_pin);
        tone(buzzer_pin, 3000, 200);
        delay(300);
        calibrateC();
        digitalWrite(LED_BUILTIN, LOW);
        while (digitalRead(START_BUTTON) == LOW)
          ;          // รอปล่อยปุ่ม
        delay(400);  // ป้องกันการเด้งของปุ่ม
        isPressed = false;
      }
    } else if (isPressed) {
      unsigned long pressDuration = millis() - pressStartTime;
      isPressed = false;

      if (pressDuration >= 50 && pressDuration < 3000) {
        readBatteryVoltage();  // ตรวจแบตค่าล่าสุดก่อนออกตัว
        if (isBatteryCritical()) {
          Serial.printf("แบตต่ำเกินไป (%.2f V) — เปลี่ยนแบตก่อนออกตัว\n", getBatteryVoltage());
          SoundError();
          continue;
        }
        Serial.println("Start Mission");
        break;
      }
    }
  }

  noTone(buzzer_pin);
  LED_Off();
  tone(buzzer_pin, 3500, 400);
  delay(500);
  // กด START = ตั้งมุมศูนย์ของภารกิจ (หุ่นต้องนิ่งตอนนี้)
  resetAngles();
}

#endif  // MYMINI_16PRO_H
