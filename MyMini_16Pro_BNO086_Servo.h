#ifndef MYMINI_16PRO_SERVO_H
#define MYMINI_16PRO_SERVO_H

/*
 * MyMini_16Pro_BNO086_Servo.h
 * เซอร์โวและแขนกล (ขาตามบอร์ด MyMINI_16Pro, คำสั่งแบบ MyRP_Pico2_mini_BNO086)
 *
 * สารบัญ:
 *   [1] ขาและตัวแปรเซอร์โว
 *   [2] ตั้งค่า trim / กลับทิศ / ช่วงพัลส์
 *   [3] สั่งเซอร์โวทีละตัว
 *   [4] ควบคุมเซอร์โวผ่าน Serial
 *   [5] สั่งแขนหลายตัวพร้อมกัน (Servo / armupdown / arm_left_right)
 */

#include <Servo.h>
#include "MyMini_16Pro_BNO086_Motor.h"

// ======================================================================
// [1] ขาและตัวแปรเซอร์โว
// ======================================================================

#define SERVO_L_PIN 18   // มือซ้าย (ช่อง 1)
#define SERVO_R_PIN 22   // มือขวา (ช่อง 2)
#define SERVO_UD_PIN 28  // แขนขึ้น-ลง (ช่อง 3)
#define SERVO_SPARE0 0   // ช่องเสริม — ใช้ไม่ได้ถ้าเปิด UART0 (TX)
#define SERVO_SPARE1 1   // ช่องเสริม — ใช้ไม่ได้ถ้าเปิด UART0 (RX)

#define NUM_SERVOS 5
const uint8_t SERVO_PINS[NUM_SERVOS] = { SERVO_L_PIN, SERVO_R_PIN, SERVO_UD_PIN, SERVO_SPARE0, SERVO_SPARE1 };

Servo servoCh[NUM_SERVOS];
int servoTrim[NUM_SERVOS] = { 0, 0, 0, 0, 0 };
// กลับทิศ = เขียน 180 - มุม (ค่าเริ่มต้นตาม arm.ino ของ MyMINI_16Pro: มือขวา / แขนขึ้น-ลง กลับทิศ)
bool servoReverse[NUM_SERVOS] = { false, true, true, false, false };
int servoMinUs[NUM_SERVOS] = { 500, 500, 500, 500, 500 };
int servoMaxUs[NUM_SERVOS] = { 2500, 2500, 2500, 2500, 2500 };
int servoAngle[NUM_SERVOS] = { 90, 90, 90, 90, 90 };  // มุมล่าสุดที่สั่ง (ก่อนกลับทิศ / trim)

int servoIndex(int pin) {
  for (int i = 0; i < NUM_SERVOS; i++) {
    if (SERVO_PINS[i] == pin) return i;
  }
  return -1;
}

// ======================================================================
// [2] ตั้งค่า trim / กลับทิศ / ช่วงพัลส์
// ======================================================================

void set_ServoTrim(int pin, int trim) {
  int i = servoIndex(pin);
  if (i >= 0) servoTrim[i] = trim;
}

void S18_trim(int x) {
  set_ServoTrim(18, x);
}

void S22_trim(int x) {
  set_ServoTrim(22, x);
}

void S28_trim(int x) {
  set_ServoTrim(28, x);
}

void S0_trim(int x) {
  set_ServoTrim(0, x);
}

void S1_trim(int x) {
  set_ServoTrim(1, x);
}

void set_ServoReverse(int pin, bool reverse) {
  int i = servoIndex(pin);
  if (i >= 0) servoReverse[i] = reverse;
}

// ช่วงพัลส์ 0°/180° (500-2500 us) — ตรวจระยะหมุนสูงสุดของเซอร์โว/กลไกจริงก่อนขยาย
void set_ServoPulse(int pin, int minUs, int maxUs) {
  int i = servoIndex(pin);
  if (i < 0 || minUs < 500 || maxUs > 2500 || minUs >= maxUs) return;
  servoMinUs[i] = minUs;
  servoMaxUs[i] = maxUs;
  if (servoCh[i].attached()) servoCh[i].detach();  // attach ใหม่ด้วยช่วงใหม่ตอนสั่งครั้งถัดไป
}

// ======================================================================
// [3] สั่งเซอร์โวทีละตัว
// ======================================================================

int pos[3] = { 90, 90, 90 };  // มุมแขน { ขึ้น-ลง, ซ้าย, ขวา } ใช้กับคำสั่งหมวด [5]

// pin: 18 (ซ้าย), 22 (ขวา), 28 (ขึ้น-ลง), 0, 1 | angle 0-180
void Servo(int pin, int angle) {
  int i = servoIndex(pin);
  if (i < 0) return;
  angle = constrain(angle, 0, 180);
  servoAngle[i] = angle;
  if (pin == SERVO_UD_PIN) pos[0] = angle;
  else if (pin == SERVO_L_PIN) pos[1] = angle;
  else if (pin == SERVO_R_PIN) pos[2] = angle;

  if (!servoCh[i].attached()) servoCh[i].attach(pin, servoMinUs[i], servoMaxUs[i]);
  int out = servoReverse[i] ? 180 - angle - servoTrim[i] : angle + servoTrim[i];
  servoCh[i].write(constrain(out, 0, 180));
}

// ปล่อยเซอร์โว (หยุดส่งพัลส์ ลดกระแส/เสียงหึ่ง)
void ServoOff(int pin) {
  int i = servoIndex(pin);
  if (i >= 0 && servoCh[i].attached()) servoCh[i].detach();
}

void ServoOff() {
  for (int i = 0; i < NUM_SERVOS; i++) {
    if (servoCh[i].attached()) servoCh[i].detach();
  }
}

// ======================================================================
// [4] ควบคุมเซอร์โวผ่าน Serial
// ======================================================================

int currentServo = -1;
int currentAngle = 90;

// พิมพ์ "ขา มุม" เช่น "18 90" | พิมพ์ exit หรือกดปุ่ม START (GP2) เพื่อออก
void SerialServoControl() {
  Serial.println("Serial Servo Control Mode");
  Serial.println("Type: servo angle  (servo pin: 18, 22, 28, 0, 1) (ex: 18 90) | exit = ออก");

  while (1) {
    if (digitalRead(2) == LOW) {
      Serial.println("Exit Servo Control");
      while (digitalRead(2) == LOW)
        ;  // รอปล่อยปุ่ม
      delay(200);
      break;
    }

    if (Serial.available()) {
      String command = Serial.readStringUntil('\n');
      command.trim();
      if (command.equalsIgnoreCase("exit")) {
        Serial.println("Exit Servo Control");
        break;
      }
      int s = -1, a = -1;
      if (sscanf(command.c_str(), "%d %d", &s, &a) == 2 && servoIndex(s) >= 0 && a >= 0 && a <= 180) {
        currentServo = s;
        currentAngle = a;
        Serial.printf("Set Servo %d -> %d deg\n", currentServo, currentAngle);
      } else {
        Serial.println("รูปแบบ: ขา(18/22/28/0/1) มุม(0-180)");
      }
    }

    // เขียนค่าเดิมซ้ำเรื่อย ๆ
    if (currentServo != -1) {
      Servo(currentServo, currentAngle);
    }
    delay(20);  // ~50Hz เหมาะกับ servo
  }
}

// ======================================================================
// [5] สั่งแขนหลายตัวพร้อมกัน (Servo / armupdown / arm_left_right)
// ======================================================================

// (ขึ้น-ลง, ซ้าย, ขวา) พร้อมกันทันที
void Servo(int ud, int l, int r) {
  MotorStop();
  Servo(SERVO_UD_PIN, ud);
  Servo(SERVO_L_PIN, l);
  Servo(SERVO_R_PIN, r);
  delay(20);
}

// (ขึ้น-ลง, ซ้าย, ขวา) ค่อย ๆ ขยับพร้อมกันทีละ 1 องศา หน่วง spd ms ต่อองศา
void Servo(int target1, int target2, int target3, int spd) {
  MotorStop();
  int target[3] = { constrain(target1, 0, 180), constrain(target2, 0, 180), constrain(target3, 0, 180) };
  const int sv[3] = { SERVO_UD_PIN, SERVO_L_PIN, SERVO_R_PIN };

  while (pos[0] != target[0] || pos[1] != target[1] || pos[2] != target[2]) {
    for (int i = 0; i < 3; i++) {
      Servo(sv[i], pos[i] + (pos[i] < target[i]) - (pos[i] > target[i]));
    }
    delay(spd);
  }

  delay(20);
}

void armupdown(int x, int spd) {
  Servo(x, pos[1], pos[2], spd);
}

void arm_left_right(int l, int r, int spd) {
  Servo(pos[0], l, r, spd);
}

void armupdown(int x) {
  Servo(SERVO_UD_PIN, x);
}

void arm_left_right(int l, int r) {
  Servo(SERVO_L_PIN, l);
  Servo(SERVO_R_PIN, r);
}

#endif  // MYMINI_16PRO_SERVO_H
