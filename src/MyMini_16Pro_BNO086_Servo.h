#ifndef MYMINI16PRO_BNO086_SERVO_H
#define MYMINI16PRO_BNO086_SERVO_H

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
 *   [6] API ต้นแบบ: MyRP (SetServoPulse, S0/S1/S10/S28_pulse, S10_trim)
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

// MyRP_Pico2 ใช้ขา GP10 เป็นเซอร์โวแขน (ตำแหน่งกลางของ Servo(x, y, z)) — บน MyMINI GP10 เป็นขาเลือก MUX หลัง
// จึงแปลงเลข 10 ไปที่ช่องแขนซ้าย GP18 (ตำแหน่งเดียวกันใน Servo(x, y, z) / arm_left_right) | ขาอื่นคงเดิม
int servoPinAlias(int pin) {
  return (pin == 10) ? SERVO_L_PIN : pin;
}

int servoIndex(int pin) {
  pin = servoPinAlias(pin);
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

// MyRP: S10_trim = เซอร์โว GP10 → MyMINI GP18
void S10_trim(int x) {
  set_ServoTrim(10, x);
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

// ช่วงมุม (ก่อนกลับทิศ) ที่เขียนได้จริงหลังบวก trim — สูตรเดียวกันทั้งเซอร์โวปกติและกลับทิศ:
//   ปกติ   out = a + t        ∈ [0,180] → a ∈ [-t, 180-t]
//   กลับทิศ out = 180 - (a + t) ∈ [0,180] → a ∈ [-t, 180-t]
// ตัดมุมก่อน trim เพื่อให้ pos[] ตรงกับมุมจริง (เดิมตัดหลัง trim → ข้างหนึ่งชนเพดานแต่ pos[] ไม่รู้)
int servoClampAngle(int i, int angle) {
  int lo = (servoTrim[i] < 0) ? -servoTrim[i] : 0;
  int hi = (servoTrim[i] > 0) ? 180 - servoTrim[i] : 180;
  if (lo > hi) lo = hi = 90;  // trim เกิน ±180 (ค่าผิด)
  return constrain(angle, lo, hi);
}

// pin: 18 (ซ้าย), 22 (ขวา), 28 (ขึ้น-ลง), 0, 1 | angle 0-180
void Servo(int pin, int angle) {
  pin = servoPinAlias(pin);
  int i = servoIndex(pin);
  if (i < 0) return;
  angle = servoClampAngle(i, angle);
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

// มุมที่สเต็ป k จาก n (Linear Interpolation ปัดเศษแบบสมมาตร) — k = n ได้ start + delta พอดี
int servoLerp(int start, int delta, int k, int n) {
  long num = (long)delta * k * 2;
  return start + (int)((num + (num >= 0 ? n : -n)) / (2L * n));
}

// (ขึ้น-ลง, ซ้าย, ขวา) ค่อย ๆ ขยับพร้อมกัน — ทุกตัวเริ่มพร้อมกันและถึงเป้าหมายพร้อมกัน
// จำนวนสเต็ป = ระยะที่ไกลที่สุด (ตัวที่ไกลสุดขยับ 1 องศา/สเต็ป ตัวอื่นขยับตามสัดส่วนระยะทาง)
// หน่วง spd ms ต่อสเต็ป | spd <= 0 = สั่งทันที
void Servo(int target1, int target2, int target3, int spd) {
  MotorStop();
  const int sv[3] = { SERVO_UD_PIN, SERVO_L_PIN, SERVO_R_PIN };
  const int in[3] = { target1, target2, target3 };
  int start[3], target[3], delta[3];
  int n = 0;
  for (int a = 0; a < 3; a++) {
    start[a] = pos[a];
    target[a] = servoClampAngle(servoIndex(sv[a]), in[a]);  // ตัดขอบเขตแบบเดียวกับ Servo(pin, angle)
    delta[a] = target[a] - start[a];
    int d = abs(delta[a]);
    if (d > n) n = d;
  }

  if (n == 0) {
    delay(20);
    return;
  }
  if (spd <= 0) {
    for (int a = 0; a < 3; a++) Servo(sv[a], target[a]);
    delay(20);
    return;
  }

  for (int k = 1; k <= n; k++) {
    // เขียน ขึ้น-ลง / ซ้าย / ขวา ติดกันในสเต็ปเดียว ไม่มี delay คั่นระหว่าง S18 กับ S22
    for (int a = 0; a < 3; a++) Servo(sv[a], servoLerp(start[a], delta[a], k, n));
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

// ======================================================================
// [6] คำสั่งเซอร์โวแบบต้นฉบับ MyMINI_16Pro (servo / set_servo_pulse_range)
// ======================================================================

// ต่างจาก Servo() ด้านบน: ไม่มี trim / กลับทิศ — เขียนมุมตรง ๆ แบบต้นฉบับ
// ⚠ ใช้คำสั่งชุดเดียวต่อขา (servo() หรือ Servo()/arm_*) — ทั้งสองใช้อ็อบเจกต์เดียวกัน ช่วงพัลส์ถูกกำหนดตอน attach ครั้งแรก
//   เช่น servo(22, ..) ก่อน → ขา 22 ค้างที่ 1000–2000 us แม้เรียก Servo(22, ..) ทีหลัง
// ยังไม่ตั้งช่วงพัลส์ → attach(pin) ค่าเริ่มต้นของ core (1000–2000 us) เหมือนต้นฉบับ
bool servoRangeSet[NUM_SERVOS] = { false, false, false, false, false };

// ตั้งช่วงพัลส์ 0°/180° ก่อนสั่ง servo() ครั้งแรกของขานั้น (500–2500 us)
// คืน false ถ้าขาไม่รองรับ / ค่าผิด / ขานั้น attach ไปแล้ว
bool set_servo_pulse_range(uint8_t pin, uint16_t minPulseUs, uint16_t maxPulseUs) {
  int i = servoIndex(pin);
  if (i < 0 || minPulseUs < 500 || maxPulseUs > 2500 || minPulseUs >= maxPulseUs) return false;
  if (servoCh[i].attached()) return false;
  servoMinUs[i] = minPulseUs;
  servoMaxUs[i] = maxPulseUs;
  servoRangeSet[i] = true;
  return true;
}

// pin: 18, 22, 28, 0, 1 | angle 0–180
void servo(uint8_t pin, int angle) {
  int i = servoIndex(pin);
  if (i < 0) return;
  if (!servoCh[i].attached()) {
    int ok = servoRangeSet[i] ? servoCh[i].attach(pin, servoMinUs[i], servoMaxUs[i]) : servoCh[i].attach(pin);
    if (ok < 0) return;
  }
  servoCh[i].write(constrain(angle, 0, 180));
}


// ======================================================================
// [6] API ตามไลบรารีต้นแบบ (MyRP_Pico2: SetServoPulse / Sx_pulse)
// ======================================================================

// ---------- MyRP: ช่วงพัลส์ต่อขา (servo = 0, 1, 10, 28 / ขา MyMINI 18, 22, 28, 0, 1) ----------
// ค่าไม่ถูกต้อง (min >= max) ไม่เปลี่ยน | detach ให้ attach ใหม่ด้วยช่วงใหม่ตอนสั่งครั้งถัดไป
void SetServoPulse(int servo, int minUs, int maxUs) {
  if (minUs >= maxUs) return;
  int i = servoIndex(servo);
  if (i < 0) return;
  servoMinUs[i] = minUs;
  servoMaxUs[i] = maxUs;
  servoRangeSet[i] = true;
  if (servoCh[i].attached()) servoCh[i].detach();
}

void S0_pulse(int minUs, int maxUs) {
  SetServoPulse(0, minUs, maxUs);
}

void S1_pulse(int minUs, int maxUs) {
  SetServoPulse(1, minUs, maxUs);
}

void S10_pulse(int minUs, int maxUs) {
  SetServoPulse(10, minUs, maxUs);
}

void S28_pulse(int minUs, int maxUs) {
  SetServoPulse(28, minUs, maxUs);
}

// ---------- ชื่อของ MyRP_Pico2 (ขา 0 / 1 / 10 / 28) → ช่อง MyMINI ----------
#define Servo0 SERVO_SPARE0
#define Servo1 SERVO_SPARE1
#define Servo10 SERVO_L_PIN  // MyRP GP10 → MyMINI GP18 (GP10 ของ MyMINI เป็นขา MUX)
#define Servo28 SERVO_UD_PIN
class Servo &Servo_0 = servoCh[3];
class Servo &Servo_1 = servoCh[4];
class Servo &Servo_10 = servoCh[0];
class Servo &Servo_28 = servoCh[2];
int &Servo_tim0 = servoTrim[3];
int &Servo_tim1 = servoTrim[4];
int &Servo_tim10 = servoTrim[0];
int &Servo_tim28 = servoTrim[2];
int &Servo_min0 = servoMinUs[3], &Servo_max0 = servoMaxUs[3];
int &Servo_min1 = servoMinUs[4], &Servo_max1 = servoMaxUs[4];
int &Servo_min10 = servoMinUs[0], &Servo_max10 = servoMaxUs[0];
int &Servo_min28 = servoMinUs[2], &Servo_max28 = servoMaxUs[2];

#endif  // MYMINI16PRO_BNO086_SERVO_H
