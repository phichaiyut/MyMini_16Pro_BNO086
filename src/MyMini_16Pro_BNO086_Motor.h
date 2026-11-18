#ifndef MYMINI16PRO_BNO086_MOTOR_H
#define MYMINI16PRO_BNO086_MOTOR_H

/*
 * MyMini_16Pro_BNO086_Motor.h
 * ขับมอเตอร์ DRV8874 (ค่าเริ่มต้น) / TB6612FNG — เขียนใหม่จาก MotorDriver.cpp + DRV8874Driver.cpp
 * ของ MyMINI_16Pro โดยคงขา ความถี่ ความละเอียด ลำดับการเขียนขา และความหมายของค่า 0 / ±1 ไว้ทุกจุด
 *
 * PWM slice ของ RP2040/RP2350: GP19 = slice 1B, GP6 = slice 3A (ไม่ชนกัน)
 * buzzer GP9 ใช้ tone() (PIO) จึงไม่เปลี่ยนความถี่ PWM ของมอเตอร์
 *
 * สารบัญ:
 *   [1] ขามอเตอร์ / ค่าคงที่
 *   [2] ตัวแปร (Global Variables)
 *   [3] DRV8874 (PH/EN + nSLEEP)
 *   [4] TB6612FNG (PWM + IN1/IN2)
 *   [5] เริ่มต้น / เลือกไดรเวอร์
 *   [6] สั่งมอเตอร์ (Motor, Motor1, Motor2, Move, MotorStop, Stop, MotorCoast, TestMotor, MotorShot, set_Freq)
 *   [6.1] Safety timeout ลูปหาเส้น (set_line_timeout / LINE_WHILE)
 */

#include <Arduino.h>
#include "MyMini_16Pro_BNO086_Decl.h"
#include "MyMini_16Pro_BNO086_Buzzer.h"

// ======================================================================
// [1] ขามอเตอร์ / ค่าคงที่
// ======================================================================

// ---------- DRV8874 (ค่าเริ่มต้น) — PWM 15 kHz ----------
#define DRV_L_EN 19       // ล้อซ้าย EN/IN1 (PWM)
#define DRV_L_PH 20       // ล้อซ้าย PH/IN2 (ทิศ)
#define DRV_L_SLEEP 21    // ล้อซ้าย nSLEEP
#define DRV_R_EN 6        // ล้อขวา EN/IN1 (PWM)
#define DRV_R_PH 7        // ล้อขวา PH/IN2 (ทิศ)
#define DRV_R_SLEEP 8     // ล้อขวา nSLEEP
#define DRV_L_INVERT false
#define DRV_R_INVERT false
#define DRV_PWM_FREQ 16000  // MOTOR_PWM_FREQUENCY_HZ

// ตัวเลือกทดสอบ (ไม่มีในต้นฉบับ, ปิดเป็นค่าเริ่มต้น): ทุกครั้งที่ล้อเปลี่ยนจาก หยุด/เบรก → ขับ
// ให้ nSLEEP LOW 1 ms แล้ว HIGH เพื่อล้าง fault ที่ DRV8874 latch ค้างไว้ (เช่น OCP จากกระแสกระชาก)
// เปิดโดยใส่ #define DRV_FAULT_CLEAR 1 ไว้ก่อน #include <MyMini_16Pro_BNO086.h>
#ifndef DRV_FAULT_CLEAR
#define DRV_FAULT_CLEAR 0
#endif

// ---------- TB6612FNG — PWM 20 kHz (STBY ต่อ HIGH ตายตัว) ----------
// ล้อซ้ายของคำสั่ง Motor() ใช้ขาชุด RIGHT_MOTOR_* ของบอร์ด (ตามต้นฉบับ writeTb6612Both)
#define TB_L_PWM 19
#define TB_L_IN1 21
#define TB_L_IN2 20
#define TB_R_PWM 6
#define TB_R_IN1 8
#define TB_R_IN2 7
#define TB_L_INVERT false
#define TB_R_INVERT false
#define TB_PWM_FREQ 20000

#define MOTOR_PWM_BITS 12
#define MOTOR_PWM_MAX 4095
#define MOTOR_CMD_MAX 100

// ชนิดไดรเวอร์ (เลือกด้วย SetMotorDriver() ก่อน RobotSetup())
// ---------- ชื่อขาของไลบรารีต้นแบบ → ขา MyMINI_16Pro (ห้ามใช้เลขขาของบอร์ดต้นแบบ) ----------
// (PH/EN ล้อ 1 = ซ้าย, ล้อ 2 = ขวา) → DRV8874 ของ MyMINI
#define PH1 DRV_L_PH
#define EN1 DRV_L_EN
#define PH2 DRV_R_PH
#define EN2 DRV_R_EN
// MyRP_Pico2 (TB6612FNG: PWMA/AIN = ซ้าย, PWMB/BIN = ขวา) → TB6612FNG ของ MyMINI
#define PWMA TB_L_PWM
#define AIN1 TB_L_IN1
#define AIN2 TB_L_IN2
#define PWMB TB_R_PWM
#define BIN1 TB_R_IN1
#define BIN2 TB_R_IN2

#define DRIVER_DRV8874 0
#define DRIVER_TB6612 1

// ======================================================================
// [2] ตัวแปร (Global Variables)
// ======================================================================

uint8_t MotorDriverType = DRIVER_DRV8874;
bool motorReady = false;
int motorCmdL = 0, motorCmdR = 0;  // คำสั่งที่ส่งออกขาจริง (หลังแปลง 0 / ±1 แล้ว)
int motorReqL = 0, motorReqR = 0;  // คำสั่งที่ผู้ใช้ขอ (ก่อนแปลง) — ใช้กับ Motor1 / Motor2

bool drvSleepL = true, drvSleepR = true;  // DRV8874: true = nSLEEP LOW (Hi-Z)
bool drvDriveL = false, drvDriveR = false;  // DRV8874: true = กำลังขับอยู่ (ใช้กับ DRV_FAULT_CLEAR)
int8_t tbDirL = 0, tbDirR = 0;            // TB6612: 0 หยุด, 1 หน้า, -1 ถอย

// ======================================================================
// [3] DRV8874 (PH/EN + nSLEEP)
// ======================================================================

// 0 = nSLEEP LOW → ปล่อยล้อ (Hi-Z) | ±1 = ตื่น + EN 0 → เบรก | อื่น ๆ = ขับ
void drvWrite(uint8_t enPin, uint8_t phPin, uint8_t sleepPin, bool invert, int cmd, bool &asleep, bool &driving) {
  cmd = constrain(cmd, -MOTOR_CMD_MAX, MOTOR_CMD_MAX);
  bool wasDriving = driving;
  driving = !(cmd == 0 || cmd == 1 || cmd == -1);

  if (cmd == 0) {
    analogWrite(enPin, 0);
    if (!asleep) {
      digitalWrite(sleepPin, LOW);
      delay(1);  // tSLEEP สูงสุด 1 ms
      asleep = true;
    }
    return;
  }

  if (cmd == 1 || cmd == -1) {
    analogWrite(enPin, 0);
    if (asleep) {
      digitalWrite(sleepPin, HIGH);
      delay(1);  // tWAKE สูงสุด 1 ms
      asleep = false;
    }
    return;
  }

  bool forward = cmd > 0;
  if (invert) forward = !forward;
  analogWrite(enPin, 0);
#if DRV_FAULT_CLEAR
  if (!wasDriving && !asleep) {  // เบรก → ขับ: หลับ 1 ms เพื่อล้าง fault แล้วค่อยปลุก
    digitalWrite(sleepPin, LOW);
    delay(1);
    asleep = true;
  }
#else
  (void)wasDriving;
#endif
  if (asleep) {
    digitalWrite(sleepPin, HIGH);
    delay(1);
    asleep = false;
  }
  digitalWrite(phPin, forward ? HIGH : LOW);
  analogWrite(enPin, map(abs(cmd), 0, MOTOR_CMD_MAX, 0, MOTOR_PWM_MAX));
}

void drvBegin() {
  // ให้ทั้งสองบริดจ์หลับก่อนตั้งขาอื่น (PMODE / IMODE ถูก latch ตอน nSLEEP ขึ้น)
  pinMode(DRV_L_SLEEP, OUTPUT);
  digitalWrite(DRV_L_SLEEP, LOW);
  pinMode(DRV_R_SLEEP, OUTPUT);
  digitalWrite(DRV_R_SLEEP, LOW);

  pinMode(DRV_L_EN, OUTPUT);
  pinMode(DRV_L_PH, OUTPUT);
  pinMode(DRV_R_EN, OUTPUT);
  pinMode(DRV_R_PH, OUTPUT);
  digitalWrite(DRV_L_PH, LOW);
  digitalWrite(DRV_R_PH, LOW);

  analogWriteResolution(MOTOR_PWM_BITS);
  analogWriteFreq(DRV_PWM_FREQ);
  analogWrite(DRV_L_EN, 0);
  analogWrite(DRV_R_EN, 0);
  delay(1);

  drvSleepL = true;
  drvSleepR = true;
  drvWrite(DRV_L_EN, DRV_L_PH, DRV_L_SLEEP, DRV_L_INVERT, 0, drvSleepL, drvDriveL);
  drvWrite(DRV_R_EN, DRV_R_PH, DRV_R_SLEEP, DRV_R_INVERT, 0, drvSleepR, drvDriveR);
}

// ======================================================================
// [4] TB6612FNG (PWM + IN1/IN2)
// ======================================================================

// 0 = IN1/IN2 LOW (Hi-Z) | ±1 = IN1/IN2 HIGH (เบรกสั้น) — PWM ค้าง HIGH (4095) ทั้งสองกรณี
void tbWrite(uint8_t pwmPin, uint8_t in1Pin, uint8_t in2Pin, bool invert, int cmd, int8_t &dir) {
  cmd = constrain(cmd, -MOTOR_CMD_MAX, MOTOR_CMD_MAX);

  if (cmd == 0 || cmd == 1 || cmd == -1) {
    bool brake = cmd != 0;
    analogWrite(pwmPin, 0);
    digitalWrite(in1Pin, brake ? HIGH : LOW);
    digitalWrite(in2Pin, brake ? HIGH : LOW);
    analogWrite(pwmPin, MOTOR_PWM_MAX);
    dir = 0;
    return;
  }

  bool forward = cmd > 0;
  if (invert) forward = !forward;
  int8_t next = forward ? 1 : -1;
  if (next != dir) analogWrite(pwmPin, 0);  // กลับทิศ: ตัด PWM ก่อนสลับ IN
  digitalWrite(in1Pin, forward ? HIGH : LOW);
  digitalWrite(in2Pin, forward ? LOW : HIGH);
  analogWrite(pwmPin, map(abs(cmd), 0, MOTOR_CMD_MAX, 0, MOTOR_PWM_MAX));
  dir = next;
}

void tbBegin() {
  pinMode(TB_R_PWM, OUTPUT);
  pinMode(TB_R_IN1, OUTPUT);
  pinMode(TB_R_IN2, OUTPUT);
  pinMode(TB_L_PWM, OUTPUT);
  pinMode(TB_L_IN1, OUTPUT);
  pinMode(TB_L_IN2, OUTPUT);
  analogWriteResolution(MOTOR_PWM_BITS);
  analogWriteFreq(TB_PWM_FREQ);
  tbDirL = 0;
  tbDirR = 0;
}

// ======================================================================
// [5] เริ่มต้น / เลือกไดรเวอร์
// ======================================================================

void Motor(int leftspeed, int rightspeed);

// เรียกก่อน RobotSetup(): SetMotorDriver(DRIVER_TB6612) สำหรับหุ่นที่ใช้ TB6612FNG
void SetMotorDriver(uint8_t type) {
  MotorDriverType = (type == DRIVER_TB6612) ? DRIVER_TB6612 : DRIVER_DRV8874;
  motorReady = false;
}

void MotorBegin() {
  if (MotorDriverType == DRIVER_TB6612) tbBegin();
  else drvBegin();
  motorReady = true;
  Motor(0, 0);
}

// แบบ MyMini_16Pro_BNO086: "DRV8874" (ค่าเริ่มต้น) หรือ "TB6612" — เรียกหลัง RobotSetup() ได้ (เริ่มขาใหม่ทันที)
void set_MotorDriver(String type) {
  bool tb = (type == "TB6612" || type == "TB6612FNG" || type == "tb6612");
  SetMotorDriver(tb ? DRIVER_TB6612 : DRIVER_DRV8874);
  Serial.println(tb ? "Motor driver: TB6612FNG" : "Motor driver: DRV8874");
  MotorBegin();
}

// ======================================================================
// [6] สั่งมอเตอร์ (Motor, Motor1, Motor2, Move, MotorStop, MotorCoast, TestMotor)
// ======================================================================

// leftspeed / rightspeed = -100..100
// (0, 0) = ปล่อยล้อ | (1, 1) / (-1, -1) = เบรก
// คำสั่งผสม: ล้อที่เป็น 0 → เบรก (1), ค่า ±1 → ±2 (ค่าขับจริงค่าแรก)
bool robotStopped = true;  // true = ล้อทั้งสองหยุด (เบรก/ปล่อย) — ใช้ตัดสินว่าต้อง soft-start ไหม

// ---------- ชดเชยล้อ (Motor() บวก BalanceMotorLeft / Right ให้ทุกคำสั่ง) ----------
// ใส่เฉพาะคำสั่งขับจริง (|ค่า| > 1) ไม่แตะรหัสหยุด 0 / เบรก ±1 ของ MyMINI
int BalanceMotorLeft = 0, BalanceMotorRight = 0;

// ---------- ชดเชยแรงดันแบต (MyRP_Pico2: scale = (VNOM / V)^0.95 จำกัด 0.95..1.05) ----------
// แบต 3S แบบเดียวกับ MyRP (MyMINI ไฟระดับแบต 11.0–12.3 V, ห้ามออกตัว ≤ 10.85 V)
const float VMAX = 12.6f;
const float VMIN = 7.4f;
const float VNOM = 11.55f;
bool DC_Motors = true;  // true = มอเตอร์แปรงถ่าน (DC), false = Coreless — เลือกด้วย set_Freq()
float scale = 1.0f;     // ตัวคูณกำลังจาก bat_control() (1.0 = ไม่ชดเชย)

// อ่านแรงดันแบตใหม่จาก ADS1115 AIN0 (ตัวจริง readBatteryVoltage() อยู่ใน Battery.h)
inline void updateBattery() {
  readBatteryVoltage();
}

// คำนวณ scale จากแรงดันล่าสุด (ไม่อ่าน I2C ซ้ำ — ใช้ค่าจาก updateBattery() ครั้งล่าสุด)
void bat_control() {
  float voltage = getBatteryVoltage();
  if (voltage > 0.5f && voltage >= VMIN && voltage <= VMAX) {
    scale = powf(VNOM / voltage, 0.95f);
  } else {
    scale = 1.0f;
  }
  scale = constrain(scale, 0.95f, 1.05f);
}

// ใส่ BalanceMotor + scale แบตให้คำสั่งขับจริง (|cmd| > 1) แล้วจำกัด ±100 โดยไม่ตกไปเป็นรหัส 0 / ±1
int motorApplyComp(int cmd, int balance) {
  if (cmd >= -1 && cmd <= 1) return cmd;
  int sign = cmd > 0 ? 1 : -1;
  int out = (int)lroundf((cmd + balance) * scale);
  if (out * sign < 2) out = 2 * sign;  // balance ห้ามกลับทิศ / ห้ามกลายเป็นคำสั่งเบรก
  return constrain(out, -MOTOR_CMD_MAX, MOTOR_CMD_MAX);
}

void Motor(int leftspeed, int rightspeed) {
  static unsigned long lastBatCtl = 0;
  unsigned long nowMs = millis();
  if (nowMs - lastBatCtl > 70) {  // แบบ MyRP: อัปเดต scale ทุก 70 ms
    bat_control();
    lastBatCtl = nowMs;
  }
  int cl = constrain(leftspeed, -MOTOR_CMD_MAX, MOTOR_CMD_MAX);
  int cr = constrain(rightspeed, -MOTOR_CMD_MAX, MOTOR_CMD_MAX);
  motorReqL = cl;  // จำคำสั่ง "ก่อน" ชดเชย — Motor1 / Motor2 ส่งค่านี้กลับเข้ามา จะได้ไม่ชดเชยซ้ำ
  motorReqR = cr;
  robotStopped = (cl == 0 && cr == 0);
  cl = motorApplyComp(cl, BalanceMotorLeft);
  cr = motorApplyComp(cr, BalanceMotorRight);
  if (!((cl == 0 && cr == 0) || (cl == 1 && cr == 1) || (cl == -1 && cr == -1))) {
    if (cl == 0) cl = 1;
    else if (cl == 1) cl = 2;
    else if (cl == -1) cl = -2;
    if (cr == 0) cr = 1;
    else if (cr == 1) cr = 2;
    else if (cr == -1) cr = -2;
  }
  motorCmdL = cl;
  motorCmdR = cr;
  if (!motorReady) return;

  if (MotorDriverType == DRIVER_TB6612) {
    tbWrite(TB_L_PWM, TB_L_IN1, TB_L_IN2, TB_L_INVERT, cl, tbDirL);
    tbWrite(TB_R_PWM, TB_R_IN1, TB_R_IN2, TB_R_INVERT, cr, tbDirR);
  } else {
    drvWrite(DRV_L_EN, DRV_L_PH, DRV_L_SLEEP, DRV_L_INVERT, cl, drvSleepL, drvDriveL);
    drvWrite(DRV_R_EN, DRV_R_PH, DRV_R_SLEEP, DRV_R_INVERT, cr, drvSleepR, drvDriveR);
  }
}

// สั่งทีละล้อ ล้ออีกข้างคงคำสั่งเดิม (แบบ TB6612Driver::set ต้นฉบับ)
void Motor1(int Pow) {
  Motor(Pow, motorReqR);
}

void Motor2(int Pow) {
  Motor(motorReqL, Pow);
}

void Move(int leftspeed, int rightspeed, int Movedelay) {
  Motor(leftspeed, rightspeed);
  delay(Movedelay);
}

// เบรกทั้งสองล้อ = motor_stop() ต้นฉบับ = motor(1, 1)
void MotorStop() {
  Motor(1, 1);
  motorReqL = 0;  // แบบ TB6612Driver::stopAll(): คำสั่งที่จำไว้ = 0 หลังเบรก
  motorReqR = 0;
  robotStopped = true;
}

void MotorStop(int StopDelay) {
  MotorStop();
  delay(StopDelay);
}

// หยุดแล้วรอ StopDelay ms (เหมือน MotorStop(int))
void Stop(int StopDelay) {
  MotorStop();
  delay(StopDelay);
}

// ปล่อยล้อหมุนอิสระ = motor(0, 0)
void MotorCoast() {
  Motor(0, 0);
}

// ทดสอบทิศล้อ (ยกล้อลอยก่อน) — TestMotor() = ซ้าย 20% 300 ms → หยุด 0.5 s → ขวา 20% 300 ms, จำกัด ≤30%
void TestMotor(int power = 20, int ms = 300) {
  power = constrain(power, -30, 30);
  ms = constrain(ms, 0, 1000);
  Move(power, 0, ms);  // ล้อซ้าย
  MotorStop(500);
  Move(0, power, ms);  // ล้อขวา
  MotorStop();
}

// เบรกช็อตแบบดึงกลับ: สั่งล้อสวนทิศ (l, r) สั้น ๆ t ms แล้วเบรก
// จำกัดเวลาไม่เกิน 50 ms + cooldown กันเรียกรัว เพื่อไม่ให้กระแสค้างจนไดรเวอร์ร้อน
void MotorShot(int l, int r, int t) {
  static unsigned long lastShot = 0;
  unsigned long now = millis();
  if (now - lastShot < 50) {
    MotorStop();
    return;
  }
  lastShot = now;
  Motor(l, r);
  delay(constrain(t, 1, 50));
  MotorStop();
}

// MyRP_Pico2: Active short-brake t ms (1..15) แรง power % แล้วกลับเป็น MotorStop()
// TB6612FNG: IN1 = IN2 = HIGH + PWM = power (short brake แบบ MyRP)
// DRV8874  : EN = 0 ขณะตื่น = เบรก low-side เต็มแรง (ชิปปรับแรงเบรกไม่ได้ จึงไม่ใช้ power)
void MotorShot(int t = 3, int power = 90) {
  static unsigned long lastShot = 0;
  unsigned long now = millis();
  if (now - lastShot < 50) {  // cooldown กันเบรกซ้อนถี่เกินไป
    MotorStop();
    return;
  }
  lastShot = now;
  if (MotorDriverType == DRIVER_TB6612 && motorReady) {
    int pwm = map(constrain(power, 0, 100), 0, 100, 0, MOTOR_PWM_MAX);
    pwm = constrain((int)(pwm * scale), 0, 3850);  // Safety limit เดียวกับ MyRP
    digitalWrite(TB_L_IN1, HIGH);
    digitalWrite(TB_L_IN2, HIGH);
    digitalWrite(TB_R_IN1, HIGH);
    digitalWrite(TB_R_IN2, HIGH);
    analogWrite(TB_L_PWM, pwm);
    analogWrite(TB_R_PWM, pwm);
    tbDirL = 0;
    tbDirR = 0;
  } else {
    Motor(1, 1);
  }
  delay(constrain(t, 1, 15));  // จำกัดเวลาเบรกสูงสุด กันค้างที่กระแสสูง
  MotorStop();
}

// MyRP_Pico2: เลือกชนิดมอเตอร์ "Coreless_Motors" / "coreless" = PWM 20 kHz, อื่น ๆ = DC 1 kHz
// (servo ของ MyMINI ใช้ Servo library บน PIO และ buzzer ใช้ tone() จึงไม่โดนผลจากการเปลี่ยนความถี่)
void set_Freq(const String &type) {
  if (type == "Coreless_Motors" || type == "coreless") {
    DC_Motors = false;
    analogWriteFreq(20000);
    Serial.println("Motor: Coreless_Motors (20kHz)");
  } else {
    DC_Motors = true;
    analogWriteFreq(1000);
    Serial.println("Motor: DC_Motors (1kHz)");
  }
}

// ======================================================================
// [6.1] Safety timeout ของลูปวิ่งหาเส้น / เซนเซอร์ (LINE_WHILE) — จาก MyRP_Pico2
// ======================================================================

// set_line_timeout(ms [, halt]) | ms = 0 ปิด (ค่าเริ่มต้น = ลูปไม่จำกัดเวลาแบบเดิม)
// halt = true: หมดเวลาแล้วหยุดมอเตอร์ + ค้าง (กระพริบ LED + เสียง) จนกด reset
// halt = false: หมดเวลาแล้วหยุดมอเตอร์ ออกจากลูป ทำคำสั่งถัดไปต่อ
uint32_t line_timeout_ms = 0;
bool line_timeout_halt = true;
uint32_t line_timeout_count = 0;  // จำนวนครั้งที่ timeout ทำงาน (ไว้ debug)

void set_line_timeout(uint32_t ms, bool halt = true) {
  line_timeout_ms = ms;
  line_timeout_halt = halt;
}

struct LineGuard {
  uint32_t t0;
};

LineGuard lineGuardStart() {
  LineGuard g;
  g.t0 = millis();
  return g;
}

// คืน true = หมดเวลา (หยุดมอเตอร์ให้แล้ว) | โหมด halt จะไม่คืนค่า (ค้างจนกด reset)
bool lineGuardExpired(LineGuard &g) {
  if (line_timeout_ms == 0 || millis() - g.t0 <= line_timeout_ms) return false;
  MotorStop();
  BZoff();
  line_timeout_count++;
  Serial.println("Line timeout: line / sensor not found -> motor stop");
  if (line_timeout_halt) {
    Serial.println("Robot halted. Press reset.");
    pinMode(LED_BUILTIN, OUTPUT);
    while (1) {
      MotorStop();
      digitalWrite(LED_BUILTIN, HIGH);
      Beep(200);
      digitalWrite(LED_BUILTIN, LOW);
      delay(800);
    }
  }
  Beep(100);
  return true;
}

// ใช้แทน while (1) ในลูปวิ่งหาเส้น: break / continue ใช้ได้เหมือนเดิม
#define LINE_GUARD_CAT2(a, b) a##b
#define LINE_GUARD_CAT(a, b) LINE_GUARD_CAT2(a, b)
#define LINE_GUARD_VAR LINE_GUARD_CAT(lg_, __LINE__)
#define LINE_WHILE for (LineGuard LINE_GUARD_VAR = lineGuardStart(); !lineGuardExpired(LINE_GUARD_VAR);)

// ======================================================================
// [7] Soft-start ตอนออกตัว (จาก MyMini_16Pro_BNO086)
// ======================================================================

// ใช้ใน InitialSpeed / MoveSoft / PIDF / PIDB — set_MotorSoftStart(0) = ปิด
#define SOFT_START_MIN_SPEED 40
int softStartMs = 200;
bool softStartActive = false;
unsigned long softStartBegin = 0;
int softStartBase = 100;

// ตั้งเวลา soft-start (ms) ตอนออกตัวด้วยความเร็ว > 40 จากหยุดนิ่ง | 0 = ปิด
void set_MotorSoftStart(int ms) {
  softStartMs = max(ms, 0);
  if (softStartMs == 0) softStartActive = false;
}

void beginSoftStart(int baseSpeed) {
  softStartBase = abs(baseSpeed);
  softStartActive = robotStopped && softStartMs > 0 && softStartBase > SOFT_START_MIN_SPEED;
  softStartBegin = millis();
}

// ตัวคูณความเร็วช่วงออกตัว: 40/Base → 1.0 แบบ smoothstep ภายใน softStartMs
float softStartFactor() {
  if (!softStartActive) return 1.0f;
  float x = (millis() - softStartBegin) / (float)softStartMs;
  if (x >= 1.0f) {
    softStartActive = false;
    return 1.0f;
  }
  float f0 = (float)SOFT_START_MIN_SPEED / softStartBase;
  return f0 + (1.0f - f0) * (x * x * (3.0f - 2.0f * x));
}

// ======================================================================
// [8] ตารางความเร็ว / PID / เวลาเบรก ตามช่วงความเร็ว (จาก MyMini_16Pro_BNO086)
// ======================================================================

int BaseSpeed, LeftBaseSpeed, RightBaseSpeed, BackLeftBaseSpeed, BackRightBaseSpeed;
float PID_KP_Front, PID_KD_Front;
float PID_KP_Back, PID_KD_Back;
int delay_break_f = 30;
int delay_break_b = 30;
int L[10], R[10];
int BL[10], BR[10];
float KP[10], KD[10];
float KP_Back[10], KD_Back[10];
int delay_break_f_table[10] = { 30, 30, 30, 30, 30, 30, 30, 30, 30, 30 };
int delay_break_b_table[10] = { 30, 30, 30, 30, 30, 30, 30, 30, 30, 30 };

#define SPD_10 0
#define SPD_20 1
#define SPD_30 2
#define SPD_40 3
#define SPD_50 4
#define SPD_60 5
#define SPD_70 6
#define SPD_80 7
#define SPD_90 8
#define SPD_100 9

// แปลง ch เป็นดัชนีตาราง 0-9 (MyRP_Pico2): รับ SPD_10..SPD_100 (0-9) และเลขความเร็ว 10, 20, ..., 100
// ค่าอื่นคืน -1 = ไม่บันทึก (กันเขียนเกินขอบอาร์เรย์)
static int speedTableIndex(int ch) {
  if (ch >= SPD_10 && ch <= SPD_100) return ch;
  if (ch >= 10 && ch <= 100 && ch % 10 == 0) return ch / 10 - 1;
  Serial.print("Speed table: invalid index ");
  Serial.println(ch);
  return -1;
}

void setBalanceSpeed(int ch, int spdL, int spdR) {
  int i = speedTableIndex(ch);
  if (i < 0) return;
  L[i] = spdL;
  R[i] = spdR;
}

void setBalanceBackSpeed(int ch, int spdL, int spdR) {
  int i = speedTableIndex(ch);
  if (i < 0) return;
  BL[i] = spdL;
  BR[i] = spdR;
}

void Set_KP_KD(int ch, float kp, float kd) {
  int i = speedTableIndex(ch);
  if (i < 0) return;
  KP[i] = kp;
  KD[i] = kd;
}

void Set_KP_KD_Back(int ch, float kp, float kd) {
  int i = speedTableIndex(ch);
  if (i < 0) return;
  KP_Back[i] = kp;
  KD_Back[i] = kd;
}

void SetDelayBreak(int ch, int delay_f, int delay_b) {
  int i = speedTableIndex(ch);
  if (i < 0) return;
  delay_break_f_table[i] = delay_f;
  delay_break_b_table[i] = delay_b;
}

int LastError_F, LastError_B;            // error ก่อนหน้าของ PIDF / PIDB
int LastError_F_none, LastError_B_none;  // error ก่อนหน้าของ PID แบบ _none
float FilteredD_F = 0, FilteredD_B = 0;  // สถานะ D-term ของ PIDF / PIDB (ใช้ใน PID.h)
bool PidFreshF = true, PidFreshB = true;
void resetGyroRunError();                // อยู่ใน Gyro.h

// เลือกชุดค่าความเร็ว/PID ตามช่วงของ BaseSpeed (ปัดขึ้นเป็นสิบ เช่น 35 ใช้ชุดของ 40)
void InitialSpeed() {
  int idx = constrain((BaseSpeed - 1) / 10, SPD_10, SPD_100);
  LeftBaseSpeed = BaseSpeed - L[idx];
  RightBaseSpeed = BaseSpeed - R[idx];
  BackLeftBaseSpeed = BaseSpeed - BL[idx];
  BackRightBaseSpeed = BaseSpeed - BR[idx];
  PID_KP_Front = KP[idx];
  PID_KD_Front = KD[idx];
  PID_KP_Back = KP_Back[idx];
  PID_KD_Back = KD_Back[idx];
  delay_break_f = delay_break_f_table[idx];
  delay_break_b = delay_break_b_table[idx];
  LastError_F = 0;
  LastError_B = 0;
  LastError_F_none = 0;
  LastError_B_none = 0;
  FilteredD_F = 0;
  FilteredD_B = 0;
  PidFreshF = true;
  PidFreshB = true;
  resetGyroRunError();
  beginSoftStart(BaseSpeed);
}

// Move แบบมี soft-start: ไต่ความเร็วตาม softStartFactor() แล้วค้างไว้จนครบ t ms
void MoveSoft(int l, int r, int t) {
  unsigned long start = millis();
  while (softStartActive && millis() - start < (unsigned long)t) {
    float k = softStartFactor();
    Motor(lroundf(l * k), lroundf(r * k));
    delay(1);
  }
  unsigned long used = millis() - start;
  Motor(l, r);
  if (used < (unsigned long)t) delay(t - used);
}

// ======================================================================
// [9] เคลื่อนที่พื้นฐาน (fd, bk, sl, sr, tl, tr)
// ======================================================================

void fd(int Speed, int time_ms) {
  BaseSpeed = Speed;
  InitialSpeed();
  MoveSoft(LeftBaseSpeed, RightBaseSpeed, time_ms);
}

void bk(int Speed, int time_ms) {
  BaseSpeed = Speed;
  InitialSpeed();
  MoveSoft(-BackLeftBaseSpeed, -BackRightBaseSpeed, time_ms);
}

void sl(int Speed, int time_ms) {
  MotorStop(0);
  Move(-Speed, Speed, time_ms);
  MotorStop(0);
}

void sr(int Speed, int time_ms) {
  MotorStop(0);
  Move(Speed, -Speed, time_ms);
  MotorStop(0);
}

void tl(int Speed, int time_ms) {
  MotorStop(0);
  Move(0, Speed, time_ms);
  MotorStop(0);
}

void tr(int Speed, int time_ms) {
  MotorStop(0);
  Move(Speed, 0, time_ms);
  MotorStop(0);
}

#endif  // MYMINI16PRO_BNO086_MOTOR_H
