#ifndef MYMINI_16PRO_MOTOR_H
#define MYMINI_16PRO_MOTOR_H

/*
 * MyMini_16Pro_BNO086_Motor.h
 * ขับมอเตอร์ (DRV8874 / TB6612FNG) ชดเชยแบตเตอรี่ และตารางสมดุลความเร็ว
 *
 * สารบัญ:
 *   [1] ขามอเตอร์ / เลือกไดรเวอร์ / ความถี่ PWM
 *   [2] ชดเชยแรงดันแบตเตอรี่
 *   [3] เขียนขาไดรเวอร์ทีละล้อ (DRV8874 / TB6612FNG)
 *   [4] ขับมอเตอร์ (Motor / Move / MotorStop / MotorCoast / MotorShot)
 *   [5] ตารางความเร็ว / PID / เวลาเบรก ตามช่วงความเร็ว
 *   [6] เคลื่อนที่พื้นฐาน (fd, bk, sl, sr, tl, tr)
 */

#include "MyMini_16Pro_BNO086_Battery.h"

// ======================================================================
// [1] ขามอเตอร์ / เลือกไดรเวอร์ / ความถี่ PWM
// ======================================================================

// DRV8874 (ค่าเริ่มต้น) ต่อแบบ PH/EN — ขา PMODE ต้องต่อ GND ที่บอร์ด
// nSLEEP แยกขาของแต่ละล้อ: LOW = ปล่อยล้อ (Hi-Z), HIGH + EN=0 = เบรก
#define DRV_L_EN 19
#define DRV_L_PH 20
#define DRV_L_SLEEP 21
#define DRV_R_EN 6
#define DRV_R_PH 7
#define DRV_R_SLEEP 8

// TB6612FNG (ใช้ขาชุดเดียวกัน) — STBY ต่อ HIGH ไว้ที่บอร์ด
#define TB_L_PWM 19
#define TB_L_IN1 21
#define TB_L_IN2 20
#define TB_R_PWM 6
#define TB_R_IN1 8
#define TB_R_IN2 7

#define MOTOR_DRV8874 0
#define MOTOR_TB6612 1
#define PWM_MAX 4095

#define WHEEL_COAST 0
#define WHEEL_BRAKE 1
#define WHEEL_FWD 2
#define WHEEL_REV 3

int MotorDriverType = MOTOR_DRV8874;
bool DC_Motors = false;     // มอเตอร์ของหุ่นนี้เป็น coreless 12.4 V
long motorPwmFreq = 15000;  // DRV8874 = 15 kHz (ค่าเดิมของ MyMINI_16Pro)

// กลับทิศล้อ — แก้หลังทดสอบยกล้อลอยแล้วเท่านั้น
bool MOTOR_L_INVERT = false;
bool MOTOR_R_INVERT = false;

// สถานะภายในของไดรเวอร์
bool drvAsleepL = true, drvAsleepR = true;
int tbModeL = WHEEL_COAST, tbModeR = WHEEL_COAST;

void MotorCoast();

void MotorBegin() {
  analogWriteResolution(12);
  analogWriteFreq(motorPwmFreq);

  if (MotorDriverType == MOTOR_DRV8874) {
    // ให้ไดรเวอร์หลับไว้ก่อนตั้งขาอื่น (PMODE / IMODE ถูกอ่านตอน nSLEEP ขึ้น HIGH)
    pinMode(DRV_L_SLEEP, OUTPUT);
    pinMode(DRV_R_SLEEP, OUTPUT);
    digitalWrite(DRV_L_SLEEP, LOW);
    digitalWrite(DRV_R_SLEEP, LOW);
    pinMode(DRV_L_PH, OUTPUT);
    pinMode(DRV_R_PH, OUTPUT);
    digitalWrite(DRV_L_PH, LOW);
    digitalWrite(DRV_R_PH, LOW);
    pinMode(DRV_L_EN, OUTPUT);
    pinMode(DRV_R_EN, OUTPUT);
    analogWrite(DRV_L_EN, 0);
    analogWrite(DRV_R_EN, 0);
    drvAsleepL = true;
    drvAsleepR = true;
    delay(1);  // tSLEEP สูงสุด 1 ms
  } else {
    pinMode(TB_L_PWM, OUTPUT);
    pinMode(TB_L_IN1, OUTPUT);
    pinMode(TB_L_IN2, OUTPUT);
    pinMode(TB_R_PWM, OUTPUT);
    pinMode(TB_R_IN1, OUTPUT);
    pinMode(TB_R_IN2, OUTPUT);
    tbModeL = WHEEL_COAST;
    tbModeR = WHEEL_COAST;
  }
  MotorCoast();
}

// เลือกไดรเวอร์: "DRV8874" (ค่าเริ่มต้น) หรือ "TB6612" — เรียกใน Setting() ได้เลย
// ตั้งความถี่ PWM ตามไดรเวอร์ (ค่าเดิมของ MyMINI_16Pro): TB6612 = 20 kHz, DRV8874 = 15 kHz
// set_Freq("DC_Motors") = 1 kHz ให้เรียกหลังคำสั่งนี้
void set_MotorDriver(String type) {
  if (type == "TB6612" || type == "TB6612FNG" || type == "tb6612") {
    MotorDriverType = MOTOR_TB6612;
    motorPwmFreq = 20000;
    Serial.println("Motor driver: TB6612FNG");
  } else {
    MotorDriverType = MOTOR_DRV8874;
    motorPwmFreq = 15000;
    Serial.println("Motor driver: DRV8874");
  }
  MotorBegin();
}

void set_Freq(String type) {
  if (type == "DC_Motors" || type == "dc") {
    DC_Motors = true;
    motorPwmFreq = 1000;
    Serial.println("Motor: DC_Motors (1kHz)");
  } else {
    DC_Motors = false;
    motorPwmFreq = MotorDriverType == MOTOR_TB6612 ? 20000 : 15000;
    Serial.printf("Motor: Coreless_Motors (%ldkHz)\n", motorPwmFreq / 1000);
  }
  analogWriteFreq(motorPwmFreq);
}

void set_MotorInvert(bool left, bool right) {
  MOTOR_L_INVERT = left;
  MOTOR_R_INVERT = right;
}

// ======================================================================
// [2] ชดเชยแรงดันแบตเตอรี่
// ======================================================================

const float VMAX = 12.6f;
const float VMIN = 9.0f;
const float VNOM = 11.8f;
bool batCompensate = true;
float scale = 1.0f;

void set_BatteryCompensate(bool enable) {
  batCompensate = enable;
  if (!enable) scale = 1.0f;
}

void bat_control() {
  updateBattery();
  float voltage = getBatteryVoltage();

  if (batCompensate && voltage >= VMIN && voltage <= VMAX) {
    scale = pow(VNOM / voltage, 0.95f);
  } else {
    scale = 1.0f;
  }

  // จำกัดช่วงชดเชยไม่ให้กระแสพุ่ง (มอเตอร์ coreless กระแสล็อกล้อ ~5 A)
  scale = constrain(scale, 0.95f, 1.05f);
}

// ======================================================================
// [3] เขียนขาไดรเวอร์ทีละล้อ (DRV8874 / TB6612FNG)
// ======================================================================

void writeWheelDRV(uint8_t en, uint8_t ph, uint8_t slp, bool &asleep, int mode, int pwm) {
  if (mode == WHEEL_COAST) {
    analogWrite(en, 0);
    if (!asleep) {
      digitalWrite(slp, LOW);  // nSLEEP LOW = ทั้งสองขาออก Hi-Z
      asleep = true;
    }
    return;
  }
  if (asleep) {
    analogWrite(en, 0);
    digitalWrite(slp, HIGH);
    delay(1);  // tWAKE สูงสุด 1 ms ก่อนสั่ง EN
    asleep = false;
  }
  if (mode == WHEEL_BRAKE) {
    analogWrite(en, 0);  // ตื่นอยู่ + EN = 0 → เบรก (low-side)
    return;
  }
  digitalWrite(ph, mode == WHEEL_FWD ? HIGH : LOW);
  analogWrite(en, pwm);
}

void writeWheelTB(uint8_t pwmPin, uint8_t in1, uint8_t in2, int &lastMode, int mode, int pwm) {
  if (mode == WHEEL_COAST || mode == WHEEL_BRAKE) {
    // truth table: IN1/IN2 = H/H → เบรก, L/L → ปล่อย (ให้ PWM ค้าง HIGH เสมอ)
    uint8_t level = mode == WHEEL_BRAKE ? HIGH : LOW;
    analogWrite(pwmPin, 0);
    digitalWrite(in1, level);
    digitalWrite(in2, level);
    analogWrite(pwmPin, PWM_MAX);
    lastMode = mode;
    return;
  }
  if (mode != lastMode) analogWrite(pwmPin, 0);  // กลับทิศ: ตัด PWM ก่อนสลับ IN
  digitalWrite(in1, mode == WHEEL_FWD ? HIGH : LOW);
  digitalWrite(in2, mode == WHEEL_FWD ? LOW : HIGH);
  analogWrite(pwmPin, pwm);
  lastMode = mode;
}

int invertMode(int mode, bool invert) {
  if (!invert) return mode;
  if (mode == WHEEL_FWD) return WHEEL_REV;
  if (mode == WHEEL_REV) return WHEEL_FWD;
  return mode;
}

void setWheelL(int mode, int pwm) {
  mode = invertMode(mode, MOTOR_L_INVERT);
  if (MotorDriverType == MOTOR_DRV8874) writeWheelDRV(DRV_L_EN, DRV_L_PH, DRV_L_SLEEP, drvAsleepL, mode, pwm);
  else writeWheelTB(TB_L_PWM, TB_L_IN1, TB_L_IN2, tbModeL, mode, pwm);
}

void setWheelR(int mode, int pwm) {
  mode = invertMode(mode, MOTOR_R_INVERT);
  if (MotorDriverType == MOTOR_DRV8874) writeWheelDRV(DRV_R_EN, DRV_R_PH, DRV_R_SLEEP, drvAsleepR, mode, pwm);
  else writeWheelTB(TB_R_PWM, TB_R_IN1, TB_R_IN2, tbModeR, mode, pwm);
}

// ======================================================================
// [4] ขับมอเตอร์ (Motor / Move / MotorStop / MotorCoast / MotorShot)
// ======================================================================

int speedToPwm(int spd) {
  int pwm = map(abs(spd), 0, 100, 0, PWM_MAX);
  return constrain((int)(pwm * scale), 0, PWM_MAX);
}

// left / right = -100..100
// (0, 0) = ปล่อยล้อทั้งสอง | ล้อที่เป็น 0 ขณะอีกล้อหมุน = เบรกล้อนั้น (ให้เลี้ยวล้อเดียวได้คม)
void Motor(int left, int right) {
  static unsigned long lastBatUpdate = 0;
  unsigned long now = millis();
  if (now - lastBatUpdate > 70) {
    bat_control();
    lastBatUpdate = now;
  }

  left = constrain(left, -100, 100);
  right = constrain(right, -100, 100);
  int zeroMode = (left == 0 && right == 0) ? WHEEL_COAST : WHEEL_BRAKE;

  int modeL = left > 0 ? WHEEL_FWD : (left < 0 ? WHEEL_REV : zeroMode);
  int modeR = right > 0 ? WHEEL_FWD : (right < 0 ? WHEEL_REV : zeroMode);
  setWheelL(modeL, speedToPwm(left));
  setWheelR(modeR, speedToPwm(right));
}

void Move(int l, int r, int t) {
  Motor(l, r);
  delay(t);
}

// เบรกทั้งสองล้อ (หยุดทันที)
void MotorStop() {
  setWheelL(WHEEL_BRAKE, 0);
  setWheelR(WHEEL_BRAKE, 0);
}

// เบรกค้างไว้ t ms
void MotorStop(int t) {
  MotorStop();
  delay(t);
}

// ปล่อยล้อหมุนอิสระ (ไม่เบรก)
void MotorCoast() {
  setWheelL(WHEEL_COAST, 0);
  setWheelR(WHEEL_COAST, 0);
}

// เบรกช็อตแบบดึงกลับ: สั่งล้อสวนทิศ (l, r) สั้น ๆ t ms แล้วเบรก
// จำกัดเวลาไม่เกิน 50 ms + cooldown กันเรียกรัว เพื่อไม่ให้กระแสค้างจนไดรเวอร์ร้อน
// ตัวอย่าง: วิ่งหน้า 60 แล้ว MotorShot(-30, -30, 10);
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

// ======================================================================
// [5] ตารางความเร็ว / PID / เวลาเบรก ตามช่วงความเร็ว
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

// ดัชนีตารางความเร็ว (ใช้กับ setBalanceSpeed / setBalanceBackSpeed)
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

void setBalanceSpeed(int ch, int spdL, int spdR) {
  L[ch] = spdL;
  R[ch] = spdR;
}

void setBalanceBackSpeed(int ch, int spdL, int spdR) {
  BL[ch] = spdL;
  BR[ch] = spdR;
}

void Set_KP_KD(int ch, float kp, float kd) {
  KP[ch] = kp;
  KD[ch] = kd;
}

void Set_KP_KD_Back(int ch, float kp, float kd) {
  KP_Back[ch] = kp;
  KD_Back[ch] = kd;
}

void SetDelayBreak(int ch, int delay_f, int delay_b) {
  delay_break_f_table[ch] = delay_f;
  delay_break_b_table[ch] = delay_b;
}

int LastError_F, LastError_B;            // ค่า error ก่อนหน้าของ PIDF / PIDB
int LastError_F_none, LastError_B_none;  // ค่า error ก่อนหน้าของ PID แบบ _none
void resetGyroRunError();                // อยู่ใน Gyro.h: รีเซ็ต previous_errorG / previous_errorGB

// เลือกชุดค่าความเร็ว/PID ตามช่วงของ BaseSpeed (ปัดขึ้นเป็นสิบ เช่น 35 ใช้ชุดของ 40)
void InitialSpeed() {
  int idx = constrain((BaseSpeed - 1) / 10, SPD_10, SPD_100);

  LeftBaseSpeed = BaseSpeed - L[idx];
  RightBaseSpeed = BaseSpeed - R[idx];
  BackLeftBaseSpeed = BaseSpeed - BL[idx];
  BackRightBaseSpeed = BaseSpeed - BR[idx];
  PID_KP_Front = KP[idx];  // forward PID
  PID_KD_Front = KD[idx];
  PID_KP_Back = KP_Back[idx];  // backward PID
  PID_KD_Back = KD_Back[idx];
  delay_break_f = delay_break_f_table[idx];
  delay_break_b = delay_break_b_table[idx];
  LastError_F = 0;  // รีเซ็ต D-term ไม่ให้ค้างจากคำสั่งก่อนหน้า
  LastError_B = 0;
  LastError_F_none = 0;
  LastError_B_none = 0;
  resetGyroRunError();  // previous_errorG / previous_errorGB = error ปัจจุบัน
}

// ======================================================================
// [6] เคลื่อนที่พื้นฐาน (fd, bk, sl, sr, tl, tr)
// ======================================================================

void fd(int Speed, int time_ms) {
  BaseSpeed = Speed;
  InitialSpeed();
  Move(LeftBaseSpeed, RightBaseSpeed, time_ms);
}

void bk(int Speed, int time_ms) {
  BaseSpeed = Speed;
  InitialSpeed();
  Move(-BackLeftBaseSpeed, -BackRightBaseSpeed, time_ms);
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

#endif  // MYMINI_16PRO_MOTOR_H
