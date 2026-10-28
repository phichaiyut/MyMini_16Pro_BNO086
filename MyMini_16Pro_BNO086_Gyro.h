#ifndef MYMINI_16PRO_GYRO_H
#define MYMINI_16PRO_GYRO_H

/*
 * MyMini_16Pro_BNO086_Gyro.h
 * ไจโร BNO086: อ่านมุม, หมุน/เลี้ยวตามองศา, วิ่งตรง และทางแยกด้วยไจโร
 * (ตรรกะจาก MyRP_Pico2_mini_BNO086 — ดัชนีเซนเซอร์เป็น 16 ตัวแบบ STORMXII: ผ่านแยกใช้ F[3] / F[12])
 *
 * สารบัญ:
 *   [1] IMU BNO08x และตัวแปรปรับค่าไจโร
 *   [2] เริ่มต้น / อ่านมุม IMU (gyro_Begin / yaw / reset)
 *   [3] ตั้งค่า PID ไจโร (SetGyroTurn / SetGyroSpin)
 *   [4] ล็อกมุม (HoldAngle / SetG)
 *   [5] หมุน / เลี้ยวตามองศา (spindegree / turndegree)
 *   [6] หันไปทิศสัมบูรณ์ (0 / 90 / 180 / 270 อ้างอิงตอน resetAngles)
 *   [7] วิ่งตรงด้วยไจโร (RunG / RunGB)
 *   [8] วิ่งตรงตามระยะทาง (ffcmg / bbcmg)
 *   [9] วิ่งตรง + ล็อกทิศ (direction)
 *   [10] คำสั่งย่อ หมุน / เลี้ยว (spinlg, slg, tlg ...)
 *   [11] เข้ากลาง / ชิดเส้น ด้วยไจโร
 *   [12] ทางแยก (TrackSelectG / TrackSelectGB)
 *   [13] วิ่งตรงจนเจอเส้น / ระยะ + ทางแยก
 *   [14] ทางแยก + ล็อกทิศ
 *   [15] วิ่งตามเส้นจนหันถึงทิศ (ffdir / bbdir / ffdircl / ffdircr ...)
 */

#include <Wire.h>
#include <SparkFun_BNO08x_Arduino_Library.h>
#include "MyMini_16Pro_BNO086_Buzzer.h"
#include "MyMini_16Pro_BNO086_I2C.h"
#include "MyMini_16Pro_BNO086_PID.h"

// ======================================================================
// [1] IMU BNO08x และตัวแปรปรับค่าไจโร
// ======================================================================

BNO08x myIMU;
bool imuReady = false;  // true = เจอ BNO086 (ไม่เจอ → คำสั่งไจโรอ่านมุมเป็น 0)
float yawOffset = 0;
float lastYaw = 0;
float yawgyro = 0;  // มุมสะสมไม่ตัดรอบ (เลี้ยวขวา 2 รอบ = 720)
const int YAW_SIGN = -1;

// ---------- ค่าปรับไจโร: หมุนตัว (Spin) ----------

float gyro_Kp_Spin = 1.2f;
float gyro_Kd_Spin = 0.25f;
int gyro_MaxSpd_Spin = 50;
int gyro_MinSpd_Spin = 15;
float gyro_SmallAngle_Spin = 15.0f;
float gyro_StopThr_Spin = 2.0f;

// ---------- ค่าปรับไจโร: เลี้ยว (Turn) / วิ่งตรง (Run) ----------

float gyro_Kp_Turn = 1.2f;
float gyro_Kd_Turn = 0.25f;
int gyro_MaxSpd_Turn = 50;
int gyro_MaxSpd_TurnB = 50;  // ความเร็วเริ่มต้นของ turndegreeb / turndirectionb
int gyro_MinSpd_Turn = 15;
float gyro_SmallAngle_Turn = 15.0f;
float gyro_StopThr_Turn = 2.0f;
float gyro_StopThr_Rotate = 1.0f;

float current_degree = 0;
float previous_errorG = 0;
float previous_errorGB = 0;
float power_factor = 1.0;


float speed_scale_fw_g = 1.5;  // ค่าตัวคูณความเร็ววิ่งตรง (RunG / RunGB)
float speed_scale_bw_g = 1.5;
float speed_scale_fw_g_normal = 0.99;  // ค่าตัวคูณความเร็ววิ่งตรง (RunG / RunGB)
float speed_scale_bw_g_normal = 0.99;

float speed_scale_fw_gs = 1.65;  // ค่าตัวคูณความเร็ววิ่งตรง (RunG / RunGB)
float speed_scale_bw_gs = 1.65;

int brake_spin_g = 5;
int brake_turn_g = 5;
int brake_turn_b_g = 5;

int gyro_BounceSpd_spin = 12;
int gyro_BounceMs_spin = 5;

int gyro_BounceSpd_turn = 12;
int gyro_BounceMs_turn = 5;

int gyro_BounceSpd_turn_b = 12;
int gyro_BounceMs_turn_b = 5;

// ======================================================================
// [2] เริ่มต้น / อ่านมุม IMU (gyro_Begin / yaw / reset)
// ======================================================================

// ลองหา BNO08x ที่ 0x4A และ 0x4B ซ้ำได้ไม่เกิน timeoutMs ถ้าไม่เจอให้ทำงานต่อโดยไม่มี gyro
bool gyro_Begin(unsigned long timeoutMs = 10000) {
  const uint8_t addresses[] = { 0x4A, 0x4B };
  imuReady = false;
  unsigned long start = millis();
  while (!imuReady && millis() - start < timeoutMs) {
    for (uint8_t addr : addresses) {
      if (probeI2C(addr) && myIMU.begin(addr, Wire, -1, -1)) {
        imuReady = true;
        Serial.printf("BNO08x found at 0x%02X!\n", addr);
        break;
      }
    }
    if (!imuReady) delay(100);
  }

  if (!imuReady) {
    Serial.println("BNO08x not detected at 0x4A / 0x4B. Check your wiring.");
    for (int i = 0; i < 4; i++) {
      Beep(500);
      delay(200);
    }
    return false;
  }

  myIMU.enableRotationVector(5);  // ตั้งค่าให้ BNO08x ส่งข้อมูล rotation vector ทุก 5ms (200Hz)
  // myIMU.begin() เรียก Wire.begin() ซ้ำข้างใน → ตั้งความเร็วบัสกลับเป็น 100 kHz
  Wire.setClock(I2C_CLOCK);
  return true;
}

float wrap180(float a) {
  while (a > 180.0f) a -= 360.0f;
  while (a < -180.0f) a += 360.0f;
  return a;
}

bool waitForRotVec(uint32_t timeout_ms = 200) {
  if (!imuReady) return false;
  uint32_t t0 = millis();
  while (millis() - t0 < timeout_ms) {
    if (myIMU.getSensorEvent() && myIMU.getSensorEventID() == SENSOR_REPORTID_ROTATION_VECTOR) {
      return true;
    }
    delay(1);
  }
  return false;
}

float quatToYawDeg() {
  float q0 = myIMU.getQuatReal();
  float q1 = myIMU.getQuatI();
  float q2 = myIMU.getQuatJ();
  float q3 = myIMU.getQuatK();

  float yaw = atan2(2.0f * (q0 * q3 + q1 * q2),
                    1.0f - 2.0f * (q2 * q2 + q3 * q3));
  yaw = yaw * 180.0f / PI;
  yaw = wrap180(yaw);

  yaw *= (float)YAW_SIGN;
  yaw = wrap180(yaw);

  return yaw;
}

int updateYawFromEvent() {
  float yaw = quatToYawDeg();
  float yawZeroed = wrap180(yaw - yawOffset);
  float delta = wrap180(yawZeroed - lastYaw);
  yawgyro += delta;
  return lastYaw = yawZeroed;
}

int angleRead() {
  if (!imuReady) return lastYaw;
  if (!myIMU.getSensorEvent()) return lastYaw;
  if (myIMU.getSensorEventID() != SENSOR_REPORTID_ROTATION_VECTOR) return lastYaw;
  return updateYawFromEvent();
}

// รอค่ามุมใหม่จาก IMU ก่อนคืนค่า (ใช้ตอนตั้งมุมเป้าหมาย)
// angleRead() คืน lastYaw เก่าถ้ายังไม่มี packet ใหม่ ถ้าคำสั่งก่อนหน้าไม่ได้อ่านไจโร (เช่น วิ่งตามเส้น)
// lastYaw จะค้างเป็นมุมเดิม ทำให้เป้าหมายผิด หุ่นหมุนกลับไปมุมเก่าก่อนแล้วค่อยหมุนต่อ
int angleReadFresh() {
  if (!waitForRotVec(50)) return lastYaw;
  return updateYawFromEvent();
}

// คืน true ถ้า reset สำเร็จ, false = gyro ไม่ส่งข้อมูล (บี๊บเตือน และ offset เป็นค่าเดิม)
bool resetYaw() {
  if (!imuReady) return false;
  if (!waitForRotVec(200)) {
    Serial.println("ResetYaw timeout: no rotation vector");
    Beep(500);
    return false;
  }
  float yawNow = quatToYawDeg();
  yawOffset = yawNow;
  lastYaw = 0.0f;
  yawgyro = 0.0f;
  return true;
}

// void SetRobotAngle() {
//   //  MotorStop();delay(20);
//   for (int i = 0; i < 10; i++) {
//     resetYaw();
//   }
// }

void setAngleOffset() {
  SetRobotAngle();
}

// ควรเรียกตอนหุ่นหยุดนิ่งแล้ว เช่น MotorStop(); delay(100); resetAngles();
void resetAngles() {
  resetYaw();
  current_degree = angleRead();
  previous_errorG = 0;
  previous_errorGB = 0;
}

// ffcmg: fw_normal = ระยะ >= 30 cm (มีเร่ง/ชะลอ), fw = ระยะ < 30 cm (ไม่มี ramp)
void set_speed_scale_ff_g(float fw_normal, float fw) {
  speed_scale_fw_g_normal = fw_normal;
  speed_scale_fw_g = fw;
}

// bbcmg: bb_normal = ระยะ >= 30 cm (มีเร่ง/ชะลอ), bb = ระยะ < 30 cm (ไม่มี ramp)
void set_speed_scale_bb_g(float bb_normal, float bb) {
  speed_scale_bw_g_normal = bb_normal;
  speed_scale_bw_g = bb;
}

// ffcmgs / bbcmgs
void set_speed_scale_ff_bb_gs(float ff, float bb) {
  speed_scale_fw_gs = ff;
  speed_scale_bw_gs = bb;
}

float gyroZ() {
  return angleRead();
}

void SetRobotAngle() {
  current_degree = gyroZ();
}

// ======================================================================
// [3] ตั้งค่า PID ไจโร (SetGyroTurn / SetGyroSpin)
// ======================================================================

void SetGyroTurn(float kp, float kd, int maxSpd, int minSpd, float smallAngle, float stopThr) {
  gyro_Kp_Turn = kp;
  gyro_Kd_Turn = kd;
  gyro_MaxSpd_Turn = maxSpd;
  gyro_MinSpd_Turn = minSpd;
  gyro_SmallAngle_Turn = smallAngle;
  gyro_StopThr_Turn = stopThr;
}
void SetGyroSpin(float kp, float kd, int maxSpd, int minSpd, float smallAngle, float stopThr) {
  gyro_Kp_Spin = kp;
  gyro_Kd_Spin = kd;
  gyro_MaxSpd_Spin = maxSpd;
  gyro_MinSpd_Spin = minSpd;
  gyro_SmallAngle_Spin = smallAngle;
  gyro_StopThr_Spin = stopThr;
}

// ======================================================================
// [4] ล็อกมุม (HoldAngle / SetG)
// ======================================================================

float kpHold = 2.5;
float kdHold = 1.5;
float kpFHold = 2.5;
float kdFHold = 1.5;
float kpBHold = 2.5;
float kdBHold = 1.2;
float holdAngle = 0;
float prevErrHold = 0;
float prevErrHoldB = 0;
float prevErrHoldF = 0;
float prevErrHoldBF = 0;

void HoldAngle() {
  float error = current_degree - gyroZ();
  // wrap -180 ถึง 180
  if (error > 180) error -= 360;
  else if (error < -180) error += 360;
  float d = error - prevErrHold;
  int power = (error * kpHold) + (d * kdHold);
  power = constrain(power, -50, 50);  // แรงหมุน
  Motor(power, -power);               // หมุนอยู่กับที่
  prevErrHold = error;
}

void HoldAngleB() {
  float error = gyroZ() - current_degree;
  // wrap -180 ถึง 180
  if (error > 180) error -= 360;
  else if (error < -180) error += 360;
  float d = error - prevErrHoldB;
  int power = (error * kpBHold) + (d * kdBHold);
  power = constrain(power, -50, 50);  // แรงหมุน
  Motor(power, -power);               // หมุนอยู่กับที่
  prevErrHoldB = error;
}

void SetHoldAngle() {
  holdAngle = gyroZ();  // มุมที่ต้องการให้หุ่น "จำ"
  MotorStop();
  prevErrHoldF = 0;
}

void HoldAngleF() {
  float error = holdAngle - gyroZ();
  // wrap -180 ถึง 180
  if (error > 180) error -= 360;
  else if (error < -180) error += 360;
  float d = error - prevErrHoldF;
  int power = (error * kpFHold) + (d * kdFHold);
  power = constrain(power, -50, 50);  // แรงหมุน
  Motor(power, -power);               // หมุนอยู่กับที่
  prevErrHoldF = error;
}

void SetFG(int totalTime) {
  BZon();
  SetHoldAngle();
  unsigned long endTime = millis() + totalTime;
  while (millis() <= endTime) {
    HoldAngleF();
  }
  BZoff();
}

void setfg(int totalTime) {
  SetFG(totalTime);
}

void HoldAngleBF() {
  float error = gyroZ() - holdAngle;
  // wrap -180 ถึง 180
  if (error > 180) error -= 360;
  else if (error < -180) error += 360;
  float d = error - prevErrHoldBF;
  int power = (error * kpBHold) + (d * kdBHold);
  power = constrain(power, -50, 50);  // แรงหมุน
  Motor(-power, power);               // หมุนอยู่กับที่
  prevErrHoldBF = error;
}

void SetBG(int totalTime) {
  BZon();
  SetHoldAngle();
  prevErrHoldBF = 0;
  unsigned long endTime = millis() + totalTime;
  while (millis() <= endTime) {
    HoldAngleBF();
  }
  BZoff();
}

void setbg(int totalTime) {
  SetBG(totalTime);
}

void SetG(int totalTime) {
  BZon();
  unsigned long endTime = millis() + totalTime;
  while (millis() <= endTime) {
    HoldAngle();
  }
  BZoff();
}

void SetGB(int totalTime) {
  BZon();
  unsigned long endTime = millis() + totalTime;
  while (millis() <= endTime) {
    HoldAngleB();
  }
  BZoff();
}



void set_brake_spin_gyro_setg(int brake){
  brake_spin_g = brake;
}

void set_brake_spin_gyro(int spd, int t){
  gyro_BounceSpd_spin = spd;
  gyro_BounceMs_spin = t;
}

void set_brake_turn_gyro_setg(int brake){
  brake_turn_g = brake;
}
void set_brake_turn_gyro(int spd, int t){
  gyro_BounceSpd_turn = spd;
  gyro_BounceMs_turn = t;
}

void set_brake_turn_b_gyro_setg(int brake){
  brake_turn_b_g = brake;
}
void set_brake_turn_b_gyro(int spd, int t){
  gyro_BounceSpd_turn_b = spd;
  gyro_BounceMs_turn_b = t;
}
// ======================================================================
// [5] หมุน / เลี้ยวตามองศา (spindegree / turndegree)
// ======================================================================

void spindegree(int Speed, int relative_degree) {
  int min_speed = gyro_MinSpd_Spin;
  int max_speed = Speed;
  float kp = gyro_Kp_Spin;                             // เพิ่มจาก 0.9: เร่งแรงบิดตาม error ให้มากขึ้น
  float kd = gyro_Kd_Spin;                             // เพิ่มจาก 0.35: ยังหมุนเกิน 90° อยู่ จึงเพิ่มแรงหน่วงตามอัตราหมุนให้มากขึ้นอีก
  float small_angle_threshold = gyro_SmallAngle_Spin;  // ลดจาก 25: ช่วงคลานที่ min_speed แคบลง วิ่งเร็วได้นานขึ้นก่อนเข้าเบรก
  float stop_threshold = gyro_StopThr_Spin;
  float previous_error = 0;
  float target_degree = angleReadFresh() + relative_degree;
  if (target_degree > 180.0f) target_degree -= 360.0f;
  if (target_degree < -180.0f) target_degree += 360.0f;
  current_degree = target_degree;
  int spin_dir = 0;  // ทิศคำสั่งล่าสุด: 1 = หมุนขวา, -1 = หมุนซ้าย, 0 = ไม่ได้หมุน
  while (1) {
    float current_angle = gyroZ();
    float error = target_degree - current_angle;

    if (error > 180.0f) error -= 360.0f;
    else if (error < -180.0f) error += 360.0f;
    int pd_value = (kp * error) + (kd * (error - previous_error));

    if (pd_value > max_speed) pd_value = max_speed;
    else if (pd_value < -max_speed) pd_value = -max_speed;

    if (error > stop_threshold && error < small_angle_threshold) {
      Motor(min_speed, -min_speed);
      spin_dir = 1;
    } else if (error < -stop_threshold && error > -small_angle_threshold) {
      Motor(-min_speed, min_speed);
      spin_dir = -1;
    } else if (error >= -stop_threshold && error <= stop_threshold) {
      break;
    } else {
      Motor(pd_value, -pd_value);
      if (pd_value != 0) spin_dir = (pd_value > 0) ? 1 : -1;
    }
    previous_error = error;
  }
  // เบรกสวนทิศที่หมุนจริง (ไม่ใช้ relative_degree เพราะหลัง wrap ±180 ทิศอาจกลับด้าน)
  if (spin_dir != 0) {
    Motor(-spin_dir * gyro_BounceSpd_spin, spin_dir * gyro_BounceSpd_spin);
    delay(gyro_BounceMs_spin);
  }
  MotorStop();
  SetG(brake_spin_g);
}

void turndegree(int Speed, int relative_degree) {
  int min_speed = gyro_MinSpd_Turn;  // เพิ่มจาก 10: ล้อเดียวแรงไม่พอหมุนถึงเป้าหมายบางครั้ง
  int max_speed = Speed;
  float kp = gyro_Kp_Turn;                             // เพิ่มจาก 0.9: เร่งแรงบิดตาม error ให้มากขึ้น
  float kd = gyro_Kd_Turn;                             // เพิ่มจาก 0.35: ยังหมุนเกิน 90° อยู่ จึงเพิ่มแรงหน่วงตามอัตราหมุนให้มากขึ้นอีก
  float small_angle_threshold = gyro_SmallAngle_Turn;  // ลดจาก 25: ช่วงคลานที่ min_speed แคบลง วิ่งเร็วได้นานขึ้นก่อนเข้าเบรก
  float stop_threshold = gyro_StopThr_Turn;
  float previous_error = 0;
  float target_degree = angleReadFresh() + relative_degree;

  if (target_degree > 180.0f) target_degree -= 360.0f;
  if (target_degree < -180.0f) target_degree += 360.0f;
  current_degree = target_degree;
  int turn_dir = 0;  // ทิศคำสั่งล่าสุด: 1 = เลี้ยวขวา (ล้อซ้ายเดิน), -1 = เลี้ยวซ้าย (ล้อขวาเดิน), 0 = ไม่ได้เลี้ยว

  while (1) {
    float current_angle = gyroZ();
    float error = target_degree - current_angle;

    if (error > 180.0f) error -= 360.0f;
    else if (error < -180.0f) error += 360.0f;

    int pd_value = (kp * error) + (kd * (error - previous_error));

    if (pd_value > max_speed) pd_value = max_speed;
    else if (pd_value < -max_speed) pd_value = -max_speed;

    if (error > stop_threshold && error < small_angle_threshold) {
      Motor(min_speed, 1);
      turn_dir = 1;
    } else if (error < -stop_threshold && error > -small_angle_threshold) {
      Motor(-1, min_speed);
      turn_dir = -1;
    } else if (error >= -stop_threshold && error <= stop_threshold) {
      MotorStop();
      break;
    } else {
      if (error <= 0) {
        Motor(-1, -pd_value);
        turn_dir = -1;
      } else {
        Motor(pd_value, 1);
        turn_dir = 1;
      }
    }

    previous_error = error;
  }
  // เบรกล้อที่เดินอยู่ สวนทิศที่เลี้ยวจริง
  if (turn_dir > 0) Motor(-gyro_BounceSpd_turn, 0);
  else if (turn_dir < 0) Motor(0, -gyro_BounceSpd_turn);
  if (turn_dir != 0) delay(gyro_BounceMs_turn);
  MotorStop();
  SetG(brake_turn_g);
}

void turndegreeb(int Speed, int relative_degree) {
  int min_speed = gyro_MinSpd_Turn;  // เพิ่มจาก 10: ล้อเดียวแรงไม่พอหมุนถึงเป้าหมายบางครั้ง
  int max_speed = Speed;
  float kp = gyro_Kp_Turn;                             // เพิ่มจาก 0.9: เร่งแรงบิดตาม error ให้มากขึ้น
  float kd = gyro_Kd_Turn;                             // เพิ่มจาก 0.35: ยังหมุนเกิน 90° อยู่ จึงเพิ่มแรงหน่วงตามอัตราหมุนให้มากขึ้นอีก
  float small_angle_threshold = gyro_SmallAngle_Turn;  // ลดจาก 25: ช่วงคลานที่ min_speed แคบลง วิ่งเร็วได้นานขึ้นก่อนเข้าเบรก
  float stop_threshold = gyro_StopThr_Turn;
  float previous_error = 0;
  float target_degree = angleReadFresh() + relative_degree;
  if (target_degree > 180.0f) target_degree -= 360.0f;
  if (target_degree < -180.0f) target_degree += 360.0f;
  current_degree = target_degree;
  int turn_dir = 0;  // ทิศคำสั่งล่าสุด: 1 = ล้อขวาถอย, -1 = ล้อซ้ายถอย, 0 = ไม่ได้เลี้ยว

  while (1) {
    float current_angle = gyroZ();
    float error = target_degree - current_angle;

    if (error > 180.0f) error -= 360.0f;
    else if (error < -180.0f) error += 360.0f;

    int pd_value = (kp * error) + (kd * (error - previous_error));

    if (pd_value > max_speed) pd_value = max_speed;
    else if (pd_value < -max_speed) pd_value = -max_speed;

    if (error > stop_threshold && error < small_angle_threshold) {
      Motor(-1, -min_speed);
      turn_dir = 1;
    } else if (error < -stop_threshold && error > -small_angle_threshold) {
      Motor(-min_speed, 1);
      turn_dir = -1;
    } else if (error >= -stop_threshold && error <= stop_threshold) {
      break;
    } else {
      if (error <= 0) {
        Motor(pd_value, 1);
        turn_dir = -1;
      } else {
        Motor(-1, -pd_value);
        turn_dir = 1;
      }
    }

    previous_error = error;
  }
  // เบรกล้อที่ถอยอยู่ ด้วยการเดินหน้าสวนสั้น ๆ
  if (turn_dir > 0) Motor(0, gyro_BounceSpd_turn_b);
  else if (turn_dir < 0) Motor(gyro_BounceSpd_turn_b, 0);
  if (turn_dir != 0) delay(gyro_BounceMs_turn_b);
  MotorStop();

  SetG(brake_turn_b_g);
}

// ---------- rotatedegree: วิ่งโค้ง ล้อซ้าย/ขวาความเร็วต่างกัน ----------

void rotatedegree(int SpeedL, int SpeedR, int relative_degree, float kp, float kd) {
  float stop_threshold = gyro_StopThr_Rotate;
  float previous_error = 0;
  float target_degree = angleReadFresh() + relative_degree;

  if (target_degree > 180.0f) target_degree -= 360.0f;
  if (target_degree < -180.0f) target_degree += 360.0f;
  current_degree = target_degree;

  while (1) {
    float current_angle = gyroZ();
    float error = target_degree - current_angle;

    if (error > 180.0f) error -= 360.0f;
    else if (error < -180.0f) error += 360.0f;

    if (error >= -stop_threshold && error <= stop_threshold) {
      MotorStop();
      break;
    }

    float derivative = error - previous_error;
    int pd_value = (int)((error * kp) + (derivative * kd));

    int leftPow = constrain(SpeedL + pd_value, -100, 100);
    int rightPow = constrain(SpeedR - pd_value, -100, 100);

    Motor(leftPow, rightPow);
    previous_error = error;
  }
  SetG(5);
}

void rotatedegree(int SpeedL, int SpeedR, int relative_degree) {
  rotatedegree(SpeedL, SpeedR, relative_degree, 0.9, 0.6);
}

void turndegree_none(int Speed, int relative_degree) {
  float stop_threshold = gyro_StopThr_Turn;
  float previous_error = 0;
  float target_degree = current_degree + relative_degree;
  if (target_degree > 180.0f) target_degree -= 360.0f;
  if (target_degree < -180.0f) target_degree += 360.0f;
  current_degree = target_degree;

  while (1) {
    float current_angle = gyroZ();
    float error = target_degree - current_angle;

    if (error > 180.0f) error -= 360.0f;
    else if (error < -180.0f) error += 360.0f;

    // BNO086 ส่งมุมทุก 10ms ถ้าหมุนเร็ว มุมจะกระโดดข้ามช่วง ±stop_threshold ได้
    // จึงถือว่าถึงเป้าเมื่อ error เปลี่ยนเครื่องหมาย (หมุนเลยเป้าแล้ว) ด้วย ไม่ให้หมุนกลับไปมา
    // (|error| < 90 กันกรณี error กระโดดจาก +180 เป็น -180 ตอน wrap)
    bool crossed = fabsf(error) < 90.0f && ((error > 0 && previous_error < 0) || (error < 0 && previous_error > 0));
    previous_error = error;

    if ((error >= -stop_threshold && error <= stop_threshold) || crossed) {
      break;
    } else if (error > 0) {
      Motor(Speed, -1);
    } else {
      Motor(-1, Speed);
    }
  }
}

void turndegreeb_none(int Speed, int relative_degree) {
  float stop_threshold = gyro_StopThr_Turn;
  float previous_error = 0;
  float target_degree = current_degree + relative_degree;
  if (target_degree > 180.0f) target_degree -= 360.0f;
  if (target_degree < -180.0f) target_degree += 360.0f;
  current_degree = target_degree;

  while (1) {
    float current_angle = gyroZ();
    float error = target_degree - current_angle;

    if (error > 180.0f) error -= 360.0f;
    else if (error < -180.0f) error += 360.0f;

    // BNO086 ส่งมุมทุก 10ms ถ้าหมุนเร็ว มุมจะกระโดดข้ามช่วง ±stop_threshold ได้
    // จึงถือว่าถึงเป้าเมื่อ error เปลี่ยนเครื่องหมาย (หมุนเลยเป้าแล้ว) ด้วย ไม่ให้หมุนกลับไปมา
    // (|error| < 90 กันกรณี error กระโดดจาก +180 เป็น -180 ตอน wrap)
    bool crossed = fabsf(error) < 90.0f && ((error > 0 && previous_error < 0) || (error < 0 && previous_error > 0));
    previous_error = error;

    if ((error >= -stop_threshold && error <= stop_threshold) || crossed) {
      break;
    } else if (error > 0) {
      Motor(1, -Speed);
    } else {
      Motor(-Speed, 1);
    }
  }
}

void spindegree(int relative_degree) {
  spindegree(gyro_MaxSpd_Spin, relative_degree);
}

void turndegree(int relative_degree) {
  turndegree(gyro_MaxSpd_Turn, relative_degree);
}

void turndegreeb(int relative_degree) {
  turndegreeb(gyro_MaxSpd_Turn, relative_degree);
}

void turndegree_none(int relative_degree) {
  turndegree_none(gyro_MaxSpd_Turn, relative_degree);
}

void turndegreeb_none(int relative_degree) {
  turndegreeb_none(gyro_MaxSpd_Turn, relative_degree);
}

// ======================================================================
// [6] หันไปทิศสัมบูรณ์ (0 / 90 / 180 / 270 อ้างอิงตอน resetAngles)
// ======================================================================

// คำนวณมุมที่ต้องหมุนจากมุมปัจจุบันไปยังทิศทางสัมบูรณ์ (ทางที่สั้นที่สุด)
int relativeToDirection(int direction) {
  float relative = fmodf((float)direction, 360.0f) - angleReadFresh();
  while (relative > 180.0f) relative -= 360.0f;
  while (relative < -180.0f) relative += 360.0f;
  return (int)roundf(relative);
}

void spindirection(int Speed, int direction) {
  spindegree(Speed, relativeToDirection(direction));
}

void spindirection(int direction) {
  spindirection(gyro_MaxSpd_Spin, direction);
}

void turndirection(int Speed, int direction) {
  turndegree(Speed, relativeToDirection(direction));
}

void turndirectionb(int Speed, int direction) {
  turndegreeb(Speed, relativeToDirection(direction));
}

void turndirection(int direction) {
  turndirection(gyro_MaxSpd_Turn, direction);
}

void turndirectionb(int direction) {
  turndirectionb(gyro_MaxSpd_Turn, direction);
}

void turndirection_none(int Speed, int direction) {
  turndegree_none(Speed, relativeToDirection(direction));
}

void turndirectionb_none(int Speed, int direction) {
  turndegreeb_none(Speed, relativeToDirection(direction));
}

void turndirection_none(int direction) {
  turndirection_none(gyro_MaxSpd_Turn, direction);
}

void turndirectionb_none(int direction) {
  turndirectionb_none(gyro_MaxSpd_Turn, direction);
}

// ======================================================================
// [7] วิ่งตรงด้วยไจโร (RunG / RunGB)
// ======================================================================

// ค่าเริ่มต้น = ค่าที่ RunG / RunGB ใช้อยู่เดิม (ถ้าไม่เรียก SetGyroRun / SetGyroRunB)
float kpG = 1.2f;   // เดินหน้า (RunG)
float kdG = 2.5f;
float kpGB = 1.2f;  // ถอยหลัง (RunGB)
float kdGB = 2.5f;

void SetGyroRun(float kp, float kd) {
  kpG = kp;
  kdG = kd;
}

void SetGyroRunB(float kp, float kd) {
  kpGB = kp;
  kdGB = kd;
}

// โหมดจำกัดกำลังมอเตอร์ของ RunG (เดินหน้า) / RunGB (ถอยหลัง) แยกจาก ModePidStatus ของ PID เส้น
// ค่าเริ่มต้น: RunG และ RunGB = โหมด 4 (0..Speed+up ไม่ถอยล้อ)
int MaxSpeedG = 100;
int MinSpeedG = -5;
int ModeGyroStatus = 4;
int ModeGyroBStatus = 4;
int gyro_limit_up = 20, gyro_limit_down = 20;  // โหมด 2: -(Speed+down)..(Speed+up)
int gyro_limit4_up = 20;                       // โหมด 4: 0..(Speed+up)

// ตั้งโหมดจำกัดกำลังของ gyro (ใช้ทั้ง RunG และ RunGB) แบบเดียวกับ ModeSpdPID()
void ModeSpdGyro(int moD, int maX, int miN) {
  ModeGyroStatus = moD;
  ModeGyroBStatus = moD;
  MaxSpeedG = maX;
  MinSpeedG = miN;
}

// ตั้งโหมดแยกเดินหน้า (RunG) / ถอยหลัง (RunGB)
void ModeSpdGyro(int moDF, int moDB, int maX, int miN) {
  ModeGyroStatus = moDF;
  ModeGyroBStatus = moDB;
  MaxSpeedG = maX;
  MinSpeedG = miN;
}

// ช่วงกำลังล้อของโหมด 2 (ใช้ทั้ง RunG และ RunGB)
void set_gyro_mode2_limit(int up, int down) {
  gyro_limit_up = up;
  gyro_limit_down = down;
}

// ช่วงกำลังล้อของโหมด 4: 0..(Speed+up) (ใช้ทั้ง RunG และ RunGB)
void set_gyro_mode4_limit(int up) {
  gyro_limit4_up = up;
}

// จำกัดค่า LeftPower/RightPower ของ gyro ตามโหมด (เคส 0-3 เหมือน ClampPIDPower, เคส 4 = 0..Speed)
void ClampGyroPower(float &LeftPower, float &RightPower, int SpeedL, int SpeedR, int mode) {
  switch (mode) {
    case 0:
      if (LeftPower > MaxSpeedG) LeftPower = MaxSpeedG;
      if (LeftPower < 0) LeftPower = MinSpeedG;
      if (RightPower > MaxSpeedG) RightPower = MaxSpeedG;
      if (RightPower < 0) RightPower = MinSpeedG;
      break;
    case 1:
      if (LeftPower > MaxSpeedG) LeftPower = MaxSpeedG;
      if (LeftPower < MinSpeedG) LeftPower = MinSpeedG;
      if (RightPower > MaxSpeedG) RightPower = MaxSpeedG;
      if (RightPower < MinSpeedG) RightPower = MinSpeedG;
      break;
    case 2:
      if (LeftPower > SpeedL + gyro_limit_up) LeftPower = SpeedL + gyro_limit_up;
      if (LeftPower < -SpeedL - gyro_limit_down) LeftPower = -SpeedL - gyro_limit_down;
      if (RightPower > SpeedR + gyro_limit_up) RightPower = SpeedR + gyro_limit_up;
      if (RightPower < -SpeedR - gyro_limit_down) RightPower = -SpeedR - gyro_limit_down;
      break;
    case 3:
      if (LeftPower > MaxSpeedG) LeftPower = MaxSpeedG;
      if (LeftPower < 0) LeftPower = -BaseSpeed;
      if (RightPower > MaxSpeedG) RightPower = MaxSpeedG;
      if (RightPower < 0) RightPower = -BaseSpeed;
      break;
    case 4:
      if (LeftPower > SpeedL + gyro_limit4_up) LeftPower = SpeedL + gyro_limit4_up;
      if (LeftPower < 0) LeftPower = 0;
      if (RightPower > SpeedR + gyro_limit4_up) RightPower = SpeedR + gyro_limit4_up;
      if (RightPower < 0) RightPower = 0;
      break;
    default:
      if (LeftPower > MaxSpeedG) LeftPower = MaxSpeedG;
      if (LeftPower < 0) LeftPower = 0;
      if (RightPower > MaxSpeedG) RightPower = MaxSpeedG;
      if (RightPower < 0) RightPower = 0;
  }
}

void RunG(int SpeedL, int SpeedR) {
  float error = current_degree - gyroZ();

  if (error > 180.0f) error -= 360.0f;
  else if (error < -180.0f) error += 360.0f;

  float derivative = error - previous_errorG;
  int pd_value = (int)((error * kpG) + (derivative * kdG));
  float leftPow = SpeedL + pd_value;
  float rightPow = SpeedR - pd_value;

  ClampGyroPower(leftPow, rightPow, SpeedL, SpeedR, ModeGyroStatus);

  Motor(leftPow, rightPow);
  previous_errorG = error;
}

void RunGB(int SpeedL, int SpeedR) {
  float error = current_degree - gyroZ();
  if (error > 180.0f) error -= 360.0f;
  else if (error < -180.0f) error += 360.0f;
  float derivative = error - previous_errorGB;
  int pd_value = (int)((error * kpGB) + (derivative * kdGB));
  float leftPow = SpeedL - pd_value;
  float rightPow = SpeedR + pd_value;

  ClampGyroPower(leftPow, rightPow, SpeedL, SpeedR, ModeGyroBStatus);

  Motor(-leftPow, -rightPow);
  previous_errorGB = error;
}

void fftimerg(int Speed, int totalTime) {
  BaseSpeed = Speed;
  InitialSpeed();
  unsigned long endTime = millis() + totalTime;
  while (millis() <= endTime) {
    RunG(LeftBaseSpeed, RightBaseSpeed);
  }
}

// ตั้งทิศทางสัมบูรณ์ (อ้างอิงจากตอน resetAngles) เช่น 0, 90, 180, 270, 360
void SetDirectionG(int direction) {
  float target = fmod((float)direction, 360.0f);
  if (target > 180.0f) target -= 360.0f;
  else if (target < -180.0f) target += 360.0f;
  current_degree = target;
  resetGyroRunError();
}

// รีเซ็ต D-term ของ RunG / RunGB ให้เท่ากับ error ปัจจุบัน (ไม่ใช่ 0) กันกระชากตอนออกตัว
// เรียกจาก InitialSpeed() ด้วย | ใช้ lastYaw (ไม่อ่านเซนเซอร์ใหม่) จึงปลอดภัยแม้ยังไม่เริ่ม IMU
void resetGyroRunError() {
  float error = current_degree - (int)lastYaw;  // (int) ให้ตรงกับค่าที่ gyroZ() คืน
  if (error > 180.0f) error -= 360.0f;
  else if (error < -180.0f) error += 360.0f;
  previous_errorG = error;
  previous_errorGB = error;
}

void bbtimerg(int Speed, int totalTime) {
  BaseSpeed = Speed;
  InitialSpeed();
  unsigned long endTime = millis() + totalTime;
  while (millis() <= endTime) {
    RunGB(BackLeftBaseSpeed, BackRightBaseSpeed);
  }
}

// ======================================================================
// [8] วิ่งตรงตามระยะทาง (ffcmg / bbcmg)
// ======================================================================

void ffcmgs(int Speed, float distance) {
  BaseSpeed = Speed;
  InitialSpeed();
  int target_speed = min(LeftBaseSpeed, RightBaseSpeed);
  float traveled_distance = 0;
  unsigned long last_time = millis();
  // float speed_scale = 1.65;

  unsigned long prevT = millis();

  while (1) {
    unsigned long now = millis();
    float dt = (now - prevT) / 1000.0;
    if (dt <= 0) dt = 0.001;
    prevT = now;

    RunG(LeftBaseSpeed, RightBaseSpeed);

    if (distance > 0) {
      unsigned long current_time = millis();
      float delta_time = (current_time - last_time) / 1000.0;
      traveled_distance += (target_speed * speed_scale_fw_gs) * delta_time;
      last_time = current_time;

      if (traveled_distance >= distance) break;
    }
    delayMicroseconds(80);
  }
}

void ffcmg(int Speed, float distance_cm) {
  BaseSpeed = Speed;
  InitialSpeed();

  if (distance_cm <= 0) {
    Motor(0, 0);
    return;
  }

  int base_speed = min(abs(LeftBaseSpeed), abs(RightBaseSpeed));
  float traveled_distance = 0.0;
  unsigned long last_time = millis();

  const float ACCEL_DISTANCE_CM = 20.0;
  const float DECEL_DISTANCE_CM = 25.0;
  const float MIN_SPEED = 10.0;

  float speed_scale = speed_scale_fw_g_normal;
  bool enableRamp = (distance_cm >= 30.0);

  if (!enableRamp) {
    speed_scale = speed_scale_fw_g;
  }

  while (true) {
    unsigned long current_time = millis();
    float delta_time = (current_time - last_time) / 1000.0;
    traveled_distance += (base_speed * speed_scale) * delta_time;
    last_time = current_time;

    float remaining_cm = distance_cm - traveled_distance;
    if (remaining_cm <= 0.7f) break;

    float target_speed = base_speed;

    if (enableRamp) {
      if (traveled_distance < ACCEL_DISTANCE_CM) {
        target_speed = MIN_SPEED + (base_speed - MIN_SPEED) * (traveled_distance / ACCEL_DISTANCE_CM);
      } else if (remaining_cm < DECEL_DISTANCE_CM) {
        target_speed = MIN_SPEED + (base_speed - MIN_SPEED) * (remaining_cm / DECEL_DISTANCE_CM);
      }
    }
    RunG(target_speed, target_speed);
  }
}

void bbcmgs(int Speed, float distance) {
  BaseSpeed = Speed;
  InitialSpeed();
  int target_speed = min(BackLeftBaseSpeed, BackRightBaseSpeed);
  float traveled_distance = 0;
  unsigned long last_time = millis();
  //float speed_scale = 1.65;

  unsigned long prevT = millis();

  while (1) {
    unsigned long now = millis();
    float dt = (now - prevT) / 1000.0;
    if (dt <= 0) dt = 0.001;
    prevT = now;

    RunGB(BackLeftBaseSpeed, BackRightBaseSpeed);

    if (distance > 0) {
      unsigned long current_time = millis();
      float delta_time = (current_time - last_time) / 1000.0;
      traveled_distance += (target_speed * speed_scale_bw_gs) * delta_time;
      last_time = current_time;

      if (traveled_distance >= distance) break;
    }
    delayMicroseconds(80);
  }
}

void bbcmg(int Speed, float distance_cm) {
  BaseSpeed = Speed;
  InitialSpeed();

  if (distance_cm <= 0) {
    Motor(0, 0);
    return;
  }

  int base_speed = min(abs(BackLeftBaseSpeed), abs(BackRightBaseSpeed));
  float traveled_distance = 0.0;
  unsigned long last_time = millis();

  const float ACCEL_DISTANCE_CM = 20.0;
  const float DECEL_DISTANCE_CM = 25.0;
  const float MIN_SPEED = 10.0;

  float speed_scale = speed_scale_bw_g_normal;
  bool enableRamp = (distance_cm >= 30.0);

  if (!enableRamp) {
    speed_scale = speed_scale_bw_g;
  }

  while (true) {
    unsigned long current_time = millis();
    float delta_time = (current_time - last_time) / 1000.0;
    traveled_distance += (base_speed * speed_scale) * delta_time;
    last_time = current_time;

    float remaining_cm = distance_cm - traveled_distance;
    if (remaining_cm <= 0.8f) break;

    float target_speed = base_speed;

    if (enableRamp) {
      if (traveled_distance < ACCEL_DISTANCE_CM) {
        target_speed = MIN_SPEED + (base_speed - MIN_SPEED) * (traveled_distance / ACCEL_DISTANCE_CM);
      } else if (remaining_cm < DECEL_DISTANCE_CM) {
        target_speed = MIN_SPEED + (base_speed - MIN_SPEED) * (remaining_cm / DECEL_DISTANCE_CM);
      }
    }
    RunGB(target_speed, target_speed);
  }
}

// ======================================================================
// [9] วิ่งตรง + ล็อกทิศ (direction)
// ======================================================================

void fftimerg(int Speed, int totalTime, int direction) {
  SetDirectionG(direction);
  fftimerg(Speed, totalTime);
}
void bbtimerg(int Speed, int totalTime, int direction) {
  SetDirectionG(direction);
  bbtimerg(Speed, totalTime);
}

void ffcmgs(int Speed, float distance_cm, int direction) {
  SetDirectionG(direction);
  ffcmgs(Speed, distance_cm);
}
void bbcmgs(int Speed, float distance_cm, int direction) {
  SetDirectionG(direction);
  bbcmgs(Speed, distance_cm);
}

void ffcmg(int Speed, float distance_cm, int direction) {
  SetDirectionG(direction);
  ffcmg(Speed, distance_cm);
}
void bbcmg(int Speed, float distance_cm, int direction) {
  SetDirectionG(direction);
  bbcmg(Speed, distance_cm);
}

// ======================================================================
// [10] คำสั่งย่อ หมุน / เลี้ยว (spinlg, slg, tlg ...)
// ======================================================================

void spinlg(int Angle) {
  spindegree(-abs(Angle));
}

void spinrg(int Angle) {
  spindegree(abs(Angle));
}

void turnlg(int Angle) {
  turndegree(-abs(Angle));
}

void turnrg(int Angle) {
  turndegree(abs(Angle));
}

void turnlbg(int Angle) {
  turndegreeb(abs(Angle));
}

void turnrbg(int Angle) {
  turndegreeb(-abs(Angle));
}

void spinlg(int spd, int Angle) {
  spindegree(spd, -abs(Angle));
}

void spinrg(int spd, int Angle) {
  spindegree(spd, abs(Angle));
}

void turnlg(int spd, int Angle) {
  turndegree(spd, -abs(Angle));
}

void turnrg(int spd, int Angle) {
  turndegree(spd, abs(Angle));
}

void turnlbg(int spd, int Angle) {
  turndegreeb(spd, abs(Angle));
}

void turnrbg(int spd, int Angle) {
  turndegreeb(spd, -abs(Angle));
}

void slg(int Angle) {
  spindegree(-abs(Angle));
}

void srg(int Angle) {
  spindegree(abs(Angle));
}

void tlg(int Angle) {
  turndegree(-abs(Angle));
}

void trg(int Angle) {
  turndegree(abs(Angle));
}

void tlbg(int Angle) {
  turndegreeb(abs(Angle));
}

void trbg(int Angle) {
  turndegreeb(-abs(Angle));
}

void slg(int spd, int Angle) {
  spindegree(spd, -abs(Angle));
}

void srg(int spd, int Angle) {
  spindegree(spd, abs(Angle));
}

void tlg(int spd, int Angle) {
  turndegree(spd, -abs(Angle));
}

void trg(int spd, int Angle) {
  turndegree(spd, abs(Angle));
}

void tlbg(int spd, int Angle) {
  turndegreeb(spd, abs(Angle));
}

void trbg(int spd, int Angle) {
  turndegreeb(spd, -abs(Angle));
}

// ---------- ต่อเนื่อง (chainable, ไม่หยุดกลางทาง): เลี้ยวซ้ายแล้วขวา / ขวาแล้วซ้าย ----------

void tlrg(int Angle) {
  turndegree_none(-abs(Angle));
  turndegree(abs(Angle));
}
void trlg(int Angle) {
  turndegree_none(abs(Angle));
  turndegree(-abs(Angle));
}

void tlrg(int spd, int Angle) {
  turndegree_none(spd, -abs(Angle));
  turndegree(spd, abs(Angle));
}
void trlg(int spd, int Angle) {
  turndegree_none(spd, abs(Angle));
  turndegree(spd, -abs(Angle));
}

void tlrg(int spd, int Angle, int Angle2) {
  turndegree_none(spd, -abs(Angle));
  turndegree(spd, abs(Angle2)); /*SetG(spd);*/
}
void trlg(int spd, int Angle, int Angle2) {
  turndegree_none(spd, abs(Angle));
  turndegree(spd, -abs(Angle2)); /*SetG(spd);*/
}

void tlrbg(int Angle) {
  turndegreeb_none(abs(Angle));
  turndegreeb(-abs(Angle)); /*SetGB(50);*/
}
void trlbg(int Angle) {
  turndegreeb_none(-abs(Angle));
  turndegreeb(abs(Angle)); /*SetGB(50);*/
}

void tlrbg(int spd, int Angle) {
  turndegreeb_none(spd, abs(Angle));
  turndegreeb(spd, -abs(Angle)); /*SetGB(spd);*/
}
void trlbg(int spd, int Angle) {
  turndegreeb_none(spd, -abs(Angle));
  turndegreeb(spd, abs(Angle)); /*SetGB(spd);*/
}

void tlrbg(int spd, int Angle, int Angle2) {
  turndegreeb_none(spd, abs(Angle));
  turndegreeb(spd, -abs(Angle2)); /*SetG(spd);*/
}
void trlbg(int spd, int Angle, int Angle2) {
  turndegreeb_none(spd, -abs(Angle));
  turndegreeb(spd, abs(Angle2)); /*SetG(spd);*/
}

// ======================================================================
// [11] เข้ากลาง / ชิดเส้น ด้วยไจโร
// ======================================================================

void ToCenterG() {
  BZon();
  for (int i = 0; i < 20; i++) {
    RunG(tctL, tctR);
  }
  while (1) {
    RunG(tctL, tctR);
    ReadCalibrateC();
    if (C[CCL] >= RefC || C[CCR] >= RefC) {
      Motor(-tct, -tct);
      delay(5);
      MotorStop();
      BZoff();
      break;
    }
  }
}

void BackCenterG() {
  BZon();
  for (int i = 0; i < 20; i++) {
    RunGB(bctL, bctR);
  }
  while (1) {
    RunGB(bctL, bctR);
    ReadCalibrateC();
    if (C[CCL] >= RefC || C[CCR] >= RefC) {
      Motor(bctL, bctR);
      delay(5);
      MotorStop();
      BZoff();
      break;
    }
  }
}

void ToFrontG() {
  while (1) {
    RunG(tctL, tctR);
    ReadCalibrateF();
    if (F[2] > Ref || F[4] > Ref || F[6] > Ref || F[9] > Ref || F[11] > Ref || F[13] > Ref) break;
  }
}

void ToBackG() {
  while (1) {
    RunGB(bctL, bctR);
    ReadCalibrateB();
    if (B[2] > Ref || B[4] > Ref || B[6] > Ref || B[9] > Ref || B[11] > Ref || B[13] > Ref) break;
  }
}

// ======================================================================
// [12] ทางแยก (TrackSelectG / TrackSelectGB)
// ======================================================================

void TrackSelectG(int spd, char select) {
  if (select == 'L') {
    spindegree(-90);
  } else if (select == 'l') {
    ToCenterG();
    spindegree(-90);
  } else if (select == 'R') {
    spindegree(90);
  } else if (select == 'r') {
    ToCenterG();
    spindegree(90);
  } else if (select == 'q') {
    turndegree(-90);
  } else if (select == 'Q') {
    ToFrontG();
    turndegree(-90);
  } else if (select == 'e') {
    turndegree(90);
  } else if (select == 'E') {
    ToFrontG();
    turndegree(90);
  } else if (select == 'p') {
    BZon();
    ReadCalibrateF();
    while (1) {
      RunG(LeftBaseSpeed, RightBaseSpeed);
      ReadCalibrateF();
      if (F[3] < Ref && F[12] < Ref) break;
    }
    while (1) {
      RunG(LeftBaseSpeed, RightBaseSpeed);
      ReadCalibrateF();
      if (F[3] < Ref && F[12] < Ref) break;
    }
    BZoff();
  } else if (select == 'P') {
    ToFrontG();
    BZon();
    ReadCalibrateF();
    while (1) {
      RunG(LeftBaseSpeed, RightBaseSpeed);
      ReadCalibrateF();
      if (F[3] < Ref && F[12] < Ref) break;
    }
    while (1) {
      RunG(LeftBaseSpeed, RightBaseSpeed);
      ReadCalibrateF();
      if (F[3] < Ref && F[12] < Ref) break;
    }
    BZoff();
  } else if (select == 'c') {
    BZon();
    for (int i = 0; i < 20; i++) {
      RunG(LeftBaseSpeed, RightBaseSpeed);
    }
    while (1) {
      RunG(LeftBaseSpeed, RightBaseSpeed);
      ReadCalibrateC();
      if (C[CCL] >= RefC || C[CCR] >= RefC) {
        Motor(-spd, -spd);
        delay(delay_break_f);
        MotorStop();
        BZoff();
        break;
      }
    }
  } else if (select == 'C') {
    ToFrontG();
    BZon();
    for (int i = 0; i < 20; i++) {
      RunG(LeftBaseSpeed, RightBaseSpeed);
    }
    while (1) {
      RunG(LeftBaseSpeed, RightBaseSpeed);
      ReadCalibrateC();
      if (C[CCL] >= RefC || C[CCR] >= RefC) {
        Motor(-spd, -spd);
        delay(delay_break_f);
        MotorStop();
        BZoff();
        break;
      }
    }
  } else if (select == 'b') {
    BZon();
    for (int i = 0; i < 20; i++) {
      RunG(tctL, tctR);
    }
    while (1) {
      RunG(tctL, tctR);
      ReadCalibrateC();
      if (C[CCL] >= RefC || C[CCR] >= RefC) {
        break;
      }
    }
    while (1) {
      RunG(tctL, tctR);
      ReadCalibrateB();
      if ((B[3] > Ref || B[12] > Ref)) {
        Motor(-tctL, -tctR);
        delay(tct_delay_break);
        Motor(-1, -1);
        delay(1);
        MotorStop();
        BZoff();
        break;
      }
    }
  } else if (select == 'B') {
    ToFrontG();
    BZon();
    for (int i = 0; i < 20; i++) {
      RunG(tctL, tctR);
    }
    while (1) {
      RunG(tctL, tctR);
      ReadCalibrateC();
      if (C[CCL] >= RefC || C[CCR] >= RefC) {
        break;
      }
    }
    while (1) {
      RunG(tctL, tctR);
      ReadCalibrateB();
      if ((B[3] > Ref || B[12] > Ref)) {
        Motor(-tctL, -tctR);
        delay(tct_delay_break);
        Motor(-1, -1);
        delay(1);
        MotorStop();
        BZoff();
        break;
      }
    }
  } else if (select == 's') {
    Motor(-spd, -spd);
    delay(delay_break_f);
    Motor(-1, -1);
    delay(1);
    MotorStop();
  } else if (select == 'S') {
    ToFrontG();
    Motor(-tctL, -tctR);
    delay(tct_delay_break);
    Motor(-1, -1);
    delay(1);
    MotorStop();
  } else if (select == 'G') {
    ToFrontG();
    SetG(tct_delay_break);
  } else {
    SetG(100);
  }
}

void TrackSelectGB(int spd, char select) {
  if (select == 'L') {
    spindegree(-90);
  } else if (select == 'l') {
    BackCenterG();
    spindegree(-90);
  } else if (select == 'R') {
    spindegree(90);
  } else if (select == 'r') {
    BackCenterG();
    spindegree(90);
  } else if (select == 'q') {
    turndegreeb(90);
  } else if (select == 'Q') {
    ToBackG();
    turndegreeb(90);
  } else if (select == 'e') {
    turndegreeb(-90);
  } else if (select == 'E') {
    ToBackG();
    turndegreeb(-90);
  } else if (select == 'p') {
    BZon();
    ReadCalibrateB();
    while (1) {
      RunGB(BackLeftBaseSpeed, BackRightBaseSpeed);
      ReadCalibrateB();
      if (B[3] < Ref && B[12] < Ref) break;
    }
    while (1) {
      RunGB(BackLeftBaseSpeed, BackRightBaseSpeed);
      ReadCalibrateB();
      if (B[3] < Ref && B[12] < Ref) break;
    }
    BZoff();
  } else if (select == 'P') {
    ToBackG();
    BZon();
    ReadCalibrateB();
    while (1) {
      RunGB(BackLeftBaseSpeed, BackRightBaseSpeed);
      ReadCalibrateB();
      if (B[3] < Ref && B[12] < Ref) break;
    }
    while (1) {
      RunGB(BackLeftBaseSpeed, BackRightBaseSpeed);
      ReadCalibrateB();
      if (B[3] < Ref && B[12] < Ref) break;
    }
    BZoff();
  } else if (select == 'c') {
    BZon();
    for (int i = 0; i < 20; i++) {
      RunGB(BackLeftBaseSpeed, BackRightBaseSpeed);
    }
    while (1) {
      RunGB(BackLeftBaseSpeed, BackRightBaseSpeed);
      ReadCalibrateC();
      if (C[CCL] >= RefC || C[CCR] >= RefC) {
        Motor(spd, spd);
        delay(delay_break_b);
        MotorStop();
        BZoff();
        break;
      }
    }
  } else if (select == 'C') {
    ToBackG();
    BZon();
    for (int i = 0; i < 20; i++) {
      RunGB(BackLeftBaseSpeed, BackRightBaseSpeed);
    }
    while (1) {
      RunGB(BackLeftBaseSpeed, BackRightBaseSpeed);
      ReadCalibrateC();
      if (C[CCL] >= RefC || C[CCR] >= RefC) {
        Motor(spd, spd);
        delay(delay_break_b);
        MotorStop();
        BZoff();
        break;
      }
    }
  } else if (select == 'b') {
    BZon();
    for (int i = 0; i < 20; i++) {
      RunGB(bctL, bctR);
    }
    while (1) {
      RunGB(bctL, bctR);
      ReadCalibrateC();
      if (C[CCL] >= RefC || C[CCR] >= RefC) {
        break;
      }
    }
    while (1) {
      RunGB(bctL, bctR);
      ReadCalibrateB();
      if ((B[3] > Ref || B[12] > Ref)) {
        Motor(bctL, bctR);
        delay(bct_delay_break);
        Motor(1, 1);
        delay(1);
        MotorStop();
        BZoff();
        break;
      }
    }
  } else if (select == 'B') {
    ToBackG();
    BZon();
    for (int i = 0; i < 20; i++) {
      RunGB(bctL, bctR);
    }
    while (1) {
      RunGB(bctL, bctR);
      ReadCalibrateC();
      if (C[CCL] >= RefC || C[CCR] >= RefC) {
        break;
      }
    }
    while (1) {
      RunGB(bctL, bctR);
      ReadCalibrateB();
      if ((B[3] > Ref || B[12] > Ref)) {
        Motor(bctL, bctR);
        delay(bct_delay_break);
        Motor(1, 1);
        delay(1);
        MotorStop();
        BZoff();
        break;
      }
    }
  } else if (select == 's') {
    Motor(spd, spd);
    delay(delay_break_b);
    Motor(1, 1);
    delay(1);
    MotorStop();
  } else if (select == 'S') {
    ToBackG();
    Motor(bctL, bctR);
    delay(bct_delay_break);
    Motor(1, 1);
    delay(1);
    MotorStop();
  } else if (select == 'G') {
    ToBackG();
    SetG(bct_delay_break);
  } else {
    SetG(spd);
  }
}

// ======================================================================
// [13] วิ่งตรงจนเจอเส้น / ระยะ + ทางแยก
// ======================================================================

void ffbg(int Speed, char select) {
  BaseSpeed = Speed;
  InitialSpeed();
  while (1) {
    RunG(LeftBaseSpeed, RightBaseSpeed);
    ReadCalibrateF();
    if (F[2] > Ref || F[4] > Ref || F[6] > Ref || F[9] > Ref || F[11] > Ref || F[13] > Ref) break;
  }
  TrackSelectG(Speed, select);
}

void bbbg(int Speed, char select) {
  BaseSpeed = Speed;
  InitialSpeed();
  while (1) {
    RunGB(BackLeftBaseSpeed, BackRightBaseSpeed);
    ReadCalibrateB();
    if (B[2] > Ref || B[4] > Ref || B[6] > Ref || B[9] > Ref || B[11] > Ref || B[13] > Ref) break;
  }
  TrackSelectGB(Speed, select);
}

void ffbg(int Speed, char select, int direction) {
  SetDirectionG(direction);
  ffbg(Speed, select);
}
void bbbg(int Speed, char select, int direction) {
  SetDirectionG(direction);
  bbbg(Speed, select);
}

void ffdg(int Speed, char select, float distance_cm) {
  BaseSpeed = Speed;
  InitialSpeed();
  while (1) {
    RunG(LeftBaseSpeed, RightBaseSpeed);
    if (readDistance() >= distance_cm) break;
  }
  TrackSelectG(Speed, select);
}

void bbdg(int Speed, char select, float distance_cm) {
  BaseSpeed = Speed;
  InitialSpeed();
  while (1) {
    RunGB(BackLeftBaseSpeed, BackRightBaseSpeed);
    if (readDistance() <= distance_cm) break;
  }
  TrackSelectGB(Speed, select);
}

void ffdgs(int Speed, char select, float distance_cm) {
  BaseSpeed = Speed;
  InitialSpeed();
  while (1) {
    RunG(LeftBaseSpeed, RightBaseSpeed);
    if (readDistance() >= distance_cm) break;
  }
  TrackSelectG(Speed, select);
}

void bbdgs(int Speed, char select, float distance_cm) {
  BaseSpeed = Speed;
  InitialSpeed();
  while (1) {
    RunGB(BackLeftBaseSpeed, BackRightBaseSpeed);
    if (readDistance() <= distance_cm) break;
  }
  TrackSelectGB(Speed, select);
}

void fftimerg(int Speed, int totalTime, char select) {
  fftimerg(Speed, totalTime);
  TrackSelectG(Speed, select);
}
void bbtimerg(int Speed, int totalTime, char select) {
  bbtimerg(Speed, totalTime);
  TrackSelectGB(Speed, select);
}

void fftg(int Speed, int totalTime, char select) {
  fftimerg(Speed, totalTime);
  TrackSelectG(Speed, select);
}
void bbtg(int Speed, int totalTime, char select) {
  bbtimerg(Speed, totalTime);
  TrackSelectGB(Speed, select);
}

void ffcmgs(int Speed, float distance_cm, char select) {
  ffcmgs(Speed, distance_cm);
  TrackSelectG(Speed, select);
}
void bbcmgs(int Speed, float distance_cm, char select) {
  bbcmgs(Speed, distance_cm);
  TrackSelectGB(Speed, select);
}

void ffcmg(int Speed, float distance_cm, char select) {
  ffcmg(Speed, distance_cm);
  TrackSelectG(Speed, select);
}
void bbcmg(int Speed, float distance_cm, char select) {
  bbcmg(Speed, distance_cm);
  TrackSelectGB(Speed, select);
}

void setg(int time) {
  SetG(time);
}
void setgb(int time) {
  SetGB(time);
}

// ======================================================================
// [14] ทางแยก + ล็อกทิศ
// ======================================================================

void fftimerg(int Speed, int totalTime, char select, int direction) {
  fftimerg(Speed, totalTime, direction);
  TrackSelectG(Speed, select);
}
void bbtimerg(int Speed, int totalTime, char select, int direction) {
  bbtimerg(Speed, totalTime, direction);
  TrackSelectGB(Speed, select);
}

void fftg(int Speed, int totalTime, char select, int direction) {
  fftimerg(Speed, totalTime, direction);
  TrackSelectG(Speed, select);
}
void bbtg(int Speed, int totalTime, char select, int direction) {
  bbtimerg(Speed, totalTime, direction);
  TrackSelectGB(Speed, select);
}

void ffcmgs(int Speed, float distance_cm, char select, int direction) {
  ffcmgs(Speed, distance_cm, direction);
  TrackSelectG(Speed, select);
}
void bbcmgs(int Speed, float distance_cm, char select, int direction) {
  bbcmgs(Speed, distance_cm, direction);
  TrackSelectGB(Speed, select);
}

void ffcmg(int Speed, float distance_cm, char select, int direction) {
  ffcmg(Speed, distance_cm, direction);
  TrackSelectG(Speed, select);
}
void bbcmg(int Speed, float distance_cm, char select, int direction) {
  bbcmg(Speed, distance_cm, direction);
  TrackSelectGB(Speed, select);
}

// ======================================================================
// [15] วิ่งตามเส้นจนหันถึงทิศ (ffdir / bbdir)
// ======================================================================
// วิ่งตามเส้น (PIDF / PIDB) ไปเรื่อย ๆ จนมุมไจโรถึงทิศสัมบูรณ์ (0 / 90 / 180 / 270 อ้างอิงตอน resetAngles)
// ใช้กับเส้นโค้ง เช่น ffdir(50, 270); = วิ่งตามเส้นโค้งไปจนหุ่นหันไปทิศ 270
// ไม่มี select: จบแล้วมอเตอร์ยังหมุนอยู่ ต่อคำสั่งถัดไปได้ทันที

float line_dir_threshold = 5.0f;  // ถือว่าถึงทิศเมื่อ |error| <= ค่านี้ (องศา)

void set_line_dir_threshold(float thr) {
  line_dir_threshold = thr;
}

// true = มุมปัจจุบันถึงทิศ direction แล้ว (อยู่ในช่วง threshold หรือหมุนเลยเป้าไปแล้ว)
static bool reachedDirection(int direction, float &previous_error) {
  float error = wrap180(fmodf((float)direction, 360.0f) - angleRead());
  // มุมอาจกระโดดข้ามช่วง threshold ตอนเลี้ยวเร็ว จึงถือว่าถึงเมื่อ error เปลี่ยนเครื่องหมายด้วย
  // (|error| < 90 กันกรณี error กระโดดจาก +180 เป็น -180 ตอน wrap)
  bool crossed = fabsf(error) < 90.0f && ((error > 0 && previous_error < 0) || (error < 0 && previous_error > 0));
  previous_error = error;
  return fabsf(error) <= line_dir_threshold || crossed;
}

// Kp / Kd ติดลบ = ใช้ค่าจากตารางความเร็ว (PID_KP_Front / PID_KD_Front)
static void ffdir_pid(int Speed, int direction, float Kp, float Kd) {
  BaseSpeed = Speed;
  InitialSpeed();
  if (Kp < 0) Kp = PID_KP_Front;
  if (Kd < 0) Kd = PID_KD_Front;
  float previous_error = wrap180(fmodf((float)direction, 360.0f) - angleReadFresh());
  while (!reachedDirection(direction, previous_error)) PIDF(LeftBaseSpeed, RightBaseSpeed, Kp, Kd);
}

// Kp / Kd ติดลบ = ใช้ค่าจากตารางความเร็ว (PID_KP_Back / PID_KD_Back)
static void bbdir_pid(int Speed, int direction, float Kp, float Kd) {
  BaseSpeed = Speed;
  InitialSpeed();
  if (Kp < 0) Kp = PID_KP_Back;
  if (Kd < 0) Kd = PID_KD_Back;
  float previous_error = wrap180(fmodf((float)direction, 360.0f) - angleReadFresh());
  while (!reachedDirection(direction, previous_error)) PIDB(BackLeftBaseSpeed, BackRightBaseSpeed, Kp, Kd);
}

void ffdir(int Speed, int direction) {
  ffdir_pid(Speed, direction, -1, -1);
}

void bbdir(int Speed, int direction) {
  bbdir_pid(Speed, direction, -1, -1);
}

void ffdir(int Speed, int direction, char select) {
  ffdir(Speed, direction);
  TrackSelectF(Speed, select);
}

void bbdir(int Speed, int direction, char select) {
  bbdir(Speed, direction);
  TrackSelectB(Speed, select);
}

// --- กำหนด Kp / Kd เอง: (speed, kp, [kd,] ทิศ [, select]) ---
void ffdir(int Speed, float Kp, int direction) {
  ffdir_pid(Speed, direction, Kp, -1);
}

void bbdir(int Speed, float Kp, int direction) {
  bbdir_pid(Speed, direction, Kp, -1);
}

void ffdir(int Speed, float Kp, int direction, char select) {
  ffdir_pid(Speed, direction, Kp, -1);
  TrackSelectF(Speed, select);
}

void bbdir(int Speed, float Kp, int direction, char select) {
  bbdir_pid(Speed, direction, Kp, -1);
  TrackSelectB(Speed, select);
}

void ffdir(int Speed, float Kp, float Kd, int direction) {
  ffdir_pid(Speed, direction, Kp, Kd);
}

void bbdir(int Speed, float Kp, float Kd, int direction) {
  bbdir_pid(Speed, direction, Kp, Kd);
}

void ffdir(int Speed, float Kp, float Kd, int direction, char select) {
  ffdir_pid(Speed, direction, Kp, Kd);
  TrackSelectF(Speed, select);
}

void bbdir(int Speed, float Kp, float Kd, int direction, char select) {
  bbdir_pid(Speed, direction, Kp, Kd);
  TrackSelectB(Speed, select);
}

// --- วิ่งโค้ง cl = เบี่ยงซ้าย (set_positionL), cr = เบี่ยงขวา (set_positionR) ---

void ffdircl(int Speed, int direction) {
  int temp = set_position;
  set_position = set_positionL;
  ffdir(Speed, direction);
  set_position = temp;
}

void ffdircr(int Speed, int direction) {
  int temp = set_position;
  set_position = set_positionR;
  ffdir(Speed, direction);
  set_position = temp;
}

void bbdircl(int Speed, int direction) {
  int temp = set_position;
  set_position = set_positionL;
  bbdir(Speed, direction);
  set_position = temp;
}

void bbdircr(int Speed, int direction) {
  int temp = set_position;
  set_position = set_positionR;
  bbdir(Speed, direction);
  set_position = temp;
}

void ffdircl(int Speed, int direction, char select) {
  int temp = set_position;
  set_position = set_positionL;
  ffdir(Speed, direction, select);
  set_position = temp;
}

void ffdircr(int Speed, int direction, char select) {
  int temp = set_position;
  set_position = set_positionR;
  ffdir(Speed, direction, select);
  set_position = temp;
}

void bbdircl(int Speed, int direction, char select) {
  int temp = set_position;
  set_position = set_positionL;
  bbdir(Speed, direction, select);
  set_position = temp;
}

void bbdircr(int Speed, int direction, char select) {
  int temp = set_position;
  set_position = set_positionR;
  bbdir(Speed, direction, select);
  set_position = temp;
}

// --- *dircl/cr แบบกำหนด Kp / Kd เอง: (speed, kp, [kd,] ทิศ [, select]) ---

void ffdircl(int Speed, float Kp, float Kd, int direction) {
  int temp = set_position;
  set_position = set_positionL;
  ffdir(Speed, Kp, Kd, direction);
  set_position = temp;
}

void ffdircr(int Speed, float Kp, float Kd, int direction) {
  int temp = set_position;
  set_position = set_positionR;
  ffdir(Speed, Kp, Kd, direction);
  set_position = temp;
}

void bbdircl(int Speed, float Kp, float Kd, int direction) {
  int temp = set_position;
  set_position = set_positionL;
  bbdir(Speed, Kp, Kd, direction);
  set_position = temp;
}

void bbdircr(int Speed, float Kp, float Kd, int direction) {
  int temp = set_position;
  set_position = set_positionR;
  bbdir(Speed, Kp, Kd, direction);
  set_position = temp;
}

void ffdircl(int Speed, float Kp, float Kd, int direction, char select) {
  int temp = set_position;
  set_position = set_positionL;
  ffdir(Speed, Kp, Kd, direction, select);
  set_position = temp;
}

void ffdircr(int Speed, float Kp, float Kd, int direction, char select) {
  int temp = set_position;
  set_position = set_positionR;
  ffdir(Speed, Kp, Kd, direction, select);
  set_position = temp;
}

void bbdircl(int Speed, float Kp, float Kd, int direction, char select) {
  int temp = set_position;
  set_position = set_positionL;
  bbdir(Speed, Kp, Kd, direction, select);
  set_position = temp;
}

void bbdircr(int Speed, float Kp, float Kd, int direction, char select) {
  int temp = set_position;
  set_position = set_positionR;
  bbdir(Speed, Kp, Kd, direction, select);
  set_position = temp;
}

void ffdircl(int Speed, float Kp, int direction) {
  ffdircl(Speed, Kp, -1.0f, direction);
}

void ffdircr(int Speed, float Kp, int direction) {
  ffdircr(Speed, Kp, -1.0f, direction);
}

void bbdircl(int Speed, float Kp, int direction) {
  bbdircl(Speed, Kp, -1.0f, direction);
}

void bbdircr(int Speed, float Kp, int direction) {
  bbdircr(Speed, Kp, -1.0f, direction);
}

void ffdircl(int Speed, float Kp, int direction, char select) {
  ffdircl(Speed, Kp, -1.0f, direction, select);
}

void ffdircr(int Speed, float Kp, int direction, char select) {
  ffdircr(Speed, Kp, -1.0f, direction, select);
}

void bbdircl(int Speed, float Kp, int direction, char select) {
  bbdircl(Speed, Kp, -1.0f, direction, select);
}

void bbdircr(int Speed, float Kp, int direction, char select) {
  bbdircr(Speed, Kp, -1.0f, direction, select);
}

#endif  // MYMINI_16PRO_GYRO_H
