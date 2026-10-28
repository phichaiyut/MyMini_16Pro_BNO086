#ifndef MYMINI_16PRO_PID_H
#define MYMINI_16PRO_PID_H

/*
 * MyMini_16Pro_BNO086_PID.h
 * เดินตามเส้นด้วย PID, เลี้ยว/หมุนด้วยเซนเซอร์ และคำสั่งทางแยก
 *
 * ที่มา: ตรรกะและชื่อคำสั่งจาก MyRP_Pico2_mini_BNO086 (8 เซนเซอร์)
 *        ปรับเป็นเซนเซอร์จริง 16 ตัว F[0..15] / B[0..15] อ้างอิง UnknowRobot_STORMXII_V3_BNO085
 *   - ตำแหน่งเส้น 0-15000 (เซนเซอร์ละ 1000) กลาง = 7500, เลือกช่วงเซนเซอร์ด้วย set_sensor_track_line()
 *   - ตรวจผ่านแยก / ชิดเส้น (TrackSelect, ToFront, ff, bb) ใช้ F[3] / F[12]
 *   - เจอแยกตัด (ffc / bbc) ใช้เงื่อนไขเส้นกว้างทั้งสองฝั่งแบบ STORMXII
 *   - หมุน / เลี้ยว / บาลานซ์ ใช้ขอบนอก F[0] / F[15] และเซนเซอร์หยุด 2 / 4 / 6 (13 / 11 / 9)
 *   - B[] กลับลำดับ (B_PIN = 15..0) แบบ Pico2 / STORMXII
 *
 * สารบัญ:
 *   [1] ตัวแปรและค่าเริ่มต้น
 *   [2] ฟังก์ชันตั้งค่า (Config)
 *   [3] อ่านตำแหน่งเส้น (readPosition)
 *   [4] PID เดินตามเส้น (PIDF / PIDB)
 *   [5] เดินตามเวลา / ระยะทาง (fftimer, ffcm, lf, lb)
 *   [6] เข้ากลางหุ่น / ชิดเส้น (ToCenter, ToFront, ToBack)
 *   [7] เลี้ยว / หมุน ด้วยเซนเซอร์หน้า
 *   [8] เลี้ยว / หมุน ด้วยเซนเซอร์หลัง
 *   [9] ทางแยก (TrackSelectF / TrackSelectB)
 *   [10] เดินตามเวลา / ระยะทาง + ทางแยก
 *   [11] เดินตามเส้นจนเจอแยก (ff, bb, ffl, ffr, ffb, ffw ...)
 *   [12] หยุดตามระยะเซนเซอร์ (ff_distances)
 *   [13] ออกจากจุดเริ่ม / เข้าจุดจบ (gostart / goend)
 *   [14] ตั้งหุ่นตรงเส้น (balance / set)
 *   [15] Serial debug ตำแหน่งเส้น
 *   [16] วิ่งโค้ง (CL / CR: เบี่ยงซ้าย/ขวา ด้วย set_positionL/R ชั่วคราว)
 */

#include "MyMini_16Pro_BNO086_Buzzer.h"
#include "MyMini_16Pro_BNO086_Motor.h"
#include "MyMini_16Pro_BNO086_Sensor.h"

// ======================================================================
// [1] ตัวแปรและค่าเริ่มต้น
// ======================================================================

// LastError_F / LastError_B / LastError_F_none / LastError_B_none ประกาศไว้ใน Motor.h (InitialSpeed() รีเซ็ตค่า)
int tct, bct, tspd;
int tctL, tctR, bctL, bctR;
int LTurnSpdL, LTurnSpdR, TurnDelayL;
int RTurnSpdL, RTurnSpdR, TurnDelayR;
int LTurnBackSpdL, LTurnBackSpdR, TurnBackDelayL;
int RTurnBackSpdL, RTurnBackSpdR, TurnBackDelayR;
int LTurnBackFSpdL, LTurnBackFSpdR, TurnBackFDelayL;
int RTurnBackFSpdL, RTurnBackFSpdR, TurnBackFDelayR;
int LTurnBackBSpdL, LTurnBackBSpdR, TurnBackBDelayL;
int RTurnBackBSpdL, RTurnBackBSpdR, TurnBackBDelayR;
int LChTurn = 5, RChTurn = 10;  // เซนเซอร์หน้า (0-15) ที่ใช้หยุด TurnLeft / TurnRight (ค่าเริ่มต้นแบบ STORMXII)
int tct_delay_break = 0;
int bct_delay_break = 0;
int break_ff = 5, break_fc = 30, break_bf = 10, break_bc = 20;  // การหน่วง
// ตำแหน่งเส้น 0-15000 (เซนเซอร์ละ 1000) กลาง = 7500
int set_position = 7500;
int set_positionL = 1500;
int set_positionR = 13500;
int setsensortracklineL = 0, setsensortracklineR = 15;  // ช่วงเซนเซอร์ที่ใช้คำนวณตำแหน่งเส้น
float slow_kp_f = 0.005, slow_kd_f = 0.05;
float slow_kp_b = 0.005, slow_kd_b = 0.05;
float slow_kpf = 0.005, slow_kdf = 0.05;
float slow_kpb = 0.005, slow_kdb = 0.05;
float lf_kp = 0.015, lf_kd = 0.10;  // PID อยู่กับที่ lf()
float lb_kp = 0.015, lb_kd = 0.10;  // PID อยู่กับที่ lb()
int line_centor = 0;

int MaxSpeed = 100;
int MinSpeed = -5;
int ModePidStatus = 0;
int pid_mode2_up = 20, pid_mode2_down = 20;  // ModePidStatus 2: ช่วงกำลังล้อ = -(Speed+down)..(Speed+up)
int dottedline = 0;

float speed_scale_fw = 1.75;  // <-- ใช้ค่าที่คำนวณจากการวัดจริง
float speed_scale_bw = 1.75;  // <-- ใช้ค่าที่คำนวณจากการวัดจริง

// ค่า Error ที่เล็กกว่านี้ (หน่วยเดียวกับ set_position, 0-15000) จะถูกมองเป็น 0
// เพื่อกันไม่ให้ PID สั่นตามสัญญาณรบกวนตอนวิ่งตรงกลางเส้น
int PID_DeadBand = 20;

void SetPIDDeadBand(int db) {
  PID_DeadBand = db;
}

// จำนวนครั้งที่ต้องอ่านค่าซ้ำ ๆ ให้ผลตรงกันติดต่อกัน ก่อนจะยอมรับว่า
// "พ้นเส้นเดิม" หรือ "เจอเส้นใหม่" แล้วจริง ๆ (กันสัญญาณรบกวนที่ขอบเส้นทำให้
// spinl()/spinr() หยุดเร็วเกินไปโดยยังไม่พ้นเส้นเดิม)
int SpinDebounceCount = 5;

// ======================================================================
// [2] ฟังก์ชันตั้งค่า (Config)
// ======================================================================

void SetFG(int time);
void SetGB(int totalTime);
void SetBG(int time);
void SetRobotAngle();

void SetToCenterSpeed(int tctv) {
  tct = tctv;
  bct = tctv;
  BaseSpeed = tctv;
  InitialSpeed();
  // Front / back base speed
  tctL = LeftBaseSpeed;
  tctR = RightBaseSpeed;
  bctL = BackLeftBaseSpeed;
  bctR = BackRightBaseSpeed;

  // Slow PID
  slow_kp_f = PID_KP_Front;
  slow_kd_f = PID_KD_Front;
  slow_kp_b = PID_KP_Back;
  slow_kd_b = PID_KD_Back;

  tct_delay_break = delay_break_f;
  bct_delay_break = delay_break_b;
}

void set_slow_kp_kd(float kp_f, float kd_f, float kp_b, float kd_b) {
  slow_kp_f = kp_f;
  slow_kd_f = kd_f;
  slow_kp_b = kp_b;
  slow_kd_b = kd_b;
}

void Dottedline(int x) {
  dottedline = x;
}

void SetSensorTurnLeftRight(int l, int r) {
  LChTurn = l;
  RChTurn = r;
}

void SetTurnSpeed(int tspdv) {
  tspd = tspdv;
}

void TurnSpeedLeft(int l, int r, int de) {
  LTurnSpdL = l;
  LTurnSpdR = r;
  TurnDelayL = de;
}

void TurnSpeedRight(int l, int r, int de) {
  RTurnSpdL = l;
  RTurnSpdR = r;
  TurnDelayR = de;
}

void TurnBackSpeedLeft(int l, int r, int de) {
  LTurnBackSpdL = l;
  LTurnBackSpdR = r;
  TurnBackDelayL = de;
}

void TurnBackSpeedRight(int l, int r, int de) {
  RTurnBackSpdL = l;
  RTurnBackSpdR = r;
  TurnBackDelayR = de;
}

// ความเร็ว/ดีเลย์ของ TurnLeftBackF()/TurnRightBackF() แยกต่างหากจาก
// TurnSpeedLeft()/TurnSpeedRight() (ที่ใช้กับ TurnLeft()/TurnRight() ฝั่งเดินหน้า)
// เพื่อให้ปรับความเร็วตอนเลี้ยวล้อเดียวถอยหลังได้โดยไม่กระทบการเลี้ยวเดินหน้า
void TurnSpeedLeftBackF(int l, int r, int de) {
  LTurnBackFSpdL = l;
  LTurnBackFSpdR = r;
  TurnBackFDelayL = de;
}

void TurnSpeedRightBackF(int l, int r, int de) {
  RTurnBackFSpdL = l;
  RTurnBackFSpdR = r;
  TurnBackFDelayR = de;
}

// ความเร็ว/ดีเลย์ของ TurnLeftBackB()/TurnRightBackB() (เลี้ยวล้อเดียวถอยหลัง
// เช็คเส้นด้วยเซนเซอร์หลัง B[]) แยกต่างหากจากทุกชุดข้างต้น
void TurnSpeedLeftBackB(int l, int r, int de) {
  LTurnBackBSpdL = l;
  LTurnBackBSpdR = r;
  TurnBackBDelayL = de;
}

void TurnSpeedRightBackB(int l, int r, int de) {
  RTurnBackBSpdL = l;
  RTurnBackBSpdR = r;
  TurnBackBDelayR = de;
}


void set_brake_fc(int ff, int fc) {
  break_ff = ff;
  break_fc = fc;
}
void set_brake_bc(int ff, int fc) {
  break_bf = ff;
  break_bc = fc;
}

void ModeSpdPID(int moD, int maX, int miN) {
  ModePidStatus = moD;
  MaxSpeed = maX;
  MinSpeed = miN;
}

void set_position_line(int _pos) {
  if (_pos < 0) {
    set_position = 0;
  } else if (_pos > 15000) {
    set_position = 15000;
  } else {
    set_position = _pos;
  }
}

void set_position_line_l(int _pos) {
  if (_pos < 0) {
    set_positionL = 0;
  } else if (_pos > 15000) {
    set_positionL = 15000;
  } else {
    set_positionL = _pos;
  }
}

void set_position_line_r(int _pos) {
  if (_pos < 0) {
    set_positionR = 0;
  } else if (_pos > 15000) {
    set_positionR = 15000;
  } else {
    set_positionR = _pos;
  }
}

// เลือกช่วงเซนเซอร์ที่ใช้คำนวณตำแหน่งเส้น (แบบ STORMXII): 0,15 | 2,13 | 3,12 | 4,11
// ตำแหน่งยังเป็น 0-15000 เท่าเดิม (เซนเซอร์ตัวที่ i = i * 1000)
void set_sensor_track_line(int L, int R) {
  setsensortracklineL = constrain(L, 0, NUM_SENSORS - 1);
  setsensortracklineR = constrain(R, setsensortracklineL, NUM_SENSORS - 1);
}

void set_line_center(int x) {
  line_centor = x;
}

void set_speed_scale(float fw, float bw) {
  speed_scale_fw = fw;
  speed_scale_bw = bw;
}

void set_pid_lf(float Kp, float Kd) {
  lf_kp = Kp;
  lf_kd = Kd;
}

void set_pid_lb(float Kp, float Kd) {
  lb_kp = Kp;
  lb_kd = Kd;
}

void set_pid_mode2_limit(int up, int down) {
  pid_mode2_up = up;
  pid_mode2_down = down;
}

// ======================================================================
// [3] อ่านตำแหน่งเส้น (readPosition)
// ======================================================================

// ตามต้นฉบับ UnknowRobot_STORMXII_V3 (16CH): ตำแหน่ง 0-15000 ใช้เซนเซอร์ setsensortracklineL..R
int readPositionF(int Track, int noise) {
  int i;
  bool online = false;
  unsigned long avg = 0;
  unsigned long sum = 0;
  static int last_value = (NUM_SENSORS - 1) * 1000 / 2;
  ReadCalibrateF();
  for (i = setsensortracklineL; i <= setsensortracklineR; i++) {
    int values = F[i];
    if (values > Track) online = true;
    if (values > noise) {
      avg += (long)(values) * (i * 1000);
      sum += values;
    }
  }
  if (!online) {
    if (last_value < (NUM_SENSORS - 1) * 1000 / 2)
      return setsensortracklineL * 1000;
    else
      return setsensortracklineR * 1000;
  }
  if (sum == 0) return last_value;
  last_value = avg / sum;
  return last_value;
}

int readPositionB(int Track, int noise) {
  int i;
  bool online = false;
  unsigned long avg = 0;
  unsigned long sum = 0;
  static int last_value = (NUM_SENSORS - 1) * 1000 / 2;
  ReadCalibrateB();
  for (i = setsensortracklineL; i <= setsensortracklineR; i++) {
    int values = B[i];
    if (values > Track) online = true;
    if (values > noise) {
      avg += (long)(values) * (i * 1000);
      sum += values;
    }
  }
  if (!online) {
    if (last_value < (NUM_SENSORS - 1) * 1000 / 2)
      return setsensortracklineL * 1000;
    else
      return setsensortracklineR * 1000;
  }
  if (sum == 0) return last_value;
  last_value = avg / sum;
  return last_value;
}

int readPositionF_none(int Track, int noise) {
  unsigned char i, online = 0;
  unsigned long avg = 0;
  unsigned long sum = 0;
  static int last_value = (NUM_SENSORS - 1) * 1000 / 2;
  ReadCalibrateF();
  for (i = setsensortracklineL; i < setsensortracklineR; i++) {
    int values = F[i];
    if (values > Track) online = 1;
    if (values > noise) {
      avg += (long)(values) * (i * 1000);
      sum += values;
    }
  }
  if (!online) {
    if (last_value < (NUM_SENSORS - 1) * 1000 / 2)
      return 7500;
    else
      return 7500;
  }
  last_value = avg / sum;
  return last_value;
}

int readPositionB_none(int Track, int noise) {
  unsigned char i, online = 0;
  unsigned long avg = 0;
  unsigned long sum = 0;
  static int last_value = (NUM_SENSORS - 1) * 1000 / 2;
  ReadCalibrateB();
  for (i = setsensortracklineL; i < setsensortracklineR; i++) {
    int values = B[i];
    if (values > Track) online = 1;
    if (values > noise) {
      avg += (long)(values) * (i * 1000);
      sum += values;
    }
  }
  if (!online) {
    if (last_value < (NUM_SENSORS - 1) * 1000 / 2)
      return 7500;
    else
      return 7500;
  }
  last_value = avg / sum;
  return last_value;
}

// ======================================================================
// [4] PID เดินตามเส้น (PIDF / PIDB)
// ======================================================================

// จำกัดค่า LeftPower/RightPower ตาม ModePidStatus (ใช้ร่วมกันทั้ง PIDF และ PIDB)
void ClampPIDPower(float &LeftPower, float &RightPower, int SpeedL, int SpeedR) {
  switch (ModePidStatus) {
    case 0:
      if (LeftPower > MaxSpeed) LeftPower = MaxSpeed;
      if (LeftPower < 0) LeftPower = MinSpeed;
      if (RightPower > MaxSpeed) RightPower = MaxSpeed;
      if (RightPower < 0) RightPower = MinSpeed;
      break;
    case 1:
      if (LeftPower > MaxSpeed) LeftPower = MaxSpeed;
      if (LeftPower < MinSpeed) LeftPower = MinSpeed;
      if (RightPower > MaxSpeed) RightPower = MaxSpeed;
      if (RightPower < MinSpeed) RightPower = MinSpeed;
      break;
    case 2:
      if (LeftPower > SpeedL + pid_mode2_up) LeftPower = SpeedL + pid_mode2_up;
      if (LeftPower < -SpeedL - pid_mode2_down) LeftPower = -SpeedL - pid_mode2_down;
      if (RightPower > SpeedR + pid_mode2_up) RightPower = SpeedR + pid_mode2_up;
      if (RightPower < -SpeedR - pid_mode2_down) RightPower = -SpeedR - pid_mode2_down;
      break;
    case 3:
      if (LeftPower > MaxSpeed) LeftPower = MaxSpeed;
      if (LeftPower < 0) LeftPower = -BaseSpeed;
      if (RightPower > MaxSpeed) RightPower = MaxSpeed;
      if (RightPower < 0) RightPower = -BaseSpeed;
      break;
    default:
      if (LeftPower > MaxSpeed) LeftPower = MaxSpeed;
      if (LeftPower < 0) LeftPower = 0;
      if (RightPower > MaxSpeed) RightPower = MaxSpeed;
      if (RightPower < 0) RightPower = 0;
  }
}

void PIDF(int SpeedL, int SpeedR, float Kp, float Kd) {
  float Pos = readPositionF(200, 50);
  float Error = Pos - set_position;
  if (fabs(Error) < PID_DeadBand) Error = 0;
  float PID_Value = (Kp * Error) + (Kd * (Error - LastError_F));
  LastError_F = Error;
  float LeftPower = SpeedL + PID_Value;
  float RightPower = SpeedR - PID_Value;
  ClampPIDPower(LeftPower, RightPower, SpeedL, SpeedR);
  Motor(LeftPower, RightPower);
}

void PIDB(int SpeedL, int SpeedR, float Kp, float Kd) {
  float Pos = readPositionB(200, 50);
  float Error = Pos - set_position;
  if (fabs(Error) < PID_DeadBand) Error = 0;
  float PID_Value = (Kp * Error) + (Kd * (Error - LastError_B));
  LastError_B = Error;
  float LeftPower = SpeedL + PID_Value;
  float RightPower = SpeedR - PID_Value;
  ClampPIDPower(LeftPower, RightPower, SpeedL, SpeedR);
  Motor(-LeftPower, -RightPower);
}

static bool frontCenterLine() {
  return (F[6] > Ref && F[7] > Ref)
         || (F[7] > Ref && F[8] > Ref)
         || (F[8] > Ref && F[9] > Ref);
}

void PIDF_none(int SpeedL, int SpeedR, float Kp, float Kd) {
  ReadCalibrateF();
  float Pos;
  if (frontCenterLine()) {
    Pos = set_position;
  } else {
    Pos = readPositionF_none(200, 50);
  }

  float Error = Pos - set_position;
  float PID_Value = (Kp * Error) + (Kd * (Error - LastError_F_none));
  LastError_F_none = Error;

  float LeftPower = SpeedL + PID_Value;
  float RightPower = SpeedR - PID_Value;

  LeftPower = constrain(LeftPower, -100, 100);
  RightPower = constrain(RightPower, -100, 100);

  Motor(LeftPower, RightPower);
}

static bool backCenterLine() {
  return (B[6] > Ref && B[7] > Ref)
         || (B[7] > Ref && B[8] > Ref)
         || (B[8] > Ref && B[9] > Ref);
}

void PIDB_none(int SpeedL, int SpeedR, float Kp, float Kd) {
  float Pos;
  ReadCalibrateB();
  if (backCenterLine()) {
    Pos = set_position;
  } else {
    Pos = readPositionB_none(200, 50);
  }
  float Error = Pos - set_position;
  float PID_Value = (Kp * Error) + (Kd * (Error - LastError_B_none));
  LastError_B_none = Error;
  float LeftPower = SpeedL + PID_Value;
  float RightPower = SpeedR - PID_Value;
  LeftPower = constrain(LeftPower, -100, 100);
  RightPower = constrain(RightPower, -100, 100);
  Motor(-LeftPower, -RightPower);
}

// ======================================================================
// [5] เดินตามเวลา / ระยะทาง (fftimer, ffcm, lf, lb)
// ======================================================================

// Kp / Kd ติดลบ = ใช้ค่าจากตารางความเร็ว (PID_KP_Front / PID_KD_Front)
static void fftimer_pid(int baseSpeed, int totalTime, float Kp, float Kd) {
  BaseSpeed = baseSpeed;
  InitialSpeed();
  if (Kp < 0) Kp = PID_KP_Front;
  if (Kd < 0) Kd = PID_KD_Front;
  unsigned long endTime = millis() + totalTime;
  while (millis() <= endTime) PIDF(LeftBaseSpeed, RightBaseSpeed, Kp, Kd);
}

void fftimer(int baseSpeed, int totalTime) {
  fftimer_pid(baseSpeed, totalTime, -1, -1);
}

// Kp / Kd ติดลบ = ใช้ค่าจากตารางความเร็ว (PID_KP_Back / PID_KD_Back)
static void bbtimer_pid(int baseSpeed, int totalTime, float Kp, float Kd) {
  BaseSpeed = baseSpeed;
  InitialSpeed();
  if (Kp < 0) Kp = PID_KP_Back;
  if (Kd < 0) Kd = PID_KD_Back;
  unsigned long endTime = millis() + totalTime;
  while (millis() <= endTime) PIDB(BackLeftBaseSpeed, BackRightBaseSpeed, Kp, Kd);
}

void bbtimer(int baseSpeed, int totalTime) {
  bbtimer_pid(baseSpeed, totalTime, -1, -1);
}

void lf(int totalTime) {
  unsigned long endTime = millis() + totalTime;
  while (millis() <= endTime) PIDF(0, 0, lf_kp, lf_kd);
  MotorStop();
}

void lb(int totalTime) {
  unsigned long endTime = millis() + totalTime;
  while (millis() <= endTime) PIDB(0, 0, lb_kp, lb_kd);
  MotorStop();
}

// Kp / Kd ติดลบ = ใช้ค่าจากตารางความเร็ว (PID_KP_Front / PID_KD_Front)
static void ffcm_pid(int Speed, float distance, float Kp, float Kd) {
  BaseSpeed = Speed;
  InitialSpeed();
  if (Kp < 0) Kp = PID_KP_Front;
  if (Kd < 0) Kd = PID_KD_Front;
  int target_speed = min(LeftBaseSpeed, RightBaseSpeed);
  float traveled_distance = 0;
  unsigned long last_time = millis();
  while (1) {
    if (distance > 0) {
      unsigned long current_time = millis();
      float delta_time = (current_time - last_time) / 1000.0;
      traveled_distance += (target_speed * speed_scale_fw) * delta_time;
      last_time = current_time;

      if (traveled_distance >= distance) break;
    }
    PIDF(LeftBaseSpeed, RightBaseSpeed, Kp, Kd);
  }
}

void ffcm(int Speed, float distance) {
  ffcm_pid(Speed, distance, -1, -1);
}

// Kp / Kd ติดลบ = ใช้ค่าจากตารางความเร็ว (PID_KP_Back / PID_KD_Back)
static void bbcm_pid(int Speed, float distance, float Kp, float Kd) {
  BaseSpeed = Speed;
  InitialSpeed();
  if (Kp < 0) Kp = PID_KP_Back;
  if (Kd < 0) Kd = PID_KD_Back;
  int target_speed = min(BackLeftBaseSpeed, BackRightBaseSpeed);
  float traveled_distance = 0;
  unsigned long last_time = millis();
  while (1) {
    if (distance > 0) {
      unsigned long current_time = millis();
      float delta_time = (current_time - last_time) / 1000.0;
      traveled_distance += (target_speed * speed_scale_bw) * delta_time;
      last_time = current_time;

      if (traveled_distance >= distance) break;
    }
    PIDB(BackLeftBaseSpeed, BackRightBaseSpeed, Kp, Kd);
  }
}

void bbcm(int Speed, float distance) {
  bbcm_pid(Speed, distance, -1, -1);
}

// ======================================================================
// [6] เข้ากลางหุ่น / ชิดเส้น (ToCenter, ToFront, ToBack)
// ======================================================================

void ModeToCenter() {
  if (line_centor == 0) {
    Motor(tctL, tctR);
    delay(20);
  } else {
    for (int i = 0; i <= 20; i++) {
      PIDF_none(tctL, tctR, slow_kp_f, slow_kd_f);
    }
  }
}

void ModeToCenterBack() {
  if (line_centor == 0) {
    Motor(-bctL, -bctR);
    delay(20);
  } else {
    for (int i = 0; i <= 20; i++) {
      PIDB_none(bctL, bctR, slow_kp_b, slow_kd_b);
    }
  }
}

void ModeToCenterLine() {
  if (line_centor == 0) {
    Motor(tctL, tctR);
  } else {
    PIDF_none(tctL, tctR, slow_kp_f, slow_kd_f);
  }
}

void ModeToCenterBackLine() {
  if (line_centor == 0) {
    Motor(-bctL, -bctR);
  } else {
    PIDB_none(bctL, bctR, slow_kp_b, slow_kd_b);
  }
}

void ToCenter() {
  BZon();
  ModeToCenter();
  while (1) {
    ModeToCenterLine();
    ReadCalibrateC();
    if (C[CCL] >= RefC || C[CCR] >= RefC) {
      Motor(-tctL, -tctR);
      delay(break_fc);
      MotorStop();
      BZoff();
      break;
    }
  }
}

void ToCenterL() {
  BZon();
  ModeToCenter();
  while (1) {
    ModeToCenterLine();
    ReadCalibrateC();
    if (C[CCL] >= RefC) {
      Motor(-tctL, -tctR);
      delay(break_fc);
      MotorStop();
      BZoff();
      break;
    }
  }
}

void ToCenterR() {
  BZon();
  ModeToCenter();
  while (1) {
    ModeToCenterLine();
    ReadCalibrateC();
    if (C[CCR] >= RefC) {
      Motor(-tctL, -tctR);
      delay(break_fc);
      MotorStop();
      BZoff();
      break;
    }
  }
}

void BackCenter() {
  BZon();
  ModeToCenterBack();
  while (1) {
    ModeToCenterBackLine();
    ReadCalibrateC();
    if (C[CCL] >= RefC || C[CCR] >= RefC) {
      Motor(bctL, bctR);
      delay(break_bc);
      MotorStop();
      BZoff();
      break;
    }
  }
}

void ToFront() {
  while (1) {
    PIDF(tctL, tctR, slow_kp_f, slow_kd_f);
    ReadCalibrateF();
    if (F[3] > Ref || F[12] > Ref) break;
  }
}

void ToBack() {
  while (1) {
    PIDB(bctL, bctR, slow_kp_b, slow_kd_b);
    ReadCalibrateB();
    if (B[3] > Ref || B[12] > Ref) break;
  }
}

// ======================================================================
// [7] เลี้ยว / หมุน ด้วยเซนเซอร์หน้า
// ======================================================================

void TurnLeft() {
  // Motor(LTurnSpdL, LTurnSpdR);
  // delay(TurnDelayL);
  // while (1) {
  //   Motor(LTurnSpdL, LTurnSpdR);
  //   ReadCalibrateF();
  //   if (F[4] >= Ref) break;
  // }
  for (int i = 0; i <= LChTurn; i++) {
    do {
      Motor(LTurnSpdL, LTurnSpdR);
      delayMicroseconds(80);
      ReadCalibrateF();
    } while (F[i] <= Ref);
  }
}

void TurnRight() {
  // Motor(RTurnSpdL, RTurnSpdR);
  // delay(TurnDelayR);
  for (int i = 15; i >= RChTurn; i--) {
    do {
      Motor(RTurnSpdL, RTurnSpdR);
      delayMicroseconds(80);
      ReadCalibrateF();
    } while (F[i] <= Ref);
  }
}

// เลี้ยวล้อเดียวแบบถอยหลัง เช็คเส้นด้วยเซนเซอร์หน้า F[] (ต่างจาก TurnLeft_B()/
// TurnRight_B() ที่เปลี่ยนไปใช้เซนเซอร์หลัง B[]) ตั้งความเร็ว/ดีเลย์แยกต่างหากจาก
// TurnLeft()/TurnRight() ผ่าน TurnSpeedLeftBackF()/TurnSpeedRightBackF()
// (ให้ล้อฝั่งที่ไม่ต้องการหมุน = 0 เพื่อให้เป็นการเลี้ยวล้อเดียว)
void TurnLeftBackF() {
  // เลือกเซนเซอร์ตามความเร็วล้อที่หมุน เหมือน spinl(): ยิ่งหมุนช้า ยิ่งใช้
  // เซนเซอร์ที่ห่างออกไป (เผื่อระยะเหวี่ยงที่แคบลง)
  int spd = max(abs(LTurnBackFSpdL), abs(LTurnBackFSpdR));
  int sensorIdx;
  if (spd >= 80) sensorIdx = 13;
  else if (spd <= 50) sensorIdx = 9;
  else if (spd <= 70) sensorIdx = 11;
  else sensorIdx = 2;

  Motor(LTurnBackFSpdL, -LTurnBackFSpdR);
  delay(TurnBackFDelayL);
  while (1) {
    Motor(LTurnBackFSpdL, -LTurnBackFSpdR);
    ReadCalibrateF();
    if (F[sensorIdx] >= Ref) MotorStop();
    lf(spd);
    break;
  }
}

void TurnRightBackF() {
  // mirror ของ TurnLeftBackF() ไปฝั่งขวา (F[4]<->F[11], F[2]<->F[13], F[6]<->F[9])
  int spd = max(abs(RTurnBackFSpdL), abs(RTurnBackFSpdR));
  int sensorIdx;
  if (spd >= 80) sensorIdx = 2;
  else if (spd <= 50) sensorIdx = 6;
  else if (spd <= 70) sensorIdx = 4;
  else sensorIdx = 13;

  Motor(RTurnBackFSpdL, RTurnBackFSpdR);
  delay(TurnBackFDelayR);
  while (1) {
    Motor(RTurnBackFSpdL, RTurnBackFSpdR);
    ReadCalibrateF();
    if (F[sensorIdx] >= Ref) MotorStop();
    lf(spd);
    break;
  }
}

void spinl(int speed) {
  MotorStop();
  // delay(10);
  // Motor(-speed, speed);
  // delay(60);

  // เลือกเซนเซอร์ตามความเร็ว tspd: ยิ่งหมุนช้า ยิ่งใช้เซนเซอร์ที่ห่างออกไป
  // (เผื่อระยะเหวี่ยงที่แคบลง) ตรวจสอบจากช่วงแคบไปกว้างเพื่อให้ทุกเงื่อนไข
  // มีโอกาสถูกใช้จริง (เช็ค <=50 ก่อน <=70 ไม่งั้น <=50 จะไม่มีทางถูกเลือก)
  int sensorIdx;
  if (speed >= 80) sensorIdx = 2;
  else if (speed <= 60) sensorIdx = 6;
  else if (speed <= 70) sensorIdx = 4;
  else sensorIdx = 2;

  while (1) {
    ReadCalibrateF();
    Motor(-speed, speed);
    if (F[0] <= Ref) break;
  }
  for (int i = 0; i <= sensorIdx; i++) {
    do {
      Motor(-speed, speed);
      delayMicroseconds(80);
      ReadCalibrateF();
    } while (F[i] <= Ref);
  }
  Motor(speed, -speed);
  delay(5);
  lf(speed);
  MotorStop();
}

void spinl() {
  spinl(tspd);
}

void spinl2(int speed) {
  MotorStop();
  int sensorIdx;
  if (speed >= 80) sensorIdx = 2;
  else if (speed <= 60) sensorIdx = 6;
  else if (speed <= 70) sensorIdx = 4;
  else sensorIdx = 2;
  while (1) {
    ReadCalibrateF();
    Motor(-speed, speed);
    if (F[0] <= Ref) break;
  }
  for (int pass = 0; pass < 2; pass++) {
    for (int i = 0; i <= sensorIdx; i++) {
      do {
        Motor(-speed, speed);
        delayMicroseconds(80);
        ReadCalibrateF();
      } while (F[i] <= Ref);
    }
  }
  Motor(speed, -speed);
  delay(5);
  lf(speed);
  MotorStop();
}

void spinl2() {
  spinl2(tspd);
}

void spinr(int speed) {
  MotorStop();
  int sensorIdx;
  if (speed >= 80) sensorIdx = 13;
  else if (speed <= 60) sensorIdx = 9;
  else if (speed <= 70) sensorIdx = 11;
  else sensorIdx = 13;

  while (1) {
    ReadCalibrateF();
    Motor(speed, -speed);
    if (F[15] <= Ref) break;
  }
  for (int i = 15; i >= sensorIdx; i--) {
    do {
      Motor(speed, -speed);
      delayMicroseconds(80);
      ReadCalibrateF();
    } while (F[i] <= Ref);
  }
  Motor(-speed, speed);
  delay(5);
  lf(speed);
  MotorStop();
}

void spinr() {
  spinr(tspd);
}

void spinr2(int speed) {
  MotorStop();

  int sensorIdx;
  if (speed >= 80) sensorIdx = 13;
  else if (speed <= 60) sensorIdx = 9;
  else if (speed <= 70) sensorIdx = 11;
  else sensorIdx = 13;

  while (1) {
    ReadCalibrateF();
    Motor(speed, -speed);
    if (F[15] <= Ref) break;
  }

  for (int pass = 0; pass < 2; pass++) {
    for (int i = 15; i >= sensorIdx; i--) {
      do {
        Motor(speed, -speed);
        delayMicroseconds(80);
        ReadCalibrateF();
      } while (F[i] <= Ref);
    }
  }

  Motor(-speed, speed);
  delay(5);
  lf(speed);
  MotorStop();
}

void spinr2() {
  spinr2(tspd);
}

// ======================================================================
// [8] เลี้ยว / หมุน ด้วยเซนเซอร์หลัง
// ======================================================================
// แบบ STORMXII (16CH, B_PIN = 15..0): ฝั่งซ้ายไล่ B[0..] / ฝั่งขวาไล่ B[15..]

int LChTurnB = 5, RChTurnB = 10;  // เซนเซอร์หลังที่ใช้หยุด TurnLeft_B (B[0..5]) / TurnRight_B (B[15..10])

void SetSensorTurnLeftRight_B(int l, int r) {
  LChTurnB = l;
  RChTurnB = r;
}

// เลือกเซนเซอร์หยุดของการหมุนตามความเร็ว: ยิ่งหมุนเร็ว ยิ่งหยุดที่เซนเซอร์ขอบนอก
// (เผื่อระยะเหวี่ยง) ฝั่งขวาใช้ 15 - ค่านี้
int SpinStopSensor(int Speed) {
  if (Speed >= 80) return 2;
  else if (Speed <= 60) return 6;
  else if (Speed <= 70) return 4;
  else return 2;
}

// ---------- เลี้ยว (ความเร็วจาก TurnBackSpeedLeft / TurnBackSpeedRight) ----------
// หมุนจนเซนเซอร์หลังเจอเส้นไล่ทีละตัว B[0..LChTurnB] (ซ้าย) / B[15..RChTurnB] (ขวา)
void TurnLeft_B() {
  for (int i = 0; i <= LChTurnB; i++) {
    do {
      Motor(LTurnBackSpdL, LTurnBackSpdR);
      delayMicroseconds(80);
      ReadCalibrateB();
    } while (B[i] <= Ref);
  }
  MotorStop();
}

void TurnRight_B() {
  for (int i = 15; i >= RChTurnB; i--) {
    do {
      Motor(RTurnBackSpdL, RTurnBackSpdR);
      delayMicroseconds(80);
      ReadCalibrateB();
    } while (B[i] <= Ref);
  }
  MotorStop();
}

// ชื่อใหม่ของ MyRP_Pico2_mini_BNO086
void TurnLeftB() {
  TurnLeft_B();
}

void TurnRightB() {
  TurnRight_B();
}

// ---------- เลี้ยวล้อเดียวถอยหลัง เช็คเซนเซอร์หลัง (TurnSpeedLeftBackB / TurnSpeedRightBackB) ----------
void TurnLeftBackB() {
  int spd = max(abs(LTurnBackBSpdL), abs(LTurnBackBSpdR));
  int sensorIdx = SpinStopSensor(spd);

  Motor(LTurnBackBSpdL, LTurnBackBSpdR);
  delay(TurnBackBDelayL);
  while (1) {
    Motor(LTurnBackBSpdL, LTurnBackBSpdR);
    ReadCalibrateB();
    if (B[sensorIdx] >= Ref) break;
  }
  MotorStop();
}

void TurnRightBackB() {
  int spd = max(abs(RTurnBackBSpdL), abs(RTurnBackBSpdR));
  int sensorIdx = 15 - SpinStopSensor(spd);

  Motor(RTurnBackBSpdL, RTurnBackBSpdR);
  delay(TurnBackBDelayR);
  while (1) {
    Motor(RTurnBackBSpdL, RTurnBackBSpdR);
    ReadCalibrateB();
    if (B[sensorIdx] >= Ref) break;
  }
  MotorStop();
}

// ---------- หมุนอยู่กับที่ ----------
// หมุนจนเซนเซอร์ขอบนอกพ้นเส้นเดิม แล้วไล่เซนเซอร์ทีละตัวจากขอบนอกเข้าหาเซนเซอร์หยุด
// จนเจอเส้น จากนั้นเบรกสวนสั้น ๆ และจัดตรงเส้นด้วย lb()
void spinl_B(int speed) {
  MotorStop();
  int stopIdx = SpinStopSensor(speed);
  while (1) {
    ReadCalibrateB();
    Motor(-speed, speed);
    if (B[0] <= Ref) break;
  }
  for (int i = 0; i <= stopIdx; i++) {
    do {
      Motor(-speed, speed);
      delayMicroseconds(80);
      ReadCalibrateB();
    } while (B[i] <= Ref);
  }
  Motor(speed, -speed);
  delay(5);
  lb(speed);
  MotorStop();
}

void spinl_B() {
  spinl_B(tspd);
}

void spinl2_B(int speed) {
  MotorStop();
  int stopIdx = SpinStopSensor(speed);
  while (1) {
    ReadCalibrateB();
    Motor(-speed, speed);
    if (B[0] <= Ref) break;
  }
  for (int pass = 0; pass < 2; pass++) {
    for (int i = 0; i <= stopIdx; i++) {
      do {
        Motor(-speed, speed);
        delayMicroseconds(80);
        ReadCalibrateB();
      } while (B[i] <= Ref);
    }
  }
  Motor(speed, -speed);
  delay(5);
  lb(speed);
  MotorStop();
}

void spinl2_B() {
  spinl2_B(tspd);
}

void spinr_B(int speed) {
  MotorStop();
  int stopIdx = 15 - SpinStopSensor(speed);
  while (1) {
    ReadCalibrateB();
    Motor(speed, -speed);
    if (B[15] <= Ref) break;
  }
  for (int i = 15; i >= stopIdx; i--) {
    do {
      Motor(speed, -speed);
      delayMicroseconds(80);
      ReadCalibrateB();
    } while (B[i] <= Ref);
  }
  Motor(-speed, speed);
  delay(5);
  lb(speed);
  MotorStop();
}

void spinr_B() {
  spinr_B(tspd);
}

void spinr2_B(int speed) {
  MotorStop();
  int stopIdx = 15 - SpinStopSensor(speed);
  while (1) {
    ReadCalibrateB();
    Motor(speed, -speed);
    if (B[15] <= Ref) break;
  }
  for (int pass = 0; pass < 2; pass++) {
    for (int i = 15; i >= stopIdx; i--) {
      do {
        Motor(speed, -speed);
        delayMicroseconds(80);
        ReadCalibrateB();
      } while (B[i] <= Ref);
    }
  }
  Motor(-speed, speed);
  delay(5);
  lb(speed);
  MotorStop();
}

void spinr2_B() {
  spinr2_B(tspd);
}

// ======================================================================
// [9] ทางแยก (TrackSelectF / TrackSelectB)
// ======================================================================

void TrackSelectF(int spd, char x) {
  switch (x) {
    case 's':
      Motor(-spd, -spd);
      delay(delay_break_f);
      MotorStop();
      break;

    case 'S':
      while (1) {
        PIDF(tctL, tctR, slow_kp_f, slow_kd_f);
        ReadCalibrateF();
        if (F[3] > Ref || F[12] > Ref) {
          Motor(-tctL, -tctR);
          delay(tct_delay_break);
          MotorStop();
          break;
        }
      }
      break;

    case 'p':
      BZon();
      ReadCalibrateF();
      while (1) {
        Motor(spd, spd);
        ReadCalibrateF();
        if (F[3] < Ref && F[12] < Ref) break;
      }
      while (1) {
        Motor(spd, spd);
        ReadCalibrateF();
        if (F[3] < Ref && F[12] < Ref) {
          BZoff();
          break;
        }
      }
      break;

    case 'P':
      ToFront();
      BZon();
      ReadCalibrateF();
      while (1) {
        Motor(spd, spd);
        ReadCalibrateF();
        if (F[3] < Ref && F[12] < Ref) break;
      }
      while (1) {
        Motor(spd, spd);
        ReadCalibrateF();
        if (F[3] < Ref && F[12] < Ref) {
          BZoff();
          break;
        }
      }
      break;

    case 'l':
      ToCenter();
      spinl();
      break;
    case 'L':
      ToFront();
      ToCenter();
      spinl();
      break;

    case 'r':
      ToCenter();
      spinr();
      break;
    case 'R':
      ToFront();
      ToCenter();
      spinr();
      break;

    case 'q':
      BZon();
      while (1) {
        Motor(tctL, tctR);
        ReadCalibrateF();
        if (F[3] < Ref && F[12] < Ref) break;
      }
      while (1) {
        Motor(tctL, tctR);
        ReadCalibrateF();
        if (F[3] < Ref && F[12] < Ref) {
          BZoff();
          break;
        }
      }
      TurnLeft();
      break;

    case 'Q':
      ToFront();
      BZon();
      while (1) {
        Motor(tctL, tctR);
        ReadCalibrateF();
        if (F[3] < Ref && F[12] < Ref) break;
      }
      while (1) {
        Motor(tctL, tctR);
        ReadCalibrateF();
        if (F[3] < Ref && F[12] < Ref) {
          BZoff();
          break;
        }
      }
      TurnLeft();
      break;

    case 'e':
      BZon();
      while (1) {
        Motor(tctL, tctR);
        ReadCalibrateF();
        if (F[3] < Ref && F[12] < Ref) break;
      }
      while (1) {
        Motor(tctL, tctR);
        ReadCalibrateF();
        if (F[3] < Ref && F[12] < Ref) {
          BZoff();
          break;
        }
      }
      TurnRight();
      break;

    case 'E':
      ToFront();
      BZon();
      while (1) {
        Motor(tctL, tctR);
        ReadCalibrateF();
        if (F[3] < Ref && F[12] < Ref) break;
      }
      while (1) {
        Motor(tctL, tctR);
        ReadCalibrateF();
        if (F[3] < Ref && F[12] < Ref) {
          BZoff();
          break;
        }
      }
      TurnRight();
      break;

    case 'c':
      ToCenter();
      break;

    case 'C':
      ToFront();
      ToCenter();
      break;

    case 'd':
      ToCenter();
      spinr_B();
      break;

    case 'D':
      ToFront();
      ToCenter();
      spinr_B();
      break;

    case 'a':
      ToCenter();
      spinl_B();
      break;

    case 'A':
      ToFront();
      ToCenter();
      spinl_B();
      break;

    case 'b':
      BZon();
      ModeToCenter();
      while (1) {
        ModeToCenterLine();
        ReadCalibrateB();
        if (B[3] > Ref || B[12] > Ref) {
          Motor(-tctL, -tctR);
          delay(tct_delay_break);
          Motor(-1, -1);
          delay(1);
          MotorStop();
          BZoff();
          break;
        }
      }
      break;

    case 'B':
      ToFront();
      BZon();
      ModeToCenter();
      while (1) {
        ModeToCenterLine();
        ReadCalibrateB();
        if (B[3] > Ref || B[12] > Ref) {
          Motor(-tctL, -tctR);
          delay(tct_delay_break);
          Motor(-1, -1);
          delay(1);
          MotorStop();
          BZoff();
          break;
        }
      }
      break;

    case 'g':
      SetFG(spd);
      break;

    case 'G':
      ToFront();
      SetFG(spd);
      break;

    default:
      MotorStop(spd);
      break;
  }
}

// ถอยหลังจนเซนเซอร์หลัง B[3] / B[12] พ้นเส้นแยก (อ่านสองรอบกันสัญญาณกระตุก)
static void passBackJunction(int spdL, int spdR) {
  for (int n = 0; n < 2; n++) {
    while (1) {
      Motor(-spdL, -spdR);
      delayMicroseconds(80);
      ReadCalibrateB();
      if (B[3] < Ref && B[12] < Ref) break;
    }
  }
}

// คำสั่งทางแยกตอนถอยหลัง (แบบ STORMXII) — ตัวพิมพ์ใหญ่ = ToBack() จัดตรงเส้นก่อน
void TrackSelectB(int spd, char x) {
  switch (x) {
    case 's':  // หยุด
      Motor(spd, spd);
      delay(delay_break_b);
      MotorStop();
      break;

    case 'S':  // วิ่งถึงขอบเส้นแล้วหยุด
      ToBack();
      Motor(bctL, bctR);
      delay(bct_delay_break);
      MotorStop();
      break;

    case 'p':  // วิ่งผ่าน
      BZon();
      passBackJunction(spd, spd);
      BZoff();
      break;

    case 'P':
      ToBack();
      BZon();
      passBackJunction(spd, spd);
      BZoff();
      break;

    case 'l':  // เข้ากลาง + หมุนซ้าย (เซนเซอร์หน้า)
      BackCenter();
      spinl();
      break;
    case 'L':
      ToBack();
      BackCenter();
      spinl();
      break;

    case 'r':  // เข้ากลาง + หมุนขวา (เซนเซอร์หน้า)
      BackCenter();
      spinr();
      break;
    case 'R':
      ToBack();
      BackCenter();
      spinr();
      break;

    case 'c':  // เข้ากลางแล้วหยุด
      BackCenter();
      break;
    case 'C':
      ToBack();
      BackCenter();
      break;

    case 'a':  // เข้ากลาง + หมุนซ้ายด้วยเซนเซอร์หลัง
      BackCenter();
      spinl_B();
      break;
    case 'A':
      ToBack();
      BackCenter();
      spinl_B();
      break;

    case 'd':  // เข้ากลาง + หมุนขวาด้วยเซนเซอร์หลัง
      BackCenter();
      spinr_B();
      break;
    case 'D':
      ToBack();
      BackCenter();
      spinr_B();
      break;

    case 'e':  // เลี้ยวโค้งซ้าย (เซนเซอร์หลัง)
      BZon();
      passBackJunction(bctL, bctR);
      delay(TurnBackDelayL);
      BZoff();
      TurnLeft_B();
      lb(5);
      break;
    case 'E':
      ToBack();
      BZon();
      passBackJunction(bctL, bctR);
      delay(TurnBackDelayL);
      BZoff();
      TurnLeft_B();
      lb(5);
      break;

    case 'q':  // เลี้ยวโค้งขวา (เซนเซอร์หลัง)
      BZon();
      passBackJunction(bctL, bctR);
      delay(TurnBackDelayR);
      BZoff();
      TurnRight_B();
      lb(5);
      break;
    case 'Q':
      ToBack();
      BZon();
      passBackJunction(bctL, bctR);
      delay(TurnBackDelayR);
      BZoff();
      TurnRight_B();
      lb(5);
      break;

    case 'b':  // เข้ากลางแล้ววิ่งต่อถึงเส้นอีกฝั่ง (เซนเซอร์หน้า)
    case 'B':
      BZon();
      ModeToCenterBack();
      while (1) {
        ModeToCenterBackLine();
        ReadCalibrateF();
        if (F[3] > Ref || F[12] > Ref) {
          Motor(bctL, bctR);
          delay(bct_delay_break);
          MotorStop();
          BZoff();
          break;
        }
      }
      break;

    case 'g':  // ปรับตรงด้วย gyro (ล็อกมุมแบบถอยหลัง)
      SetGB(spd);
      break;
    case 'G':
      ToBack();
      SetGB(spd);
      break;

    default:
      MotorStop();
      break;
  }
}

// ======================================================================
// [10] เดินตามเวลา / ระยะทาง + ทางแยก
// ======================================================================

void fftimer(int Speed, int totalTime, char select) {
  fftimer(Speed, totalTime);
  TrackSelectF(Speed, select);
}

void bbtimer(int Speed, int totalTime, char select) {
  bbtimer(Speed, totalTime);
  TrackSelectB(Speed, select);
}

void fft(int Speed, int totalTime, char select) {
  fftimer(Speed, totalTime);
  TrackSelectF(Speed, select);
}

// วิ่งตามเส้นตามเวลา ไม่มี select: จบแล้วมอเตอร์ยังหมุนอยู่ ต่อคำสั่งถัดไปได้ทันที
void fft(int Speed, int totalTime) {
  fftimer(Speed, totalTime);
}

void bbt(int Speed, int totalTime, char select) {
  bbtimer(Speed, totalTime);
  TrackSelectB(Speed, select);
}

// ถอยตามเส้นตามเวลา ไม่มี select: จบแล้วมอเตอร์ยังหมุนอยู่ ต่อคำสั่งถัดไปได้ทันที
void bbt(int Speed, int totalTime) {
  bbtimer(Speed, totalTime);
}

void ffcm(int Speed, float distance, char select) {
  ffcm(Speed, distance);
  TrackSelectF(Speed, select);
}

void bbcm(int Speed, float distance, char select) {
  bbcm(Speed, distance);
  TrackSelectB(Speed, select);
}

// --- กำหนด Kp / Kd เอง: (speed, kp, [kd,] เวลา/ระยะ [, select]) ---
// เวลา/ระยะ เป็น int เพื่อไม่ให้ชนกับ fftimer(speed, ms, 'p') / ffcm(speed, cm, 'p')

void fftimer(int Speed, float Kp, int totalTime) {
  fftimer_pid(Speed, totalTime, Kp, -1);
}

void fftimer(int Speed, float Kp, float Kd, int totalTime) {
  fftimer_pid(Speed, totalTime, Kp, Kd);
}

void fftimer(int Speed, float Kp, int totalTime, char select) {
  fftimer_pid(Speed, totalTime, Kp, -1);
  TrackSelectF(Speed, select);
}

void fftimer(int Speed, float Kp, float Kd, int totalTime, char select) {
  fftimer_pid(Speed, totalTime, Kp, Kd);
  TrackSelectF(Speed, select);
}

void bbtimer(int Speed, float Kp, int totalTime) {
  bbtimer_pid(Speed, totalTime, Kp, -1);
}

void bbtimer(int Speed, float Kp, float Kd, int totalTime) {
  bbtimer_pid(Speed, totalTime, Kp, Kd);
}

void bbtimer(int Speed, float Kp, int totalTime, char select) {
  bbtimer_pid(Speed, totalTime, Kp, -1);
  TrackSelectB(Speed, select);
}

void bbtimer(int Speed, float Kp, float Kd, int totalTime, char select) {
  bbtimer_pid(Speed, totalTime, Kp, Kd);
  TrackSelectB(Speed, select);
}

void ffcm(int Speed, float Kp, int distance) {
  ffcm_pid(Speed, distance, Kp, -1);
}

void ffcm(int Speed, float Kp, float Kd, int distance) {
  ffcm_pid(Speed, distance, Kp, Kd);
}

void ffcm(int Speed, float Kp, int distance, char select) {
  ffcm_pid(Speed, distance, Kp, -1);
  TrackSelectF(Speed, select);
}

void ffcm(int Speed, float Kp, float Kd, int distance, char select) {
  ffcm_pid(Speed, distance, Kp, Kd);
  TrackSelectF(Speed, select);
}

void bbcm(int Speed, float Kp, int distance) {
  bbcm_pid(Speed, distance, Kp, -1);
}

void bbcm(int Speed, float Kp, float Kd, int distance) {
  bbcm_pid(Speed, distance, Kp, Kd);
}

void bbcm(int Speed, float Kp, int distance, char select) {
  bbcm_pid(Speed, distance, Kp, -1);
  TrackSelectB(Speed, select);
}

void bbcm(int Speed, float Kp, float Kd, int distance, char select) {
  bbcm_pid(Speed, distance, Kp, Kd);
  TrackSelectB(Speed, select);
}

// fft / bbt (ชื่อย่อ) แบบกำหนด Kp / Kd เอง

void fft(int Speed, float Kp, int totalTime) {
  fftimer(Speed, Kp, totalTime);
}

void fft(int Speed, float Kp, float Kd, int totalTime) {
  fftimer(Speed, Kp, Kd, totalTime);
}

void fft(int Speed, float Kp, int totalTime, char select) {
  fftimer(Speed, Kp, totalTime, select);
}

void fft(int Speed, float Kp, float Kd, int totalTime, char select) {
  fftimer(Speed, Kp, Kd, totalTime, select);
}

void bbt(int Speed, float Kp, int totalTime) {
  bbtimer(Speed, Kp, totalTime);
}

void bbt(int Speed, float Kp, float Kd, int totalTime) {
  bbtimer(Speed, Kp, Kd, totalTime);
}

void bbt(int Speed, float Kp, int totalTime, char select) {
  bbtimer(Speed, Kp, totalTime, select);
}

void bbt(int Speed, float Kp, float Kd, int totalTime, char select) {
  bbtimer(Speed, Kp, Kd, totalTime, select);
}

// ======================================================================
// [11] เดินตามเส้นจนเจอแยก (ff, bb, ffl, ffr, ffb, ffw ...)
// ======================================================================

void ff(int Speed, float Kp, float Kd, char select) {
  BaseSpeed = Speed;
  InitialSpeed();
  if (Kp < 0) Kp = PID_KP_Front;
  if (Kd < 0) Kd = PID_KD_Front;
  while (1) {
    PIDF(LeftBaseSpeed, RightBaseSpeed, Kp, Kd);
    ReadCalibrateF();
    if (F[3] > Ref || F[12] > Ref || (F[4] > Ref && F[11] > Ref)) {
      break;
    }
  }
  TrackSelectF(Speed, select);
}

void ff(int Speed, float Kp, char select) {
  ff(Speed, Kp, -1, select);
}

void ff(int Speed, char select) {
  ff(Speed, -1, -1, select);
}

void bb(int Speed, float Kp, float Kd, char select) {
  BaseSpeed = Speed;
  InitialSpeed();
  if (Kp < 0) Kp = PID_KP_Back;
  if (Kd < 0) Kd = PID_KD_Back;
  while (1) {
    PIDB(BackLeftBaseSpeed, BackRightBaseSpeed, Kp, Kd);
    ReadCalibrateB();
    if (B[3] > Ref || B[12] > Ref || (B[4] > Ref && B[11] > Ref)) {
      break;
    }
  }
  TrackSelectB(Speed, select);
}

void bb(int Speed, float Kp, char select) {
  bb(Speed, Kp, -1, select);
}

void bb(int Speed, char select) {
  bb(Speed, -1, -1, select);
}

void ffc2(int Speed, float Kp, float Kd, char select) {
  BaseSpeed = Speed;
  InitialSpeed();
  if (Kp < 0) Kp = PID_KP_Front;
  if (Kd < 0) Kd = PID_KD_Front;
  while (1) {
    PIDF(LeftBaseSpeed, RightBaseSpeed, Kp, Kd);
    ReadCalibrateF();
    if ((F[0] > Ref && F[15] > Ref) || (F[2] > Ref && F[13] > Ref) || (F[4] > Ref && F[6] > Ref && F[9] > Ref && F[11] > Ref)) break;
  }
  TrackSelectF(Speed, select);
}

void ffc2(int Speed, float Kp, char select) {
  ffc2(Speed, Kp, -1, select);
}

void ffc2(int Speed, char select) {
  ffc2(Speed, -1, -1, select);
}

void ffc(int Speed, float Kp, float Kd, char select) {
  BaseSpeed = Speed;
  InitialSpeed();
  if (Kp < 0) Kp = PID_KP_Front;
  if (Kd < 0) Kd = PID_KD_Front;
  while (1) {
    PIDF(LeftBaseSpeed, RightBaseSpeed, Kp, Kd);
    ReadCalibrateF();
    if ((F[5] >= Ref && F[10] >= Ref) || (F[4] >= Ref && F[11] >= Ref) || (F[3] >= Ref && F[12] >= Ref) || (F[5] >= Ref && F[6] >= Ref && F[9] >= Ref && F[10] >= Ref)) break;
  }
  TrackSelectF(Speed, select);
}

void ffc(int Speed, float Kp, char select) {
  ffc(Speed, Kp, -1, select);
}

void ffc(int Speed, char select) {
  ffc(Speed, -1, -1, select);
}

void bbc2(int Speed, float Kp, float Kd, char select) {
  BaseSpeed = Speed;
  InitialSpeed();
  if (Kp < 0) Kp = PID_KP_Back;
  if (Kd < 0) Kd = PID_KD_Back;
  while (1) {
    PIDB(BackLeftBaseSpeed, BackRightBaseSpeed, Kp, Kd);
    ReadCalibrateB();
    if ((B[0] > Ref && B[15] > Ref) || (B[2] > Ref && B[13] > Ref) || (B[4] > Ref && B[6] > Ref && B[9] > Ref && B[11] > Ref)) break;
  }
  TrackSelectB(Speed, select);
}

void bbc2(int Speed, float Kp, char select) {
  bbc2(Speed, Kp, -1, select);
}

void bbc2(int Speed, char select) {
  bbc2(Speed, -1, -1, select);
}

void bbc(int Speed, float Kp, float Kd, char select) {
  BaseSpeed = Speed;
  InitialSpeed();
  if (Kp < 0) Kp = PID_KP_Back;
  if (Kd < 0) Kd = PID_KD_Back;
  while (1) {
    PIDB(BackLeftBaseSpeed, BackRightBaseSpeed, Kp, Kd);
    ReadCalibrateB();
    if ((B[5] >= Ref && B[10] >= Ref) || (B[4] >= Ref && B[11] >= Ref) || (B[3] >= Ref && B[12] >= Ref) || (B[5] >= Ref && B[6] >= Ref && B[9] >= Ref && B[10] >= Ref)) break;
  }
  TrackSelectB(Speed, select);
}

void bbc(int Speed, float Kp, char select) {
  bbc(Speed, Kp, -1, select);
}

void bbc(int Speed, char select) {
  bbc(Speed, -1, -1, select);
}

void ffl(int Speed, float Kp, float Kd, char select) {
  BaseSpeed = Speed;
  InitialSpeed();
  if (Kp < 0) Kp = PID_KP_Front;
  if (Kd < 0) Kd = PID_KD_Front;
  while (1) {
    PIDF(LeftBaseSpeed, RightBaseSpeed, Kp, Kd);
    ReadCalibrateF();
    if (F[0] > Ref) {
      break;
    }
  }
  TrackSelectF(Speed, select);
}

void ffl(int Speed, float Kp, char select) {
  ffl(Speed, Kp, -1, select);
}

void ffl(int Speed, char select) {
  ffl(Speed, -1, -1, select);
}

void ffl2(int Speed, float Kp, float Kd, char select) {
  BaseSpeed = Speed;
  InitialSpeed();
  if (Kp < 0) Kp = PID_KP_Front;
  if (Kd < 0) Kd = PID_KD_Front;
  while (1) {
    PIDF(LeftBaseSpeed, RightBaseSpeed, Kp, Kd);
    ReadCalibrateF();
    if (F[0] > Ref && F[2] > Ref && F[4] > Ref && F[6] > Ref && F[9] > Ref) {
      break;
    }
  }
  TrackSelectF(Speed, select);
}

void ffl2(int Speed, float Kp, char select) {
  ffl2(Speed, Kp, -1, select);
}

void ffl2(int Speed, char select) {
  ffl2(Speed, -1, -1, select);
}

void bbl(int Speed, float Kp, float Kd, char select) {
  BaseSpeed = Speed;
  InitialSpeed();
  if (Kp < 0) Kp = PID_KP_Back;
  if (Kd < 0) Kd = PID_KD_Back;
  while (1) {
    PIDB(BackLeftBaseSpeed, BackRightBaseSpeed, Kp, Kd);
    ReadCalibrateB();
    if (B[0] > Ref) {
      break;
    }
  }
  TrackSelectB(Speed, select);
}

void bbl(int Speed, float Kp, char select) {
  bbl(Speed, Kp, -1, select);
}

void bbl(int Speed, char select) {
  bbl(Speed, -1, -1, select);
}

void bbl2(int Speed, float Kp, float Kd, char select) {
  BaseSpeed = Speed;
  InitialSpeed();
  if (Kp < 0) Kp = PID_KP_Back;
  if (Kd < 0) Kd = PID_KD_Back;
  while (1) {
    PIDB(BackLeftBaseSpeed, BackRightBaseSpeed, Kp, Kd);
    ReadCalibrateB();
    if (B[0] > Ref && B[2] > Ref && B[4] > Ref && B[6] > Ref && B[9] > Ref) {
      break;
    }
  }
  TrackSelectB(Speed, select);
}

void bbl2(int Speed, float Kp, char select) {
  bbl2(Speed, Kp, -1, select);
}

void bbl2(int Speed, char select) {
  bbl2(Speed, -1, -1, select);
}

void ffr(int Speed, float Kp, float Kd, char select) {
  BaseSpeed = Speed;
  InitialSpeed();
  if (Kp < 0) Kp = PID_KP_Front;
  if (Kd < 0) Kd = PID_KD_Front;
  while (1) {
    PIDF(LeftBaseSpeed, RightBaseSpeed, Kp, Kd);
    ReadCalibrateF();
    if (F[15] > Ref) break;
  }
  TrackSelectF(Speed, select);
}

void ffr(int Speed, float Kp, char select) {
  ffr(Speed, Kp, -1, select);
}

void ffr(int Speed, char select) {
  ffr(Speed, -1, -1, select);
}

void ffr2(int Speed, float Kp, float Kd, char select) {
  BaseSpeed = Speed;
  InitialSpeed();
  if (Kp < 0) Kp = PID_KP_Front;
  if (Kd < 0) Kd = PID_KD_Front;
  while (1) {
    PIDF(LeftBaseSpeed, RightBaseSpeed, Kp, Kd);
    ReadCalibrateF();
    if (F[6] > Ref && F[9] > Ref && F[11] > Ref && F[13] > Ref && F[15] > Ref) break;
  }
  TrackSelectF(Speed, select);
}

void ffr2(int Speed, float Kp, char select) {
  ffr2(Speed, Kp, -1, select);
}

void ffr2(int Speed, char select) {
  ffr2(Speed, -1, -1, select);
}

void bbr(int Speed, float Kp, float Kd, char select) {
  BaseSpeed = Speed;
  InitialSpeed();
  if (Kp < 0) Kp = PID_KP_Back;
  if (Kd < 0) Kd = PID_KD_Back;
  while (1) {
    PIDB(BackLeftBaseSpeed, BackRightBaseSpeed, Kp, Kd);
    ReadCalibrateB();
    if (B[15] > Ref) {
      break;
    }
  }
  TrackSelectB(Speed, select);
}

void bbr(int Speed, float Kp, char select) {
  bbr(Speed, Kp, -1, select);
}

void bbr(int Speed, char select) {
  bbr(Speed, -1, -1, select);
}

void bbr2(int Speed, float Kp, float Kd, char select) {
  BaseSpeed = Speed;
  InitialSpeed();
  if (Kp < 0) Kp = PID_KP_Back;
  if (Kd < 0) Kd = PID_KD_Back;
  while (1) {
    PIDB(BackLeftBaseSpeed, BackRightBaseSpeed, Kp, Kd);
    ReadCalibrateB();
    if (B[6] > Ref && B[9] > Ref && B[11] > Ref && B[13] > Ref && B[15] > Ref) {
      break;
    }
  }
  TrackSelectB(Speed, select);
}

void bbr2(int Speed, float Kp, char select) {
  bbr2(Speed, Kp, -1, select);
}

void bbr2(int Speed, char select) {
  bbr2(Speed, -1, -1, select);
}

void ffblack(int SpeedL, int SpeedR, char select) {
  Move(SpeedL, SpeedR, 50);
  while (1) {
    Motor(SpeedL, SpeedR);
    ReadCalibrateF();
    if (F[2] > Ref || F[4] > Ref || F[6] > Ref || F[9] > Ref || F[11] > Ref || F[13] > Ref) break;
  }
  TrackSelectF(SpeedL, select);
}

void ffb(int SpeedL, int SpeedR, char select) {
  ffblack(SpeedL, SpeedR, select);
}

void ffblack(int Speed, char select) {
  BaseSpeed = Speed;
  InitialSpeed();
  ffblack(LeftBaseSpeed, RightBaseSpeed, select);
}

void ffb(int Speed, char select) {
  ffblack(Speed, select);
}

void bbblack(int SpeedL, int SpeedR, char select) {
  Move(-SpeedL, -SpeedR, 50);
  while (1) {
    Motor(-SpeedL, -SpeedR);
    ReadCalibrateB();
    if (B[2] > Ref || B[4] > Ref || B[6] > Ref || B[9] > Ref || B[11] > Ref || B[13] > Ref) break;
  }
  TrackSelectB(SpeedL, select);
}

void bbb(int SpeedL, int SpeedR, char select) {
  bbblack(SpeedL, SpeedR, select);
}

void bbblack(int Speed, char select) {
  BaseSpeed = Speed;
  InitialSpeed();
  bbblack(BackLeftBaseSpeed, BackRightBaseSpeed, select);
}

void bbb(int Speed, char select) {
  bbblack(Speed, select);
}

void ffwhite(int Speed, float Kp, float Kd, char select) {
  BaseSpeed = Speed;
  InitialSpeed();
  if (Kp < 0) Kp = PID_KP_Front;
  if (Kd < 0) Kd = PID_KD_Front;
  while (1) {
    PIDF(LeftBaseSpeed, RightBaseSpeed, Kp, Kd);
    ReadCalibrateF();
    if ((F[0] < Ref && F[2] < Ref && F[4] < Ref && F[6] < Ref && F[9] < Ref && F[11] < Ref && F[13] < Ref && F[15] < Ref)) break;
  }
  TrackSelectF(Speed, select);
}

void ffwhite(int Speed, float Kp, char select) {
  ffwhite(Speed, Kp, -1, select);
}

void ffwhite(int Speed, char select) {
  ffwhite(Speed, -1, -1, select);
}

void ffw(int Speed, float Kp, float Kd, char select) {
  ffwhite(Speed, Kp, Kd, select);
}

void ffw(int Speed, float Kp, char select) {
  ffw(Speed, Kp, -1, select);
}

void ffw(int Speed, char select) {
  ffw(Speed, -1, -1, select);
}

void bbwhite(int Speed, float Kp, float Kd, char select) {
  BaseSpeed = Speed;
  InitialSpeed();
  if (Kp < 0) Kp = PID_KP_Back;
  if (Kd < 0) Kd = PID_KD_Back;
  while (1) {
    PIDB(BackLeftBaseSpeed, BackRightBaseSpeed, Kp, Kd);
    ReadCalibrateB();
    if ((B[0] < Ref && B[2] < Ref && B[4] < Ref && B[6] < Ref && B[9] < Ref && B[11] < Ref && B[13] < Ref && B[15] < Ref)) break;
  }
  TrackSelectB(Speed, select);
}

void bbwhite(int Speed, float Kp, char select) {
  bbwhite(Speed, Kp, -1, select);
}

void bbwhite(int Speed, char select) {
  bbwhite(Speed, -1, -1, select);
}

void bbw(int Speed, float Kp, float Kd, char select) {
  bbwhite(Speed, Kp, Kd, select);
}

void bbw(int Speed, float Kp, char select) {
  bbw(Speed, Kp, -1, select);
}

void bbw(int Speed, char select) {
  bbw(Speed, -1, -1, select);
}

void ffnum(int Speed, float Kp, float Kd, char select, int numm) {
  BaseSpeed = Speed;
  InitialSpeed();
  if (Kp < 0) Kp = PID_KP_Front;
  if (Kd < 0) Kd = PID_KD_Front;
  while (1) {
    PIDF(LeftBaseSpeed, RightBaseSpeed, Kp, Kd);
    ReadCalibrateF();
    if (F[numm] > Ref) break;
  }
  TrackSelectF(Speed, select);
}

void ffnum(int Speed, float Kp, char select, int numm) {
  ffnum(Speed, Kp, -1, select, numm);
}

void ffnum(int Speed, char select, int numm) {
  ffnum(Speed, -1, -1, select, numm);
}

void bbnum(int Speed, float Kp, float Kd, char select, int numm) {
  BaseSpeed = Speed;
  InitialSpeed();
  if (Kp < 0) Kp = PID_KP_Back;
  if (Kd < 0) Kd = PID_KD_Back;
  while (1) {
    PIDB(BackLeftBaseSpeed, BackRightBaseSpeed, Kp, Kd);
    ReadCalibrateB();
    if (B[numm] > Ref) break;
  }
  TrackSelectB(Speed, select);
}

void bbnum(int Speed, float Kp, char select, int numm) {
  bbnum(Speed, Kp, -1, select, numm);
}

void bbnum(int Speed, char select, int numm) {
  bbnum(Speed, -1, -1, select, numm);
}

void ffn(int Speed, float Kp, float Kd, char select, int numm) {
  ffnum(Speed, Kp, Kd, select, numm);
}

void ffn(int Speed, float Kp, char select, int numm) {
  ffn(Speed, Kp, -1, select, numm);
}

void ffn(int Speed, char select, int numm) {
  ffn(Speed, -1, -1, select, numm);
}
void bbn(int Speed, float Kp, float Kd, char select, int numm) {
  bbnum(Speed, Kp, Kd, select, numm);
}

void bbn(int Speed, float Kp, char select, int numm) {
  bbn(Speed, Kp, -1, select, numm);
}

void bbn(int Speed, char select, int numm) {
  bbn(Speed, -1, -1, select, numm);
}

void ff_distance(int Speed, float Kp, float Kd, char select, float distance) {
  BaseSpeed = Speed;
  InitialSpeed();
  if (Kp < 0) Kp = PID_KP_Front;
  if (Kd < 0) Kd = PID_KD_Front;
  while (1) {
    PIDF(LeftBaseSpeed, RightBaseSpeed, Kp, Kd);
    if (readDistance() >= distance) break;
  }
  TrackSelectF(Speed, select);
}

void ff_distance(int Speed, float Kp, char select, float distance) {
  ff_distance(Speed, Kp, -1, select, distance);
}

void ff_distance(int Speed, char select, float distance) {
  ff_distance(Speed, -1, -1, select, distance);
}

void bb_distance(int Speed, float Kp, float Kd, char select, float distance) {
  BaseSpeed = Speed;
  InitialSpeed();
  if (Kp < 0) Kp = PID_KP_Back;
  if (Kd < 0) Kd = PID_KD_Back;
  while (1) {
    PIDB(BackLeftBaseSpeed, BackRightBaseSpeed, Kp, Kd);
    if (readDistance() <= distance) break;
  }
  TrackSelectB(Speed, select);
}

void bb_distance(int Speed, float Kp, char select, float distance) {
  bb_distance(Speed, Kp, -1, select, distance);
}

void bb_distance(int Speed, char select, float distance) {
  bb_distance(Speed, -1, -1, select, distance);
}

void ffd(int Speed, float Kp, float Kd, char select, float distance) {
  ff_distance(Speed, Kp, Kd, select, distance);
}

void ffd(int Speed, float Kp, char select, float distance) {
  ffd(Speed, Kp, -1, select, distance);
}

void ffd(int Speed, char select, float distance) {
  ffd(Speed, -1, -1, select, distance);
}
void bbd(int Speed, float Kp, float Kd, char select, float distance) {
  bb_distance(Speed, Kp, Kd, select, distance);
}

void bbd(int Speed, float Kp, char select, float distance) {
  bbd(Speed, Kp, -1, select, distance);
}

void bbd(int Speed, char select, float distance) {
  bbd(Speed, -1, -1, select, distance);
}

// ======================================================================
// [12] หยุดตามระยะเซนเซอร์ (ff_distances)
// ======================================================================

float PID_KP_STOP = 0.35;
float PID_KD_STOP = 0.8;

int PID_Stop(int ir, int ir_target, int maxSpeed) {
  static int lastError = 0;

  int error = ir_target - ir;
  int derivative = error - lastError;
  lastError = error;

  int out = (PID_KP_STOP * error) + (PID_KD_STOP * derivative);

  return constrain(out, 0, maxSpeed);
}

void ff_distances(int Speed, char select, int ir_target) {
  BaseSpeed = Speed;
  InitialSpeed();  // จำเป็นเสมอ: เปลี่ยน BaseSpeed แล้วต้องเรียก InitialSpeed() ใหม่

  while (1) {
    ReadCalibrateF();
    int ir = readDistance();
    // ===== เข้าโหมดหยุด =====
    if (ir >= ir_target) {
      int newBase = PID_Stop(ir, ir_target, Speed);
      if (newBase != BaseSpeed) {
        BaseSpeed = newBase;
        int keepErr = LastError_F;  // เก็บ error เดิมไว้ ไม่ให้ D-term ถูกรีเซ็ตกลางทาง
        InitialSpeed();             // BaseSpeed เปลี่ยน ต้อง InitialSpeed() ใหม่
        LastError_F = keepErr;
      }
    }
    // ===== PID ตามเส้น =====
    PIDF(LeftBaseSpeed, RightBaseSpeed, PID_KP_Front, PID_KD_Front);
    // ===== หยุดจริง =====
    if (BaseSpeed <= 1) break;
  }
  MotorStop();                  // เบรกนิ่ง
  TrackSelectF(Speed, select);  // คำสั่งถัดไป
}

void ffds(int Speed, char select, int ir_target) {
  ff_distances(Speed, select, ir_target);
}

// ======================================================================
// [13] ออกจากจุดเริ่ม / เข้าจุดจบ (gostart / goend)
// ======================================================================

void gostart(int LeftSpeed, int RightSpeed) {
  Move(LeftSpeed, RightSpeed, 100);
  while (1) {
    ReadCalibrateF();
    Motor(LeftSpeed, RightSpeed);
    if (F[0] < Ref && F[15] < Ref) {
      Move(LeftSpeed, RightSpeed, 1);
      break;
    }
  }
}

void gostart(int Speed) {
  BaseSpeed = Speed;
  InitialSpeed();
  gostart(LeftBaseSpeed, RightBaseSpeed);
}

void goend(int LeftSpeed, int RightSpeed) {
  while (1) {
    ReadCalibrateC();
    Motor(LeftSpeed, RightSpeed);
    if (C[0] > RefC && C[1] > RefC) {
      Move(LeftSpeed, RightSpeed, 50);
      break;
    }
  }
  Move(-15, -15, 15);
  Move(-10, -10, 10);
  Move(-1, -1, 1);
  MotorStop();
}

void goend(int Speed) {
  BaseSpeed = Speed;
  InitialSpeed();
  goend(LeftBaseSpeed, RightBaseSpeed);
  MotorStop();
}

// ======================================================================
// [14] ตั้งหุ่นตรงเส้น (balance / set)
// ======================================================================

void balancef(int Counter) {
  Move(-10, -10, 50);
  for (int i = 0; i <= Counter; i++) {
    Move(-10, -10, 50);
    while (1) {
      Motor(10, 10);
      ReadCalibrateF();
      if (F[0] > Ref) {
        while (1) {
          Motor(0, 10);
          ReadCalibrateF();
          if (F[15] > Ref) {
            MotorStop();
            break;
          }
        }
      }
      if (F[15] > Ref) {
        while (1) {
          Motor(10, 0);
          ReadCalibrateF();
          if (F[0] > Ref) {
            MotorStop();
            break;
          }
        }
      }
      if (F[0] > Ref && F[15] > Ref) {
        MotorStop();
        break;
      }
    }
    MotorStop();
  }
  SetRobotAngle();
}
void setf(int Counter) {
  balancef(Counter);
}

void balanceb(int Counter) {
  Move(10, 10, 50);
  for (int i = 0; i <= Counter; i++) {
    Move(10, 10, 50);
    while (1) {
      Motor(-12, -12);
      ReadCalibrateB();
      if (B[0] > Ref) {
        while (1) {
          Motor(0, -10);
          ReadCalibrateB();
          if (B[15] > Ref) {
            MotorStop();
            break;
          }
        }
      }
      if (B[15] > Ref) {
        while (1) {
          Motor(-10, 0);
          ReadCalibrateB();
          if (B[0] > Ref) {
            MotorStop();
            break;
          }
        }
      }
      if (B[0] > Ref && B[15] > Ref) {
        MotorStop();
        break;
      }
    }
    MotorStop();
  }
  SetRobotAngle();
}

void setb(int Counter) {
  balanceb(Counter);
}

void balancefc(int Counter) {
  Move(-10, -10, 50);
  for (int i = 0; i <= Counter; i++) {
    Move(-10, -10, 50);
    while (1) {
      Motor(10, 10);
      ReadCalibrateC();
      if (C[1] > RefC) {
        while (1) {
          Motor(0, 10);
          ReadCalibrateC();
          if (C[0] > RefC) {
            MotorStop();
            break;
          }
        }
      }
      if (C[0] > RefC) {
        while (1) {
          Motor(10, 0);
          ReadCalibrateC();
          if (C[1] > RefC) {
            MotorStop();
            break;
          }
        }
      }
      if (C[1] > RefC && C[0] > RefC) {
        MotorStop();
        break;
      }
    }
    MotorStop();
  }
  SetRobotAngle();
}

void balancebc(int Counter) {
  Move(10, 10, 50);
  for (int i = 0; i <= Counter; i++) {
    Move(10, 10, 50);
    while (1) {
      Motor(-12, -12);
      ReadCalibrateC();
      if (C[1] > RefC) {
        while (1) {
          Motor(0, -10);
          ReadCalibrateC();
          if (C[0] > RefC) {
            MotorStop();
            break;
          }
        }
      }
      if (C[0] > RefC) {
        while (1) {
          Motor(-10, 0);
          ReadCalibrateC();
          if (C[1] > RefC) {
            MotorStop();
            break;
          }
        }
      }
      if (C[1] > RefC && C[0] > RefC) {
        MotorStop();
        break;
      }
    }
    MotorStop();
  }
  SetRobotAngle();
}

void setfc(int Counter) {
  balancefc(Counter);
}
void setbc(int Counter) {
  balancebc(Counter);
}
void set_f(int num) {
  for (int i = 0; i < num; i++) {
    while (1) {
      ReadCalibrateF();
      delay(5);
      if (F[0] > Ref && F[15] < Ref) {
        Motor(-5, 15);
      } else if (F[0] < Ref && F[15] > Ref) {
        Motor(15, -5);
      } else if (F[0] < Ref && F[15] < Ref) {
        Motor(15, 15);
      } else {
        Motor(-1, -1);
        MotorStop();
        break;
      }
    }
    if (num > 1) {
      Motor(-15, -15);
      delay(50);
      Motor(-1, -1);
    }
  }
  SetRobotAngle();
}

void set_b(int num) {
  for (int i = 0; i < num; i++) {
    while (1) {
      ReadCalibrateB();
      delay(5);
      if (B[0] > Ref && B[15] < Ref) {
        Motor(5, -15);
      } else if (B[0] < Ref && B[15] > Ref) {
        Motor(-15, 5);
      } else if (B[0] < Ref && B[15] < Ref) {
        Motor(-15, -15);
      } else {
        Motor(1, 1);
        MotorStop();
        break;
      }
    }
    if (num > 1) {
      Motor(15, 15);
      delay(50);
      Motor(1, 1);
    }
  }
  SetRobotAngle();
}

void set_fc(int num) {
  for (int i = 0; i < num; i++) {
    while (1) {
      ReadCalibrateC();
      delay(5);
      if (C[1] > RefC && C[0] < RefC) {
        Motor(-5, 15);
      } else if (C[1] < RefC && C[0] > RefC) {
        Motor(15, -5);
      } else if (C[1] < RefC && C[0] < RefC) {
        Motor(15, 15);
      } else {
        Motor(-1, -1);
        MotorStop();
        break;
      }
    }
    if (num > 1) {
      Motor(-15, -15);
      delay(50);
      Motor(-1, -1);
    }
  }
  SetRobotAngle();
}

void set_bc(int num) {
  for (int i = 0; i < num; i++) {
    while (1) {
      ReadCalibrateC();
      delay(5);
      if (C[1] > RefC && C[0] < RefC) {
        Motor(5, -15);
      } else if (C[1] < RefC && C[0] > RefC) {
        Motor(-15, 5);
      } else if (C[1] < RefC && C[0] < RefC) {
        Motor(-15, -15);
      } else {
        Motor(1, 1);
        MotorStop();
        break;
      }
    }
    if (num > 1) {
      Motor(15, 15);
      delay(50);
      Motor(1, 1);
    }
  }
  SetRobotAngle();
}

// ======================================================================
// [15] Serial debug ตำแหน่งเส้น
// ======================================================================

void SerialPositionF() {
  while (1) {
    int pos = readPositionF(200, 50);
    Serial.print("Position F : ");
    Serial.println(pos);
    delay(100);
  }
}

void SerialPositionB() {
  while (1) {
    int pos = readPositionB(200, 50);
    Serial.print("Position  B : ");
    Serial.println(pos);
    delay(100);
  }
}

void SerialPositionFB() {
  while (1) {
    int posF = readPositionF(200, 50);
    int posB = readPositionB(200, 50);
    Serial.print("Position F : ");
    Serial.print(posF);
    Serial.print("  |  Position B : ");
    Serial.println(posB);
    delay(100);
  }
}

// ======================================================================
// [16] วิ่งโค้ง (CL / CR: เบี่ยงซ้าย/ขวา ด้วย set_positionL/R ชั่วคราว)
// ======================================================================

// Ramp เร่งความเร็วช่วงต้นระยะทาง (ไม่ผ่อนท้าย) ตอนนี้ยังไม่มีฟังก์ชันไหนเรียกใช้
// (ffcmcl/ffcmcr/bbcmcl/bbcmcr เปลี่ยนไปใช้ ffcm/bbcm ธรรมดาแล้ว) สเกล LeftBaseSpeed/
// RightBaseSpeed (หรือฝั่ง Back) ด้วยอัตราส่วนเดียวกัน เพื่อคง balance ซ้าย-ขวาเดิมไว้ระหว่าง ramp
[[maybe_unused]] static void ffcm_ramped(int Speed, int distance) {
  BaseSpeed = Speed;
  InitialSpeed();
  int target_speed = min(LeftBaseSpeed, RightBaseSpeed);
  float traveled_distance = 0;
  unsigned long last_time = millis();
  float speed_scale = 1.75;  // <-- ใช้ค่าที่คำนวณจากการวัดจริง (เหมือน ffcm)

  // เร่งความเร็วเฉพาะช่วงต้น ไม่ผ่อนท้าย: จบด้วยความเร็วเต็มเพื่อต่อคำสั่งถัดไปได้ลื่น
  const float ACCEL_DISTANCE_CM = 20.0;
  const int MIN_SPEED = 10;
  bool enableRamp = (distance >= 30 && target_speed > MIN_SPEED);
  float rampSpeed = enableRamp ? MIN_SPEED : target_speed;

  while (1) {
    unsigned long current_time = millis();
    float delta_time = (current_time - last_time) / 1000.0;
    // คิดระยะจากความเร็วที่สั่งจริงในรอบก่อน (ช่วงเร่งวิ่งช้ากว่า target_speed) ไม่ให้ระยะขาด
    traveled_distance += (rampSpeed * speed_scale) * delta_time;
    last_time = current_time;

    if (distance > 0 && traveled_distance >= distance) break;

    rampSpeed = target_speed;
    if (enableRamp && traveled_distance < ACCEL_DISTANCE_CM) {
      rampSpeed = MIN_SPEED + (target_speed - MIN_SPEED) * (traveled_distance / ACCEL_DISTANCE_CM);
    }
    float scale = rampSpeed / target_speed;

    PIDF(LeftBaseSpeed * scale, RightBaseSpeed * scale, PID_KP_Front, PID_KD_Front);
  }
}

[[maybe_unused]] static void bbcm_ramped(int Speed, int distance) {
  BaseSpeed = Speed;
  InitialSpeed();
  int target_speed = min(BackLeftBaseSpeed, BackRightBaseSpeed);
  float traveled_distance = 0;
  unsigned long last_time = millis();
  float speed_scale = 1.75;  // <-- ใช้ค่าที่คำนวณจากการวัดจริง (เหมือน bbcm)

  // เร่งความเร็วเฉพาะช่วงต้น ไม่ผ่อนท้าย: จบด้วยความเร็วเต็มเพื่อต่อคำสั่งถัดไปได้ลื่น
  const float ACCEL_DISTANCE_CM = 20.0;
  const int MIN_SPEED = 10;
  bool enableRamp = (distance >= 30 && target_speed > MIN_SPEED);
  float rampSpeed = enableRamp ? MIN_SPEED : target_speed;

  while (1) {
    unsigned long current_time = millis();
    float delta_time = (current_time - last_time) / 1000.0;
    // คิดระยะจากความเร็วที่สั่งจริงในรอบก่อน (ช่วงเร่งวิ่งช้ากว่า target_speed) ไม่ให้ระยะขาด
    traveled_distance += (rampSpeed * speed_scale) * delta_time;
    last_time = current_time;

    if (distance > 0 && traveled_distance >= distance) break;

    rampSpeed = target_speed;
    if (enableRamp && traveled_distance < ACCEL_DISTANCE_CM) {
      rampSpeed = MIN_SPEED + (target_speed - MIN_SPEED) * (traveled_distance / ACCEL_DISTANCE_CM);
    }
    float scale = rampSpeed / target_speed;

    PIDB(BackLeftBaseSpeed * scale, BackRightBaseSpeed * scale, PID_KP_Back, PID_KD_Back);
  }
}

// Ramp แบบเดียวกับ ffcm_ramped/bbcm_ramped ข้างบน แต่ใช้เวลาที่เหลือ (totalTime) แทนระยะทาง
// เพราะ fftimercl/fftimercr/bbtimercl/bbtimercr วิ่งตามเวลาคงที่ ไม่ใช่ระยะทาง จึงรู้ "เวลารวม"
// ล่วงหน้าแทน ปิด ramp อัตโนมัติถ้า totalTime สั้นเกินไป (< 400ms)
// กันไม่ให้ช่วงเร่งกินเวลาที่สั่งวิ่งทั้งหมด
[[maybe_unused]] static void fftimer_ramped(int Speed, int totalTime) {
  BaseSpeed = Speed;
  InitialSpeed();
  int target_speed = min(LeftBaseSpeed, RightBaseSpeed);

  // เร่งความเร็วเฉพาะช่วงต้น ไม่ผ่อนท้าย: จบด้วยความเร็วเต็มเพื่อต่อคำสั่งถัดไปได้ลื่น
  const unsigned long ACCEL_TIME_MS = 150;
  const int MIN_SPEED = 10;
  bool enableRamp = (totalTime >= 400 && target_speed > MIN_SPEED);

  unsigned long startTime = millis();
  unsigned long endTime = startTime + totalTime;

  while (millis() <= endTime) {
    float scale = 1.0;
    unsigned long elapsed = millis() - startTime;
    if (enableRamp && elapsed < ACCEL_TIME_MS) {
      float rampSpeed = MIN_SPEED + (target_speed - MIN_SPEED) * ((float)elapsed / ACCEL_TIME_MS);
      scale = rampSpeed / target_speed;
    }
    PIDF(LeftBaseSpeed * scale, RightBaseSpeed * scale, PID_KP_Front, PID_KD_Front);
  }
}

[[maybe_unused]] static void bbtimer_ramped(int Speed, int totalTime) {
  BaseSpeed = Speed;
  InitialSpeed();
  int target_speed = min(BackLeftBaseSpeed, BackRightBaseSpeed);

  // เร่งความเร็วเฉพาะช่วงต้น ไม่ผ่อนท้าย: จบด้วยความเร็วเต็มเพื่อต่อคำสั่งถัดไปได้ลื่น
  const unsigned long ACCEL_TIME_MS = 150;
  const int MIN_SPEED = 10;
  bool enableRamp = (totalTime >= 400 && target_speed > MIN_SPEED);

  unsigned long startTime = millis();
  unsigned long endTime = startTime + totalTime;

  while (millis() <= endTime) {
    float scale = 1.0;
    unsigned long elapsed = millis() - startTime;
    if (enableRamp && elapsed < ACCEL_TIME_MS) {
      float rampSpeed = MIN_SPEED + (target_speed - MIN_SPEED) * ((float)elapsed / ACCEL_TIME_MS);
      scale = rampSpeed / target_speed;
    }
    PIDB(BackLeftBaseSpeed * scale, BackRightBaseSpeed * scale, PID_KP_Back, PID_KD_Back);
  }
}

void ffcl(int Speed, float Kp, float Kd, char select) {
  int temp = set_position;
  set_position = set_positionL;
  BaseSpeed = Speed;
  InitialSpeed();
  if (Kp < 0) Kp = PID_KP_Front;
  if (Kd < 0) Kd = PID_KD_Front;
  while (1) {
    PIDF(LeftBaseSpeed, RightBaseSpeed, Kp, Kd);
    ReadCalibrateF();
    if (F[0] > Ref) break;
  }
  TrackSelectF(Speed, select);
  set_position = temp;
}

void ffcl(int Speed, float Kp, char select) {
  ffcl(Speed, Kp, -1, select);
}

void ffcl(int Speed, char select) {
  ffcl(Speed, -1, -1, select);
}

void ffcr(int Speed, float Kp, float Kd, char select) {
  int temp = set_position;
  set_position = set_positionR;
  BaseSpeed = Speed;
  InitialSpeed();
  if (Kp < 0) Kp = PID_KP_Front;
  if (Kd < 0) Kd = PID_KD_Front;
  while (1) {
    PIDF(LeftBaseSpeed, RightBaseSpeed, Kp, Kd);
    ReadCalibrateF();
    if (F[15] > Ref || (F[2] < Ref && F[4] < Ref && F[6] < Ref && F[9] < Ref && F[11] < Ref && F[13] < Ref)) break;
  }
  TrackSelectF(Speed, select);
  set_position = temp;
}

void ffcr(int Speed, float Kp, char select) {
  ffcr(Speed, Kp, -1, select);
}

void ffcr(int Speed, char select) {
  ffcr(Speed, -1, -1, select);
}

void fftimercl(int Speed, int totalTime) {
  int temp = set_position;
  set_position = set_positionL;
  fftimer(Speed, totalTime);
  set_position = temp;
}

void fftimercr(int Speed, int totalTime) {
  int temp = set_position;
  set_position = set_positionR;
  fftimer(Speed, totalTime);
  set_position = temp;
}

void ffcmcl(int Speed, int distance) {
  int temp = set_position;
  set_position = set_positionL;
  ffcm(Speed, distance);
  set_position = temp;
}

void ffcmcr(int Speed, int distance) {
  int temp = set_position;
  set_position = set_positionR;
  ffcm(Speed, distance);
  set_position = temp;
}

void fftimercl(int Speed, int totalTime, char select) {
  int temp = set_position;
  set_position = set_positionL;
  fftimer(Speed, totalTime, select);
  set_position = temp;
}

void fftimercr(int Speed, int totalTime, char select) {
  int temp = set_position;
  set_position = set_positionR;
  fftimer(Speed, totalTime, select);
  set_position = temp;
}

void ffcmcl(int Speed, int distance, char select) {
  int temp = set_position;
  set_position = set_positionL;
  ffcm(Speed, distance, select);
  set_position = temp;
}

void ffcmcr(int Speed, int distance, char select) {
  int temp = set_position;
  set_position = set_positionR;
  ffcm(Speed, distance, select);
  set_position = temp;
}

void bbcl(int Speed, float Kp, float Kd, char select) {
  int temp = set_position;
  set_position = set_positionL;
  BaseSpeed = Speed;
  InitialSpeed();
  if (Kp < 0) Kp = PID_KP_Back;
  if (Kd < 0) Kd = PID_KD_Back;
  while (1) {
    PIDB(BackLeftBaseSpeed, BackRightBaseSpeed, Kp, Kd);
    ReadCalibrateB();
    if (B[0] > Ref) break;
  }
  TrackSelectB(Speed, select);
  set_position = temp;
}

void bbcl(int Speed, float Kp, char select) {
  bbcl(Speed, Kp, -1, select);
}

void bbcl(int Speed, char select) {
  bbcl(Speed, -1, -1, select);
}

void bbcr(int Speed, float Kp, float Kd, char select) {
  int temp = set_position;
  set_position = set_positionR;
  BaseSpeed = Speed;
  InitialSpeed();
  if (Kp < 0) Kp = PID_KP_Back;
  if (Kd < 0) Kd = PID_KD_Back;
  while (1) {
    PIDB(BackLeftBaseSpeed, BackRightBaseSpeed, Kp, Kd);
    ReadCalibrateB();
    if (B[15] > Ref) break;
  }
  TrackSelectB(Speed, select);
  set_position = temp;
}

void bbcr(int Speed, float Kp, char select) {
  bbcr(Speed, Kp, -1, select);
}

void bbcr(int Speed, char select) {
  bbcr(Speed, -1, -1, select);
}

void bbtimercl(int Speed, int totalTime) {
  int temp = set_position;
  set_position = set_positionL;
  bbtimer(Speed, totalTime);
  set_position = temp;
}

void bbtimercr(int Speed, int totalTime) {
  int temp = set_position;
  set_position = set_positionR;
  bbtimer(Speed, totalTime);
  set_position = temp;
}

void bbcmcl(int Speed, int distance) {
  int temp = set_position;
  set_position = set_positionL;
  bbcm(Speed, distance);
  set_position = temp;
}

void bbcmcr(int Speed, int distance) {
  int temp = set_position;
  set_position = set_positionR;
  bbcm(Speed, distance);
  set_position = temp;
}

void bbtimercl(int Speed, int totalTime, char select) {
  int temp = set_position;
  set_position = set_positionL;
  bbtimer(Speed, totalTime, select);
  set_position = temp;
}

void bbtimercr(int Speed, int totalTime, char select) {
  int temp = set_position;
  set_position = set_positionR;
  bbtimer(Speed, totalTime, select);
  set_position = temp;
}

void bbcmcl(int Speed, int distance, char select) {
  int temp = set_position;
  set_position = set_positionL;
  bbcm(Speed, distance, select);
  set_position = temp;
}

void bbcmcr(int Speed, int distance, char select) {
  int temp = set_position;
  set_position = set_positionR;
  bbcm(Speed, distance, select);
  set_position = temp;
}

// --- *timercl/cr, *cmcl/cr แบบกำหนด Kp / Kd เอง: (speed, kp, [kd,] เวลา/ระยะ [, select]) ---

void fftimercl(int Speed, float Kp, int totalTime) {
  int temp = set_position;
  set_position = set_positionL;
  fftimer(Speed, Kp, totalTime);
  set_position = temp;
}

void fftimercl(int Speed, float Kp, float Kd, int totalTime) {
  int temp = set_position;
  set_position = set_positionL;
  fftimer(Speed, Kp, Kd, totalTime);
  set_position = temp;
}

void fftimercl(int Speed, float Kp, int totalTime, char select) {
  int temp = set_position;
  set_position = set_positionL;
  fftimer(Speed, Kp, totalTime, select);
  set_position = temp;
}

void fftimercl(int Speed, float Kp, float Kd, int totalTime, char select) {
  int temp = set_position;
  set_position = set_positionL;
  fftimer(Speed, Kp, Kd, totalTime, select);
  set_position = temp;
}

void ffcmcl(int Speed, float Kp, int distance) {
  int temp = set_position;
  set_position = set_positionL;
  ffcm(Speed, Kp, distance);
  set_position = temp;
}

void ffcmcl(int Speed, float Kp, float Kd, int distance) {
  int temp = set_position;
  set_position = set_positionL;
  ffcm(Speed, Kp, Kd, distance);
  set_position = temp;
}

void ffcmcl(int Speed, float Kp, int distance, char select) {
  int temp = set_position;
  set_position = set_positionL;
  ffcm(Speed, Kp, distance, select);
  set_position = temp;
}

void ffcmcl(int Speed, float Kp, float Kd, int distance, char select) {
  int temp = set_position;
  set_position = set_positionL;
  ffcm(Speed, Kp, Kd, distance, select);
  set_position = temp;
}

void fftimercr(int Speed, float Kp, int totalTime) {
  int temp = set_position;
  set_position = set_positionR;
  fftimer(Speed, Kp, totalTime);
  set_position = temp;
}

void fftimercr(int Speed, float Kp, float Kd, int totalTime) {
  int temp = set_position;
  set_position = set_positionR;
  fftimer(Speed, Kp, Kd, totalTime);
  set_position = temp;
}

void fftimercr(int Speed, float Kp, int totalTime, char select) {
  int temp = set_position;
  set_position = set_positionR;
  fftimer(Speed, Kp, totalTime, select);
  set_position = temp;
}

void fftimercr(int Speed, float Kp, float Kd, int totalTime, char select) {
  int temp = set_position;
  set_position = set_positionR;
  fftimer(Speed, Kp, Kd, totalTime, select);
  set_position = temp;
}

void ffcmcr(int Speed, float Kp, int distance) {
  int temp = set_position;
  set_position = set_positionR;
  ffcm(Speed, Kp, distance);
  set_position = temp;
}

void ffcmcr(int Speed, float Kp, float Kd, int distance) {
  int temp = set_position;
  set_position = set_positionR;
  ffcm(Speed, Kp, Kd, distance);
  set_position = temp;
}

void ffcmcr(int Speed, float Kp, int distance, char select) {
  int temp = set_position;
  set_position = set_positionR;
  ffcm(Speed, Kp, distance, select);
  set_position = temp;
}

void ffcmcr(int Speed, float Kp, float Kd, int distance, char select) {
  int temp = set_position;
  set_position = set_positionR;
  ffcm(Speed, Kp, Kd, distance, select);
  set_position = temp;
}

void bbtimercl(int Speed, float Kp, int totalTime) {
  int temp = set_position;
  set_position = set_positionL;
  bbtimer(Speed, Kp, totalTime);
  set_position = temp;
}

void bbtimercl(int Speed, float Kp, float Kd, int totalTime) {
  int temp = set_position;
  set_position = set_positionL;
  bbtimer(Speed, Kp, Kd, totalTime);
  set_position = temp;
}

void bbtimercl(int Speed, float Kp, int totalTime, char select) {
  int temp = set_position;
  set_position = set_positionL;
  bbtimer(Speed, Kp, totalTime, select);
  set_position = temp;
}

void bbtimercl(int Speed, float Kp, float Kd, int totalTime, char select) {
  int temp = set_position;
  set_position = set_positionL;
  bbtimer(Speed, Kp, Kd, totalTime, select);
  set_position = temp;
}

void bbcmcl(int Speed, float Kp, int distance) {
  int temp = set_position;
  set_position = set_positionL;
  bbcm(Speed, Kp, distance);
  set_position = temp;
}

void bbcmcl(int Speed, float Kp, float Kd, int distance) {
  int temp = set_position;
  set_position = set_positionL;
  bbcm(Speed, Kp, Kd, distance);
  set_position = temp;
}

void bbcmcl(int Speed, float Kp, int distance, char select) {
  int temp = set_position;
  set_position = set_positionL;
  bbcm(Speed, Kp, distance, select);
  set_position = temp;
}

void bbcmcl(int Speed, float Kp, float Kd, int distance, char select) {
  int temp = set_position;
  set_position = set_positionL;
  bbcm(Speed, Kp, Kd, distance, select);
  set_position = temp;
}

void bbtimercr(int Speed, float Kp, int totalTime) {
  int temp = set_position;
  set_position = set_positionR;
  bbtimer(Speed, Kp, totalTime);
  set_position = temp;
}

void bbtimercr(int Speed, float Kp, float Kd, int totalTime) {
  int temp = set_position;
  set_position = set_positionR;
  bbtimer(Speed, Kp, Kd, totalTime);
  set_position = temp;
}

void bbtimercr(int Speed, float Kp, int totalTime, char select) {
  int temp = set_position;
  set_position = set_positionR;
  bbtimer(Speed, Kp, totalTime, select);
  set_position = temp;
}

void bbtimercr(int Speed, float Kp, float Kd, int totalTime, char select) {
  int temp = set_position;
  set_position = set_positionR;
  bbtimer(Speed, Kp, Kd, totalTime, select);
  set_position = temp;
}

void bbcmcr(int Speed, float Kp, int distance) {
  int temp = set_position;
  set_position = set_positionR;
  bbcm(Speed, Kp, distance);
  set_position = temp;
}

void bbcmcr(int Speed, float Kp, float Kd, int distance) {
  int temp = set_position;
  set_position = set_positionR;
  bbcm(Speed, Kp, Kd, distance);
  set_position = temp;
}

void bbcmcr(int Speed, float Kp, int distance, char select) {
  int temp = set_position;
  set_position = set_positionR;
  bbcm(Speed, Kp, distance, select);
  set_position = temp;
}

void bbcmcr(int Speed, float Kp, float Kd, int distance, char select) {
  int temp = set_position;
  set_position = set_positionR;
  bbcm(Speed, Kp, Kd, distance, select);
  set_position = temp;
}

#endif  // MYMINI_16PRO_PID_H
