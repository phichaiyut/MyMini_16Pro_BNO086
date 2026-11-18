#ifndef MYMINI16PRO_BNO086_GYRO_H
#define MYMINI16PRO_BNO086_GYRO_H
// โมดูลที่ไฟล์นี้ใช้ (include ซ้ำได้ มี guard) → include ไฟล์นี้เดี่ยว ๆ หรือสลับลำดับได้
#include "MyMini_16Pro_BNO086_Decl.h"
#include "MyMini_16Pro_BNO086_Buzzer.h"
#include "MyMini_16Pro_BNO086_I2C.h"
#include "MyMini_16Pro_BNO086_Sensor.h"
#include "MyMini_16Pro_BNO086_Motor.h"
#include "MyMini_16Pro_BNO086_PID.h"

/*
 * MyMini_16Pro_BNO086_Gyro.h
 * ไจโร BNO086: อ่านมุม, หมุน/เลี้ยวตามองศา, วิ่งตรง และทางแยกด้วยไจโร
 * พอร์ตจาก MyRP_Pico2_mini_BNO086_TEST (Gyro.h) ทั้งไฟล์ — ลอจิก / ชื่อ / overload / ค่าเริ่มต้นตามต้นแบบ
 * ผูกกับฮาร์ดแวร์ MyMINI_16Pro:
 *   - BNO086 บน Wire (SDA GP4 / SCL GP5, 100 kHz, timeout 25 ms) — ขาและ clock จาก MyMini_16Pro_BNO086_I2C.h
 *   - เซนเซอร์เส้น 16 ช่อง: ดัชนีแบบ 8 ช่องของ MyRP แปลงเป็น 16 ช่อง
 *       ผ่านแยก F[0] / F[7] → F[3] / F[12] | ชิดเส้น F[1..6] → F[2], F[4], F[6], F[9], F[11], F[13] (B เหมือนกัน)
 *   - ทิศ yaw: MYMINI_YAW_SIGN (ค่าเริ่มต้น -1) | โหมดเข็มทิศ: MYMINI_BNO_USE_MAG | รอบรายงาน: IMU_REPORT_MS
 *   - [15] ชื่อเดิมของ MyMini_16Pro_BNO086 (gyro_Begin, imuReady, spindirection ...) เรียกตัวใหม่
 *
 * สารบัญ:
 *   [1] IMU BNO08x และตัวแปรปรับค่าไจโร
 *   [2] อ่านมุม IMU (yaw / reset)
 *   [3] ตั้งค่าไจโร (PID / ตัวคูณระยะทาง / เบรก)
 *   [4] ล็อกมุม (HoldAngle / SetG)
 *   [5] หมุน / เลี้ยวตามองศา (spindegree / turndegree)
 *   [6] วิ่งตรงด้วยไจโร (RunG / RunGB)
 *   [7] วิ่งตรงตามระยะทาง (ffcmg / bbcmg)
 *   [8] คำสั่งย่อ หมุน / เลี้ยว (spinlg, slg, tlg ...)
 *   [9] เข้ากลาง / ชิดเส้น ด้วยไจโร
 *   [10] ทางแยก (TrackSelectG / TrackSelectGB)
 *   [11] วิ่งตรงจนเจอเส้น / ระยะ + ทางแยก
 *   [12] ควบคุมตามทิศทางจริง / เข็มทิศ (Absolute Heading)
 *   [13] วิ่งตามเส้นจนหันถึงทิศ / องศา (ffdir, bbdir, ffdeg, bbdeg)
 *   [14] วิ่งเกาะเส้นโค้งจนหันถึงทิศ / องศา (ffdircl, ffdircr, bbdircl, ffdegcl ...)
 *   [15] ชื่อเดิมของ MyMini_16Pro_BNO086 (gyro_Begin, imuReady, spindirection, turndirection ...)
 */

#include <Wire.h>
#include <SparkFun_BNO08x_Arduino_Library.h>
#include <type_traits>

// ======================================================================
// [1] IMU BNO08x และตัวแปรปรับค่าไจโร
// ======================================================================

BNO08x myIMU;
float yawOffset = 0;
float lastYaw = 0;
float yawgyro = 0;
// ทิศของมุม yaw ขึ้นกับการติดตั้ง BNO086 บนหุ่น: หมุนหุ่นไปทางขวาแล้ว gyroZ() ต้องเป็นบวก
// ถ้าเป็นลบ ให้ใส่ #define MYMINI_YAW_SIGN 1 ไว้ก่อน #include <MyMini_16Pro_BNO086.h> (ตรวจด้วย examples/GyroTest)
#ifndef MYMINI_YAW_SIGN
#define MYMINI_YAW_SIGN -1
#endif
const int YAW_SIGN = MYMINI_YAW_SIGN;

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
int gyro_MinSpd_Turn = 15;
float gyro_SmallAngle_Turn = 15.0f;
float gyro_StopThr_Turn = 2.0f;
float gyro_StopThr_Rotate = 1.0f;

// ---------- ค่าปรับไจโร: วิ่งโค้งกำหนดล้อเอง (Rotate) — rotatedegree / rotatedir ----------
// ช่วงชะลอ: scale = kp × (มุมที่เหลือ + kd × อัตราเข้าหาเป้า[องศา/10ms]) / smallAngle
//   kp > 1 = ชะลอช้าลง (เข้าเร็ว) | kd = มองล่วงหน้ากี่ × 10ms (หมุนเลยมุม → เพิ่ม kd)

float gyro_Kp_Rotate = 1.0f;
float gyro_Kd_Rotate = 2.0f;
int gyro_MinSpd_Rotate = 15;           // ล้อที่เร็วกว่าไม่ต่ำกว่านี้ตอนชะลอ (แรงขั้นต่ำที่ล้อยังขยับ)
float gyro_SmallAngle_Rotate = 25.0f;  // เหลือน้อยกว่านี้ (องศา) เริ่มชะลอ

// ---------- ค่าปรับไจโร: เลี้ยวถอยหลัง (TurnB) — turndegreeb (+ _none) ----------

float gyro_Kp_TurnB = 1.2f;
float gyro_Kd_TurnB = 0.25f;
int gyro_MaxSpd_TurnB = 50;
int gyro_MinSpd_TurnB = 15;
float gyro_SmallAngle_TurnB = 15.0f;
float gyro_StopThr_TurnB = 2.0f;

// ---------- สถานะมุมเป้าหมาย / error ก่อนหน้า ----------

float current_degree = 0;
float previous_errorG = 0;
float previous_errorGB = 0;

// ---------- ตัวคูณแปลงความเร็ว → ระยะทาง (ตั้งด้วย set_speed_scale_*) ----------

float speed_scale_ff_g = 1.5f;         // ค่าตัวคูณความเร็ววิ่งตรงระยะสั้น < 30 cm
float speed_scale_bb_g = 1.5f;
float speed_scale_ff_g_normal = 0.99f; // ค่าตัวคูณความเร็ววิ่งตรงระยะ >= 30 cm (มี ramp)
float speed_scale_bb_g_normal = 0.99f;

float speed_scale_ff_gs = 1.65f;       // ค่าตัวคูณความเร็ววิ่งตรงคงที่ (ffcmgs / bbcmgs)
float speed_scale_bb_gs = 1.65f;

// ---------- เบรกหลังหมุน / เลี้ยว (ตั้งด้วย set_brake_*_gyro*) ----------

int brake_spin_g = 5;
int brake_turn_g = 5;
int brake_turn_b_g = 5;

int gyro_BounceSpd_spin = 12;
int gyro_BounceMs_spin = 5;

int gyro_BounceSpd_turn = 12;
int gyro_BounceMs_turn = 5;

int gyro_BounceSpd_turn_b = 12;
int gyro_BounceMs_turn_b = 5;

int brake_rotate_g = 20;        // rotatedegree / rotatedir: เวลาล็อกมุม (SetG) หลังเบรก (ms)
int gyro_BounceSpd_rotate = 15;  // เบรกสวนทิศล้อ (ล้อที่เร็วกว่า = ค่านี้ ล้อช้ากว่าลดตามสัดส่วน)
int gyro_BounceMs_rotate = 10;

// ======================================================================
// [2] อ่านมุม IMU (yaw / reset)
// ======================================================================

float wrap180(float a) {
  while (a > 180.0f) a -= 360.0f;
  while (a < -180.0f) a += 360.0f;
  return a;
}

// ---------- โหมด report มุมของ BNO08x ----------
// GYRO_MODE_GAME_RV (ค่าเริ่มต้น): Game Rotation Vector = gyro + accel ไม่ใช้เข็มทิศ
//   มุม 0° นิ่งตั้งแต่เปิดเครื่อง ไม่โดนแม่เหล็กมอเตอร์ดึง (ดริฟต์ช้าๆ ไม่กี่องศาต่อนาที)
// GYRO_MODE_RV: Rotation Vector = ผสม magnetometer มุมจะค่อยๆ ดึงเข้าหาทิศเหนือหลังบูต
// เลือกโหมดด้วย SetGyroMode(...) "ก่อน" RobotSetup() หรือเรียกทีหลังก็ได้ (จะสลับ report ให้)
#define GYRO_MODE_GAME_RV 0
#define GYRO_MODE_RV 1

// MyMINI: #define MYMINI_BNO_USE_MAG 1 ก่อน #include = เริ่มด้วย Rotation Vector (ใช้เข็มทิศ)
#ifndef MYMINI_BNO_USE_MAG
#define MYMINI_BNO_USE_MAG 0
#endif
// รอบรายงานมุม (ms) — เปลี่ยนได้: #define IMU_REPORT_MS 20 ไว้ก่อน #include
#ifndef IMU_REPORT_MS
#define IMU_REPORT_MS 5
#endif
uint8_t gyro_report_mode = MYMINI_BNO_USE_MAG ? GYRO_MODE_RV : GYRO_MODE_GAME_RV;
uint16_t gyro_report_interval_ms = IMU_REPORT_MS;  // รอบส่งมุม (ms) 5 = 200Hz หน่วงน้อย หมุนไม่เลยมุม | 10 = 100Hz
bool gyro_imu_ready = false;            // true หลัง gyroBegin() เจอ BNO08x
bool &imu_connected = gyro_imu_ready;   // ชื่อเรียกอีกชื่อของแฟล็กเดียวกัน (ไม่มีสองแฟล็กที่ค่าไม่ตรงกัน)
uint32_t gyro_reset_count = 0;          // จำนวนครั้งที่ BNO08x รีเซ็ตตัวเอง (ไว้ debug)
bool gyro_rebase_pending = false;       // true = เพิ่งรีเซ็ต รอค่ามุมแรกเพื่อต่อมุมเดิม (ดู updateYawFromEvent)

// ---------- บัส I2C (Wire) ----------
// Wire ของ arduino-pico มี timeout เริ่มต้น 1000 ms ต่อ "หนึ่งทรานแซกชัน" และไม่กู้บัสเอง
// ถ้าบัสค้าง (มีชิปดึง SDA/SCL ต่ำไว้) ทุกคำสั่ง I2C จะค้างครั้งละ 1 วินาที → ลูป sw() ค้าง ~6 วินาที
// จึงตั้ง timeout สั้น + ให้ core กู้บัสอัตโนมัติเมื่อ timeout (Wire.setTimeout(ms, true))
uint32_t i2c_clock_hz = I2C_CLOCK;  // MyMINI: I2C_CLOCK (100 kHz)  // 100 kHz: BNO086 ใช้ clock stretching, 400 kHz ทดสอบแล้วมุมไม่มา
uint32_t i2c_timeout_ms = I2C_TIMEOUT_MS;  // MyMINI: I2C_TIMEOUT_MS (25 ms)    // อ่านทีละ 32 ไบต์ที่ 100 kHz ใช้ ~3 ms → 25 ms เผื่อ clock stretching แล้ว

// ตั้ง clock + timeout ของ Wire (เรียกซ้ำได้ทุกครั้งหลัง Wire.begin())
void i2cApplyConfig() {
  Wire.setClock(i2c_clock_hz);
  Wire.setTimeout(i2c_timeout_ms, true);  // true = timeout แล้วให้ core reset + กู้บัสเอง
}

// อ่านสถานะสาย SDA/SCL ตอนปล่อยว่าง (ต้องเป็น HIGH ทั้งคู่) — ต้องเรียกตอน Wire ยังไม่ทำงาน
void i2cReadLines(bool &sdaHigh, bool &sclHigh) {
  pinMode(I2C_SDA_PIN, INPUT_PULLUP);
  pinMode(I2C_SCL_PIN, INPUT_PULLUP);
  delayMicroseconds(50);
  sdaHigh = digitalRead(I2C_SDA_PIN) == HIGH;
  sclHigh = digitalRead(I2C_SCL_PIN) == HIGH;
}

// I2C Bus Recovery แบบ open-drain แล้วเริ่ม Wire ใหม่ | คืน true ถ้าหลังกู้ SDA/SCL ว่าง (HIGH) ทั้งคู่
// "ดึงลง" = OUTPUT LOW, "ปล่อย" = INPUT_PULLUP (ห้ามขับ HIGH แบบ push-pull ชนกับ slave)
// หมายเหตุ: ถ้า SCL ถูกดึงต่ำค้าง (slave ค้าง clock stretching) master แก้ไม่ได้ ต้องตัดไฟ/รีเซ็ตชิป
bool i2cBusRecover() {
  Wire.end();  // ปล่อยขาจากฮาร์ดแวร์ I2C ก่อน (ถ้ายังไม่ begin จะไม่ทำอะไร)
  bool sdaHigh, sclHigh;
  i2cReadLines(sdaHigh, sclHigh);

  // 9-10 clock ที่ SCL ให้ slave ที่ค้างกลางไบต์ส่งบิตที่เหลือจนปล่อย SDA
  for (int i = 0; i < 10; i++) {
    pinMode(I2C_SCL_PIN, OUTPUT);
    digitalWrite(I2C_SCL_PIN, LOW);
    delayMicroseconds(5);
    pinMode(I2C_SCL_PIN, INPUT_PULLUP);
    delayMicroseconds(5);
  }
  // STOP: SCL ต่ำ → SDA ต่ำ → ปล่อย SCL → ปล่อย SDA (SDA ขึ้นตอน SCL สูง)
  pinMode(I2C_SCL_PIN, OUTPUT);
  digitalWrite(I2C_SCL_PIN, LOW);
  delayMicroseconds(5);
  pinMode(I2C_SDA_PIN, OUTPUT);
  digitalWrite(I2C_SDA_PIN, LOW);
  delayMicroseconds(5);
  pinMode(I2C_SCL_PIN, INPUT_PULLUP);
  delayMicroseconds(5);
  pinMode(I2C_SDA_PIN, INPUT_PULLUP);
  delayMicroseconds(5);

  i2cReadLines(sdaHigh, sclHigh);
  Wire.setSDA(I2C_SDA_PIN);
  Wire.setSCL(I2C_SCL_PIN);
  Wire.begin();
  i2cApplyConfig();
  return sdaHigh && sclHigh;
}

// เช็กว่ามีอุปกรณ์ตอบ ACK ที่ address นี้ไหม (ไม่เจอ = NACK กลับทันที, บัสค้าง = รอไม่เกิน i2c_timeout_ms)
bool i2cProbe(uint8_t addr) {
  Wire.beginTransmission(addr);
  return Wire.endTransmission() == 0;
}

const char *i2cDeviceName(uint8_t addr) {
  switch (addr) {
    case 0x20: return "PCF8574 (LED ระดับแบต)";
    case 0x48: return "ADS1115 (แบต + เซนเซอร์กลาง CL/CR)";
    case 0x4A:
    case 0x4B: return "BNO08x (IMU)";
    case 0x50: return "CAT24C128 EEPROM (คาลิเบรต)";
    case 0x68: return "MCP3421 (ปุ่ม ADCcal)";
    default: return "?";
  }
}

// สแกนบัส I2C แล้วพิมพ์อุปกรณ์ที่เจอ | คืนจำนวนอุปกรณ์ที่เจอ
// ถ้าเจอ timeout (บัสค้าง) จะหยุดสแกนทันที ไม่รอครบ 112 address
int i2cScan() {
  int found = 0;
  Serial.println("I2C scan (SDA=GP4, SCL=GP5):");
  for (uint8_t addr = 0x08; addr <= 0x77; addr++) {
    Wire.clearTimeoutFlag();
    bool ack = i2cProbe(addr);
    if (Wire.getTimeoutFlag()) {
      Serial.println("  I2C TIMEOUT -> bus stuck (SDA/SCL held LOW?) - scan aborted");
      return found;
    }
    if (ack) {
      Serial.print("  Found I2C device at 0x");
      if (addr < 16) Serial.print('0');
      Serial.print(addr, HEX);
      Serial.print("  ");
      Serial.println(i2cDeviceName(addr));
      found++;
    }
  }
  if (found == 0) Serial.println("  No I2C devices found! (check wiring / pull-ups / power)");
  return found;
}

// ---------- เชื่อมต่อ BNO08x ----------
uint8_t imu_i2c_addr = 0;           // address ที่เจอ BNO08x (0 = ยังไม่เคยเจอ)
bool gyro_sh2_open = false;          // ไลบรารี SparkFun เปิด SHTP ได้ "ครั้งเดียว" ต้อง sh2_close() ก่อน begin ซ้ำ
bool gyro_auto_reconnect = true;     // IMU หลุดแล้วให้ลองต่อใหม่เองเป็นระยะ
uint32_t gyro_reconnect_interval_ms = 2000;
uint8_t gyro_i2c_err_count = 0;      // I2C timeout ติดกันตอนอ่าน IMU
uint32_t gyro_last_reconnect_ms = 0;

// เรียก myIMU.begin() ที่ address เดียว แบบปลอดภัย: probe ก่อน (เร็ว), ปิด SHTP เดิมก่อนเปิดใหม่
bool imuBeginAt(uint8_t addr) {
  if (!i2cProbe(addr)) return false;  // ไม่มีใครตอบ → ไม่ต้องเรียก begin (ซึ่งช้า ~300+ ms)
  if (gyro_sh2_open) {
    sh2_close();
    gyro_sh2_open = false;
  }
  bool ok = myIMU.begin(addr, Wire, -1, -1);
  gyro_sh2_open = true;  // probe ผ่านแล้ว → begin เปิด SHTP แน่นอน (แม้ init จะไม่สำเร็จ)
  if (ok) imu_i2c_addr = addr;
  return ok;
}

// หา BNO08x ที่ 0x4A แล้ว 0x4B (ลองซ้ำได้ไม่เกิน timeout_ms) | คืน address ที่เจอ หรือ 0 ถ้าไม่เจอ
uint8_t imuDetect(uint32_t timeout_ms = 3000) {
  uint32_t t0 = millis();
  do {
    if (imuBeginAt(0x4A)) return 0x4A;
    if (imuBeginAt(0x4B)) return 0x4B;
    delay(100);
  } while (millis() - t0 < timeout_ms);
  return 0;
}

// ทำเครื่องหมายว่า IMU หลุด: ฟังก์ชันอ่านมุมทุกตัวจะคืน lastYaw ทันที (0 ms) ไม่แตะ I2C
void imuMarkOffline(const char *why) {
  if (!gyro_imu_ready) return;
  gyro_imu_ready = false;
  gyro_last_reconnect_ms = millis();
  Serial.print("BNO08x offline: ");
  Serial.println(why);
}

bool gyroEnableReports();
// ลองต่อ BNO08x ใหม่ (เว้นช่วง gyro_reconnect_interval_ms) — ไม่เจอ = เสียเวลาแค่ probe สั้นๆ
bool imuTryReconnect() {
  if (gyro_imu_ready || !gyro_auto_reconnect) return gyro_imu_ready;
  uint32_t now = millis();
  if (now - gyro_last_reconnect_ms < gyro_reconnect_interval_ms) return false;
  gyro_last_reconnect_ms = now;

  Wire.clearTimeoutFlag();
  uint8_t addr = imu_i2c_addr ? imu_i2c_addr : 0x4A;
  bool ok = imuBeginAt(addr);
  if (!ok && !imu_i2c_addr) ok = imuBeginAt(0x4B);
  if (!ok) {
    if (Wire.getTimeoutFlag()) i2cBusRecover();  // บัสค้าง → กู้บัส รอรอบหน้า
    gyro_last_reconnect_ms = millis();
    return false;
  }
  myIMU.wasReset();  // ล้างแฟล็ก reset จาก begin()
  gyroEnableReports();
  gyro_rebase_pending = true;  // ชิปนับมุมใหม่ → ต่อมุมจาก lastYaw เดิม
  gyro_i2c_err_count = 0;
  gyro_imu_ready = true;
  gyro_last_reconnect_ms = millis();
  Serial.print("BNO08x reconnected at 0x");
  Serial.println(imu_i2c_addr, HEX);
  return true;
}

// อ่าน event จาก BNO08x หนึ่งครั้ง "อย่างปลอดภัย" — ใช้แทน myIMU.getSensorEvent() ทุกที่
// IMU ไม่พร้อม → คืน false ทันที (ไม่แตะ I2C) | I2C timeout ติดกัน 3 ครั้ง → ถือว่าหลุด
// การต่อใหม่ (imuTryReconnect) เรียกเฉพาะใน sw() ตอนมอเตอร์หยุด เพราะ begin() ใช้เวลา ~0.5 วินาที
bool imuPoll() {
  if (!gyro_imu_ready) return false;
  Wire.clearTimeoutFlag();
  bool got = myIMU.getSensorEvent();
  if (Wire.getTimeoutFlag()) {
    Wire.clearTimeoutFlag();
    if (++gyro_i2c_err_count >= 3) imuMarkOffline("I2C timeout x3 (bus stuck / wire loose)");
    return false;
  }
  gyro_i2c_err_count = 0;
  return got;
}

uint8_t gyroReportID() {
  return (gyro_report_mode == GYRO_MODE_RV) ? SENSOR_REPORTID_ROTATION_VECTOR
                                            : SENSOR_REPORTID_GAME_ROTATION_VECTOR;
}

bool gyroEnableReports() {
  if (gyro_report_mode == GYRO_MODE_RV) return myIMU.enableRotationVector(gyro_report_interval_ms);
  return myIMU.enableGameRotationVector(gyro_report_interval_ms);
}

// BNO08x รีเซ็ตตัวเอง (เช่นไฟกระตุก) report จะถูกปิดหมด → เปิดใหม่อัตโนมัติ
// หลังรีเซ็ต ชิปนับมุม 0 ใหม่จากทิศตอนนั้น → ค่ามุมแรกที่ได้จะถูกใช้ตั้ง yawOffset ใหม่
// ให้มุมต่อจาก lastYaw เดิม (ไม่กระโดด) | คืน true ถ้าเพิ่งเจอ reset
bool gyroCheckReset() {
  if (!gyro_imu_ready) return false;
  if (!myIMU.wasReset()) return false;
  gyro_reset_count++;
  gyro_rebase_pending = true;
  gyroEnableReports();
  Serial.println("BNO08x reset detected -> reports re-enabled");
  return true;
}

void SetGyroMode(uint8_t mode) {
  if (mode == gyro_report_mode) return;
  if (gyro_imu_ready) {
    // ปิด report เดิมก่อน (interval 0 = ปิด) กัน quaternion สองโหมดปนกัน
    if (gyro_report_mode == GYRO_MODE_RV) myIMU.enableRotationVector(0);
    else myIMU.enableGameRotationVector(0);
  }
  gyro_report_mode = mode;
  if (gyro_imu_ready) gyroEnableReports();
}

bool isYawEvent() {
  return myIMU.getSensorEventID() == gyroReportID();
}

bool waitForRotVec(uint32_t timeout_ms = 200) {
  if (!gyro_imu_ready) return false;
  uint32_t t0 = millis();
  while (millis() - t0 < timeout_ms && gyro_imu_ready) {
    gyroCheckReset();
    if (imuPoll() && isYawEvent()) {
      return true;
    }
    delay(1);
  }
  return false;
}

float quatToYawDeg() {
  float q0, q1, q2, q3;
  if (gyro_report_mode == GYRO_MODE_RV) {
    q0 = myIMU.getQuatReal();
    q1 = myIMU.getQuatI();
    q2 = myIMU.getQuatJ();
    q3 = myIMU.getQuatK();
  } else {
    q0 = myIMU.getGameQuatReal();
    q1 = myIMU.getGameQuatI();
    q2 = myIMU.getGameQuatJ();
    q3 = myIMU.getGameQuatK();
  }

  // ใช้ฟังก์ชัน/ค่าคงที่แบบ float ทั้งหมด: RP2350 มี FPU เฉพาะ single precision
  float yaw = atan2f(2.0f * (q0 * q3 + q1 * q2),
                     1.0f - 2.0f * (q2 * q2 + q3 * q3));
  yaw = yaw * (180.0f / (float)PI);
  yaw = wrap180(yaw);

  yaw *= (float)YAW_SIGN;
  yaw = wrap180(yaw);

  return yaw;
}

// นับจำนวนค่ามุมใหม่ที่ได้จาก IMU (ใช้เช็กว่ารอบนี้ได้ค่าจริงใหม่หรือเป็นค่าเดิมค้าง)
uint32_t yawSampleCount = 0;
unsigned long lastImuEventMs = 0;  // เวลาที่ได้ค่ามุมใหม่ล่าสุด (ใช้ใน gyroAbort)

// ---------- D-term ของ RunG / RunGB แบบคิดเฉพาะตอนได้มุมใหม่ (set_gyro_run_dterm_fresh) ----------
// false (ค่าเริ่มต้น) = แบบเดิม: คิด D ทุกรอบลูป (รอบที่ไม่มีมุมใหม่ D = 0, รอบที่มี D กระโดดครั้งเดียว
//                        → Kd แทบไม่มีผล + กำลังล้อกระตุก) ค่า kdG / kdGB เดิมที่จูนไว้ใช้ได้เหมือนเดิม
// true = คิด PD เฉพาะตอนได้มุมใหม่ แล้วคงค่าไว้จนถึงค่าถัดไป, D = องศาที่เปลี่ยนต่อ 10ms
//        (เหมือน HoldAngle / spindegree) → Kd มีผลจริง ต้องจูน SetGyroRun / SetGyroRunB ใหม่ (เริ่มจาก kd น้อย ๆ)
bool gyro_run_dterm_fresh = false;
uint32_t runGSeen = 0, runGBSeen = 0;
float runGPdHeld = 0.0f, runGBPdHeld = 0.0f;

void set_gyro_run_dterm_fresh(bool enable) {
  gyro_run_dterm_fresh = enable;
}

// บังคับให้ RunG / RunGB รอบถัดไปคำนวณ PD ใหม่ทันที (ไม่ใช้ค่าที่คงไว้จากคำสั่งก่อน)
void resetRunGFreshState() {
  runGSeen = yawSampleCount - 1;
  runGBSeen = yawSampleCount - 1;
}

float updateYawFromEvent() {
  yawSampleCount++;
  lastImuEventMs = millis();
  float yaw = quatToYawDeg();
  if (gyro_rebase_pending) {
    // ค่าแรกหลังชิปรีเซ็ต: ตั้ง offset ให้มุมนี้ = lastYaw (มุมก่อนรีเซ็ต) → ทิศอ้างอิงต่อเนื่อง
    yawOffset = wrap180(yaw - lastYaw);
    gyro_rebase_pending = false;
  }
  float yawZeroed = wrap180(yaw - yawOffset);
  float delta = wrap180(yawZeroed - lastYaw);
  yawgyro += delta;
  return lastYaw = yawZeroed;
}

float angleRead() {
  if (!gyro_imu_ready) return lastYaw;  // ไม่มี IMU → คืนค่าเดิมทันที ไม่แตะ I2C
  gyroCheckReset();
  if (!imuPoll()) return lastYaw;
  if (!isYawEvent()) return lastYaw;
  return updateYawFromEvent();
}

// ล้างคิว (Flush FIFO): อ่านแพ็กเก็ตที่ค้างใน BNO08x ออกจนหมด
// ระหว่างล้างยังอัปเดต lastYaw / yawgyro ไปด้วย (มุมสะสมไม่ขาดช่วง)
// หยุดเมื่ออ่านว่างติดกัน 2 ครั้ง (ครั้งเดียวอาจเป็นแพ็กเก็ตคำสั่ง ไม่ใช่คิวหมด) หรือครบ max_ms
float gyroFlushFIFO(uint32_t max_ms = 60) {
  if (!gyro_imu_ready) return lastYaw;
  uint32_t t0 = millis();
  uint8_t emptyCount = 0;
  while (millis() - t0 < max_ms && gyro_imu_ready) {
    gyroCheckReset();
    if (imuPoll()) {
      emptyCount = 0;
      if (isYawEvent()) updateYawFromEvent();
    } else if (++emptyCount >= 2) {
      break;
    }
  }
  return lastYaw;
}

// ล้างคิวแล้วคืนมุมล่าสุดที่มีในคิว (ไม่รอแพ็กเก็ตใหม่) — ใช้แสดงผลในลูปช้าๆ เช่น sw()
float angleReadLatest() {
  return gyroFlushFIFO();
}

// ล้างคิวเก่าทิ้ง แล้วรอแพ็กเก็ตใหม่จริงๆ ก่อนคืนค่า (ใช้ตอนตั้งมุมเป้าหมาย)
float angleReadFresh() {
  if (!gyro_imu_ready) return lastYaw;
  gyroFlushFIFO();
  if (!waitForRotVec(50)) return lastYaw;
  return updateYawFromEvent();
}

// อ่านมุมดิบ (ก่อนหัก offset) แบบสดใหม่: ล้างคิวก่อน แล้วเก็บตัวอย่างห่างกัน ~20ms
// จนได้ GYRO_STABLE_N ตัวติดกันที่แกว่งไม่เกิน GYRO_STABLE_TOL องศา แล้วคืนค่าเฉลี่ย
// ถ้าไม่นิ่งภายใน timeout_ms ใช้ค่าล่าสุด | คืน false = ไม่มีข้อมูลจากไจโรเลย
#define GYRO_STABLE_N 5
float GYRO_STABLE_TOL = 0.5f;
uint16_t GYRO_STABLE_SPACING_MS = 20;

bool readStableRawYaw(float &out, uint32_t timeout_ms = 600) {
  if (!gyro_imu_ready) return false;
  gyroFlushFIFO();
  float buf[GYRO_STABLE_N];
  int n = 0;
  bool got = false;
  float last = 0;
  uint32_t t0 = millis();
  while (millis() - t0 < timeout_ms) {
    gyroFlushFIFO(30);  // ทิ้งของค้างระหว่างรอ ให้ตัวอย่างถัดไปเป็นค่าสดเสมอ
    if (!waitForRotVec(50)) continue;
    float y = quatToYawDeg();
    got = true;
    last = y;
    if (n < GYRO_STABLE_N) {
      buf[n++] = y;
    } else {
      for (int i = 0; i < GYRO_STABLE_N - 1; i++) buf[i] = buf[i + 1];
      buf[GYRO_STABLE_N - 1] = y;
    }
    if (n == GYRO_STABLE_N) {
      // เทียบกับตัวแรกด้วย wrap180 กันปัญหามุมข้าม ±180
      float mn = 0, mx = 0, sum = 0;
      for (int i = 0; i < GYRO_STABLE_N; i++) {
        float d = wrap180(buf[i] - buf[0]);
        if (d < mn) mn = d;
        if (d > mx) mx = d;
        sum += d;
      }
      if (mx - mn <= GYRO_STABLE_TOL) {
        out = wrap180(buf[0] + sum / GYRO_STABLE_N);
        return true;
      }
    }
    delay(GYRO_STABLE_SPACING_MS);
  }
  if (got) {
    Serial.println("Gyro not settled in time -> use latest sample");
    out = last;
    return true;
  }
  return false;
}

// คืน true ถ้า reset สำเร็จ, false = gyro ไม่ส่งข้อมูล (บี๊บเตือน และ offset เป็นค่าเดิม)
bool resetYaw() {
  if (!gyro_imu_ready) {
    // ไม่มี IMU: ตั้งมุมอ้างอิงเป็น 0 เฉยๆ ไม่แตะ I2C (กันค้าง)
    lastYaw = 0.0f;
    yawgyro = 0.0f;
    return false;
  }
  float yawNow;
  if (!readStableRawYaw(yawNow)) {
    Serial.println("ResetYaw timeout: no rotation vector");
    Beep(500);
    return false;
  }
  yawOffset = yawNow;
  gyro_rebase_pending = false;  // ตั้ง 0° ใหม่แล้ว ไม่ต้องต่อมุมเดิม
  lastYaw = 0.0f;
  yawgyro = 0.0f;
  return true;
}

// เริ่มไจโรหลัง myIMU.begin() สำเร็จ: เปิด report → warm-up ให้ฟิลเตอร์นิ่ง → ตั้ง 0°
// warm_ms: อ่านทิ้งต่อเนื่องอย่างน้อยกี่ ms (ถ้าชิปรีเซ็ตระหว่างนั้นจะนับใหม่ สูงสุด 2.5 วินาที)
void gyroBegin(uint32_t warm_ms = 500) {
  gyro_imu_ready = true;
  myIMU.wasReset();  // ล้างแฟล็ก reset จาก begin() เอง แล้วเปิด report ด้านล่าง
  if (!gyroEnableReports()) {
    delay(50);
    gyroEnableReports();
  }
  uint32_t tStart = millis();
  uint32_t t0 = tStart;
  while (millis() - t0 < warm_ms && millis() - tStart < 2500) {
    if (!gyro_imu_ready) break;           // I2C timeout ระหว่าง warm-up → IMU หลุด
    if (gyroCheckReset()) t0 = millis();  // ชิปรีเซ็ตตัวเอง → เริ่ม warm-up ใหม่
    if (imuPoll() && isYawEvent()) updateYawFromEvent();
    delay(1);
  }
  resetYaw();
}

float gyroZ() {
  return angleRead();
}

void SetRobotAngle() {
  current_degree = gyroZ();
}

void setAngleOffset() {
  SetRobotAngle();
}

// ตั้งมุมปัจจุบัน = 0° — เรียกบรรทัดเดียวต้น Mission() ครบทุกขั้นตอน:
// หยุดมอเตอร์ → รอ 200ms ให้หุ่นนิ่งจากแรงกดนิ้ว → ล้างคิว BNO08x → อ่านมุมสดจนนิ่ง → ตั้ง 0°
void resetAngles() {
  MotorStop();
  delay(200);
  resetYaw();
  current_degree = lastYaw;  // = 0 หลัง reset (ไม่อ่านแพ็กเก็ตเพิ่ม กันได้ค่าค้าง)
  previous_errorG = 0;
  previous_errorGB = 0;
  resetRunGFreshState();
}

// ======================================================================
// [3] ตั้งค่าไจโร (PID / ตัวคูณระยะทาง / เบรก)
// ======================================================================

void SetGyroTurn(float kp, float kd, int maxSpd, int minSpd, float smallAngle, float stopThr) {
  gyro_Kp_Turn = kp;
  gyro_Kd_Turn = kd;
  gyro_MaxSpd_Turn = maxSpd;
  gyro_MinSpd_Turn = minSpd;
  gyro_SmallAngle_Turn = smallAngle;
  gyro_StopThr_Turn = stopThr;
}

void SetGyroTurnB(float kp, float kd, int maxSpd, int minSpd, float smallAngle, float stopThr) {
  gyro_Kp_TurnB = kp;
  gyro_Kd_TurnB = kd;
  gyro_MaxSpd_TurnB = maxSpd;
  gyro_MinSpd_TurnB = minSpd;
  gyro_SmallAngle_TurnB = smallAngle;
  gyro_StopThr_TurnB = stopThr;
}

void SetGyroSpin(float kp, float kd, int maxSpd, int minSpd, float smallAngle, float stopThr) {
  gyro_Kp_Spin = kp;
  gyro_Kd_Spin = kd;
  gyro_MaxSpd_Spin = maxSpd;
  gyro_MinSpd_Spin = minSpd;
  gyro_SmallAngle_Spin = smallAngle;
  gyro_StopThr_Spin = stopThr;
}

// rotatedegree / rotatedir (วิ่งโค้งกำหนดล้อเอง) — ไม่มี maxSpd เพราะใช้ SpeedL / SpeedR ที่สั่ง
void SetGyroRotate(float kp, float kd, int minSpd, float smallAngle, float stopThr) {
  gyro_Kp_Rotate = kp;
  gyro_Kd_Rotate = kd;
  gyro_MinSpd_Rotate = minSpd;
  gyro_SmallAngle_Rotate = smallAngle;
  gyro_StopThr_Rotate = stopThr;
}

// ---------- ตัวคูณแปลงความเร็ว → ระยะทาง (ffcmg / bbcmg / ffcmgs / bbcmgs) ----------

// ffcmg: fw_normal = ระยะ >= 30 cm (มีเร่ง/ชะลอ), fw = ระยะ < 30 cm (ไม่มี ramp)
void set_speed_scale_ff_g(float fw_normal, float fw) {
  speed_scale_ff_g_normal = fw_normal;
  speed_scale_ff_g = fw;
}

// bbcmg: bb_normal = ระยะ >= 30 cm (มีเร่ง/ชะลอ), bb = ระยะ < 30 cm (ไม่มี ramp)
void set_speed_scale_bb_g(float bb_normal, float bb) {
  speed_scale_bb_g_normal = bb_normal;
  speed_scale_bb_g = bb;
}

// ffcmgs / bbcmgs
void set_speed_scale_ff_bb_gs(float ff, float bb) {
  speed_scale_ff_gs = ff;
  speed_scale_bb_gs = bb;
}

// ---------- เบรกหลังหมุน / เลี้ยวด้วยไจโร ----------
// *_setg(ms) = เวลาล็อกมุม (SetG) หลังถึงมุม | (spd, ms) = เบรกสวนทิศสั้น ๆ ทันทีที่ถึงมุม

void set_brake_spin_gyro_setg(int brake) {
  brake_spin_g = brake;
}

void set_brake_spin_gyro(int spd, int t) {
  gyro_BounceSpd_spin = spd;
  gyro_BounceMs_spin = t;
}

void set_brake_turn_gyro_setg(int brake) {
  brake_turn_g = brake;
}

void set_brake_turn_gyro(int spd, int t) {
  gyro_BounceSpd_turn = spd;
  gyro_BounceMs_turn = t;
}

void set_brake_turn_b_gyro_setg(int brake) {
  brake_turn_b_g = brake;
}

void set_brake_turn_b_gyro(int spd, int t) {
  gyro_BounceSpd_turn_b = spd;
  gyro_BounceMs_turn_b = t;
}

void set_brake_rotate_gyro_setg(int brake) {
  brake_rotate_g = brake;
}

void set_brake_rotate_gyro(int spd, int t) {
  gyro_BounceSpd_rotate = spd;
  gyro_BounceMs_rotate = t;
}

// ---------- ตัวช่วย PD ของลูปหมุน / เลี้ยว ----------
// ลูป while ของ RP2350 วิ่งเร็วกว่า BNO08x ส่งมุมมาก รอบที่ยังไม่มีมุมใหม่ error จะเท่าเดิม
// ถ้าคิด D ทุกรอบ (error - previous_error) จะเป็น 0 เกือบตลอด → Kd ไม่ช่วยเบรก
// จึงคำนวณ error / D เฉพาะตอนได้แพ็กเก็ตมุมใหม่จริง และ D = "องศาที่เปลี่ยนต่อ 10ms"
// (ไม่ขึ้นกับรอบส่งของ IMU — เปลี่ยน gyro_report_interval_ms แล้วไม่ต้องจูน Kd ใหม่)

float gyro_StopLead_ms = 10.0f;  // หยุดก่อนถึงมุมล่วงหน้า (ms) ชดเชยแรงเฉื่อย + หน่วงของ IMU | 0 = ปิด
int gyro_PreTurnStop_ms = 40;    // ffcmg(...,'r') ฯลฯ: หยุดนิ่งก่อนเริ่มหมุนที่ทางแยก (ms) | 0 = ปิด
int gyro_PreTurnBrakeSpd = 0;    // เบรกถอยสั้นๆ ก่อนหยุด (ความเร็ว) | 0 = ไม่เบรกถอย
int gyro_PreTurnBrakeMs = 0;

// lead_ms: หยุดล่วงหน้ากี่ ms ก่อนถึงมุม (ตามความเร็วหมุนจริง) — ใช้ได้ทั้งหมุนและเลี้ยว
void set_gyro_stop_lead(float lead_ms) {
  gyro_StopLead_ms = lead_ms;
}

// ก่อนหมุน/เลี้ยวที่ทางแยกของ TrackSelectG / TrackSelectGB: (เบรกถอย spd, ms) แล้วหยุดนิ่ง stop_ms
void set_pre_turn_stop(int stop_ms, int brake_spd = 0, int brake_ms = 0) {
  gyro_PreTurnStop_ms = stop_ms;
  gyro_PreTurnBrakeSpd = brake_spd;
  gyro_PreTurnBrakeMs = brake_ms;
}

// dir: 1 = เพิ่งเดินหน้า (เบรกด้วยการถอย), -1 = เพิ่งถอยหลัง (เบรกด้วยการเดินหน้า)
void gyroPreTurnStop(int dir) {
  if (gyro_PreTurnBrakeSpd > 0 && gyro_PreTurnBrakeMs > 0) {
    Motor(-dir * gyro_PreTurnBrakeSpd, -dir * gyro_PreTurnBrakeSpd);
    delay(gyro_PreTurnBrakeMs);
  }
  MotorStop();
  if (gyro_PreTurnStop_ms > 0) delay(gyro_PreTurnStop_ms);
}

struct GyroStep {
  uint32_t seen;
  bool first;
  float prev;
};

void gyroStepBegin(GyroStep &s) {
  s.seen = yawSampleCount - 1;  // รอบแรกใช้ lastYaw ที่เพิ่งอ่านสดได้ทันที
  s.first = true;
  s.prev = 0;
}

// คืน false = ยังไม่มีมุมใหม่ (ให้คงคำสั่งมอเตอร์เดิมไว้)
// คืน true  = error = target - มุมปัจจุบัน (wrap ±180), d = error ที่เปลี่ยนต่อ 10ms
bool gyroStep(GyroStep &s, float target, float &error, float &d) {
  gyroZ();
  if (yawSampleCount == s.seen) return false;
  s.seen = yawSampleCount;
  error = wrap180(target - lastYaw);
  d = s.first ? 0.0f : (error - s.prev) * (10.0f / (float)gyro_report_interval_ms);
  s.first = false;
  s.prev = error;
  return true;
}

// ถึงมุมแล้ว หรือกำลังพุ่งเข้าหาเป้าเร็วพอที่จะถึงภายใน gyro_StopLead_ms → หยุดเลย
bool gyroStopNow(float error, float d, float stop_threshold, float small_angle_threshold) {
  if (fabsf(error) <= stop_threshold) return true;
  if (gyro_StopLead_ms <= 0.0f || fabsf(error) >= small_angle_threshold) return false;
  float e_pred = error + d * (gyro_StopLead_ms / 10.0f);
  if (error > 0 && d < 0 && e_pred <= stop_threshold) return true;
  if (error < 0 && d > 0 && e_pred >= -stop_threshold) return true;
  return false;
}

// ---------- Safety timeout ของลูปไจโร (กันหุ่นหมุนค้างไม่จบ) ----------
// stale: ไม่มีค่ามุมใหม่จาก BNO08x นานเกินนี้ (ms) → IMU หลุด / ไม่ได้ต่อ → หยุดมอเตอร์ออกจากลูป
// total: หมุน / เลี้ยวนานเกินนี้ (ms) → ติดขอบสนาม / ล้อฟรี → หยุดมอเตอร์ออกจากลูป
// ตอนทำงานปกติไม่ถึงค่าเหล่านี้ จึงไม่กระทบโค้ดเดิม | 0 = ปิด
uint32_t gyro_stale_timeout_ms = 200;
uint32_t gyro_turn_timeout_ms = 6000;
uint32_t gyro_timeout_count = 0;  // จำนวนครั้งที่ timeout ทำงาน (ไว้ debug)

void set_gyro_timeout(uint32_t stale_ms, uint32_t total_ms) {
  gyro_stale_timeout_ms = stale_ms;
  gyro_turn_timeout_ms = total_ms;
}

struct GyroGuard {
  uint32_t t_start;
  uint32_t t_last;
  uint32_t seen;
};

void gyroGuardBegin(GyroGuard &g) {
  g.t_start = g.t_last = millis();
  g.seen = yawSampleCount;
}

// เรียกทุกรอบหลังอ่านไจโร | คืน true = หมดเวลา (หยุดมอเตอร์ + บี๊บให้แล้ว) ให้ออกจากลูป
// useTotal = false: เช็กเฉพาะ IMU หยุดส่งค่า (ใช้กับลูปวิ่งตามเส้นที่อาจนานได้จริง)
bool gyroGuardExpired(GyroGuard &g, bool useTotal) {
  uint32_t now = millis();
  if (yawSampleCount != g.seen) {
    g.seen = yawSampleCount;
    g.t_last = now;
  }
  bool stale = gyro_stale_timeout_ms > 0 && now - g.t_last > gyro_stale_timeout_ms;
  bool total = useTotal && gyro_turn_timeout_ms > 0 && now - g.t_start > gyro_turn_timeout_ms;
  if (!stale && !total) return false;
  MotorStop();
  gyro_timeout_count++;
  Serial.println(stale ? "Gyro timeout: no IMU data -> motor stop" : "Gyro timeout: turn too long -> motor stop");
  Beep(100);
  return true;
}

// ======================================================================
// [4] ล็อกมุม (HoldAngle / SetG)
// ======================================================================

float kpHold = 2.5f;
float kdHold = 1.5f;
float kpFHold = 2.5f;
float kdFHold = 1.5f;
float kpBHold = 2.5f;
float kdBHold = 1.2f;
float holdAngle = 0;
float prevErrHold = 0;
float prevErrHoldB = 0;
float prevErrHoldF = 0;
float prevErrHoldBF = 0;


float hold_deadband_deg = 0.5f;  // |error| <= ค่านี้ = ถือว่าตรงแล้ว สั่ง Motor(0, 0) กันมอเตอร์ครางสั่น

void set_hold_deadband(float deg) {
  hold_deadband_deg = deg;
}

uint32_t holdSampleSeen = 0;
uint32_t holdSampleSeenB = 0;
uint32_t holdSampleSeenF = 0;
uint32_t holdSampleSeenBF = 0;

// 1 สเต็ปล็อกมุม: คำนวณใหม่เฉพาะตอนได้มุมใหม่จาก IMU (D ไม่เป็น 0 ทุกรอบ)
// error = target - มุมปัจจุบัน, D = error ที่เปลี่ยนต่อ 10ms
// คืน false = ยังไม่มีมุมใหม่ (คงคำสั่งมอเตอร์เดิม) | power + = หมุนขวา → Motor(power, -power)
static bool holdStep(float target, float kp, float kd, float &prevErr, uint32_t &seen, int &power) {
  gyroZ();
  if (yawSampleCount == seen) return false;
  seen = yawSampleCount;
  float error = wrap180(target - lastYaw);
  float d = (error - prevErr) * (10.0f / (float)gyro_report_interval_ms);
  prevErr = error;
  if (fabsf(error) <= hold_deadband_deg) {
    power = 0;
  } else {
    power = constrain((int)(error * kp + d * kd), -50, 50);
  }
  return true;
}

// เริ่มล็อกมุมใหม่: D เริ่มจาก error ปัจจุบัน (ไม่กระชาก) และรอบแรกคำนวณทันที
static void holdBegin(float target, float &prevErr, uint32_t &seen) {
  prevErr = wrap180(target - lastYaw);
  seen = yawSampleCount - 1;
}

// ล็อกมุม current_degree (หมุนอยู่กับที่)
void HoldAngle() {
  int p;
  if (holdStep(current_degree, kpHold, kdHold, prevErrHold, holdSampleSeen, p)) Motor(p, -p);
}

// ล็อกมุม current_degree ด้วยค่า kpBHold / kdBHold (ใช้หลังคำสั่งถอยหลัง)
// แก้บั๊กเดิม: error กับทิศแรงบิดกลับด้านกัน → หุ่นหมุนหนีมุมเป้า
void HoldAngleB() {
  int p;
  if (holdStep(current_degree, kpBHold, kdBHold, prevErrHoldB, holdSampleSeenB, p)) Motor(p, -p);
}

// จำมุมที่หัวหุ่นหันอยู่จริง "ตอนนี้" (ล้างคิว IMU + อ่านค่าสด) แล้วหยุดมอเตอร์
void SetHoldAngle() {
  holdAngle = angleReadFresh();
  MotorStop();
  holdBegin(holdAngle, prevErrHoldF, holdSampleSeenF);
  holdBegin(holdAngle, prevErrHoldBF, holdSampleSeenBF);
}

// ล็อกมุม holdAngle (ค่า kpFHold / kdFHold)
void HoldAngleF() {
  int p;
  if (holdStep(holdAngle, kpFHold, kdFHold, prevErrHoldF, holdSampleSeenF, p)) Motor(p, -p);
}

// ล็อกมุม holdAngle (ค่า kpBHold / kdBHold)
void HoldAngleBF() {
  int p;
  if (holdStep(holdAngle, kpBHold, kdBHold, prevErrHoldBF, holdSampleSeenBF, p)) Motor(p, -p);
}

// จบการล็อกมุม: หยุดมอเตอร์ แล้วให้ RunG / ffcmg ถัดไปวิ่งตรงทิศ holdAngle ต่อ
static void holdFinish() {
  MotorStop();
  current_degree = holdAngle;
  resetGyroRunError();
}

// ---------- SetFG / SetBG: ล็อกมุมนิ่งอยู่กับที่ (ms) — แบบเดิม ----------

void SetFG(int totalTime) {
  BZon();
  SetHoldAngle();
  unsigned long endTime = millis() + totalTime;
  while (millis() <= endTime) {
    HoldAngleF();
  }
  holdFinish();
  BZoff();
}

void setfg(int totalTime) {
  SetFG(totalTime);
}

void SetBG(int totalTime) {
  BZon();
  SetHoldAngle();
  unsigned long endTime = millis() + totalTime;
  while (millis() <= endTime) {
    HoldAngleBF();
  }
  holdFinish();
  BZoff();
}

void setbg(int totalTime) {
  SetBG(totalTime);
}

// ---------- SetFG / SetBG แบบเบรก (Gyro Anti-Skid Brake) ----------
// SetFG(brakeSpd, brakeMs [, holdMs]) — หลังวิ่ง "เดินหน้า" มาเร็ว ๆ
//   จังหวะ 1 (brakeMs): ถอยล้อต้านแรงเฉื่อย brakeSpd + ผสม PD ไจโรกันหัวปัด
//   จังหวะ 2 (holdMs) : ล็อกมุมนิ่งอยู่กับที่จนหยุดสนิท แล้ว MotorStop()
// SetBG(brakeSpd, brakeMs [, holdMs]) — หลังวิ่ง "ถอยหลัง" (ดันล้อเดินหน้าต้านแทน)

static void gyroBrakeStop(int brakeSpd, int brakeMs, int holdMs, bool backward) {
  BZon();
  int base = backward ? abs(brakeSpd) : -abs(brakeSpd);
  Motor(base, base);           // เริ่มเบรกทันที ระหว่างอ่านมุมสด
  holdAngle = angleReadFresh();  // มุมที่หัวหุ่นหันอยู่จริงตอนสั่ง
  float kp = backward ? kpBHold : kpFHold;
  float kd = backward ? kdBHold : kdFHold;
  float prevErr;
  uint32_t seen;
  holdBegin(holdAngle, prevErr, seen);
  int pd = 0;
  unsigned long endTime = millis() + brakeMs;
  while (millis() <= endTime) {
    int p;
    if (holdStep(holdAngle, kp, kd, prevErr, seen, p)) pd = p;
    Motor(constrain(base + pd, -100, 100), constrain(base - pd, -100, 100));
  }
  MotorStop();
  if (backward) holdBegin(holdAngle, prevErrHoldBF, holdSampleSeenBF);
  else holdBegin(holdAngle, prevErrHoldF, holdSampleSeenF);
  endTime = millis() + holdMs;
  while (millis() <= endTime) {
    if (backward) HoldAngleBF();
    else HoldAngleF();
  }
  holdFinish();
  BZoff();
}

void SetFG(int brakeSpd, int brakeMs, int holdMs = 60) {
  gyroBrakeStop(brakeSpd, brakeMs, holdMs, false);
}

void setfg(int brakeSpd, int brakeMs, int holdMs = 60) {
  SetFG(brakeSpd, brakeMs, holdMs);
}

void SetBG(int brakeSpd, int brakeMs, int holdMs = 60) {
  gyroBrakeStop(brakeSpd, brakeMs, holdMs, true);
}

void setbg(int brakeSpd, int brakeMs, int holdMs = 60) {
  SetBG(brakeSpd, brakeMs, holdMs);
}

// ---------- SetG / SetGB: ล็อกมุม current_degree ตามเวลา ----------

void SetG(int totalTime) {
  BZon();
  holdBegin(current_degree, prevErrHold, holdSampleSeen);
  unsigned long endTime = millis() + totalTime;
  while (millis() <= endTime) {
    HoldAngle();
  }
  MotorStop();  // ไม่ปล่อยกำลังค้างจากรอบสุดท้ายของการล็อกมุม
  BZoff();
}

void SetGB(int totalTime) {
  BZon();
  holdBegin(current_degree, prevErrHoldB, holdSampleSeenB);
  unsigned long endTime = millis() + totalTime;
  while (millis() <= endTime) {
    HoldAngleB();
  }
  MotorStop();
  BZoff();
}

// ======================================================================
// [5] หมุน / เลี้ยวตามองศา (spindegree / turndegree)
// ======================================================================

// ---------- แกนกลางของการหมุน / เลี้ยว (ใช้ร่วมกัน: องศาสัมพัทธ์ *degree และทิศจริง *dir ในหมวด [12]) ----------
// เดิม spindegree / spindir (และคู่ turn / turnb / _none) มีลูปเหมือนกันทุกบรรทัดสองชุด → รวมเหลือชุดเดียว
// เรียกได้เฉพาะหลังอ่านมุมสดแล้ว (angleReadFresh() / trackTarget()) เพราะรอบแรกใช้ lastYaw ทันที
// ไม่อ่านมุมสดซ้ำเอง → trackSpin / alignHeading ไม่ต้องล้างคิว + รอแพ็กเก็ต IMU สองรอบก่อนหมุน

// หมุนตัว 2 ล้อเข้าหามุม target_degree (ทิศจริง -180..180)
static void spinToTarget(int Speed, float target_degree) {
  int min_speed = gyro_MinSpd_Spin;
  int max_speed = Speed;
  float kp = gyro_Kp_Spin;
  float kd = gyro_Kd_Spin;
  float small_angle_threshold = gyro_SmallAngle_Spin;
  float stop_threshold = gyro_StopThr_Spin;
  target_degree = wrap180(target_degree);
  current_degree = target_degree;
  int spin_dir = 0;  // 1 = หมุนขวา, -1 = หมุนซ้าย, 0 = ไม่ได้หมุน
  GyroStep gs;
  gyroStepBegin(gs);
  GyroGuard gg;
  gyroGuardBegin(gg);
  bool timedOut = false;
  while (1) {
    float error, d;
    bool fresh = gyroStep(gs, target_degree, error, d);
    if (gyroGuardExpired(gg, true)) {  // IMU หลุด / หมุนนานเกิน → หยุดมอเตอร์แล้ว
      timedOut = true;
      break;
    }
    if (!fresh) continue;  // ยังไม่มีมุมใหม่ → คงคำสั่งมอเตอร์เดิม
    if (gyroStopNow(error, d, stop_threshold, small_angle_threshold)) break;
    int pd_value = (kp * error) + (kd * d);

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
  }
  if (timedOut) return;  // หยุดมอเตอร์ไปแล้ว ไม่เบรกสวน / ไม่ล็อกมุมต่อ
  // เบรกสวนทิศที่หมุนจริง
  if (spin_dir != 0) {
    Motor(-spin_dir * gyro_BounceSpd_spin, spin_dir * gyro_BounceSpd_spin);
    delay(gyro_BounceMs_spin);
  }
  MotorStop();
  SetG(brake_spin_g);
}

// เลี้ยวล้อเดียวเดินหน้าเข้าหามุม target_degree
static void turnToTarget(int Speed, float target_degree) {
  int min_speed = gyro_MinSpd_Turn;
  int max_speed = Speed;
  float kp = gyro_Kp_Turn;
  float kd = gyro_Kd_Turn;
  float small_angle_threshold = gyro_SmallAngle_Turn;
  float stop_threshold = gyro_StopThr_Turn;
  target_degree = wrap180(target_degree);
  current_degree = target_degree;
  int turn_dir = 0;  // 1 = เลี้ยวขวา (ล้อซ้ายเดิน), -1 = เลี้ยวซ้าย (ล้อขวาเดิน)

  GyroStep gs;
  gyroStepBegin(gs);
  GyroGuard gg;
  gyroGuardBegin(gg);
  bool timedOut = false;
  while (1) {
    float error, d;
    bool fresh = gyroStep(gs, target_degree, error, d);
    if (gyroGuardExpired(gg, true)) {  // IMU หลุด / หมุนนานเกิน → หยุดมอเตอร์แล้ว
      timedOut = true;
      break;
    }
    if (!fresh) continue;  // ยังไม่มีมุมใหม่ → คงคำสั่งมอเตอร์เดิม
    if (gyroStopNow(error, d, stop_threshold, small_angle_threshold)) break;
    int pd_value = (kp * error) + (kd * d);

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
  }
  if (timedOut) return;  // หยุดมอเตอร์ไปแล้ว ไม่เบรกสวน / ไม่ล็อกมุมต่อ
  // เบรกล้อที่เดินอยู่ สวนทิศที่เลี้ยวจริง
  if (turn_dir > 0) Motor(-gyro_BounceSpd_turn, 0);
  else if (turn_dir < 0) Motor(0, -gyro_BounceSpd_turn);
  if (turn_dir != 0) delay(gyro_BounceMs_turn);
  MotorStop();
  SetG(brake_turn_g);
}

// เลี้ยวล้อเดียวถอยหลังเข้าหามุม target_degree
static void turnBToTarget(int Speed, float target_degree) {
  int min_speed = gyro_MinSpd_TurnB;
  int max_speed = Speed;
  float kp = gyro_Kp_TurnB;
  float kd = gyro_Kd_TurnB;
  float small_angle_threshold = gyro_SmallAngle_TurnB;
  float stop_threshold = gyro_StopThr_TurnB;
  target_degree = wrap180(target_degree);
  current_degree = target_degree;
  int turn_dir = 0;  // 1 = ล้อขวาถอย, -1 = ล้อซ้ายถอย, 0 = ไม่ได้เลี้ยว

  GyroStep gs;
  gyroStepBegin(gs);
  GyroGuard gg;
  gyroGuardBegin(gg);
  bool timedOut = false;
  while (1) {
    float error, d;
    bool fresh = gyroStep(gs, target_degree, error, d);
    if (gyroGuardExpired(gg, true)) {  // IMU หลุด / หมุนนานเกิน → หยุดมอเตอร์แล้ว
      timedOut = true;
      break;
    }
    if (!fresh) continue;  // ยังไม่มีมุมใหม่ → คงคำสั่งมอเตอร์เดิม
    if (gyroStopNow(error, d, stop_threshold, small_angle_threshold)) break;
    int pd_value = (kp * error) + (kd * d);

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
  }
  if (timedOut) return;  // หยุดมอเตอร์ไปแล้ว ไม่เบรกสวน / ไม่ล็อกมุมต่อ
  // เบรกล้อที่ถอยอยู่ ด้วยการเดินหน้าสวนสั้น ๆ
  if (turn_dir > 0) Motor(0, gyro_BounceSpd_turn_b);
  else if (turn_dir < 0) Motor(gyro_BounceSpd_turn_b, 0);
  if (turn_dir != 0) delay(gyro_BounceMs_turn_b);
  MotorStop();
  SetG(brake_turn_b_g);
}

// เลี้ยวล้อเดียวเดินหน้า ไม่เบรก ไหลต่อเข้าคำสั่งถัดไป
static void turnToTargetNone(int Speed, float target_degree) {
  float stop_threshold = gyro_StopThr_Turn;
  float previous_error = 0;
  target_degree = wrap180(target_degree);
  current_degree = target_degree;
  GyroGuard gg;
  gyroGuardBegin(gg);
  while (1) {
    float current_angle = gyroZ();
    if (gyroGuardExpired(gg, true)) break;  // IMU หลุด / หมุนนานเกิน → หยุดมอเตอร์แล้ว
    float error = wrap180(target_degree - current_angle);

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

// เลี้ยวล้อเดียวถอยหลัง ไม่เบรก ไหลต่อเข้าคำสั่งถัดไป
static void turnBToTargetNone(int Speed, float target_degree) {
  float stop_threshold = gyro_StopThr_TurnB;
  float previous_error = 0;
  target_degree = wrap180(target_degree);
  current_degree = target_degree;
  GyroGuard gg;
  gyroGuardBegin(gg);
  while (1) {
    float current_angle = gyroZ();
    if (gyroGuardExpired(gg, true)) break;  // IMU หลุด / หมุนนานเกิน → หยุดมอเตอร์แล้ว
    float error = wrap180(target_degree - current_angle);

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

void spindegree(int Speed, int relative_degree) {
  spinToTarget(Speed, angleReadFresh() + relative_degree);
}

void turndegree(int Speed, int relative_degree) {
  turnToTarget(Speed, angleReadFresh() + relative_degree);
}

void turndegreeb(int Speed, int relative_degree) {
  turnBToTarget(Speed, angleReadFresh() + relative_degree);
}

// ---------- rotatedegree: วิ่งโค้ง ล้อซ้าย/ขวาความเร็วต่างกัน (Ratio-Preserving Arc) ----------
// ทิศหมุนกำหนดจากความเร็วล้อเสมอ (ใช้ได้ทั้งโค้งเดินหน้า / ถอยหลัง / หมุนเยื้องศูนย์):
//   SpeedL > SpeedR → มุมเพิ่ม (ขวา) | SpeedL < SpeedR → มุมลด (ซ้าย) | เท่ากัน = ไม่เลี้ยว → ข้าม
// ช่วงไกล (มุมที่เหลือ >= gyro_SmallAngle_Rotate): สั่ง (SpeedL, SpeedR) ตรงตามผู้ใช้ 100% → รัศมีโค้งตรง
// ช่วงชะลอ: สั่ง (SpeedL × scale, SpeedR × scale) สัดส่วนล้อเท่าเดิม → ช้าลงบนเส้นโค้งเดิม
//   scale = kp × (มุมที่เหลือ + kd × อัตราเข้าหาเป้า) / gyro_SmallAngle_Rotate (คิดเฉพาะตอนมีมุมใหม่จาก IMU)
//   ล้อที่เร็วกว่าไม่ต่ำกว่า gyro_MinSpd_Rotate
// หยุดเมื่อ |error| <= gyro_StopThr_Rotate, ข้ามเป้า (error เปลี่ยนเครื่องหมายตอน |error| < 90)
//   หรือพุ่งเข้าเป้าเร็วจนจะถึงภายใน gyro_StopLead_ms
// brake = true: เบรกสวนทิศล้อตามสัดส่วน → MotorStop() → ล็อกมุม SetG(brake_rotate_g)
// brake = false: ไม่ชะลอ ไม่เบรก มอเตอร์ยังวิ่ง (SpeedL, SpeedR) ต่อ → ต่อคำสั่งถัดไปลื่น
// ต้องเรียก angleReadFresh() มาก่อน (lastYaw สด) | คืน false = ไม่ได้วิ่ง หรือ timeout (หยุดมอเตอร์แล้ว)

static bool rotateArc(int SpeedL, int SpeedR, float target_degree, bool brake, float kp, float kd) {
  SpeedL = constrain(SpeedL, -100, 100);
  SpeedR = constrain(SpeedR, -100, 100);
  if (SpeedL == SpeedR) {
    Serial.println("rotate: SpeedL == SpeedR -> no turning, skipped");
    Beep(100);
    return false;
  }
  const int rot_sign = (SpeedL > SpeedR) ? 1 : -1;  // ทิศที่ล้อพาหุ่นหมุนจริง
  const int maxAbs = max(abs(SpeedL), abs(SpeedR));
  const float minScale = (maxAbs > gyro_MinSpd_Rotate) ? (float)gyro_MinSpd_Rotate / (float)maxAbs : 1.0f;
  const float small_angle = (gyro_SmallAngle_Rotate > 1.0f) ? gyro_SmallAngle_Rotate : 1.0f;
  const float stop_threshold = gyro_StopThr_Rotate;

  target_degree = wrap180(target_degree);
  current_degree = target_degree;
  float prev_error = wrap180(target_degree - lastYaw);  // เริ่มจาก error จริง (ไม่ใช่ 0) กัน crossed / D กระชาก
  int cmdL = 0, cmdR = 0;

  GyroStep gs;
  gyroStepBegin(gs);  // D เฉพาะตอนมีแพ็กเก็ตใหม่ รอบแรก D = 0
  GyroGuard gg;
  gyroGuardBegin(gg);
  while (1) {
    float error, d;
    bool fresh = gyroStep(gs, target_degree, error, d);
    if (gyroGuardExpired(gg, true)) return false;  // IMU หลุด / หมุนนานเกิน → หยุดมอเตอร์แล้ว
    if (!fresh) continue;                          // ยังไม่มีมุมใหม่ → คงคำสั่งมอเตอร์เดิม

    bool crossed = fabsf(error) < 90.0f && ((error > 0.0f && prev_error < 0.0f) || (error < 0.0f && prev_error > 0.0f));
    prev_error = error;
    if (crossed || gyroStopNow(error, d, stop_threshold, small_angle)) break;

    // rem = มุมที่เหลือตามทิศที่ล้อพาหมุน (+ = เป้ายังอยู่ข้างหน้า, - = อีกฝั่ง ต้องหมุนอีกไกล)
    float rem = error * rot_sign;
    float scale = 1.0f;
    if (brake && rem > 0.0f && rem < small_angle) {
      float rem_pred = rem + kd * d * rot_sign;  // d × rot_sign < 0 = กำลังเข้าหาเป้า
      scale = constrain(kp * rem_pred / small_angle, minScale, 1.0f);
    }
    cmdL = (int)lroundf(SpeedL * scale);
    cmdR = (int)lroundf(SpeedR * scale);
    Motor(cmdL, cmdR);
  }

  if (!brake) {
    resetGyroRunError();  // RunG / ffcmg ถัดไปล็อกทิศ target ต่อโดย D ไม่กระชาก
    return true;
  }
  // เบรกสวนทิศที่ล้อแต่ละข้างกำลังหมุนจริง ตามสัดส่วนเดิม (ล้อเร็วกว่า = gyro_BounceSpd_rotate)
  int cmdMax = max(abs(cmdL), abs(cmdR));
  if (cmdMax > 0 && gyro_BounceSpd_rotate > 0 && gyro_BounceMs_rotate > 0) {
    int bL = -(int)lroundf((float)gyro_BounceSpd_rotate * cmdL / cmdMax);
    int bR = -(int)lroundf((float)gyro_BounceSpd_rotate * cmdR / cmdMax);
    Motor(bL, bR);
    delay(gyro_BounceMs_rotate);
  }
  MotorStop();
  SetG(brake_rotate_g);
  resetGyroRunError();
  return true;
}

// เครื่องหมายมุมตั้งอัตโนมัติจากล้อ: SpeedL > SpeedR → +|deg|, SpeedL < SpeedR → -|deg|
// จำกัดไม่เกิน 360 - 3 × stopThr (≈ 357) ไม่งั้นเป้าห่างจากมุมเริ่มไม่ถึง stopThr → หยุดทันทีไม่ได้หมุน
static float rotateRelTarget(int SpeedL, int SpeedR, int relative_degree) {
  float maxDeg = 360.0f - 3.0f * gyro_StopThr_Rotate;
  float deg = (float)abs(relative_degree);
  if (deg > maxDeg) deg = maxDeg;
  if (SpeedL < SpeedR) deg = -deg;
  return wrap180(angleReadFresh() + deg);
}

// kp / kd = ค่าชะลอเฉพาะครั้งนี้ (ความหมายดู gyro_Kp_Rotate / gyro_Kd_Rotate)
void rotatedegree(int SpeedL, int SpeedR, int relative_degree, float kp, float kd) {
  if (SpeedL == SpeedR) {
    rotateArc(SpeedL, SpeedR, 0, true, kp, kd);  // แจ้งเตือนแล้วออก
    return;
  }
  rotateArc(SpeedL, SpeedR, rotateRelTarget(SpeedL, SpeedR, relative_degree), true, kp, kd);
}

void rotatedegree(int SpeedL, int SpeedR, int relative_degree) {
  rotatedegree(SpeedL, SpeedR, relative_degree, gyro_Kp_Rotate, gyro_Kd_Rotate);
}

// ไม่ชะลอ ไม่เบรก: ถึงมุมแล้วมอเตอร์ยังวิ่ง (SpeedL, SpeedR) ต่อ → ต่อโค้งตัว S / วิ่งตรงต่อลื่น
void rotatedegree_none(int SpeedL, int SpeedR, int relative_degree) {
  if (SpeedL == SpeedR) {
    rotateArc(SpeedL, SpeedR, 0, false, gyro_Kp_Rotate, gyro_Kd_Rotate);
    return;
  }
  rotateArc(SpeedL, SpeedR, rotateRelTarget(SpeedL, SpeedR, relative_degree), false, gyro_Kp_Rotate, gyro_Kd_Rotate);
}

void turndegree_none(int Speed, int relative_degree) {
  turnToTargetNone(Speed, angleReadFresh() + relative_degree);
}

void turndegreeb_none(int Speed, int relative_degree) {
  turnBToTargetNone(Speed, angleReadFresh() + relative_degree);
}

void spindegree(int relative_degree) {
  spindegree(gyro_MaxSpd_Spin, relative_degree);
}

void turndegree(int relative_degree) {
  turndegree(gyro_MaxSpd_Turn, relative_degree);
}

void turndegreeb(int relative_degree) {
  turndegreeb(gyro_MaxSpd_TurnB, relative_degree);
}

void turndegree_none(int relative_degree) {
  turndegree_none(gyro_MaxSpd_Turn, relative_degree);
}

void turndegreeb_none(int relative_degree) {
  turndegreeb_none(gyro_MaxSpd_TurnB, relative_degree);
}

// ======================================================================
// [6] วิ่งตรงด้วยไจโร (RunG / RunGB)
// ======================================================================

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

int MaxSpeedG = 100;
int MinSpeedG = -5;
int ModeGyroStatus = 4;
int ModeGyroBStatus = 4;
int gyro_limit_up = 20, gyro_limit_down = 20;  // โหมด 2: -(Speed+down)..(Speed+up)
int gyro_limit4_up = 20;                       // โหมด 4: 0..(Speed+up)

void ModeSpdGyro(int moD, int maX, int miN) {
  ModeGyroStatus = moD;
  ModeGyroBStatus = moD;
  MaxSpeedG = maX;
  MinSpeedG = miN;
}

void ModeSpdGyro(int moDF, int moDB, int maX, int miN) {
  ModeGyroStatus = moDF;
  ModeGyroBStatus = moDB;
  MaxSpeedG = maX;
  MinSpeedG = miN;
}

void set_gyro_mode2_limit(int up, int down) {
  gyro_limit_up = up;
  gyro_limit_down = down;
}

void set_gyro_mode4_limit(int up) {
  gyro_limit4_up = up;
}

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

  int pd_value;
  if (gyro_run_dterm_fresh) {
    if (yawSampleCount != runGSeen) {  // มีมุมใหม่ → คิด PD ใหม่ แล้วคงไว้
      runGSeen = yawSampleCount;
      float d = (error - previous_errorG) * (10.0f / (float)gyro_report_interval_ms);
      runGPdHeld = (error * kpG) + (d * kdG);
      previous_errorG = error;
    }
    pd_value = (int)runGPdHeld;
  } else {
    float derivative = error - previous_errorG;
    pd_value = (int)((error * kpG) + (derivative * kdG));
    previous_errorG = error;
  }
  float leftPow = SpeedL + pd_value;
  float rightPow = SpeedR - pd_value;

  ClampGyroPower(leftPow, rightPow, SpeedL, SpeedR, ModeGyroStatus);

  Motor(leftPow, rightPow);
}

void RunGB(int SpeedL, int SpeedR) {
  float error = gyroZ() - current_degree;
  if (error > 180.0f) error -= 360.0f;
  else if (error < -180.0f) error += 360.0f;
  int pd_value;
  if (gyro_run_dterm_fresh) {
    if (yawSampleCount != runGBSeen) {  // มีมุมใหม่ → คิด PD ใหม่ แล้วคงไว้
      runGBSeen = yawSampleCount;
      float d = (error - previous_errorGB) * (10.0f / (float)gyro_report_interval_ms);
      runGBPdHeld = (error * kpGB) + (d * kdGB);
      previous_errorGB = error;
    }
    pd_value = (int)runGBPdHeld;
  } else {
    float derivative = error - previous_errorGB;
    pd_value = (int)((error * kpGB) + (derivative * kdGB));
    previous_errorGB = error;
  }
  float leftPow = SpeedL + pd_value;
  float rightPow = SpeedR - pd_value;

  ClampGyroPower(leftPow, rightPow, SpeedL, SpeedR, ModeGyroBStatus);

  Motor(-leftPow, -rightPow);
}

void fftimerg(int Speed, int totalTime) {
  BaseSpeed = Speed;
  InitialSpeed();
  unsigned long endTime = millis() + totalTime;
  while (millis() <= endTime) {
    RunG(LeftBaseSpeed, RightBaseSpeed);
  }
}

// รีเซ็ต D-term ของ RunG / RunGB ให้เท่ากับ error ปัจจุบัน กันกระชากตอนออกตัว
void resetGyroRunError() {
  float error = current_degree - lastYaw;
  if (error > 180.0f) error -= 360.0f;
  else if (error < -180.0f) error += 360.0f;
  previous_errorG = error;
  previous_errorGB = -error;
  resetRunGFreshState();
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
// [7] วิ่งตรงตามระยะทาง (ffcmg / bbcmg)
// ======================================================================
// หมายเหตุ: คำนวณระยะจาก ความเร็ว × เวลา × speed_scale_*

void ffcmgs(int Speed, float distance) {
  BaseSpeed = Speed;
  InitialSpeed();
  int target_speed = min(LeftBaseSpeed, RightBaseSpeed);
  // ระยะ / ความเร็ว / ตัวคูณ <= 0 (หรือ NaN) → ระยะสะสมไม่ถึงเป้า ลูปจะไม่จบ จึงหยุดก่อน
  if (!(distance > 0) || target_speed <= 0 || !(speed_scale_ff_gs > 0)) {
    Serial.println("ffcmgs: distance, speed and speed_scale must be > 0");
    MotorStop();
    return;
  }
  float traveled_distance = 0;
  unsigned long last_time = millis();

  while (1) {
    RunG(LeftBaseSpeed, RightBaseSpeed);

    if (distance > 0) {
      unsigned long current_time = millis();
      float delta_time = (current_time - last_time) / 1000.0f;
      traveled_distance += (target_speed * speed_scale_ff_gs) * delta_time;
      last_time = current_time;

      if (traveled_distance >= distance) break;
    }
    delayMicroseconds(80);
  }
}

void ffcmg(int Speed, float distance_cm) {
  BaseSpeed = Speed;
  InitialSpeed();

  // !(x > 0) จับทั้ง <= 0 และ NaN (NaN เดิมทำให้ลูปไม่จบ)
  if (!(distance_cm > 0) || isinf(distance_cm)) {
    Motor(0, 0);
    return;
  }

  int base_speed = min(abs(LeftBaseSpeed), abs(RightBaseSpeed));
  float traveled_distance = 0.0f;
  unsigned long last_time = millis();

  const float ACCEL_DISTANCE_CM = 20.0f;
  const float DECEL_DISTANCE_CM = 25.0f;
  const float MIN_SPEED = 10.0f;

  float speed_scale = speed_scale_ff_g_normal;
  bool enableRamp = (distance_cm >= 30.0f);

  if (!enableRamp) {
    speed_scale = speed_scale_ff_g;
  }

  // ความเร็ว / ตัวคูณ <= 0 → ระยะสะสมไม่เพิ่ม ลูปจะไม่จบ จึงหยุดก่อน
  if (base_speed <= 0 || !(speed_scale > 0)) {
    Serial.println("ffcmg: speed or speed_scale must be > 0");
    Motor(0, 0);
    return;
  }

  while (true) {
    unsigned long current_time = millis();
    float delta_time = (current_time - last_time) / 1000.0f;
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
    // คงค่าชดเชยซ้าย/ขวาจาก InitialSpeed(): ล้อที่ช้ากว่า = target_speed อีกล้อคูณสัดส่วนเดียวกัน
    float ratio = target_speed / base_speed;
    RunG((int)lroundf(LeftBaseSpeed * ratio), (int)lroundf(RightBaseSpeed * ratio));
  }
}

void bbcmgs(int Speed, float distance) {
  BaseSpeed = Speed;
  InitialSpeed();
  int target_speed = min(BackLeftBaseSpeed, BackRightBaseSpeed);
  // ระยะ / ความเร็ว / ตัวคูณ <= 0 (หรือ NaN) → ระยะสะสมไม่ถึงเป้า ลูปจะไม่จบ จึงหยุดก่อน
  if (!(distance > 0) || target_speed <= 0 || !(speed_scale_bb_gs > 0)) {
    Serial.println("bbcmgs: distance, speed and speed_scale must be > 0");
    MotorStop();
    return;
  }
  float traveled_distance = 0;
  unsigned long last_time = millis();

  while (1) {
    RunGB(BackLeftBaseSpeed, BackRightBaseSpeed);

    if (distance > 0) {
      unsigned long current_time = millis();
      float delta_time = (current_time - last_time) / 1000.0f;
      traveled_distance += (target_speed * speed_scale_bb_gs) * delta_time;
      last_time = current_time;

      if (traveled_distance >= distance) break;
    }
    delayMicroseconds(80);
  }
}

void bbcmg(int Speed, float distance_cm) {
  BaseSpeed = Speed;
  InitialSpeed();

  // !(x > 0) จับทั้ง <= 0 และ NaN (NaN เดิมทำให้ลูปไม่จบ)
  if (!(distance_cm > 0) || isinf(distance_cm)) {
    Motor(0, 0);
    return;
  }

  int base_speed = min(abs(BackLeftBaseSpeed), abs(BackRightBaseSpeed));
  float traveled_distance = 0.0f;
  unsigned long last_time = millis();

  const float ACCEL_DISTANCE_CM = 20.0f;
  const float DECEL_DISTANCE_CM = 25.0f;
  const float MIN_SPEED = 10.0f;

  float speed_scale = speed_scale_bb_g_normal;
  bool enableRamp = (distance_cm >= 30.0f);

  if (!enableRamp) {
    speed_scale = speed_scale_bb_g;
  }

  // ความเร็ว / ตัวคูณ <= 0 → ระยะสะสมไม่เพิ่ม ลูปจะไม่จบ จึงหยุดก่อน
  if (base_speed <= 0 || !(speed_scale > 0)) {
    Serial.println("bbcmg: speed or speed_scale must be > 0");
    Motor(0, 0);
    return;
  }

  while (true) {
    unsigned long current_time = millis();
    float delta_time = (current_time - last_time) / 1000.0f;
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
    // คงค่าชดเชยซ้าย/ขวาจาก InitialSpeed(): ล้อที่ช้ากว่า = target_speed อีกล้อคูณสัดส่วนเดียวกัน
    float ratio = target_speed / base_speed;
    RunGB((int)lroundf(BackLeftBaseSpeed * ratio), (int)lroundf(BackRightBaseSpeed * ratio));
  }
}

// ======================================================================
// [8] คำสั่งย่อ หมุน / เลี้ยว (spinlg, slg, tlg ...)
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
// [9] เข้ากลาง / ชิดเส้น ด้วยไจโร
// ======================================================================

// ดันผ่านเส้นด้วยไจโรก่อนเริ่มเช็กเซนเซอร์ (เวลา = center_push_ms ใน PID.h, -1 = นับ 20 รอบแบบเดิม)
static void runGPush(int SpeedL, int SpeedR, bool backward) {
  if (center_push_ms < 0) {
    for (int i = 0; i < 20; i++) {
      if (backward) RunGB(SpeedL, SpeedR);
      else RunG(SpeedL, SpeedR);
    }
    return;
  }
  unsigned long t0 = millis();
  while (millis() - t0 < (unsigned long)center_push_ms) {
    if (backward) RunGB(SpeedL, SpeedR);
    else RunG(SpeedL, SpeedR);
  }
}

void ToCenterG() {
  BZon();
  runGPush(tctL, tctR, false);
  LINE_WHILE {
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
  runGPush(bctL, bctR, true);
  LINE_WHILE {
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
  LINE_WHILE {
    RunG(tctL, tctR);
    ReadCalibrateF();
    if (F[2] > Ref || F[4] > Ref || F[6] > Ref || F[9] > Ref || F[11] > Ref || F[13] > Ref) break;
  }
}

void ToBackG() {
  LINE_WHILE {
    RunGB(bctL, bctR);
    ReadCalibrateB();
    if (B[2] > Ref || B[4] > Ref || B[6] > Ref || B[9] > Ref || B[11] > Ref || B[13] > Ref) break;
  }
}

// ======================================================================
// [10] ทางแยก (TrackSelectG / TrackSelectGB)
// ======================================================================

// ---------- ปัดมุมเข้าแกน (snap90) ก่อนเลี้ยวที่ทางแยก ----------
// TrackSelectG / TrackSelectGB จะปัดมุมจริงของหุ่นเข้าแกนที่ใกล้สุด (0, 90, -90, 180)
// แล้วเลี้ยว/หมุนไปทิศ "แกน ± 90" แบบ absolute → error มุมไม่สะสมทีละทางแยก
// ปิดได้ด้วย set_track_snap90(false) = กลับไปใช้ spindegree / turndegree แบบเดิม (สัมพัทธ์)


bool track_snap90 = true;

void set_track_snap90(bool enable) {
  track_snap90 = enable;
}

// ปัดมุมเข้าแกนที่ใกล้สุด คืนค่า 0, 90, -90 หรือ 180
float snap90(float angle) {
  float s = wrap180(roundf(wrap180(angle) / 90.0f) * 90.0f);
  if (s <= -180.0f) s = 180.0f;
  if (s == 0.0f) s = 0.0f;  // กัน -0
  return s;
}

// ทิศเป้าหมาย = แกนที่หุ่นหันอยู่ + rel
float trackTarget(float rel) {
  return wrap180(snap90((float)angleReadFresh()) + rel);
}

// trackTarget() อ่านมุมสดแล้ว → เรียกแกนกลางตรง ๆ ไม่ต้องอ่านสดซ้ำใน spindir / turndir
void trackSpin(int rel) {
  gyroPreTurnStop(1);  // หยุดแรงเฉื่อยก่อนหมุน/เลี้ยว กันล้อไถล
  if (track_snap90) spinToTarget(gyro_MaxSpd_Spin, trackTarget((float)rel));
  else spindegree(rel);
}

void trackTurn(int rel) {
  gyroPreTurnStop(1);  // หยุดแรงเฉื่อยก่อนหมุน/เลี้ยว กันล้อไถล
  if (track_snap90) turnToTarget(gyro_MaxSpd_Turn, trackTarget((float)rel));
  else turndegree(rel);
}

void trackTurnB(int rel) {
  gyroPreTurnStop(-1);  // หยุดแรงเฉื่อยก่อนหมุน/เลี้ยว กันล้อไถล
  if (track_snap90) turnBToTarget(gyro_MaxSpd_TurnB, trackTarget((float)rel));
  else turndegreeb(rel);
}

void TrackSelectG(int spd, char select) {
  if (select == 'l') {
    trackSpin(-90);
  } else if (select == 'L') {
    ToCenterG();
    trackSpin(-90);
  } else if (select == 'r') {
    trackSpin(90);
  } else if (select == 'R') {
    ToCenterG();
    trackSpin(90);
  } else if (select == 'q') {
    trackTurn(-90);
  } else if (select == 'Q') {
    ToFrontG();
    trackTurn(-90);
  } else if (select == 'e') {
    trackTurn(90);
  } else if (select == 'E') {
    ToFrontG();
    trackTurn(90);
  } else if (select == 'p') {
    BZon();
    ReadCalibrateF();
    for (int pass = 0; pass < 2; pass++) {
      LINE_WHILE {
      RunG(LeftBaseSpeed, RightBaseSpeed);
      ReadCalibrateF();
      if (F[3] < Ref && F[12] < Ref) break;
    }
    }
    BZoff();
  } else if (select == 'P') {
    ToFrontG();
    BZon();
    ReadCalibrateF();
    for (int pass = 0; pass < 2; pass++) {
      LINE_WHILE {
      RunG(LeftBaseSpeed, RightBaseSpeed);
      ReadCalibrateF();
      if (F[3] < Ref && F[12] < Ref) break;
    }
    }
    BZoff();
  } else if (select == 'c') {
    BZon();
    runGPush(LeftBaseSpeed, RightBaseSpeed, false);
    LINE_WHILE {
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
    runGPush(LeftBaseSpeed, RightBaseSpeed, false);
    LINE_WHILE {
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
    runGPush(tctL, tctR, false);
    LINE_WHILE {
      RunG(tctL, tctR);
      ReadCalibrateC();
      if (C[CCL] >= RefC || C[CCR] >= RefC) {
        break;
      }
    }
    LINE_WHILE {
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
    runGPush(tctL, tctR, false);
    LINE_WHILE {
      RunG(tctL, tctR);
      ReadCalibrateC();
      if (C[CCL] >= RefC || C[CCR] >= RefC) {
        break;
      }
    }
    LINE_WHILE {
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
  if (select == 'l') {
    trackSpin(-90);
  } else if (select == 'L') {
    BackCenterG();
    trackSpin(-90);
  } else if (select == 'r') {
    trackSpin(90);
  } else if (select == 'R') {
    BackCenterG();
    trackSpin(90);
  } else if (select == 'q') {
    trackTurnB(90);
  } else if (select == 'Q') {
    ToBackG();
    trackTurnB(90);
  } else if (select == 'e') {
    trackTurnB(-90);
  } else if (select == 'E') {
    ToBackG();
    trackTurnB(-90);
  } else if (select == 'p') {
    BZon();
    ReadCalibrateB();
    for (int pass = 0; pass < 2; pass++) {
      LINE_WHILE {
        RunGB(BackLeftBaseSpeed, BackRightBaseSpeed);
        ReadCalibrateB();
        if (B[3] < Ref && B[12] < Ref) break;
      }
    }
    BZoff();
  } else if (select == 'P') {
    ToBackG();
    BZon();
    ReadCalibrateB();
    for (int pass = 0; pass < 2; pass++) {
      LINE_WHILE {
        RunGB(BackLeftBaseSpeed, BackRightBaseSpeed);
        ReadCalibrateB();
        if (B[3] < Ref && B[12] < Ref) break;
      }
    }
    BZoff();
  } else if (select == 'c') {
    BZon();
    runGPush(BackLeftBaseSpeed, BackRightBaseSpeed, true);
    LINE_WHILE {
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
    runGPush(BackLeftBaseSpeed, BackRightBaseSpeed, true);
    LINE_WHILE {
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
    runGPush(bctL, bctR, true);
    LINE_WHILE {
      RunGB(bctL, bctR);
      ReadCalibrateC();
      if (C[CCL] >= RefC || C[CCR] >= RefC) {
        break;
      }
    }
    LINE_WHILE {
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
    runGPush(bctL, bctR, true);
    LINE_WHILE {
      RunGB(bctL, bctR);
      ReadCalibrateC();
      if (C[CCL] >= RefC || C[CCR] >= RefC) {
        break;
      }
    }
    LINE_WHILE {
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
// [11] วิ่งตรงจนเจอเส้น / ระยะ + ทางแยก
// ======================================================================

void ffbg(int Speed, char select) {
  BaseSpeed = Speed;
  InitialSpeed();
  LINE_WHILE {
    RunG(LeftBaseSpeed, RightBaseSpeed);
    ReadCalibrateF();
    if (F[2] > Ref || F[4] > Ref || F[6] > Ref || F[9] > Ref || F[11] > Ref || F[13] > Ref) break;
  }
  TrackSelectG(Speed, select);
}

void bbbg(int Speed, char select) {
  BaseSpeed = Speed;
  InitialSpeed();
  LINE_WHILE {
    RunGB(BackLeftBaseSpeed, BackRightBaseSpeed);
    ReadCalibrateB();
    if (B[2] > Ref || B[4] > Ref || B[6] > Ref || B[9] > Ref || B[11] > Ref || B[13] > Ref) break;
  }
  TrackSelectGB(Speed, select);
}

void ffdg(int Speed, char select, float distance_cm) {
  BaseSpeed = Speed;
  InitialSpeed();
  LINE_WHILE {
    RunG(LeftBaseSpeed, RightBaseSpeed);
    if (analogRead(DIST) >= distance_cm) break;
  }
  TrackSelectG(Speed, select);
}

void bbdg(int Speed, char select, float distance_cm) {
  BaseSpeed = Speed;
  InitialSpeed();
  LINE_WHILE {
    RunGB(BackLeftBaseSpeed, BackRightBaseSpeed);
    if (analogRead(DIST) <= distance_cm) break;
  }
  TrackSelectGB(Speed, select);
}

void ffdgs(int Speed, char select, float distance_cm) {
  BaseSpeed = Speed;
  InitialSpeed();
  LINE_WHILE {
    RunG(LeftBaseSpeed, RightBaseSpeed);
    if (analogRead(DIST) >= distance_cm) break;
  }
  TrackSelectG(Speed, select);
}

void bbdgs(int Speed, char select, float distance_cm) {
  BaseSpeed = Speed;
  InitialSpeed();
  LINE_WHILE {
    RunGB(BackLeftBaseSpeed, BackRightBaseSpeed);
    if (analogRead(DIST) <= distance_cm) break;
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
// [12] ควบคุมตามทิศทางจริง / เข็มทิศ (Absolute Heading)
// ======================================================================
// ทิศอ้างอิงจากตอนเรียก resetAngles() / resetAnglesDir():
//   0 = หน้าเริ่มต้น, +90 = ขวา, -90 = ซ้าย, 180 = หลัง
// ใส่ได้ทั้งช่วง -180..180 และ 0..360 (เช่น 270 = -90) ทุกคำสั่งจะ wrap180() ให้
// และหมุน / เลี้ยวเข้าหาเป้าหมายด้านที่สั้นที่สุดอัตโนมัติ
// คำสั่ง "วิ่ง" ตามทิศ (ff... / bb...): dir = ทิศที่หุ่น "เคลื่อนที่มุ่งหน้าไป" เหมือนกันทั้งเดินหน้าและถอยหลัง
//   (0 = หน้า/เหนือ, 90 = ขวา, 180 = หลัง/ใต้, 270 หรือ -90 = ซ้าย)
//   เดินหน้า: หัวหุ่นชี้ dir | ถอยหลัง: ท้ายหุ่นชี้ dir → หัวหุ่นชี้ dir + 180
//   เช่น หัวชี้ 0 อยู่ → bbcmg(60, 50, 's', 180) = ถอยตรงกลับไปทางทิศ 180 ทันที ไม่ต้องหมุนตัว
// คำสั่ง "หมุน / เลี้ยว" เข้าหาทิศ (spindir / turndir / turndirb): dir = ทิศที่ "หัวหุ่น" จะหันไป

// ---------- ค่าคงที่ทิศ ----------

constexpr int DIR_N = 0;
constexpr int DIR_NE = 45;
constexpr int DIR_E = 90;
constexpr int DIR_SE = 135;
constexpr int DIR_S = 180;
constexpr int DIR_SW = -135;
constexpr int DIR_W = -90;
constexpr int DIR_NW = -45;

// ---------- ตั้งค่าทิศอ้างอิง ----------

// รีเซ็ตไจโรเมื่อวางหุ่นเริ่มต้นหันไปทิศ start_dir_deg (ไม่ใช่ 0)
// ควรเรียกตอนหุ่นหยุดนิ่ง | คืน false = gyro ไม่ส่งข้อมูล (บี๊บเตือน ค่าเดิมไม่เปลี่ยน)
bool resetAnglesDir(float start_dir_deg) {
  float yawNow;
  if (!readStableRawYaw(yawNow)) {
    Serial.println("resetAnglesDir timeout: no rotation vector");
    Beep(500);
    return false;
  }
  float start_dir = wrap180(start_dir_deg);
  yawOffset = wrap180(yawNow - start_dir);
  gyro_rebase_pending = false;
  lastYaw = start_dir;
  yawgyro = 0.0f;
  current_degree = start_dir;
  previous_errorG = 0;
  previous_errorGB = 0;
  resetRunGFreshState();
  return true;
}

// ตั้งทิศที่จะล็อกให้ RunG / RunGB (ใช้ก่อนวิ่ง) พร้อมรีเซ็ต D-term กันกระชาก
void setHeading(float dir_deg) {
  angleReadFresh();
  current_degree = wrap180(dir_deg);
  resetGyroRunError();
}

// ทิศหัวหุ่นที่ต้องล็อกเมื่อ "ถอยหลังไปทาง" travel_dir (ท้ายหุ่นชี้ travel_dir)
float backNoseDir(float travel_dir) {
  return wrap180(travel_dir + 180.0f);
}

// ---------- ตั้งลำอัตโนมัติก่อนวิ่งตามทิศ ----------
// ถ้าหัวหุ่นต่างจากทิศที่ต้องล็อกเกิน gyro_AutoAlign_deg → หมุนอยู่กับที่ (spindir) ตั้งลำก่อน
// กันหุ่นเหวี่ยงโค้งกว้าง / PD เกินลิมิตจนล้อหมุนสวนทาง | 0 = ปิด
float gyro_AutoAlign_deg = 20.0f;

void set_auto_align(float deg) {
  gyro_AutoAlign_deg = deg;
}

void alignHeading(float nose_dir) {
  float nose = wrap180(nose_dir);
  float yaw = angleReadFresh();  // อ่านสดครั้งเดียว ใช้ทั้งตัดสินใจหมุน และเป็นมุมเริ่มของ spinToTarget
  if (gyro_AutoAlign_deg > 0.0f && fabsf(wrap180(nose - yaw)) > gyro_AutoAlign_deg) {
    spinToTarget(gyro_MaxSpd_Spin, nose);  // จบด้วย SetG ที่อ่านไจโรทุกรอบ → lastYaw สดอยู่แล้ว
  }
  // = setHeading(nose) โดยไม่ล้างคิว / รอแพ็กเก็ตใหม่ซ้ำ (เดิมอ่านสด 3 รอบต่อคำสั่ง)
  current_degree = nose;
  resetGyroRunError();
}

// เดินหน้าไปทิศ dir: ตั้งลำ (ถ้าต่างมาก) แล้วล็อกหัวหุ่น = dir
void setRunDir(float dir_deg) {
  alignHeading(dir_deg);
}

// ถอยหลังไปทิศ dir: ตั้งลำ (ถ้าต่างมาก) แล้วล็อกหัวหุ่น = dir + 180
void setRunDirB(float dir_deg) {
  alignHeading(backNoseDir(dir_deg));
}

// ---------- หมุนตัว 2 ล้อเข้าหาทิศ (ลอจิกเดียวกับ spindegree) ----------

void spindir(int Speed, float target_dir) {
  angleReadFresh();
  spinToTarget(Speed, target_dir);
}

void spindir(float target_dir) {
  spindir(gyro_MaxSpd_Spin, target_dir);
}

void sdg(float target_dir) {
  spindir(target_dir);
}

void sdg(int spd, float target_dir) {
  spindir(spd, target_dir);
}

// ---------- เลี้ยวล้อเดียวเดินหน้าเข้าหาทิศ (ลอจิกเดียวกับ turndegree) ----------

void turndir(int Speed, float target_dir) {
  angleReadFresh();
  turnToTarget(Speed, target_dir);
}

void turndir(float target_dir) {
  turndir(gyro_MaxSpd_Turn, target_dir);
}

void tdg(float target_dir) {
  turndir(target_dir);
}

void tdg(int spd, float target_dir) {
  turndir(spd, target_dir);
}

// ไม่เบรก ไหลต่อเข้าคำสั่งถัดไป (ลอจิกเดียวกับ turndegree_none)
void turndir_none(int Speed, float target_dir) {
  angleReadFresh();
  turnToTargetNone(Speed, target_dir);
}

void turndir_none(float target_dir) {
  turndir_none(gyro_MaxSpd_Turn, target_dir);
}

// ---------- เลี้ยวล้อเดียวถอยหลังเข้าหาทิศ (ลอจิกเดียวกับ turndegreeb) ----------

void turndirb(int Speed, float target_dir) {
  angleReadFresh();
  turnBToTarget(Speed, target_dir);
}

void turndirb(float target_dir) {
  turndirb(gyro_MaxSpd_TurnB, target_dir);
}

void tdbg(float target_dir) {
  turndirb(target_dir);
}

void tdbg(int spd, float target_dir) {
  turndirb(spd, target_dir);
}

// ไม่เบรก (ลอจิกเดียวกับ turndegreeb_none)
void turndirb_none(int Speed, float target_dir) {
  angleReadFresh();
  turnBToTargetNone(Speed, target_dir);
}

void turndirb_none(float target_dir) {
  turndirb_none(gyro_MaxSpd_TurnB, target_dir);
}

// ---------- วิ่งโค้งกำหนดล้อเอง เข้าหาทิศ (ลอจิกเดียวกับ rotatedegree) ----------
// rotatedir(SpeedL, SpeedR, dir): ตีโค้งตามสัดส่วนล้อจนหัวหุ่นหันทิศ dir แล้วเบรกหยุด
// ทางสั้นที่สุด: ถ้าทิศเป้าอยู่คนละฝั่งกับที่ล้อพาเลี้ยว → สลับล้อ L ↔ R ให้ (โค้งกระจกเงา
//   รัศมีเท่าเดิม เดินหน้า / ถอยหลังเหมือนเดิม) | เป้าอยู่ข้างหลังพอดี (|error| > 175°) ใช้ทิศเดิม

static bool rotateDirArc(int SpeedL, int SpeedR, float target_dir, bool brake) {
  if (SpeedL == SpeedR) return rotateArc(SpeedL, SpeedR, 0, brake, gyro_Kp_Rotate, gyro_Kd_Rotate);
  float target_degree = wrap180(target_dir);
  angleReadFresh();
  float error = wrap180(target_degree - lastYaw);
  int rot_sign = (SpeedL > SpeedR) ? 1 : -1;
  if (fabsf(error) > gyro_StopThr_Rotate && fabsf(error) <= 175.0f && error * rot_sign < 0.0f) {
    int t = SpeedL;  // สลับล้อ → หมุนกลับฝั่ง เข้าเป้าทางสั้น
    SpeedL = SpeedR;
    SpeedR = t;
  }
  return rotateArc(SpeedL, SpeedR, target_degree, brake, gyro_Kp_Rotate, gyro_Kd_Rotate);
}

void rotatedir(int SpeedL, int SpeedR, float target_dir) {
  rotateDirArc(SpeedL, SpeedR, target_dir, true);
}

// ไม่ชะลอ ไม่เบรก ถึงทิศแล้วมอเตอร์ยังวิ่งต่อ
void rotatedir_none(int SpeedL, int SpeedR, float target_dir) {
  rotateDirArc(SpeedL, SpeedR, target_dir, false);
}

// ถึงทิศแล้วทำ select ต่อ:
//   'n' / 0 = ไหลต่อ ไม่เบรก | 'f' = เบรกไจโร SetFG (โค้งถอยหลัง = SetBG) แรง / เวลาคิดจากความเร็ว
//   อื่น ๆ  = TrackSelectG (โค้งถอยหลัง = TrackSelectGB) ด้วยความเร็ว = ล้อที่เร็วกว่า
// "ถอยหลัง" = SpeedL + SpeedR < 0 | หมุนอยู่กับที่ล้วน (SpeedL + SpeedR = 0) + 'f' = เบรกแบบ rotatedir
void rotatedir(int SpeedL, int SpeedR, char select, float target_dir) {
  int sum = SpeedL + SpeedR;
  if (select == 'f' && sum == 0) {
    rotatedir(SpeedL, SpeedR, target_dir);
    return;
  }
  if (!rotateDirArc(SpeedL, SpeedR, target_dir, false)) return;  // ล้อเท่ากัน / timeout
  if (select == 'n' || select == 0) return;
  bool backward = sum < 0;
  int spd = max(abs(SpeedL), abs(SpeedR));
  if (select == 'f') {
    int bs = constrain(spd / 2, 15, 60);
    int bm = constrain(10 + spd / 3, 20, 50);
    if (backward) SetBG(bs, bm, 60);
    else SetFG(bs, bm, 60);
    // SetFG / SetBG จำมุมตอนเบรก (อาจเลยเป้ามานิด) → ให้ RunG / ffcmg ถัดไปล็อกทิศ target จริง
    current_degree = wrap180(target_dir);
    resetGyroRunError();
    return;
  }
  if (backward) TrackSelectGB(spd, select);
  else TrackSelectG(spd, select);
}

// ---------- ตัวช่วยแยก overload "ทิศ" ออกจากคำสั่งเดิม ----------
// ทิศอยู่พารามิเตอร์สุดท้ายเสมอ เช่น ffcmg(60, 40, DIR_E) / ffcmg(60, 40, 's', DIR_E)
// คำสั่ง 3 อาร์กิวเมนต์แบบมีทิศเป็น template ที่ไม่รับ char ตัวสุดท้าย
// เพื่อให้ ffcmg(60, 50, 's') ยังเรียกตัวเดิม ส่วน ffcmg(60, 40, 90) เรียกแบบมีทิศ
// (ถ้าใช้ overload ธรรมดา int → float กับ int → char จะชนกันจน compile ไม่ผ่าน)

template <typename T>
struct GyroIsDirNum {
  static constexpr bool value = std::is_arithmetic<T>::value && !std::is_same<T, char>::value;
};

#define GYRO_DIR_ARGS_OK(A, B) typename std::enable_if<GyroIsDirNum<A>::value && GyroIsDirNum<B>::value, int>::type = 0

// ---------- วิ่งตรงล็อกทิศ (เรียกใน while loop เอง) ----------
// RunG(L, R, dir) / RunGB(L, R, dir)
// เปลี่ยน current_degree + รีเซ็ต D-term เฉพาะตอนทิศเปลี่ยน ไม่ล้าง D-term ทุกสเต็ป

void RunG(int SpeedL, int SpeedR, float dir_deg) {
  float dir = wrap180(dir_deg);
  if (fabsf(wrap180(dir - current_degree)) > 0.01f) {
    current_degree = dir;
    resetGyroRunError();
  }
  RunG(SpeedL, SpeedR);
}

// dir = ทิศที่ถอยไป (ท้ายหุ่นชี้) → ล็อกหัวหุ่น dir + 180
void RunGB(int SpeedL, int SpeedR, float dir_deg) {
  float dir = backNoseDir(dir_deg);
  if (fabsf(wrap180(dir - current_degree)) > 0.01f) {
    current_degree = dir;
    resetGyroRunError();
  }
  RunGB(SpeedL, SpeedR);
}

// ---------- วิ่งตามระยะ มี ramp: ffcmg(Speed, cm, dir) / ffcmg(Speed, cm, select, dir) ----------

template <typename L, typename D, GYRO_DIR_ARGS_OK(L, D)>
void ffcmg(int Speed, L distance_cm, D dir_deg) {
  setRunDir((float)dir_deg);
  ffcmg(Speed, (float)distance_cm);
}
void ffcmg(int Speed, float distance_cm, char select, float dir_deg) {
  setRunDir(dir_deg);
  ffcmg(Speed, distance_cm, select);
}

template <typename L, typename D, GYRO_DIR_ARGS_OK(L, D)>
void bbcmg(int Speed, L distance_cm, D dir_deg) {
  setRunDirB((float)dir_deg);
  bbcmg(Speed, (float)distance_cm);
}
void bbcmg(int Speed, float distance_cm, char select, float dir_deg) {
  setRunDirB(dir_deg);
  bbcmg(Speed, distance_cm, select);
}

// ---------- วิ่งตามระยะ ความเร็วคงที่: ffcmgs(Speed, cm, dir) / ffcmgs(Speed, cm, select, dir) ----------

template <typename L, typename D, GYRO_DIR_ARGS_OK(L, D)>
void ffcmgs(int Speed, L distance_cm, D dir_deg) {
  setRunDir((float)dir_deg);
  ffcmgs(Speed, (float)distance_cm);
}
void ffcmgs(int Speed, float distance_cm, char select, float dir_deg) {
  setRunDir(dir_deg);
  ffcmgs(Speed, distance_cm, select);
}

template <typename L, typename D, GYRO_DIR_ARGS_OK(L, D)>
void bbcmgs(int Speed, L distance_cm, D dir_deg) {
  setRunDirB((float)dir_deg);
  bbcmgs(Speed, (float)distance_cm);
}
void bbcmgs(int Speed, float distance_cm, char select, float dir_deg) {
  setRunDirB(dir_deg);
  bbcmgs(Speed, distance_cm, select);
}

// ---------- วิ่งตามเวลา ms: fftg(Speed, ms, dir) / fftg(Speed, ms, select, dir) ----------

template <typename T, typename D, GYRO_DIR_ARGS_OK(T, D)>
void fftg(int Speed, T totalTime, D dir_deg) {
  setRunDir((float)dir_deg);
  fftimerg(Speed, (int)totalTime);
}
void fftg(int Speed, int totalTime, char select, float dir_deg) {
  setRunDir(dir_deg);
  fftimerg(Speed, totalTime, select);
}

template <typename T, typename D, GYRO_DIR_ARGS_OK(T, D)>
void bbtg(int Speed, T totalTime, D dir_deg) {
  setRunDirB((float)dir_deg);
  bbtimerg(Speed, (int)totalTime);
}
void bbtg(int Speed, int totalTime, char select, float dir_deg) {
  setRunDirB(dir_deg);
  bbtimerg(Speed, totalTime, select);
}

// ---------- วิ่งจนเจอเส้น + ทางแยก: ffbg(Speed, select, dir) ----------

void ffbg(int Speed, char select, float dir_deg) {
  setRunDir(dir_deg);
  ffbg(Speed, select);
}

void bbbg(int Speed, char select, float dir_deg) {
  setRunDirB(dir_deg);
  bbbg(Speed, select);
}

#undef GYRO_DIR_ARGS_OK

// ---------- คอมโบ: หมุนเข้าหาทิศ (ความเร็ว gyro_MaxSpd_Spin) แล้ววิ่งตามระยะ ----------
// spin_ffcmg(Speed, cm, dir) / spin_ffcmg(Speed, cm, select, dir)

void spin_ffcmg(int Speed, float distance_cm, float dir_deg) {
  spindir(dir_deg);
  ffcmg(Speed, distance_cm, dir_deg);
}
void spin_ffcmg(int Speed, float distance_cm, char select, float dir_deg) {
  spindir(dir_deg);
  ffcmg(Speed, distance_cm, select, dir_deg);
}

void spin_bbcmg(int Speed, float distance_cm, float dir_deg) {
  spindir(backNoseDir(dir_deg));
  bbcmg(Speed, distance_cm, dir_deg);
}
void spin_bbcmg(int Speed, float distance_cm, char select, float dir_deg) {
  spindir(backNoseDir(dir_deg));
  bbcmg(Speed, distance_cm, select, dir_deg);
}

// ---------- คอมโบ: เลี้ยวล้อเดียวเดินหน้าเข้าหาทิศ (turndir, ความเร็ว gyro_MaxSpd_Turn) แล้ววิ่งตามระยะ ----------
// turn_ffcmg(Speed, cm, dir) / turn_ffcmg(Speed, cm, select, dir)

void turn_ffcmg(int Speed, float distance_cm, float dir_deg) {
  turndir(dir_deg);
  ffcmg(Speed, distance_cm, dir_deg);
}
void turn_ffcmg(int Speed, float distance_cm, char select, float dir_deg) {
  turndir(dir_deg);
  ffcmg(Speed, distance_cm, select, dir_deg);
}

void turn_bbcmg(int Speed, float distance_cm, float dir_deg) {
  turndir(backNoseDir(dir_deg));
  bbcmg(Speed, distance_cm, dir_deg);
}
void turn_bbcmg(int Speed, float distance_cm, char select, float dir_deg) {
  turndir(backNoseDir(dir_deg));
  bbcmg(Speed, distance_cm, select, dir_deg);
}

// ---------- คอมโบ: เลี้ยวล้อเดียวถอยหลังเข้าหาทิศ (turndirb, ความเร็ว gyro_MaxSpd_TurnB) แล้ววิ่งตามระยะ ----------
// turnb_ffcmg(Speed, cm, dir) / turnb_ffcmg(Speed, cm, select, dir)

void turnb_ffcmg(int Speed, float distance_cm, float dir_deg) {
  turndirb(dir_deg);
  ffcmg(Speed, distance_cm, dir_deg);
}
void turnb_ffcmg(int Speed, float distance_cm, char select, float dir_deg) {
  turndirb(dir_deg);
  ffcmg(Speed, distance_cm, select, dir_deg);
}

void turnb_bbcmg(int Speed, float distance_cm, float dir_deg) {
  turndirb(backNoseDir(dir_deg));
  bbcmg(Speed, distance_cm, dir_deg);
}
void turnb_bbcmg(int Speed, float distance_cm, char select, float dir_deg) {
  turndirb(backNoseDir(dir_deg));
  bbcmg(Speed, distance_cm, select, dir_deg);
}

// ======================================================================
// [13] วิ่งตามเส้นจนหันถึงทิศ / องศา (ffdir, bbdir, ffdeg, bbdeg)
// ======================================================================
// วิ่งตามเส้นด้วย PID (PIDF / PIDB) ไปเรื่อย ๆ แล้วใช้ไจโรเช็กว่าเลี้ยวพ้นโค้งหรือยัง
// เหมาะกับโค้ง 90°, โค้งตัวยู 180°, โค้งตัว S
//
// ทิศ dir (เหมือน ffcmg / bbcmg): ทิศที่หุ่น "เคลื่อนที่มุ่งหน้าไป" อ้างอิง resetAngles()
//   เดินหน้า: หัวหุ่นชี้ dir | ถอยหลัง: ท้ายหุ่นชี้ dir → หัวหุ่นชี้ dir + 180
// deg: องศาสัมพัทธ์นับจากมุมตอนเริ่มคำสั่ง (+ = หัวหุ่นหมุนขวา / ตามเข็ม, - = ซ้าย, -180..180)
//
// ---- select ไว้หน้า dir: จบด้วยโหมดเส้น TrackSelectF / TrackSelectB ----
//   ffdir(Speed, select, dir [, tol])     bbdir(Speed, select, dir [, tol])
//   ffdeg(Speed, select, deg [, tol])     bbdeg(Speed, select, deg [, tol])
// ---- จบด้วยโหมดไจโร TrackSelectG / TrackSelectGB ----
//   ffdirg(Speed, select, dir [, tol])    bbdirg(Speed, select, dir [, tol])
// ---- ไม่ใส่ select: ถึงแล้วไหลต่อทันที ไม่เบรก ----
//   ffdir(Speed, dir [, tol])  bbdir(Speed, dir [, tol])  ffdeg(Speed, deg [, tol])  bbdeg(Speed, deg [, tol])
// ---- แบบเดิม (select ไว้หลัง dir) ยังใช้ได้เหมือนเดิม: จบด้วย TrackSelectG / TrackSelectGB ----
//   ffdir(Speed, dir, select [, tol])  bbdir(...)  ffdeg(Speed, deg, select [, tol])  bbdeg(...)
//   ffdirf / bbdirb / ffdegf / bbdegb(Speed, dir|deg, select [, tol]) = จบด้วย TrackSelectF / B
//
// select: 'n' หรือ 0 = ไม่หยุด ไม่เบรก ไหลต่อเข้าคำสั่งถัดไป
//         'f'        = หยุดด้วยเบรกไจโร SetFG / SetBG (แรง / เวลาเบรกคิดจาก Speed ดู set_line_dir_brake)
//         อื่น ๆ      = ส่งต่อ TrackSelect ของคำสั่งนั้น ('s','l','r','p','c','q','e',...)
//         (แบบเดิม select หลัง dir: 's' = เบรกสวนสั้น ๆ delay_break_f / delay_break_b แล้ว MotorStop())
// tol:    ถือว่าถึงเมื่อ |error| <= tol ติดกัน 2 ค่าจริงจาก IMU (ค่าเริ่มต้น 4°, ใส่ <= 0 = ใช้ค่าเริ่มต้น)
//         หรือ error เปลี่ยนเครื่องหมาย (ข้ามเป้าไปแล้ว) ตอน |error| < 45°
//
// จบคำสั่งแล้ว current_degree = ทิศหัวหุ่นเป้าหมาย + รีเซ็ต D-term (ก่อนเข้า TrackSelect / เบรก)
// → คำสั่งถัดไปเป็น ffcmg / RunG จะวิ่งตรงตามทิศนั้นต่อทันทีโดยไม่กระชาก

constexpr float LINE_DIR_TOL_DEFAULT = 4.0f;
constexpr float LINE_DIR_CROSS_LIMIT = 45.0f;  // นับ "ข้ามเป้า" เฉพาะตอน |error| ต่ำกว่านี้ (กันตอน wrap ±180)

// วิ่งตามเส้น 1 สเต็ปต่อรอบ จนมุมไจโรถึง target_degree (ทิศจริง -180..180)
// backward = true ใช้ PIDB (ถอยหลัง) ไม่งั้นใช้ PIDF
// Kp / Kd ติดลบ (ค่าเริ่มต้น) = ใช้ค่าจากตารางความเร็ว PID_KP_Front / PID_KP_Back
// เวลาสูงสุดของการวิ่งตามเส้นหาทิศ (ms) — 0 = ไม่จำกัด (ค่าเริ่มต้นแบบ MyRP) | ตั้งด้วย set_gyro_dir_timeout()
unsigned long gyro_DirTimeoutMs = 0;

static void lineToHeading(int Speed, float target_degree, float tol_deg, bool backward,
                          float Kp = -1.0f, float Kd = -1.0f) {
  if (tol_deg <= 0.0f) tol_deg = LINE_DIR_TOL_DEFAULT;
  BaseSpeed = Speed;
  InitialSpeed();
  float kp = (Kp >= 0.0f) ? Kp : (backward ? PID_KP_Back : PID_KP_Front);
  float kd = (Kd >= 0.0f) ? Kd : (backward ? PID_KD_Back : PID_KD_Front);

  angleReadFresh();  // ล้าง lastYaw เก่าที่ค้างตอนวิ่งตามเส้นโดยไม่ได้อ่านไจโร
  float previous_error = wrap180(target_degree - lastYaw);
  uint32_t last_sample = yawSampleCount;
  int in_tol_count = (fabsf(previous_error) <= tol_deg) ? 1 : 0;
  GyroGuard gg;
  gyroGuardBegin(gg);

  LINE_WHILE {
    if (backward) PIDB(BackLeftBaseSpeed, BackRightBaseSpeed, kp, kd);
    else PIDF(LeftBaseSpeed, RightBaseSpeed, kp, kd);

    gyroZ();
    // เช็กเฉพาะ IMU หยุดส่งค่า (ไม่จำกัดเวลารวม เพราะวิ่งตามเส้นยาวได้จริง)
    if (gyroGuardExpired(gg, false)) break;
    if (gyro_DirTimeoutMs > 0 && millis() - gg.t_start > gyro_DirTimeoutMs) {
      Serial.println("line dir: timeout");
      MotorStop();
      break;
    }
    if (yawSampleCount == last_sample) continue;  // ยังไม่มีค่าใหม่จาก IMU → วิ่งตามเส้นต่อ
    last_sample = yawSampleCount;

    float current_angle = lastYaw;  // มุมล่าสุดแบบ float จาก IMU
    float error = wrap180(target_degree - current_angle);

    bool crossed = fabsf(error) < LINE_DIR_CROSS_LIMIT
                   && ((error > 0.0f && previous_error < 0.0f) || (error < 0.0f && previous_error > 0.0f));
    previous_error = error;

    if (fabsf(error) <= tol_deg) in_tol_count++;
    else in_tol_count = 0;

    if (in_tol_count >= 2 || crossed) break;
  }

  current_degree = target_degree;
  resetGyroRunError();
}

// ---------- เบรกไจโรของ select 'f' ----------
// 0 = คิดอัตโนมัติจาก Speed: brakeSpd = Speed/2 (15..60), brakeMs = 10 + Speed/3 (20..50), holdMs = 60
int line_dir_brake_spd = 0;
int line_dir_brake_ms = 0;
int line_dir_hold_ms = 60;

void set_line_dir_brake(int brake_spd, int brake_ms, int hold_ms = 60) {
  line_dir_brake_spd = brake_spd;
  line_dir_brake_ms = brake_ms;
  line_dir_hold_ms = hold_ms;
}

static void lineDirGyroBrake(int Speed, bool backward) {
  int spd = abs(Speed);
  int bs = (line_dir_brake_spd > 0) ? line_dir_brake_spd : constrain(spd / 2, 15, 60);
  int bm = (line_dir_brake_ms > 0) ? line_dir_brake_ms : constrain(10 + spd / 3, 20, 50);
  if (backward) SetBG(bs, bm, line_dir_hold_ms);
  else SetFG(bs, bm, line_dir_hold_ms);
}

// ทำแอ็กชันหลังถึงทิศ (แบบเดิม: TrackSelectG / TrackSelectGB)
static void lineDirSelect(int Speed, char select, bool backward) {
  if (select == 'n' || select == 0) return;
  if (select == 'f') {
    lineDirGyroBrake(Speed, backward);
    return;
  }
  if (select == 's') {
    if (backward) {
      Motor(Speed, Speed);
      delay(delay_break_b);
    } else {
      Motor(-Speed, -Speed);
      delay(delay_break_f);
    }
    MotorStop();
    return;
  }
  if (backward) TrackSelectGB(Speed, select);
  else TrackSelectG(Speed, select);
}

// ---------- ทิศจริง (Absolute Heading) ----------

void ffdir(int Speed, float target_dir, float tol_deg = LINE_DIR_TOL_DEFAULT) {
  lineToHeading(Speed, wrap180(target_dir), tol_deg, false);
}
void ffdir(int Speed, float target_dir, char select, float tol_deg = LINE_DIR_TOL_DEFAULT) {
  ffdir(Speed, target_dir, tol_deg);
  lineDirSelect(Speed, select, false);
}

// dir = ทิศที่ถอยไป (ท้ายหุ่นชี้) → ตามเส้นจนหัวหุ่นหันถึง dir + 180
void bbdir(int Speed, float target_dir, float tol_deg = LINE_DIR_TOL_DEFAULT) {
  lineToHeading(Speed, backNoseDir(target_dir), tol_deg, true);
}
void bbdir(int Speed, float target_dir, char select, float tol_deg = LINE_DIR_TOL_DEFAULT) {
  bbdir(Speed, target_dir, tol_deg);
  lineDirSelect(Speed, select, true);
}

// ---------- องศาสัมพัทธ์ (Relative Degree) นับจากมุมตอนเริ่มคำสั่ง ----------

void ffdeg(int Speed, float relative_deg, float tol_deg = LINE_DIR_TOL_DEFAULT) {
  angleReadFresh();
  lineToHeading(Speed, wrap180(lastYaw + relative_deg), tol_deg, false);
}
void ffdeg(int Speed, float relative_deg, char select, float tol_deg = LINE_DIR_TOL_DEFAULT) {
  ffdeg(Speed, relative_deg, tol_deg);
  lineDirSelect(Speed, select, false);
}

void bbdeg(int Speed, float relative_deg, float tol_deg = LINE_DIR_TOL_DEFAULT) {
  angleReadFresh();
  lineToHeading(Speed, wrap180(lastYaw + relative_deg), tol_deg, true);
}
void bbdeg(int Speed, float relative_deg, char select, float tol_deg = LINE_DIR_TOL_DEFAULT) {
  bbdeg(Speed, relative_deg, tol_deg);
  lineDirSelect(Speed, select, true);
}

// ---------- จบด้วยโหมดเส้น (TrackSelectF / TrackSelectB) หรือโหมดไจโร (TrackSelectG / GB) ----------
// 'n' / 0 = ไหลต่อ ไม่เบรก | 'f' = เบรกไจโร SetFG / SetBG | อื่น ๆ = TrackSelect ของโหมดนั้น

static void lineDirAct(int Speed, char select, bool backward, bool gyroMode) {
  if (select == 'n' || select == 0) return;
  if (select == 'f') {
    lineDirGyroBrake(Speed, backward);
    return;
  }
  if (gyroMode) {
    if (backward) TrackSelectGB(Speed, select);
    else TrackSelectG(Speed, select);
  } else {
    if (backward) TrackSelectB(Speed, select);
    else TrackSelectF(Speed, select);
  }
}

// select ไว้หน้า dir → TrackSelectF / TrackSelectB
void ffdir(int Speed, char select, float target_dir, float tol_deg = LINE_DIR_TOL_DEFAULT) {
  ffdir(Speed, target_dir, tol_deg);
  lineDirAct(Speed, select, false, false);
}
// dir = ทิศที่ถอยไป (ท้ายหุ่นชี้)
void bbdir(int Speed, char select, float target_dir, float tol_deg = LINE_DIR_TOL_DEFAULT) {
  bbdir(Speed, target_dir, tol_deg);
  lineDirAct(Speed, select, true, false);
}
void ffdeg(int Speed, char select, float relative_deg, float tol_deg = LINE_DIR_TOL_DEFAULT) {
  ffdeg(Speed, relative_deg, tol_deg);
  lineDirAct(Speed, select, false, false);
}
void bbdeg(int Speed, char select, float relative_deg, float tol_deg = LINE_DIR_TOL_DEFAULT) {
  bbdeg(Speed, relative_deg, tol_deg);
  lineDirAct(Speed, select, true, false);
}

// โหมดไจโร → TrackSelectG / TrackSelectGB
void ffdirg(int Speed, char select, float target_dir, float tol_deg = LINE_DIR_TOL_DEFAULT) {
  ffdir(Speed, target_dir, tol_deg);
  lineDirAct(Speed, select, false, true);
}
void bbdirg(int Speed, char select, float target_dir, float tol_deg = LINE_DIR_TOL_DEFAULT) {
  bbdir(Speed, target_dir, tol_deg);
  lineDirAct(Speed, select, true, true);
}

// แบบ select หลัง dir → TrackSelectF / TrackSelectB (ชื่อเดิมจากรุ่นก่อน ยังใช้ได้)
void ffdirf(int Speed, float target_dir, char select, float tol_deg = LINE_DIR_TOL_DEFAULT) {
  ffdir(Speed, select, target_dir, tol_deg);
}
void bbdirb(int Speed, float target_dir, char select, float tol_deg = LINE_DIR_TOL_DEFAULT) {
  bbdir(Speed, select, target_dir, tol_deg);
}
void ffdegf(int Speed, float relative_deg, char select, float tol_deg = LINE_DIR_TOL_DEFAULT) {
  ffdeg(Speed, select, relative_deg, tol_deg);
}
void bbdegb(int Speed, float relative_deg, char select, float tol_deg = LINE_DIR_TOL_DEFAULT) {
  bbdeg(Speed, select, relative_deg, tol_deg);
}

// ---------- ตัวกันกำกวม: อาร์กิวเมนต์ที่ 3 เป็น int / double (เช่น ffdir(50, 90, 0) หรือ 4.0) ----------
// ถ้าไม่มี template นี้ คอมไพเลอร์จะเลือกไม่ได้ระหว่าง float tol กับ char select
// int / double → ถือเป็น tol (0 = ใช้ค่าเริ่มต้น และไม่หยุด เหมือน 'n')

template <typename T>
using LineTolArg = typename std::enable_if<std::is_arithmetic<T>::value && !std::is_same<T, char>::value
                                              && !std::is_same<T, float>::value, int>::type;

template <typename T, LineTolArg<T> = 0>
void ffdir(int Speed, float target_dir, T tol_deg) {
  ffdir(Speed, target_dir, (float)tol_deg);
}
template <typename T, LineTolArg<T> = 0>
void bbdir(int Speed, float target_dir, T tol_deg) {
  bbdir(Speed, target_dir, (float)tol_deg);
}
template <typename T, LineTolArg<T> = 0>
void ffdeg(int Speed, float relative_deg, T tol_deg) {
  ffdeg(Speed, relative_deg, (float)tol_deg);
}
template <typename T, LineTolArg<T> = 0>
void bbdeg(int Speed, float relative_deg, T tol_deg) {
  bbdeg(Speed, relative_deg, (float)tol_deg);
}

// select ไว้หน้า dir แล้วใส่ทิศเป็น int / double (เช่น ffdir(50, 'r', 90)) — รับเฉพาะ select ที่เป็น char จริง
// (ไม่งั้นจะชนกับ ffdir(50, 90, 4) ที่ตัวที่ 3 เป็น tol)
template <typename C, typename T>
using LineSelDirArg = typename std::enable_if<std::is_same<C, char>::value && std::is_arithmetic<T>::value
                                                 && !std::is_same<T, char>::value && !std::is_same<T, float>::value, int>::type;

template <typename C, typename T, LineSelDirArg<C, T> = 0>
void ffdir(int Speed, C select, T target_dir) {
  ffdir(Speed, (char)select, (float)target_dir);
}
template <typename C, typename T, LineSelDirArg<C, T> = 0>
void bbdir(int Speed, C select, T target_dir) {
  bbdir(Speed, (char)select, (float)target_dir);
}
template <typename C, typename T, LineSelDirArg<C, T> = 0>
void ffdeg(int Speed, C select, T relative_deg) {
  ffdeg(Speed, (char)select, (float)relative_deg);
}
template <typename C, typename T, LineSelDirArg<C, T> = 0>
void bbdeg(int Speed, C select, T relative_deg) {
  bbdeg(Speed, (char)select, (float)relative_deg);
}


// ======================================================================
// [14] วิ่งเกาะเส้นโค้ง (CL / CR) จนหันถึงทิศ / องศา
// ======================================================================
// ใช้ลอจิกเดียวกับ ffcmcl / ffcmcr / bbcmcl / bbcmcr ใน PID.h เป๊ะ:
//   สลับ set_position เป็น set_positionL (cl) / set_positionR (cr) ชั่วคราว
//   แล้ววิ่ง PIDF / PIDB (Kp / Kd จากตารางความเร็ว PID_KP_Front / PID_KP_Back) ทีละสเต็ป
//   ต่างกันแค่เงื่อนไขหยุด: ใช้มุมไจโรแทนระยะทาง (เช็กแบบเดียวกับ ffdir / bbdir ในหมวด [13])
// select ทำตอน set_position ยังเบี่ยงอยู่ (เหมือน ffcmcl(..., select) เดิม) แล้วคืนค่า set_position
//
//   ffdircl / ffdircr(Speed, dir [, tol])            ไหลต่อทันที ไม่เบรก (ต่อโค้งตัว S ได้เนียน)
//   ffdircl / ffdircr(Speed, select, dir [, tol])    จบด้วย TrackSelectF
//   ffdirclg / ffdircrg(Speed, select, dir [, tol])  จบด้วย TrackSelectG
//   bbdircl / bbdircr / bbdirclg / bbdircrg          ถอยหลัง (TrackSelectB / TrackSelectGB)
//   ffdegcl / ffdegcr / bbdegcl / bbdegcr(Speed, [select,] deg [, tol])  องศาสัมพัทธ์ (TrackSelectF / B)
//   ทุกตัวกำหนด Kp / Kd เองได้: X(Speed, Kp, [Kd,] select, dir|deg [, tol])  (ไม่ต้องการ select ใส่ 'n')
// dir: ทิศที่หุ่นเคลื่อนที่ไป (ff = หัวหุ่น, bb = ท้ายหุ่น → หัวหุ่นชี้ dir + 180)
// deg: + = หัวหุ่นหมุนขวา, - = ซ้าย นับจากมุมตอนเริ่มคำสั่ง
// select: 'n' / 0 = ไม่หยุดมอเตอร์ ไหลต่อ | 'f' = เบรกไจโร SetFG / SetBG | อื่น ๆ = TrackSelect

static void lineCurveToHeading(int Speed, float target_nose, float tol_deg, bool backward, bool right,
                               char select, bool gyroMode, float Kp = -1.0f, float Kd = -1.0f) {
  int temp = set_position;
  set_position = right ? set_positionR : set_positionL;
  lineToHeading(Speed, target_nose, tol_deg, backward, Kp, Kd);  // จบแล้ว current_degree = เป้า + resetGyroRunError()
  lineDirAct(Speed, select, backward, gyroMode);
  set_position = temp;
}

// เป้าหัวหุ่นจากองศาสัมพัทธ์ (นับจากมุมสดตอนเริ่มคำสั่ง)
static float lineRelTarget(float relative_deg) {
  return wrap180(angleReadFresh() + relative_deg);
}

void ffdircl(int Speed, float target_dir, float tol_deg = LINE_DIR_TOL_DEFAULT) {
  lineCurveToHeading(Speed, wrap180(target_dir), tol_deg, false, false, 0, false);
}
void ffdircl(int Speed, char select, float target_dir, float tol_deg = LINE_DIR_TOL_DEFAULT) {
  lineCurveToHeading(Speed, wrap180(target_dir), tol_deg, false, false, select, false);
}
void ffdirclg(int Speed, char select, float target_dir, float tol_deg = LINE_DIR_TOL_DEFAULT) {
  lineCurveToHeading(Speed, wrap180(target_dir), tol_deg, false, false, select, true);
}

void ffdegcl(int Speed, float relative_deg, float tol_deg = LINE_DIR_TOL_DEFAULT) {
  lineCurveToHeading(Speed, lineRelTarget(relative_deg), tol_deg, false, false, 0, false);
}
void ffdegcl(int Speed, char select, float relative_deg, float tol_deg = LINE_DIR_TOL_DEFAULT) {
  lineCurveToHeading(Speed, lineRelTarget(relative_deg), tol_deg, false, false, select, false);
}

void ffdircr(int Speed, float target_dir, float tol_deg = LINE_DIR_TOL_DEFAULT) {
  lineCurveToHeading(Speed, wrap180(target_dir), tol_deg, false, true, 0, false);
}
void ffdircr(int Speed, char select, float target_dir, float tol_deg = LINE_DIR_TOL_DEFAULT) {
  lineCurveToHeading(Speed, wrap180(target_dir), tol_deg, false, true, select, false);
}
void ffdircrg(int Speed, char select, float target_dir, float tol_deg = LINE_DIR_TOL_DEFAULT) {
  lineCurveToHeading(Speed, wrap180(target_dir), tol_deg, false, true, select, true);
}

void ffdegcr(int Speed, float relative_deg, float tol_deg = LINE_DIR_TOL_DEFAULT) {
  lineCurveToHeading(Speed, lineRelTarget(relative_deg), tol_deg, false, true, 0, false);
}
void ffdegcr(int Speed, char select, float relative_deg, float tol_deg = LINE_DIR_TOL_DEFAULT) {
  lineCurveToHeading(Speed, lineRelTarget(relative_deg), tol_deg, false, true, select, false);
}

void bbdircl(int Speed, float target_dir, float tol_deg = LINE_DIR_TOL_DEFAULT) {
  lineCurveToHeading(Speed, backNoseDir(target_dir), tol_deg, true, false, 0, false);
}
void bbdircl(int Speed, char select, float target_dir, float tol_deg = LINE_DIR_TOL_DEFAULT) {
  lineCurveToHeading(Speed, backNoseDir(target_dir), tol_deg, true, false, select, false);
}
void bbdirclg(int Speed, char select, float target_dir, float tol_deg = LINE_DIR_TOL_DEFAULT) {
  lineCurveToHeading(Speed, backNoseDir(target_dir), tol_deg, true, false, select, true);
}

void bbdegcl(int Speed, float relative_deg, float tol_deg = LINE_DIR_TOL_DEFAULT) {
  lineCurveToHeading(Speed, lineRelTarget(relative_deg), tol_deg, true, false, 0, false);
}
void bbdegcl(int Speed, char select, float relative_deg, float tol_deg = LINE_DIR_TOL_DEFAULT) {
  lineCurveToHeading(Speed, lineRelTarget(relative_deg), tol_deg, true, false, select, false);
}

void bbdircr(int Speed, float target_dir, float tol_deg = LINE_DIR_TOL_DEFAULT) {
  lineCurveToHeading(Speed, backNoseDir(target_dir), tol_deg, true, true, 0, false);
}
void bbdircr(int Speed, char select, float target_dir, float tol_deg = LINE_DIR_TOL_DEFAULT) {
  lineCurveToHeading(Speed, backNoseDir(target_dir), tol_deg, true, true, select, false);
}
void bbdircrg(int Speed, char select, float target_dir, float tol_deg = LINE_DIR_TOL_DEFAULT) {
  lineCurveToHeading(Speed, backNoseDir(target_dir), tol_deg, true, true, select, true);
}

void bbdegcr(int Speed, float relative_deg, float tol_deg = LINE_DIR_TOL_DEFAULT) {
  lineCurveToHeading(Speed, lineRelTarget(relative_deg), tol_deg, true, true, 0, false);
}
void bbdegcr(int Speed, char select, float relative_deg, float tol_deg = LINE_DIR_TOL_DEFAULT) {
  lineCurveToHeading(Speed, lineRelTarget(relative_deg), tol_deg, true, true, select, false);
}

// ---------- กำหนด Kp / Kd เอง (เหมือน ffcmcl(speed, kp, [kd,] cm, 'p')) ----------
// X(Speed, Kp, [Kd,] select, dir|deg [, tol]) — ต้องใส่ select เสมอ (ไม่ต้องการ ใส่ 'n')
// เพราะ X(Speed, ตัวเลข, ตัวเลข) ถูกใช้เป็น (dir, tol) อยู่แล้ว | Kp / Kd ติดลบ = ใช้ค่าจากตาราง

void ffdircl(int Speed, float Kp, float Kd, char select, float target_dir, float tol_deg = LINE_DIR_TOL_DEFAULT) {
  lineCurveToHeading(Speed, wrap180(target_dir), tol_deg, false, false, select, false, Kp, Kd);
}
void ffdircl(int Speed, float Kp, char select, float target_dir, float tol_deg = LINE_DIR_TOL_DEFAULT) {
  ffdircl(Speed, Kp, -1.0f, select, target_dir, tol_deg);
}

void ffdirclg(int Speed, float Kp, float Kd, char select, float target_dir, float tol_deg = LINE_DIR_TOL_DEFAULT) {
  lineCurveToHeading(Speed, wrap180(target_dir), tol_deg, false, false, select, true, Kp, Kd);
}
void ffdirclg(int Speed, float Kp, char select, float target_dir, float tol_deg = LINE_DIR_TOL_DEFAULT) {
  ffdirclg(Speed, Kp, -1.0f, select, target_dir, tol_deg);
}

void ffdegcl(int Speed, float Kp, float Kd, char select, float relative_deg, float tol_deg = LINE_DIR_TOL_DEFAULT) {
  lineCurveToHeading(Speed, lineRelTarget(relative_deg), tol_deg, false, false, select, false, Kp, Kd);
}
void ffdegcl(int Speed, float Kp, char select, float relative_deg, float tol_deg = LINE_DIR_TOL_DEFAULT) {
  ffdegcl(Speed, Kp, -1.0f, select, relative_deg, tol_deg);
}

void ffdircr(int Speed, float Kp, float Kd, char select, float target_dir, float tol_deg = LINE_DIR_TOL_DEFAULT) {
  lineCurveToHeading(Speed, wrap180(target_dir), tol_deg, false, true, select, false, Kp, Kd);
}
void ffdircr(int Speed, float Kp, char select, float target_dir, float tol_deg = LINE_DIR_TOL_DEFAULT) {
  ffdircr(Speed, Kp, -1.0f, select, target_dir, tol_deg);
}

void ffdircrg(int Speed, float Kp, float Kd, char select, float target_dir, float tol_deg = LINE_DIR_TOL_DEFAULT) {
  lineCurveToHeading(Speed, wrap180(target_dir), tol_deg, false, true, select, true, Kp, Kd);
}
void ffdircrg(int Speed, float Kp, char select, float target_dir, float tol_deg = LINE_DIR_TOL_DEFAULT) {
  ffdircrg(Speed, Kp, -1.0f, select, target_dir, tol_deg);
}

void ffdegcr(int Speed, float Kp, float Kd, char select, float relative_deg, float tol_deg = LINE_DIR_TOL_DEFAULT) {
  lineCurveToHeading(Speed, lineRelTarget(relative_deg), tol_deg, false, true, select, false, Kp, Kd);
}
void ffdegcr(int Speed, float Kp, char select, float relative_deg, float tol_deg = LINE_DIR_TOL_DEFAULT) {
  ffdegcr(Speed, Kp, -1.0f, select, relative_deg, tol_deg);
}

void bbdircl(int Speed, float Kp, float Kd, char select, float target_dir, float tol_deg = LINE_DIR_TOL_DEFAULT) {
  lineCurveToHeading(Speed, backNoseDir(target_dir), tol_deg, true, false, select, false, Kp, Kd);
}
void bbdircl(int Speed, float Kp, char select, float target_dir, float tol_deg = LINE_DIR_TOL_DEFAULT) {
  bbdircl(Speed, Kp, -1.0f, select, target_dir, tol_deg);
}

void bbdirclg(int Speed, float Kp, float Kd, char select, float target_dir, float tol_deg = LINE_DIR_TOL_DEFAULT) {
  lineCurveToHeading(Speed, backNoseDir(target_dir), tol_deg, true, false, select, true, Kp, Kd);
}
void bbdirclg(int Speed, float Kp, char select, float target_dir, float tol_deg = LINE_DIR_TOL_DEFAULT) {
  bbdirclg(Speed, Kp, -1.0f, select, target_dir, tol_deg);
}

void bbdegcl(int Speed, float Kp, float Kd, char select, float relative_deg, float tol_deg = LINE_DIR_TOL_DEFAULT) {
  lineCurveToHeading(Speed, lineRelTarget(relative_deg), tol_deg, true, false, select, false, Kp, Kd);
}
void bbdegcl(int Speed, float Kp, char select, float relative_deg, float tol_deg = LINE_DIR_TOL_DEFAULT) {
  bbdegcl(Speed, Kp, -1.0f, select, relative_deg, tol_deg);
}

void bbdircr(int Speed, float Kp, float Kd, char select, float target_dir, float tol_deg = LINE_DIR_TOL_DEFAULT) {
  lineCurveToHeading(Speed, backNoseDir(target_dir), tol_deg, true, true, select, false, Kp, Kd);
}
void bbdircr(int Speed, float Kp, char select, float target_dir, float tol_deg = LINE_DIR_TOL_DEFAULT) {
  bbdircr(Speed, Kp, -1.0f, select, target_dir, tol_deg);
}

void bbdircrg(int Speed, float Kp, float Kd, char select, float target_dir, float tol_deg = LINE_DIR_TOL_DEFAULT) {
  lineCurveToHeading(Speed, backNoseDir(target_dir), tol_deg, true, true, select, true, Kp, Kd);
}
void bbdircrg(int Speed, float Kp, char select, float target_dir, float tol_deg = LINE_DIR_TOL_DEFAULT) {
  bbdircrg(Speed, Kp, -1.0f, select, target_dir, tol_deg);
}

void bbdegcr(int Speed, float Kp, float Kd, char select, float relative_deg, float tol_deg = LINE_DIR_TOL_DEFAULT) {
  lineCurveToHeading(Speed, lineRelTarget(relative_deg), tol_deg, true, true, select, false, Kp, Kd);
}
void bbdegcr(int Speed, float Kp, char select, float relative_deg, float tol_deg = LINE_DIR_TOL_DEFAULT) {
  bbdegcr(Speed, Kp, -1.0f, select, relative_deg, tol_deg);
}

// ตัวกันกำกวม (เหมือนหมวด [13]): ตัวที่ 3 เป็น int / double → tol | select char + ทิศ int / double → แบบมี select

template <typename T, LineTolArg<T> = 0>
void ffdircl(int Speed, float target_dir, T tol_deg) {
  ffdircl(Speed, target_dir, (float)tol_deg);
}
template <typename C, typename T, LineSelDirArg<C, T> = 0>
void ffdircl(int Speed, C select, T target_dir) {
  ffdircl(Speed, (char)select, (float)target_dir);
}

template <typename T, LineTolArg<T> = 0>
void ffdegcl(int Speed, float relative_deg, T tol_deg) {
  ffdegcl(Speed, relative_deg, (float)tol_deg);
}
template <typename C, typename T, LineSelDirArg<C, T> = 0>
void ffdegcl(int Speed, C select, T relative_deg) {
  ffdegcl(Speed, (char)select, (float)relative_deg);
}

template <typename T, LineTolArg<T> = 0>
void ffdircr(int Speed, float target_dir, T tol_deg) {
  ffdircr(Speed, target_dir, (float)tol_deg);
}
template <typename C, typename T, LineSelDirArg<C, T> = 0>
void ffdircr(int Speed, C select, T target_dir) {
  ffdircr(Speed, (char)select, (float)target_dir);
}

template <typename T, LineTolArg<T> = 0>
void ffdegcr(int Speed, float relative_deg, T tol_deg) {
  ffdegcr(Speed, relative_deg, (float)tol_deg);
}
template <typename C, typename T, LineSelDirArg<C, T> = 0>
void ffdegcr(int Speed, C select, T relative_deg) {
  ffdegcr(Speed, (char)select, (float)relative_deg);
}

template <typename T, LineTolArg<T> = 0>
void bbdircl(int Speed, float target_dir, T tol_deg) {
  bbdircl(Speed, target_dir, (float)tol_deg);
}
template <typename C, typename T, LineSelDirArg<C, T> = 0>
void bbdircl(int Speed, C select, T target_dir) {
  bbdircl(Speed, (char)select, (float)target_dir);
}

template <typename T, LineTolArg<T> = 0>
void bbdegcl(int Speed, float relative_deg, T tol_deg) {
  bbdegcl(Speed, relative_deg, (float)tol_deg);
}
template <typename C, typename T, LineSelDirArg<C, T> = 0>
void bbdegcl(int Speed, C select, T relative_deg) {
  bbdegcl(Speed, (char)select, (float)relative_deg);
}

template <typename T, LineTolArg<T> = 0>
void bbdircr(int Speed, float target_dir, T tol_deg) {
  bbdircr(Speed, target_dir, (float)tol_deg);
}
template <typename C, typename T, LineSelDirArg<C, T> = 0>
void bbdircr(int Speed, C select, T target_dir) {
  bbdircr(Speed, (char)select, (float)target_dir);
}

template <typename T, LineTolArg<T> = 0>
void bbdegcr(int Speed, float relative_deg, T tol_deg) {
  bbdegcr(Speed, relative_deg, (float)tol_deg);
}
template <typename C, typename T, LineSelDirArg<C, T> = 0>
void bbdegcr(int Speed, C select, T relative_deg) {
  bbdegcr(Speed, (char)select, (float)relative_deg);
}

// ======================================================================
// [15] ชื่อเดิมของ MyMini_16Pro_BNO086 (ใช้ต่อได้ — เรียกตัวใหม่ข้างบน)
// ======================================================================

// แฟล็ก "เจอ BNO086" ชื่อเดิม = ตัวเดียวกับ gyro_imu_ready / imu_connected
bool &imuReady = gyro_imu_ready;
uint32_t &gyro_TurnTimeoutMs = gyro_turn_timeout_ms;
uint32_t &imuResetCount = gyro_reset_count;  // จำนวนครั้งที่ BNO08x รีเซ็ตตัวเอง
bool &imuSessionOpen = gyro_sh2_open;
#define IMU_REPORT_ID (gyroReportID())

// ตัวคูณระยะทางชื่อเดิม (fw / bw) = ตัวเดียวกับชื่อ MyRP (ff / bb)
float &speed_scale_fw_g = speed_scale_ff_g;
float &speed_scale_bw_g = speed_scale_bb_g;
float &speed_scale_fw_g_normal = speed_scale_ff_g_normal;
float &speed_scale_bw_g_normal = speed_scale_bb_g_normal;
float &speed_scale_fw_gs = speed_scale_ff_gs;
float &speed_scale_bw_gs = speed_scale_bb_gs;

#define IMU_STALE_MS 300  // ไม่มีค่ามุมใหม่นานกว่านี้ระหว่างหมุน = ไจโรค้าง (gyroAbort)

// เวลาสูงสุดของคำสั่งหมุน / เลี้ยวด้วยไจโร (ms) = set_gyro_timeout(stale, total) ส่วน total
void set_gyro_turn_timeout(unsigned long ms) {
  gyro_turn_timeout_ms = ms;
}

// เวลาสูงสุดของ ffdir / bbdir / ffdeg / ffdircl ... (ms) | 0 = ไม่จำกัด
void set_gyro_dir_timeout(unsigned long ms) {
  gyro_DirTimeoutMs = ms;
}

bool imuEnableReports() {
  return gyroEnableReports();
}

void imuCheckReset() {
  gyroCheckReset();
}

// ค้นหา BNO086 ที่ 0x4A / 0x4B ซ้ำได้ไม่เกิน timeoutMs → เปิดรายงานมุม + warm-up + ตั้งศูนย์ (gyroBegin)
// ไม่เจอ → เตือน 4 ครั้งแล้วทำงานต่อโดยไม่มีไจโร (คำสั่งไจโรจะหยุดเองด้วย timeout)
bool gyro_Begin(unsigned long timeoutMs = 10000) {
  i2cApplyConfig();
  uint8_t addr = imuDetect(timeoutMs);
  if (addr == 0) {
    gyro_imu_ready = false;
    Serial.println("BNO08x not detected at 0x4A / 0x4B. Check your wiring (SDA=GP4, SCL=GP5).");
    for (int i = 0; i < 4; i++) {
      Beep(500);
      delay(200);
    }
    return false;
  }
  Serial.printf("BNO08x found at 0x%02X! (%s)\n", addr,
                gyro_report_mode == GYRO_MODE_RV ? "Rotation Vector" : "Game Rotation Vector");
  i2cApplyConfig();  // ต้องเป็น 100 kHz ก่อนเปิด report / อ่านมุม
  gyroBegin(500);
  i2cApplyConfig();
  if (!gyro_imu_ready) Serial.println("BNO08x stopped responding during warm-up!");
  return gyro_imu_ready;
}

// true = ต้องยกเลิกคำสั่ง: ไม่มีไจโร / ไม่มีค่ามุมใหม่เกิน IMU_STALE_MS / เกิน timeoutMs
bool gyroAbort(unsigned long startMs, unsigned long timeoutMs) {
  if (!gyro_imu_ready) return true;
  unsigned long now = millis();
  if (now - startMs > timeoutMs) {
    Serial.println("gyro: timeout");
    return true;
  }
  unsigned long ref = (lastImuEventMs > startMs) ? lastImuEventMs : startMs;
  if (now - ref > IMU_STALE_MS) {
    Serial.println("gyro: no data (stalled)");
    return true;
  }
  return false;
}

// ---------- วิ่งตามเวลา + ล็อกทิศ (ชื่อเดิม) = fftg / bbtg ของ MyRP ----------
// fftimerg(Speed, ms, dir) / fftimerg(Speed, ms, select, dir) — dir ถอยหลัง = ทิศที่ถอยไป (แบบ MyRP)
void fftimerg(int Speed, int totalTime, int dir_deg) {
  setRunDir((float)dir_deg);
  fftimerg(Speed, totalTime);
}

void bbtimerg(int Speed, int totalTime, int dir_deg) {
  setRunDirB((float)dir_deg);
  bbtimerg(Speed, totalTime);
}

void fftimerg(int Speed, int totalTime, char select, int dir_deg) {
  fftg(Speed, totalTime, select, (float)dir_deg);
}

void bbtimerg(int Speed, int totalTime, char select, int dir_deg) {
  bbtg(Speed, totalTime, select, (float)dir_deg);
}

// ---------- หมุน / เลี้ยวเข้าหาทิศหัวหุ่น (ชื่อเดิม) = spindir / turndir / turndirb ----------
void spindirection(int Speed, int direction) {
  spindir(Speed, (float)direction);
}

void spindirection(int direction) {
  spindir((float)direction);
}

void turndirection(int Speed, int direction) {
  turndir(Speed, (float)direction);
}

void turndirection(int direction) {
  turndir((float)direction);
}

void turndirectionb(int Speed, int direction) {
  turndirb(Speed, (float)direction);
}

void turndirectionb(int direction) {
  turndirb((float)direction);
}

void turndirection_none(int Speed, int direction) {
  turndir_none(Speed, (float)direction);
}

void turndirection_none(int direction) {
  turndir_none((float)direction);
}

void turndirectionb_none(int Speed, int direction) {
  turndirb_none(Speed, (float)direction);
}

void turndirectionb_none(int direction) {
  turndirb_none((float)direction);
}

#endif  // MYMINI16PRO_BNO086_GYRO_H
