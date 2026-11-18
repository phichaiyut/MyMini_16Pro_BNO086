#ifndef MYMINI16PRO_BNO086_DECL_H
#define MYMINI16PRO_BNO086_DECL_H

/*
 * MyMini_16Pro_BNO086_Decl.h
 * ประกาศล่วงหน้า (forward declaration) ของฟังก์ชันที่ถูกเรียกข้ามไฟล์ "ย้อนลำดับ include"
 * (แบบ MyRP_Pico2_mini_BNO086_Decl.h) — เช่น Motor.h / PID.h เรียกฟังก์ชันที่ตัวจริงอยู่ใน Gyro.h
 * ห้ามใส่ค่า default argument ที่นี่ — ให้อยู่ที่ตัวจริงเท่านั้น
 */

// ---------- ตัวจริงอยู่ใน Gyro.h ----------
void resetGyroRunError();   // ใช้ใน Motor.h InitialSpeed()
void SetRobotAngle();       // ใช้ใน PID.h balance / set
void SetFG(int totalTime);  // ใช้ใน PID.h TrackSelectF / TrackSelectB 'g' 'G'
void SetGB(int totalTime);
void SetBG(int totalTime);
void spindir(int Speed, float target_dir);  // ใช้ใน Gyro.h ก่อนตัวจริง (trackSpin / alignHeading)
void turndir(int Speed, float target_dir);
void turndirb(int Speed, float target_dir);

// ---------- ตัวจริงอยู่ใน Battery.h ----------
bool readBatteryVoltage();
float getBatteryVoltage();

#endif  // MYMINI16PRO_BNO086_DECL_H
