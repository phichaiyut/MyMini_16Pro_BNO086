#include <MyMini_16Pro_BNO086.h>

//----------------------------------------------------------------------------->> ตั้งค่ามือจับ

void setup() {
  RobotSetup();

  Setting();
  //_______________
  S28_trim(0);  // ตั้งค่าองศา ขึ้น - ลง
  S18_trim(0);  // ตั้งค่าองศา แขนด้านซ้าย
  S22_trim(0);  // ตั้งค่าองศา แขนด้านขวา
  //__________________
  // arm_down_open();
  arm_up_open();
  // arm_down_close();
  // arm_up_close();
  // arm_down();
  sw();  // รอกดปุ่ม START (ระหว่างรอ Calibrate ได้)
  delay(200);

  SetRobotAngle();
  // SerialServoControl(); // 18 90 ตั้งค่าเซอร์โวผ่าน serial (ขา 18 / 22 / 28 / 0 / 1)
  Mission();

  MotorStop();
}

void loop() {
  HoldAngle();
  sw();
  delay(200);
  SetRobotAngle();
  Mission();
  MotorStop();
}
