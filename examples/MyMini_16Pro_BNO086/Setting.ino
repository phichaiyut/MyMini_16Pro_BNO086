void Setting() {
  /******************** MOTOR ********************/
  set_MotorDriver("DRV8874");     // หรือ "TB6612"
  set_Freq("Coreless_Motors");    // หรือ "DC_Motors"
  set_MotorInvert(false, false);  // กลับทิศล้อ (ซ้าย, ขวา) — แก้หลังทดสอบยกล้อลอย
  set_BatteryCompensate(true);    // ชดเชยความเร็วตามแรงดันแบต
  RobotSetupSpeed();

  /******************** SENSOR READ ********************/
  SetSensorRead(40, 1, 2);  // MUX: รอ (us), ทิ้งค่า (ครั้ง), เฉลี่ย (ครั้ง)
  SetCalibrateTime(5000);   // เวลาเก็บค่าตอน Calibrate (ms)

  /******************** CALIBRATION ********************/
  clampSensorValueF(100, 800);  //สำหรับกรองค่า  calibrate
  clampSensorValueB(100, 800);  //สำหรับกรองค่า  calibrate
  clampSensorValueC(0, 1000);   //สำหรับกรองค่า  calibrate

  /******************** LINE SENSOR CONFIG ********************/
  set_sensor_track_line(0, 15);  // ช่วงเซนเซอร์ที่ใช้คำนวณตำแหน่งเส้น 0 , 15 || 2 , 13 || 3 , 12 || 4 , 11
  RefLineValue(500);        // ค่า threshold เซนเซอร์หน้า-หลัง
  RefCenterLineValue(500);  // ค่า threshold เซนเซอร์คู่กลาง
  TrackLineColor(0);        // 0 = พื้นขาวเส้นดำ | 1 = พื้นดำเส้นขาว
  Dottedline(0);            // 0 = ไม่มีเส้นประ | 1 = มีเส้นประ

  /******************** LINE CENTERING ********************/
  // set_line_center(0);        // เดินธรรมดา เข้ากลางหุ่น
  set_line_center(1);    // เดินตามเส้น เข้ากลางหุ่น
  SetToCenterSpeed(20);  // ความเร็วเข้ากลางหุ่น
  set_brake_fc(10, 30);  // การเบรค ตอนเดินหน้า
  set_brake_bc(10, 30);  // การเบรค ตอนถอยหลัง

  /******************** LINE ALIGN (lf / lb) ********************/
  set_pid_lf(0.015, 0.10);  // PID อยู่กับที่ จัดตัวตรงเส้น lf (Kp, Kd)
  set_pid_lb(0.015, 0.10);  // PID อยู่กับที่ จัดตัวตรงเส้น lb (Kp, Kd)

  /******************** DISTANCE SCALE ********************/
  // ตัวคูณแปลงความเร็ว → ระยะทาง (ใช้ตอนสั่งเดินตามระยะ)
  // หุ่นเดินเกินระยะ ให้เพิ่มค่า | หุ่นเดินไม่ถึงระยะ ให้ลดค่า
  set_speed_scale(1.75, 1.75);  // (เดินหน้า, ถอยหลัง)

  /******************** TURN & SPEED CONFIG ********************/
  SetTurnSpeed(50);  // ความเร็วเลี้ยวหุ่น (l L ) (r R)

  TurnSpeedLeft(-15, 100, 60);   // เลี้ยวซ้าย (q Q)
  TurnSpeedRight(100, -15, 60);  // เลี้ยวขวา (e E)
  SetSensorTurnLeftRight(5, 10);  // เซนเซอร์หน้าที่ใช้หยุดเลี้ยว TurnLeft (F[0..5]) / TurnRight (F[15..10])

  TurnBackSpeedLeft(-25, 100, 60);   // เลี้ยวซ้าย (q Q)
  TurnBackSpeedRight(100, -25, 60);  // เลี้ยวขวา (e E)
  SetSensorTurnLeftRight_B(5, 10);   // เซนเซอร์หลังที่ใช้หยุดเลี้ยว TurnLeft_B (B[0..5]) / TurnRight_B (B[15..10])

  TurnSpeedLeftBackF(-25, 100, 60);   // เลี้ยวล้อเดียวถอยหลัง เช็คเซนเซอร์หน้า (TurnLeftBackF)
  TurnSpeedRightBackF(100, -25, 60);  // เลี้ยวล้อเดียวถอยหลัง เช็คเซนเซอร์หน้า (TurnRightBackF)

  TurnSpeedLeftBackB(-25, 100, 60);   // เลี้ยวล้อเดียวถอยหลัง เช็คเซนเซอร์หลัง (TurnLeftBackB)
  TurnSpeedRightBackB(100, -25, 60);  // เลี้ยวล้อเดียวถอยหลัง เช็คเซนเซอร์หลัง (TurnRightBackB)

  ModeSpdPID(0, 100, -5);  // โหมดควบคุมความเร็ว
  set_pid_mode2_limit(20, 20);  // ModeSpdPID โหมด 2: กำลังล้อ -(Speed+ถอย)..(Speed+เร่ง) (เร่ง, ถอย)

  /******************** GYRO SPEED MODE ********************/
  // เลือกเปิดใช้ทีละบรรทัด (mode, max, min)
  // ModeSpdGyro(0, 100, -0);  // 0 = 0..max        ล้อติดลบ → min
  // ModeSpdGyro(1, 100, -100);  // 1 = min..max
  // ModeSpdGyro(2, 100, -10);  // 2 = -Speed..Speed ถอยล้อได้เต็มที่
  // ModeSpdGyro(3, 100, -10);  // 3 = ..max          ล้อติดลบ → -Speed
  ModeSpdGyro(4, 100, 0);  // 4 = 0..Speed      ไม่ถอยล้อ
  // ModeSpdGyro(2, 4, 100, -5);   // แยกโหมด (เดินหน้า, ถอยหลัง, max, min)
  set_gyro_mode2_limit(20, 20);  // โหมด 2: -(Speed+ถอย)..(Speed+เร่ง) (เร่ง, ถอย)
  set_gyro_mode4_limit(20);      // โหมด 4: 0..(Speed+เร่ง) (เร่ง)

  /******************** GYRO PID CONFIG ********************/
  // (kp, kd, maxSpd, minSpd, smallAngle, stopThr)
  SetGyroTurn(1.2, 0.25, 50, 15, 25.0, 1.0);  // เลี้ยวล้อเดียวด้วยไจโร (turndegree / turndegreeb)
  SetGyroSpin(1.2, 0.25, 50, 8, 25.0, 1.0);   // หมุนตัวอยู่กับที่ด้วยไจโร (spindegree)
  // (kp, kd)
  SetGyroRun(1.2, 1.5);   // เดินหน้าตรงด้วยไจโร (RunG)
  SetGyroRunB(1.2, 1.5);  // ถอยหลังตรงด้วยไจโร (RunGB)

  /******************** GYRO DISTANCE SCALE ********************/
  // ตัวคูณแปลงความเร็ว → ระยะทาง ของคำสั่งวิ่งตรงตามระยะด้วยไจโร
  // หุ่นเดินเกินระยะ ให้เพิ่มค่า | หุ่นเดินไม่ถึงระยะ ให้ลดค่า
  set_speed_scale_ff_g(0.99, 1.5);       // ffcmg (ระยะ >= 30 cm มีเร่ง/ชะลอ, ระยะ < 30 cm)
  set_speed_scale_bb_g(0.99, 1.5);       // bbcmg (ระยะ >= 30 cm มีเร่ง/ชะลอ, ระยะ < 30 cm)
  set_speed_scale_ff_bb_gs(1.65, 1.65);  // ffcmgs / bbcmgs (เดินหน้า, ถอยหลัง)

  /******************** GYRO BRAKE ********************/
  // _setg(ms)   = เวลาล็อกมุม (SetG) หลังหมุน/เลี้ยวเสร็จ ให้หุ่นนิ่งตรงมุม
  // (spd, ms)   = จังหวะเบรกสวนทิศสั้น ๆ ทันทีที่ถึงมุม | หมุนเกินมุม ให้เพิ่ม spd/ms | เด้งกลับ ให้ลด
  set_brake_spin_gyro_setg(5);    // spindegree: เวลาล็อกมุมหลังหมุนเสร็จ (ms)
  set_brake_spin_gyro(12, 5);     // spindegree: เบรกสวนทิศ (ความเร็ว, ms)
  set_brake_turn_gyro_setg(5);    // turndegree: เวลาล็อกมุมหลังเลี้ยวเสร็จ (ms)
  set_brake_turn_gyro(12, 5);     // turndegree: เบรกล้อที่เดินอยู่ (ความเร็ว, ms)
  set_brake_turn_b_gyro_setg(5);  // turndegreeb: เวลาล็อกมุมหลังเลี้ยวถอยหลังเสร็จ (ms)
  set_brake_turn_b_gyro(12, 5);   // turndegreeb: เบรกล้อที่ถอยอยู่ (ความเร็ว, ms)

  /******************** LINE POSITION ********************/
  set_position_line(7500);     // 0–15000 | 1500=ซ้าย 7500=กลาง 13500=ขวา (เซนเซอร์ละ 1000)
  set_position_line_l(1500);   // 0–15000 | วิ่งโค้งซ้าย
  set_position_line_r(13500);  // 0–15000 | วิ่งโค้งขวา

  /******************** DISTANCE SENSOR ********************/
  SetAnalogDistance(DIST_ADS_AUX);  // ADS1115 AIN3 (ค่าเริ่มต้น) | SetAnalogDistance(28) = GP28 ถ้าไม่ได้ใช้เซอร์โวช่องนั้น

  /******************** DEBUG / SERIAL MONITOR ********************/
  // I2C_Scan();                        // ตรวจอุปกรณ์ I2C (พิมพ์ครั้งเดียว)
  // Serial_SensorTable();              // ตาราง raw/min/max/cal (พิมพ์ครั้งเดียว)
  // SerialDistance();                  // เซนเซอร์วัดระยะ
  // Serial_FrontSensor();              // เซนเซอร์หน้า
  // Serial_BackSensor();               // เซนเซอร์หลัง
  // Serial_CenterSensor();             // เซนเซอร์กลาง
  // Serial_AllSensor();                // เซนเซอร์รวม
  // SerialCalibrate_FrontSensor();     // ค่า Calibrate หน้า
  // SerialCalibrate_BackSensor();      // ค่า Calibrate หลัง
  // SerialCalibrate_CenterSensor();    // ค่า Calibrate กลาง
  // SerialCalibrate_AllSensor();       // ค่า Calibrate รวม
  // SerialPositionF();                 // ตำแหน่งเส้น (หน้า)
  // SerialPositionB();                 // ตำแหน่งเส้น (หลัง)
  // SerialPositionFB();                // ตำแหน่งเส้น (หน้า-หลัง)
}

void RobotSetupSpeed() {
  SetBalanceSpeedForward();   // ตั้งค่าความสมดุลมอเตอร์เดินหน้า
  SetBalanceSpeedBackward();  // ตั้งค่าความสมดุลมอเตอร์ถอยหลัง
  SetKpKd();                  // ตั้งค่า KP/KD เดินหน้า
  SetKpKdBack();              // ตั้งค่า KP/KD ถอยหลัง
  SetDelayBreakSpeed();       // ตั้งค่าเวลาเบรกตามความเร็ว
}

// ==================== เวลาเบรก (ms) ตามความเร็ว ====================

void SetDelayBreakSpeed() {
  // หุ่นไถลเลยเส้น ให้เพิ่มค่า | หุ่นถอยกลับเลยเส้น ให้ลดค่า
  // SetDelayBreak(SPEED, เดินหน้า, ถอยหลัง);

  SetDelayBreak(SPD_10, 30, 30);   // ความเร็ว 10
  SetDelayBreak(SPD_20, 30, 30);   // ความเร็ว 20
  SetDelayBreak(SPD_30, 30, 30);   // ความเร็ว 30
  SetDelayBreak(SPD_40, 30, 30);   // ความเร็ว 40
  SetDelayBreak(SPD_50, 30, 30);   // ความเร็ว 50
  SetDelayBreak(SPD_60, 30, 30);   // ความเร็ว 60
  SetDelayBreak(SPD_70, 30, 30);   // ความเร็ว 70
  SetDelayBreak(SPD_80, 30, 30);   // ความเร็ว 80
  SetDelayBreak(SPD_90, 30, 30);   // ความเร็ว 90
  SetDelayBreak(SPD_100, 30, 30);  // ความเร็ว 100
}

// ==================== PID เดินหน้า ====================

void SetKpKd() {
  // ค่าเริ่มต้นจาก UnknowRobot_STORMXII_V3 (16 เซนเซอร์ ตำแหน่ง 0-15000)
  Set_KP_KD(SPD_10, 0.006, 0.10);   // ความเร็ว 10
  Set_KP_KD(SPD_20, 0.007, 0.14);   // ความเร็ว 20
  Set_KP_KD(SPD_30, 0.008, 0.18);   // ความเร็ว 30
  Set_KP_KD(SPD_40, 0.009, 0.22);   // ความเร็ว 40
  Set_KP_KD(SPD_50, 0.010, 0.26);   // ความเร็ว 50
  Set_KP_KD(SPD_60, 0.010, 0.30);   // ความเร็ว 60
  Set_KP_KD(SPD_70, 0.011, 0.34);   // ความเร็ว 70
  Set_KP_KD(SPD_80, 0.011, 0.38);   // ความเร็ว 80
  Set_KP_KD(SPD_90, 0.012, 0.42);   // ความเร็ว 90
  Set_KP_KD(SPD_100, 0.012, 0.46);  // ความเร็ว 100
}

// ==================== PID ถอยหลัง ====================

void SetKpKdBack() {
  Set_KP_KD_Back(SPD_10, 0.006, 0.10);   // ความเร็ว 10
  Set_KP_KD_Back(SPD_20, 0.007, 0.14);   // ความเร็ว 20
  Set_KP_KD_Back(SPD_30, 0.008, 0.18);   // ความเร็ว 30
  Set_KP_KD_Back(SPD_40, 0.009, 0.22);   // ความเร็ว 40
  Set_KP_KD_Back(SPD_50, 0.010, 0.26);   // ความเร็ว 50
  Set_KP_KD_Back(SPD_60, 0.010, 0.30);   // ความเร็ว 60
  Set_KP_KD_Back(SPD_70, 0.011, 0.34);   // ความเร็ว 70
  Set_KP_KD_Back(SPD_80, 0.011, 0.38);   // ความเร็ว 80
  Set_KP_KD_Back(SPD_90, 0.012, 0.42);   // ความเร็ว 90
  Set_KP_KD_Back(SPD_100, 0.012, 0.46);  // ความเร็ว 100
}

// ==================== Balance เดินหน้า ====================

void SetBalanceSpeedForward() {
  // ข้างไหนแรงกว่า ให้เพิ่มค่าข้างนั้น
  // setBalanceSpeed(SPEED, LEFT, RIGHT);

  setBalanceSpeed(SPD_10, 0, 0);   // ความเร็ว 10
  setBalanceSpeed(SPD_20, 0, 0);   // ความเร็ว 20
  setBalanceSpeed(SPD_30, 0, 0);   // ความเร็ว 30
  setBalanceSpeed(SPD_40, 0, 0);   // ความเร็ว 40
  setBalanceSpeed(SPD_50, 0, 0);   // ความเร็ว 50
  setBalanceSpeed(SPD_60, 0, 0);   // ความเร็ว 60
  setBalanceSpeed(SPD_70, 0, 0);   // ความเร็ว 70
  setBalanceSpeed(SPD_80, 0, 0);   // ความเร็ว 80
  setBalanceSpeed(SPD_90, 0, 0);   // ความเร็ว 90
  setBalanceSpeed(SPD_100, 0, 0);  // ความเร็ว 100
}

// ==================== Balance ถอยหลัง ====================

void SetBalanceSpeedBackward() {
  // ข้างไหนแรงกว่า ให้เพิ่มค่าข้างนั้น
  // setBalanceBackSpeed(SPEED, LEFT, RIGHT);

  setBalanceBackSpeed(SPD_10, 0, 0);   // ความเร็ว 10
  setBalanceBackSpeed(SPD_20, 0, 0);   // ความเร็ว 20
  setBalanceBackSpeed(SPD_30, 0, 0);   // ความเร็ว 30
  setBalanceBackSpeed(SPD_40, 0, 0);   // ความเร็ว 40
  setBalanceBackSpeed(SPD_50, 0, 0);   // ความเร็ว 50
  setBalanceBackSpeed(SPD_60, 0, 0);   // ความเร็ว 60
  setBalanceBackSpeed(SPD_70, 0, 0);   // ความเร็ว 70
  setBalanceBackSpeed(SPD_80, 0, 0);   // ความเร็ว 80
  setBalanceBackSpeed(SPD_90, 0, 0);   // ความเร็ว 90
  setBalanceBackSpeed(SPD_100, 0, 0);  // ความเร็ว 100
}
