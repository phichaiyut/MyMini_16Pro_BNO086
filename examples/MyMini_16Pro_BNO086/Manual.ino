/* ==========================================================================
   📘  คู่มือคำสั่ง MyMini_16Pro_BNO086
       (ไฟล์นี้เป็น comment ล้วน ไม่มีผลกับโค้ด)
       กด Ctrl+F แล้วพิมพ์ [1] [2] ... เพื่อกระโดดไปหมวดที่ต้องการ
   --------------------------------------------------------------------------
   [1]  โครงโปรแกรม / ความหมายพารามิเตอร์     [10] Gyro หมุน / เลี้ยว
   [2]  ตั้งค่า (Setting.ino)                    [11] Gyro วิ่งตรง
   [3]  ตารางคำสั่งทางแยก 'select'              [12] มอเตอร์ / แบตเตอรี่
   [4]  เดินตามเส้น เดินหน้า  ff...              [13] แขน / เซอร์โว
   [5]  เดินตามเส้น ถอยหลัง  bb...              [14] เซนเซอร์ / Calibrate / EEPROM
   [6]  วิ่งโค้ง cl / cr                         [15] Debug (Serial Monitor)
   [7]  เข้ากึ่งกลาง / ออกตัว / เข้าเส้นชัย       [16] เสียง / ไฟ LED / I2C
   [8]  หมุน / เลี้ยว ด้วยเซนเซอร์เส้น            [17] ปุ่ม / เมนู sw()
   [9]  Gyro พื้นฐาน / ล็อกมุม
   ========================================================================== */


/* ==========================================================================
   [1] โครงโปรแกรม / ความหมายพารามิเตอร์
   ========================================================================== */
//
// setup() {  RobotSetup();  Setting();  sw();  SetRobotAngle();  Mission();  }
//
//   speed   ความเร็ว 0-100
//   L, R    ความเร็วล้อซ้าย / ขวา  (-100 ถึง 100, ติดลบ = ถอย)
//   ms      เวลา (มิลลิวินาที)
//   cm      ระยะทาง (เซนติเมตร, ประมาณจากเวลา x ความเร็ว)
//   dist    ค่าเซนเซอร์วัดระยะ analogRead (ยิ่งใกล้ ค่ายิ่งมาก)
//   'p'     ตัวอักษรคำสั่งทางแยก ดูตาราง [3]
//   dir     ทิศสัมบูรณ์ 0 / 90 / 180 / 270 (อ้างอิงตอนกด START ใน sw())
//   มุม     ลบ = ซ้าย, บวก = ขวา
//
// เซนเซอร์เส้น 16 ตัวต่อแถว F[0..15] / B[0..15] (F: MUX CH0-15, B: กลับลำดับ B_PIN = 15..0)
//   ชื่อคำสั่ง / ตรรกะจาก MyRP_Pico2_mini_BNO086, ตำแหน่งเซนเซอร์อ้างอิง STORMXII 16CH:
//     ตำแหน่งเส้น 0-15000 (เซนเซอร์ละ 1000) กลาง = 7500
//     ตรวจผ่านแยก / ชิดเส้น = F[3] / F[12]     ขอบนอก (หมุน / บาลานซ์ / ffl / ffr) = F[0] / F[15]
//
// ขาบอร์ด (Pico 2):
//   MUX หน้า S0-S3 = GP17 16 15 14, SIG = GP27   |  MUX หลัง S0-S3 = GP10 11 12 13, SIG = GP26
//   I2C SDA GP4 / SCL GP5 (100 kHz)              |  Buzzer GP9   |  เซนเซอร์วัดระยะ ADS1115 AIN3
//   ปุ่ม START GP2, ปุ่ม Cal หน้า GP3              |  ปุ่ม ADCcal ผ่าน MCP3421 (0x68)
//   เซอร์โว GP28 = แขนขึ้น-ลง, GP18 = มือซ้าย, GP22 = มือขวา, GP0 / GP1 = ช่องเสริม
//   DRV8874 ซ้าย EN19 PH20 nSLEEP21 | ขวา EN6 PH7 nSLEEP8   (TB6612 ใช้ขาเดียวกัน)


/* ==========================================================================
   [2] ตั้งค่า (Setting.ino)
   ========================================================================== */
// --- มอเตอร์ ---
// set_MotorDriver("DRV8874");              // หรือ "TB6612"
// set_Freq("Coreless_Motors");             // 20 kHz  (หรือ "DC_Motors" 1 kHz)
// set_MotorInvert(left, right);            // กลับทิศล้อ
// set_BatteryCompensate(true);             // ชดเชยความเร็วตามแรงดันแบต
//
// --- อ่านเซนเซอร์ / กรองค่า calibrate ---
// SetSensorRead(settleUs, discard, average);   // ค่าเริ่มต้น (40, 1, 2)
// SetCalibrateTime(ms);                         // เวลาเก็บค่า Calibrate (ค่าเริ่มต้น 5000)
// clampSensorValueF(min, max);   clampSensorValueB(min, max);   clampSensorValueC(min, max);
//
// --- เซนเซอร์เส้น ---
// RefLineValue(500);                       // threshold เซนเซอร์หน้า-หลัง (0-1000)
// RefCenterLineValue(500);                 // threshold เซนเซอร์คู่กลาง (0-1000)
// TrackLineColor(0);                       // 0 = พื้นขาวเส้นดำ | 1 = พื้นดำเส้นขาว
// Dottedline(0);                           // 0 = ไม่มีเส้นประ | 1 = มีเส้นประ
//
// --- เข้ากึ่งกลาง ---
// set_line_center(1);                      // 0 = วิ่งตรงเข้ากลาง | 1 = เดินตามเส้นเข้ากลาง
// SetToCenterSpeed(20);                    // ความเร็วตอนเข้ากลาง
// set_brake_fc(ff, fc);                    // เวลาเบรก (ms) ตอนเดินหน้า
// set_brake_bc(bf, bc);                    // เวลาเบรก (ms) ตอนถอยหลัง
// set_pid_lf(kp, kd);  set_pid_lb(kp, kd); // PID ของ lf / lb (ค่าเริ่มต้น 0.015, 0.10)
//
// --- ความเร็วเลี้ยว (ใช้กับ select l r q e) ---
// SetTurnSpeed(50);                        // ความเร็วหมุน 'l' 'r'
// TurnSpeedLeft(L, R, ms);                 TurnSpeedRight(L, R, ms);         // 'q' 'e'  TurnLeft / TurnRight
// SetSensorTurnLeftRight(5, 10);           // เซนเซอร์หน้าที่ใช้หยุด TurnLeft (F[0..5]) / TurnRight (F[15..10])
// TurnBackSpeedLeft(L, R, ms);             TurnBackSpeedRight(L, R, ms);     // TurnLeft_B / TurnRight_B
// SetSensorTurnLeftRight_B(5, 10);         // เซนเซอร์หลังที่ใช้หยุด TurnLeft_B (B[0..5]) / TurnRight_B (B[15..10])
// TurnSpeedLeftBackF(L, R, ms);            TurnSpeedRightBackF(L, R, ms);    // TurnLeftBackF / TurnRightBackF
// TurnSpeedLeftBackB(L, R, ms);            TurnSpeedRightBackB(L, R, ms);    // TurnLeftBackB / TurnRightBackB
//
// --- โหมดจำกัดความเร็วล้อ ---
// ModeSpdPID(mode, max, min);              SetPIDDeadBand(db);               // PID เส้น
// ModeSpdGyro(mode, max, min);             ModeSpdGyro(modeF, modeB, max, min);  // Gyro (แยกหน้า/หลังได้)
//     mode 0 = 0..max   1 = min..max   2 = -Speed..Speed   3 = ..max   4 = 0..Speed (ไม่ถอยล้อ)
// set_pid_mode2_limit(up, down);           // PID เส้น โหมด 2: -(Speed+down)..(Speed+up) (ค่าเริ่มต้น 20, 20)
// set_gyro_mode2_limit(up, down);          // Gyro โหมด 2: -(Speed+down)..(Speed+up) (ค่าเริ่มต้น 20, 20)
// set_gyro_mode4_limit(up);                // Gyro โหมด 4: 0..(Speed+up) (ค่าเริ่มต้น 20)
//
// --- PID ไจโร ---
// SetGyroTurn(kp, kd, maxSpd, minSpd, smallAngle, stopThr);   // เลี้ยวล้อเดียว (turndegree / turndegreeb)
// SetGyroSpin(kp, kd, maxSpd, minSpd, smallAngle, stopThr);   // หมุนอยู่กับที่ (spindegree)
// SetGyroRun(kp, kd);                      SetGyroRunB(kp, kd);              // วิ่งตรง RunG / RunGB
//
// --- ตำแหน่งเส้น (0-15000, กลาง = 7500) ---
// set_sensor_track_line(0, 15);            // ช่วงเซนเซอร์ที่ใช้คำนวณ 0,15 | 2,13 | 3,12 | 4,11
// set_position_line(7500);                 // ตำแหน่งปกติ
// set_position_line_l(1500);               set_position_line_r(13500);       // วิ่งโค้ง cl / cr
// set_slow_kp_kd(kpf, kdf, kpb, kdb);      // PID ช้าตอนเข้ากลาง
//
// --- เซนเซอร์วัดระยะ ---
// SetAnalogDistance(DIST_ADS_AUX);         // ADS1115 AIN3 (ค่าเริ่มต้น) | SetAnalogDistance(28) = analogRead GP28
//
// --- ตารางตามความเร็ว (ch = SPD_10, SPD_20 ... SPD_100) ---
// setBalanceSpeed(ch, L, R);               // balance เดินหน้า (ล้อไหนแรงกว่า เพิ่มค่าข้างนั้น)
// setBalanceBackSpeed(ch, L, R);           // balance ถอยหลัง
// Set_KP_KD(ch, kp, kd);                   // PID เดินหน้า
// Set_KP_KD_Back(ch, kp, kd);              // PID ถอยหลัง
// SetDelayBreak(ch, f, b);                 // เวลาเบรก (ms) ตอนหยุดที่เส้น (ไถลเลย = เพิ่ม, ถอยเลย = ลด)


/* ==========================================================================
   [3] ตารางคำสั่งทางแยก 'select'  (ตัวสุดท้ายของ ff / bb / *g)
   --------------------------------------------------------------------------
     ตัวอักษร  | ff / bb (เดินตามเส้น)               | *g / *gb (Gyro)
     ---------+-------------------------------------+----------------------------
     's'      | หยุด                                 | หยุด
     'S'      | วิ่งถึงขอบเส้นแล้วหยุด                 | หยุด
     'p' 'P'  | วิ่งผ่าน                               | วิ่งผ่าน
     'c' 'C'  | เข้ากลางแล้วหยุด                       | เข้ากลางแล้วหยุด
     'l' 'L'  | เข้ากลาง + หมุนซ้าย                    | 'l' เข้ากลาง+หมุนซ้าย  'L' หมุนซ้ายทันที
     'r' 'R'  | เข้ากลาง + หมุนขวา                    | 'r' เข้ากลาง+หมุนขวา  'R' หมุนขวาทันที
     'q' 'Q'  | เลี้ยวโค้งซ้าย  (bb = ขวา)             | เลี้ยวโค้งซ้าย  (gb = ขวา)
     'e' 'E'  | เลี้ยวโค้งขวา  (bb = ซ้าย)             | เลี้ยวโค้งขวา  (gb = ซ้าย)
     'a' 'A'  | หมุนซ้ายด้วยเซนเซอร์หลัง               | -
     'd' 'D'  | หมุนขวาด้วยเซนเซอร์หลัง               | -
     'b' 'B'  | เข้ากลางแล้ววิ่งต่อถึงเส้นอีกฝั่ง          | เหมือนกัน
     'g' 'G'  | ปรับตรงด้วย gyro                      | ตัวอื่น ๆ = SetG(100)
   --------------------------------------------------------------------------
     ตัวพิมพ์ใหญ่ (P C Q E G) = เรียก ToFront() / ToBack() จัดตรงเส้นก่อน
   ========================================================================== */


/* ==========================================================================
   [4] เดินตามเส้น เดินหน้า  ff...
   ========================================================================== */
// ff(speed, 'p');                          // วิ่งจนเจอแยกใดก็ได้ (F3 หรือ F12)
// ffc(speed, 'p');      ffc2(speed, 'p');  // เจอแยกตัด (เส้นกว้างทั้งสองฝั่ง แบบ STORMXII) | ffc2 = F0+F15 / F2+F13 / F4-F11
// ffl(speed, 'p');                         // เจอแยกซ้าย (F0)
// ffl2(speed, 'p');                        // เจอแยกซ้ายเต็ม (F0 F2 F4 F6 F9)
// ffr(speed, 'p');                         // เจอแยกขวา (F15)
// ffr2(speed, 'p');                        // เจอแยกขวาเต็ม (F6 F9 F11 F13 F15)
// ffnum(speed, 'p', n); ffn(speed, 'p', n);    // เจอเส้นที่เซนเซอร์ F[n] (0-15)
// ffwhite(speed, 'p');  ffw(speed, 'p');       // วิ่งจนไม่เจอเส้นเลย
// ffblack(speed, 'p');  ffb(speed, 'p');       // วิ่งตรง (ไม่ PID) จนเจอเส้น
// ffblack(L, R, 'p');   ffb(L, R, 'p');
// ff_distance(speed, 'p', dist);   ffd(speed, 'p', dist);     // จนเซนเซอร์วัดระยะถึง dist
// ff_distances(speed, 'p', dist);  ffds(speed, 'p', dist);    // แบบค่อย ๆ ชะลอ
// fftimer(speed, ms);   fftimer(speed, ms, 'p');              // วิ่งตามเวลา
// fft(speed, ms);       fft(speed, ms, 'p');                  // ชื่อย่อของ fftimer
// ffcm(speed, cm);      ffcm(speed, cm, 'p');                 // วิ่งตามระยะทาง
// lf(ms);                                  // PID อยู่กับที่ จัดตัวตรงเส้น
//
// --- กำหนด Kp / Kd เอง (ใส่ต่อจาก speed) ---
// ใช้ได้กับ ff ffc ffc2 ffl ffl2 ffr ffr2 ffnum/ffn ffwhite/ffw ff_distance/ffd
// และฝั่ง bb... ทุกตัวที่คู่กัน  (ffblack/ffb ไม่ใช้ PID จึงไม่มี)
// ff(speed, kp, 'p');                      // กำหนด Kp อย่างเดียว (Kd ใช้ค่าจากตารางความเร็ว)
// ff(speed, kp, kd, 'p');                  // กำหนดทั้ง Kp และ Kd
// ffn(speed, kp, kd, 'p', n);              // ตัวที่มีพารามิเตอร์ต่อท้าย ใส่ Kp/Kd หลัง speed เหมือนกัน
// ffd(speed, kp, 'p', dist);
// fftimer(speed, kp, ms);       fftimer(speed, kp, kd, ms);       // ตามเวลา (ใส่ 'p' ท้ายสุดได้)
// fftimer(speed, kp, ms, 'p');  fftimer(speed, kp, kd, ms, 'p');
// ffcm(speed, kp, cm);          ffcm(speed, kp, kd, cm);          // ตามระยะทาง (cm เป็นจำนวนเต็ม)
// ffcm(speed, kp, cm, 'p');     ffcm(speed, kp, kd, cm, 'p');
// fft(speed, kp, [kd,] ms [, 'p']);                               // ชื่อย่อ ใช้แบบเดียวกับ fftimer
// * ใส่ Kp เป็นทศนิยม เช่น 0.02  (ใส่ค่าติดลบ = ใช้ค่าจากตาราง)


/* ==========================================================================
   [5] เดินตามเส้น ถอยหลัง  bb...   (ใช้เซนเซอร์หลัง เหมือน ff ทุกอย่าง)
   ========================================================================== */
// bb(speed, 'p');
// bbc(speed, 'p');      bbc2(speed, 'p');
// bbl(speed, 'p');      bbl2(speed, 'p');
// bbr(speed, 'p');      bbr2(speed, 'p');
// bbnum(speed, 'p', n); bbn(speed, 'p', n);
// bbwhite(speed, 'p');  bbw(speed, 'p');
// bbblack(speed, 'p');  bbb(speed, 'p');
// bbblack(L, R, 'p');   bbb(L, R, 'p');
// bb_distance(speed, 'p', dist);   bbd(speed, 'p', dist);
// bbtimer(speed, ms);   bbtimer(speed, ms, 'p');
// bbt(speed, ms);       bbt(speed, ms, 'p');
// bbcm(speed, cm);      bbcm(speed, cm, 'p');
// lb(ms);
// bb(speed, kp, 'p');   bb(speed, kp, kd, 'p');   bbn(speed, kp, kd, 'p', n);   // กำหนด Kp / Kd เอง
// bbtimer(speed, kp, [kd,] ms [, 'p']);    bbcm(speed, kp, [kd,] cm [, 'p']);
// bbt(speed, kp, [kd,] ms [, 'p']);


/* ==========================================================================
   [6] วิ่งโค้ง  cl = เบี่ยงซ้าย, cr = เบี่ยงขวา
       (ตั้ง set_position_line_l / _r ใน Setting ก่อน)
   ========================================================================== */
//   เดินหน้า                         ถอยหลัง
// ffcl(speed, 'p');                 bbcl(speed, 'p');
// ffcr(speed, 'p');                 bbcr(speed, 'p');
// fftimercl(speed, ms [, 'p']);     bbtimercl(speed, ms [, 'p']);
// fftimercr(speed, ms [, 'p']);     bbtimercr(speed, ms [, 'p']);
// ffcmcl(speed, cm [, 'p']);        bbcmcl(speed, cm [, 'p']);
// ffcmcr(speed, cm [, 'p']);        bbcmcr(speed, cm [, 'p']);
// ffdircl(speed, dir [, 'p']);      bbdircl(speed, dir [, 'p']);     // จนหันถึงทิศ dir (0/90/180/270)
// ffdircr(speed, dir [, 'p']);      bbdircr(speed, dir [, 'p']);
//                                   ( [ ] = ใส่หรือไม่ใส่ก็ได้ )
//
// --- กำหนด Kp / Kd เอง (ใส่ต่อจาก speed) ---
// ffcl(speed, kp, [kd,] 'p');                  bbcl / ffcr / bbcr ใช้แบบเดียวกัน
// fftimercl(speed, kp, [kd,] ms [, 'p']);      ...timercr / bbtimercl / bbtimercr แบบเดียวกัน
// ffcmcl(speed, kp, [kd,] cm [, 'p']);         ...cmcr / bbcmcl / bbcmcr แบบเดียวกัน (cm เป็นจำนวนเต็ม)
// ffdircl(speed, kp, [kd,] dir [, 'p']);       ...dircr / bbdircl / bbdircr แบบเดียวกัน (kp ต้องมีจุดทศนิยม เช่น 0.02)


/* ==========================================================================
   [7] เข้ากึ่งกลาง / ออกตัว / เข้าเส้นชัย
   ========================================================================== */
// ToCenter();   ToCenterL();   ToCenterR();      // เดินหน้าเข้ากลาง (เซนเซอร์คู่กลาง)
// BackCenter();                                  // ถอยหลังเข้ากลาง
// ToFront();    ToBack();                        // จัดตัวตรงเส้น หน้า / หลัง
// gostart(speed);   gostart(L, R);               // ออกจากกล่องเริ่ม
// goend(speed);     goend(L, R);                 // วิ่งเข้าเส้นชัย
// setf(n);   setb(n);   setfc(n);   setbc(n);    // จัดตรงเส้น n รอบ (= balancef / b / fc / bc)
// set_f(n);  set_b(n);  set_fc(n);  set_bc(n);
// TrackSelectF(speed, 'p');   TrackSelectB(speed, 'p');   // สั่ง select ตรง ๆ


/* ==========================================================================
   [8] หมุน / เลี้ยว ด้วยเซนเซอร์เส้น  ([speed] ใส่หรือไม่ใส่ก็ได้)
   ========================================================================== */
// spinl([speed]);      spinr([speed]);       // หมุนอยู่กับที่ไปเส้นถัดไป (เซนเซอร์หน้า)
// spinl2([speed]);     spinr2([speed]);
// spinl_B([speed]);    spinr_B([speed]);     // ใช้เซนเซอร์หลัง
// spinl2_B([speed]);   spinr2_B([speed]);
// TurnLeft();          TurnRight();          // เลี้ยวโค้ง เซนเซอร์หน้า
// TurnLeft_B();        TurnRight_B();        // เลี้ยวโค้ง เซนเซอร์หลัง (ชื่อแบบ Pico2: TurnLeftB / TurnRightB)
// TurnLeftBackF();     TurnRightBackF();     // เลี้ยวล้อเดียวถอยหลัง เช็คเซนเซอร์หน้า
// TurnLeftBackB();     TurnRightBackB();     // เลี้ยวล้อเดียวถอยหลัง เช็คเซนเซอร์หลัง


/* ==========================================================================
   [9] Gyro พื้นฐาน / ล็อกมุม
   ========================================================================== */
// resetAngles();       // ตั้งมุมปัจจุบัน = 0 (เรียกตอนหุ่นหยุดนิ่ง)
// SetRobotAngle();     // จำมุมปัจจุบันเป็นมุมเป้าหมายวิ่งตรง
// gyroZ();             // อ่านมุม -180..180 (ซ้าย = ลบ, ขวา = บวก)
// SetDirectionG(dir);  relativeToDirection(dir);    // ตั้งทิศเป้าหมายแบบสัมบูรณ์
//
// SetG(ms);    setg(ms);       // ล็อกมุม (หมุนอยู่กับที่) ตามเวลา
// SetGB(ms);   setgb(ms);      // ล็อกมุม แบบถอยหลัง
// SetFG(ms);   setfg(ms);      // ล็อกมุม แบบเดินหน้า
// SetBG(ms);   setbg(ms);      // ล็อกมุม แบบถอยหลัง (จำมุม ณ ตอนเรียก เหมือน SetFG)
// SetHoldAngle();              // จำมุมปัจจุบันให้ HoldAngle...
// HoldAngle();   HoldAngleB();   HoldAngleF();   HoldAngleBF();    // ใช้ใน loop เพื่อค้างมุม


/* ==========================================================================
   [10] Gyro หมุน / เลี้ยว  (มุม: ลบ = ซ้าย, บวก = ขวา | [speed] ใส่หรือไม่ใส่ก็ได้)
   ========================================================================== */
// --- หมุน/เลี้ยวตามมุม ---
// spindegree([speed,] -90);          // หมุนอยู่กับที่
// turndegree([speed,] 90);           // เลี้ยวล้อเดียว เดินหน้า
// turndegreeb([speed,] 90);          // เลี้ยวล้อเดียว ถอยหลัง
// turndegree_none([speed,] 90);      // เหมือน turndegree แต่ไม่เบรก (ต่อคำสั่งถัดไปลื่น)
// turndegreeb_none([speed,] 90);
// rotatedegree(L, R, 90);            rotatedegree(L, R, 90, kp, kd);   // โค้งกว้าง กำหนดล้อเอง
//
// --- หันไปทิศสัมบูรณ์ (0 / 90 / 180 / 270) ---
// spindirection([speed,] dir);
// turndirection([speed,] dir);
// turndirectionb([speed,] dir);
// turndirection_none([speed,] dir);
// turndirectionb_none([speed,] dir);
//
// --- วิ่งตามเส้นจนหันถึงทิศ (ใช้กับเส้นโค้ง) ---
// ffdir(speed, dir [, 'p']);             bbdir(speed, dir [, 'p']);             // เช่น ffdir(50, 270);
// ffdir(speed, kp, [kd,] dir [, 'p']);   bbdir(speed, kp, [kd,] dir [, 'p']);   // กำหนด Kp / Kd เอง (kp ต้องมีจุดทศนิยม)
// set_line_dir_threshold(5.0);           // ระยะยอมรับ (องศา) ค่าเริ่มต้น 5
//
// --- ชื่อย่อ (ใส่มุมเป็นบวกเสมอ) ---
//   ซ้าย                ขวา                 ย่อ
// spinlg([speed,] 90);  spinrg([speed,] 90);  slg / srg      // หมุนอยู่กับที่
// turnlg([speed,] 90);  turnrg([speed,] 90);  tlg / trg      // เลี้ยวเดินหน้า
// turnlbg([speed,] 90); turnrbg([speed,] 90); tlbg / trbg    // เลี้ยวถอยหลัง
//
// --- เลี้ยว 2 จังหวะต่อเนื่อง (ไม่หยุดกลางทาง) ---
// tlrg([speed,] 90 [, 45]);    trlg([speed,] 90 [, 45]);     // ซ้ายแล้วขวา / ขวาแล้วซ้าย
// tlrbg([speed,] 90 [, 45]);   trlbg([speed,] 90 [, 45]);    // แบบถอยหลัง


/* ==========================================================================
   [11] Gyro วิ่งตรง  ( [ ] = ใส่หรือไม่ใส่ก็ได้ | ใส่ dir ท้าย = ล็อกทิศสัมบูรณ์ )
   ========================================================================== */
//   เดินหน้า                              ถอยหลัง
// fftimerg(speed, ms [, 'p'] [, dir]);   bbtimerg(speed, ms [, 'p'] [, dir]);   // ตามเวลา
// fftg(speed, ms, 'p' [, dir]);          bbtg(speed, ms, 'p' [, dir]);          // ชื่อย่อ
// ffcmg(speed, cm [, 'p'] [, dir]);      bbcmg(speed, cm [, 'p'] [, dir]);      // ตามระยะ (เร่ง/ผ่อน)
// ffcmgs(speed, cm [, 'p'] [, dir]);     bbcmgs(speed, cm [, 'p'] [, dir]);     // ตามระยะ (ความเร็วคงที่)
// ffbg(speed, 'p' [, dir]);              bbbg(speed, 'p' [, dir]);              // วิ่งจนเจอเส้น
// ffdg(speed, 'p', dist);                bbdg(speed, 'p', dist);                // จนเซนเซอร์วัดระยะถึง dist
// ffdgs(speed, 'p', dist);               bbdgs(speed, 'p', dist);               // แบบค่อย ๆ ชะลอ
// ToCenterG();                           BackCenterG();                         // เข้ากลางด้วย gyro
// ToFrontG();                            ToBackG();                             // วิ่งจนเจอเส้น หน้า / หลัง
// TrackSelectG(speed, 'p');              TrackSelectGB(speed, 'p');             // สั่ง select ตรง ๆ
// RunG(L, R);                            RunGB(L, R);                           // 1 รอบ PID ใช้ใน loop เอง


/* ==========================================================================
   [12] มอเตอร์ / แบตเตอรี่
   ========================================================================== */
// Motor(L, R);                 // สั่งล้อค้างไว้ | (0,0) = ปล่อยล้อ, ล้อเดียวเป็น 0 = เบรกล้อนั้น
// Move(L, R, ms);              // สั่งล้อตามเวลา
// MotorStop();   MotorStop(ms);            // เบรก / เบรกค้าง ms
// MotorCoast();                            // ปล่อยล้อหมุนอิสระ
// MotorShot(L, R, ms);                     // เบรกดึงกลับ (สั่งสวนทิศสั้น ๆ ไม่เกิน 50 ms)
// fd(speed, ms);   bk(speed, ms);          // เดินหน้า / ถอยหลัง (ใช้ตาราง balance)
// sl(speed, ms);   sr(speed, ms);          // หมุนซ้าย / ขวา
// tl(speed, ms);   tr(speed, ms);          // เลี้ยวซ้าย / ขวา (ล้อเดียว)
//
// getBatteryVoltage();     getBatteryLevel();      // V / ระดับ 0-8
// isLowBattery();          isBatteryCritical();    isNoBattery();
// updateBattery();         bat_update_led();       bat_warning();    // เรียกซ้ำใน loop


/* ==========================================================================
   [13] แขน / เซอร์โว
   ========================================================================== */
// Servo(pin, angle);                         // pin: 28 (ขึ้น-ลง), 18 (ซ้าย), 22 (ขวา), 0, 1
// Servo(ud, l, r);             Servo(ud, l, r, speed);       // speed = ms ต่อองศา
// armupdown(angle [, speed]);                // แขนขึ้น-ลง
// arm_left_right(l, r [, speed]);            // มือซ้าย-ขวา
// S28_trim(x);   S18_trim(x);   S22_trim(x);   S0_trim(x);   S1_trim(x);     // ชดเชยองศา
// set_ServoReverse(pin, true/false);         // กลับทิศ (ค่าเริ่มต้น 22 และ 28 กลับทิศ)
// set_ServoPulse(pin, 500, 2500);            // ช่วงพัลส์ 0-180°
// ServoOff();    ServoOff(pin);              // ปล่อยเซอร์โว
// SerialServoControl();                      // ปรับผ่าน Serial พิมพ์ "18 90" (exit / ปุ่ม START = ออก)
//
// --- ท่าสำเร็จรูปใน Servo.ino (ใส่ speed ได้ เช่น arm_close(2);) ---
// arm_ready();       arm_up();          arm_up45();         arm_down();
// arm_open();        arm_close();       arm_open_l();       arm_open_r();
// arm_open_down();   arm_down_open();   arm_open_up();      arm_up_open();
// arm_down_close();  arm_close_down();  arm_up_close();     arm_close_up();
// arm_big();         arm_big_box();     arm_big_box_up();   arm_behihd();


/* ==========================================================================
   [14] เซนเซอร์ / Calibrate / EEPROM
   ========================================================================== */
// --- ตัวแปร ---
// F[0-15]  B[0-15]                 // ค่าเซนเซอร์จริง หน้า / หลัง (หลัง calibrate 0-1000, เจอเส้น = มาก)
// C[CCL]  C[CCR]                   // เซนเซอร์กลาง ซ้าย / ขวา
// rawF[0-15] rawB[0-15] rawC[]     // ค่าดิบล่าสุดตามช่อง MUX
// Ref  RefC                        // threshold หน้า-หลัง / กลาง
// F_PIN[] B_PIN[]                  // ลำดับช่อง MUX → F[i] / B[i]
//
// --- อ่านค่า ---
// ReadCalibrateF();   ReadCalibrateB();   ReadCalibrateC();     // อ่านแบบ calibrate แล้ว
// ReadF();            ReadB();            ReadC();              // อ่านค่าดิบ
// ReadSensor();       ReadSensorRaw();                          // ทุกตัว (หน้า+หลังรอบเดียว)
// read_sensorF(ch);   read_sensorB(ch);                         // ช่องเดียว 0-15 (ค่าดิบ)
// readPositionF(200, 50);   readPositionB(200, 50);             // ตำแหน่งเส้น 0-15000
// readDistance();                                               // เซนเซอร์วัดระยะ 0-4095
//
// --- Calibrate ---
// calibrateF();        calibrateB();        calibrateC();       // เก็บค่าแล้วบันทึกให้อัตโนมัติ
// saveCalibration();                        // EEPROM ภายนอก (ตรวจอ่านกลับ) + สำรอง flash
// loadCalibration();                        // โหลด EEPROM ถ้าไม่ได้ใช้ flash
// loadCalibration_LOCAL();   saveCalibration_LOCAL();   printCalibration();
// รูปแบบข้อมูลเหมือน MyMINI_16Pro → ค่าที่เคย Calibrate ไว้โหลดได้เลย


/* ==========================================================================
   [15] Debug (Serial Monitor) — ค้าง while(1) ใช้ใน Setting() เท่านั้น
   ========================================================================== */
// Serial_FrontSensor();            Serial_BackSensor();             // ค่าดิบ
// Serial_CenterSensor();           Serial_AllSensor();
// SerialCalibrate_FrontSensor();   SerialCalibrate_BackSensor();    // ค่า calibrate
// SerialCalibrate_CenterSensor();  SerialCalibrate_AllSensor();
// SerialPositionF();   SerialPositionB();   SerialPositionFB();     // ตำแหน่งเส้น
// SerialDistance();                                                 // เซนเซอร์วัดระยะ
// Serial_SensorTable();            I2C_Scan();                      // พิมพ์ครั้งเดียว (ไม่ค้าง)


/* ==========================================================================
   [16] เสียง / ไฟ LED / I2C
   ========================================================================== */
// Beep(ms);          Beep2(freq, ms);     beep(freq, ms);     BeepScanner();
// BZon();            BZoff();             blink(n);           // blink = LED บนบอร์ด Pico
// SoundStartup();    SoundReady();        SoundSuccess();     SoundError();
//
// --- ไฟ LED 8 ดวง (PCF8574) ---
// LED_Write(mask);   LED_On();   LED_Off();   LED_Set(i, on);   LED_Toggle(i);
// LED_Bar(level 0-8);         LED_Number(n);              // แถบระดับ / เลขฐานสอง
// LED_Blink(times, ms);       LED_Chase(times, ms);       LED_ChaseBack(times, ms);
// LED_KnightRider(times, ms); LED_Fill(times, ms);        LED_Center(times, ms);
// LED_Alternate(times, ms);   LED_Random(times, ms);      LED_Ready();
// LED_ScannerUpdate(ms);      LED_BlinkUpdate(ms);        // แบบไม่บล็อก เรียกซ้ำใน loop
// LED_ShowLineF();            LED_ShowLineB();            // แสดงตำแหน่งเส้นบนไฟ (เรียกซ้ำใน loop)
// readADS1115(ch, raw);   readMCP3421(raw);   writePCF8574(v);
// writeEEPROM(0x50, addr, data, len);     readEEPROM(0x50, addr, buf, len);


/* ==========================================================================
   [17] ปุ่ม / เมนู sw()
   ========================================================================== */
// ระหว่างอยู่ในเมนู sw(): Serial แสดงค่าดิบทุก 100 ms, ไฟ LED แสดงระดับแบต
//   ปุ่ม Cal หน้า (GP3)          → Calibrate เซนเซอร์หน้า F (5 วินาที)
//   ปุ่ม ADCcal                  → Calibrate เซนเซอร์หลัง B
//   ปุ่ม START กดค้าง 3 วินาที    → Calibrate เซนเซอร์กลาง CL / CR
//   ปุ่ม START กดสั้น             → ออกตัว + ตั้งมุมไจโรเป็น 0 (resetAngles)
// ระหว่าง Calibrate: เสียงติ๊ก + ไฟวิ่ง ให้กวาดหุ่นผ่านเส้นดำ/พื้นขาวจนครบทุกตัว
// แบตวิกฤติ (<= 10.85 V): ไฟวิ่งไป-กลับ + เสียงถี่ และกด START จะไม่ยอมออกตัว
