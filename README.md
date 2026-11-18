# MyMini_16Pro_BNO086

ไลบรารี Arduino สำหรับหุ่นยนต์ **MyMINI 16Pro** (บอร์ด RP2350 / Raspberry Pi Pico 2) ที่ใช้ IMU **BNO086**

- เซนเซอร์เส้น 16 + 16 ช่อง (MUX หน้า / หลัง) และเซนเซอร์กลาง ADS1115
- เก็บค่าคาลิเบรตใน EEPROM CAT24C128 (สำรองใน flash)
- มอเตอร์ DRV8874 / TB6612, บัซเซอร์, ไฟ LED (PCF8574), วัดแบตเตอรี่
- แขนเซอร์โว
- เดินตามเส้นด้วย PID พร้อมคำสั่งเจอแยก, เลี้ยว / วิ่งตรงด้วยไจโร BNO08x

## ติดตั้ง

1. ติดตั้งบอร์ด **Raspberry Pi Pico/RP2040/RP2350** (Earle Philhower core) แล้วเลือกบอร์ด `Raspberry Pi Pico 2`
2. ติดตั้งไลบรารี **SparkFun BNO08x Cortex Based IMU** จาก Library Manager
3. คัดลอกโฟลเดอร์นี้ไปไว้ที่ `Documents/Arduino/libraries/MyMini_16Pro_BNO086`
   หรือใช้ `git clone https://github.com/phichaiyut/MyMini_16Pro_BNO086.git`

## โครงสร้างไฟล์

```
MyMini_16Pro_BNO086/
├── library.properties
├── keywords.txt
├── src/
│   ├── MyMini_16Pro_BNO086.h          ไฟล์หลัก: RobotSetup() + GyroReset() + เมนูปุ่ม sw() / swOK()
│   ├── MyMini_16Pro_BNO086_Decl.h     ประกาศล่วงหน้าฟังก์ชันที่เรียกข้ามไฟล์
│   ├── MyMini_16Pro_BNO086_Battery.h  วัดแรงดันแบต + ไฟระดับแบต / เสียงเตือน
│   ├── MyMini_16Pro_BNO086_Button.h   ปุ่ม START / Calibrate / ADCcal
│   ├── MyMini_16Pro_BNO086_Buzzer.h   เสียง (tone() บน GP9)
│   ├── MyMini_16Pro_BNO086_Gyro.h     BNO086: เลี้ยว / หมุน / วิ่งตรง / ทิศสัมบูรณ์ด้วยไจโร
│   ├── MyMini_16Pro_BNO086_I2C.h      I2C, EEPROM CAT24C128, ADS1115, MCP3421, LED PCF8574
│   ├── MyMini_16Pro_BNO086_Motor.h    มอเตอร์ DRV8874 / TB6612 (ทิศล้อ: DRV_L_INVERT / DRV_R_INVERT)
│   ├── MyMini_16Pro_BNO086_PID.h      เดินตามเส้น PID + คำสั่งเจอแยก
│   ├── MyMini_16Pro_BNO086_Sensor.h   อ่าน / คาลิเบรตเซนเซอร์เส้น (MUX 16+16)
│   ├── MyMini_16Pro_BNO086_Servo.h    เซอร์โว GP18 / GP22 / GP28 / GP0 / GP1
│   └── MyMini_16Pro_BNO086_my_MCP3008s.*  ไดรเวอร์ MCP3008 (ใช้เมื่อต่อโมดูลเพิ่ม)
└── examples/
    ├── MyMini_16Pro_BNO086/           โปรแกรมหลัก (Setting / Mission / Servo / Manual)
    ├── GyroTest/                      ทดสอบทิศมุมและการเลี้ยวของ BNO086
    └── MotorTest/                     ทดสอบมอเตอร์ทีละล้อ
```

## เริ่มใช้งาน

เปิด `File > Examples > MyMini_16Pro_BNO086 > MyMini_16Pro_BNO086` แล้วแก้ไขแท็บต่าง ๆ

| แท็บ | ใช้ทำอะไร |
|------|-----------|
| `Setting.ino` | ตั้งค่าหุ่น: ความเร็ว, Kp/Kd ต่อความเร็ว (`SetKpKd()` / `SetKpKdBack()`), ไจโร |
| `Mission.ino` | เขียนภารกิจ (ลำดับคำสั่งเดิน) |
| `Servo.ino`   | ท่าทางแขน เช่น `arm_up_open()` |
| `Manual.ino`  | คู่มือคำสั่งทั้งหมด |

```cpp
#include <MyMini_16Pro_BNO086.h>

void setup() {
  RobotSetup();    // ตั้งศูนย์ไจโร 0 องศาตอนเปิดเครื่อง (หุ่นต้องนิ่ง)
  Setting();
  sw();            // รอกดปุ่ม START (ระหว่างรอคาลิเบรตได้) — ไม่ตั้งศูนย์ไจโรซ้ำ
  // SetRobotAngle(); / resetAngles();  // ใส่เองถ้าต้องการให้มุม 0 = ทิศตอนกด START
  Mission();
  MotorStop();
}

void loop() {}
```

## คำสั่งที่ใช้บ่อย

| คำสั่ง | ความหมาย |
|--------|----------|
| `ff(50, 'p')` | เดินตามเส้นจนเจอแยก แล้ววิ่งผ่าน |
| `ffc(50, 'l')` | เจอแยกตัด → เข้ากลาง + หมุนซ้าย |
| `ffr(50, 'r')` | เจอแยกขวา → เข้ากลาง + หมุนขวา |
| `ffcm(60, 30)` | เดินตามเส้น 30 cm |
| `bb(40, 's')` / `bbc(50, 'l')` | ถอยตามเส้น (เหมือน `ff` / `ffc`) |
| `ffcmg(60, 50, 's')` | วิ่งตรงด้วยไจโร 50 cm แล้วหยุด |
| `spindegree(90)` | หมุนอยู่กับที่ 90 องศา |
| `turndegree(-90)` | เลี้ยวล้อเดียวไปทางซ้าย 90 องศา |

ดูคำสั่งทั้งหมดใน [`examples/MyMini_16Pro_BNO086/Manual.ino`](examples/MyMini_16Pro_BNO086/Manual.ino)

## ตรวจสอบหุ่น

- `TestMotorDirection()`: ล้อซ้ายต้องหมุนไปข้างหน้าก่อน (ถ้าผิดแก้ `DRV_L_INVERT` / `DRV_R_INVERT` ใน `src/MyMini_16Pro_BNO086_Motor.h`)
- `SerialPositionF()`: เส้นอยู่ทางซ้ายต้องอ่านได้ต่ำกว่า 7500
- `SerialLoopTimeF()`: ดูเวลาต่อรอบของลูป PID
- ตัวอย่าง `GyroTest`: ตรวจทิศมุม ถ้ามุมกลับทิศให้ใส่ `#define MYMINI_YAW_SIGN 1` ไว้บนสุดของ sketch ก่อน `#include`

## ผู้พัฒนา

Phichaiyut Deesungnern
