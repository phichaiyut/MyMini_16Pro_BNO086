// MotorTest — ทดสอบล้อทีละข้าง (ยกล้อให้ลอยก่อน!)
// เปิด Serial Monitor 115200 แล้วกด START (GP2) เพื่อไปขั้นถัดไปทีละขั้น
//
// ส่วน A: ผ่านไลบรารี Motor()   — ซ้ายหน้า / ซ้ายถอย / ขวาหน้า / ขวาถอย
// ส่วน B: สั่งขาตรง ๆ ไม่ผ่านไลบรารี (ล้อซ้าย GP19/20/21) + ล้าง fault ด้วย nSLEEP ก่อน
//
// อ่านผล:
//   A ซ้ายไม่หมุน แต่ B หมุน        → ปัญหาในโค้ดไลบรารี (ส่งผลกลับมา)
//   A และ B ซ้ายไม่หมุนทั้งคู่       → ฮาร์ดแวร์ (โมดูล DRV8874 / สาย / มอเตอร์)
//   A ซ้ายหมุนได้หลังรัน B แล้ว      → DRV8874 ฝั่งซ้าย latch fault ค้าง → ลองเปิด DRV_FAULT_CLEAR

// #define DRV_FAULT_CLEAR 1   // เปิดตัวล้าง fault (ทดสอบรอบที่ 2)
#include <MyMini_16Pro_BNO086.h>

#define TEST_SPEED 25    // %
#define TEST_MS 1000

void printPins(const char *label, uint8_t en, uint8_t ph, uint8_t slp) {
  Serial.printf("  %s  EN GP%d / PH GP%d=%d / nSLEEP GP%d=%d\n", label, en, ph, digitalRead(ph), slp, digitalRead(slp));
}

// รันหนึ่งขั้น: แสดงชื่อ → รอกด START → หมุน 1 วินาที (พิมพ์ระดับขาระหว่างหมุน) → เบรก
void step(const char *name, int l, int r) {
  Serial.printf("\n>> %s  Motor(%d, %d) — กด START\n", name, l, r);
  swSTART();
  Motor(l, r);
  delay(50);
  printPins("LEFT ", DRV_L_EN, DRV_L_PH, DRV_L_SLEEP);
  printPins("RIGHT", DRV_R_EN, DRV_R_PH, DRV_R_SLEEP);
  delay(TEST_MS - 50);
  MotorStop(500);
  Serial.println("  ล้อไหนหมุน? ทิศอะไร? (มองจากด้านบน หัวหุ่นชี้ไปข้างหน้า)");
}

// ล้อซ้ายแบบไม่ผ่านไลบรารี: ล้าง fault → หน้า 1 วินาที → ถอย 1 วินาที → ปล่อย
void rawLeftTest() {
  Serial.println("\n>> B: ล้อซ้ายสั่งขาตรง (GP19 PWM / GP20 PH / GP21 nSLEEP) — กด START");
  swSTART();
  analogWrite(DRV_L_EN, 0);
  digitalWrite(DRV_L_SLEEP, LOW);  // หลับ = ล้าง fault ที่ latch ค้าง
  delay(2);
  digitalWrite(DRV_L_SLEEP, HIGH);  // ตื่น
  delay(2);
  int pwm = map(TEST_SPEED, 0, 100, 0, MOTOR_PWM_MAX);

  digitalWrite(DRV_L_PH, HIGH);
  analogWrite(DRV_L_EN, pwm);
  Serial.println("  B1 ซ้ายเดินหน้า (PH HIGH)");
  delay(TEST_MS);

  analogWrite(DRV_L_EN, 0);
  delay(200);
  digitalWrite(DRV_L_PH, LOW);
  analogWrite(DRV_L_EN, pwm);
  Serial.println("  B2 ซ้ายถอยหลัง (PH LOW)");
  delay(TEST_MS);

  analogWrite(DRV_L_EN, 0);
  digitalWrite(DRV_L_SLEEP, LOW);
  drvSleepL = true;  // บอกไลบรารีว่าฝั่งซ้ายหลับแล้ว
  drvDriveL = false;
  Serial.println("  จบส่วน B");
}

void setup() {
  RobotSetup();  // ไม่เรียก sw() / ไจโร เพื่อตัดตัวแปรอื่นออก
  Serial.printf("Driver: %s, DRV_FAULT_CLEAR=%d\n", MotorDriverType == DRIVER_TB6612 ? "TB6612" : "DRV8874", DRV_FAULT_CLEAR);

  step("A1 ซ้ายเดินหน้า", TEST_SPEED, 0);
  step("A2 ซ้ายถอยหลัง", -TEST_SPEED, 0);
  step("A3 ขวาเดินหน้า", 0, TEST_SPEED);
  step("A4 ขวาถอยหลัง", 0, -TEST_SPEED);

  rawLeftTest();

  step("A5 ซ้ายเดินหน้า (ซ้ำหลังส่วน B)", TEST_SPEED, 0);
  MotorCoast();
  Serial.println("\nจบการทดสอบ");
}

void loop() {}
