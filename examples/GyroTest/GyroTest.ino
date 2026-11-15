/*
 * GyroTest.ino — ทดสอบ BNO086 บน MyMINI 16Pro แล้วบอกว่าปัญหาอยู่ตรงไหน
 *
 * วิธีใช้: อัปโหลด → เปิด Serial Monitor 115200 (เลือก Newline หรือ No line ending ก็ได้)
 *   a = ทดสอบ 1-3 รวดเดียว (ไม่หมุนมอเตอร์)
 *   1 = สแกน I2C + สถานะ IMU
 *   2 = ทดสอบทิศมุม: ใช้มือหมุนหุ่นไปทางขวา
 *   3 = วางนิ่ง 5 วินาที: วัดอัตราข้อมูล + มุมไหล
 *   4 = วางหุ่นบนพื้น ให้หุ่นหมุนขวา 90° เอง (มอเตอร์หมุน! ต้องเสียบแบต)
 *   ? = แสดงเมนู
 *
 * ถ้าผลบอกให้กลับทิศมุม ให้เพิ่มบรรทัดนี้ไว้ "บนสุด" ของ sketch หลัก ก่อน #include:
 *   #define MYMINI_YAW_SIGN 1
 */

#include <MyMini_16Pro_BNO086.h>

// ผลทดสอบ 2: 0 = ยังไม่ทดสอบ, 1 = ทิศถูก, -1 = ทิศกลับ, 2 = มุมไม่เปลี่ยน
int signResult = 0;

// ---------- เครื่องมือ ----------

void flushInput() {
  delay(20);
  while (Serial.available()) Serial.read();
}

// รอให้พิมพ์ตัวอักษร (ข้าม Enter) ไม่เกิน timeoutMs; คืน 0 ถ้าหมดเวลา
char waitKey(unsigned long timeoutMs) {
  unsigned long t0 = millis();
  while (millis() - t0 < timeoutMs) {
    if (Serial.available()) {
      char c = Serial.read();
      if (c != '\r' && c != '\n' && c != ' ') return c;
    }
    delay(5);
  }
  return 0;
}

bool imuFresh() {
  return imuReady && millis() - lastImuEventMs <= IMU_STALE_MS;
}

void printLine() {
  Serial.println("------------------------------------------------------------");
}

void printMenu() {
  printLine();
  Serial.println("GyroTest  |  a = ทดสอบ 1-3  |  1 สแกน  |  2 ทิศมุม  |  3 นิ่ง/อัตรา  |  4 หมุนจริง  |  ? เมนู");
  Serial.printf("ตั้งค่าตอนนี้: YAW_SIGN = %d, รายงาน = %s\n", YAW_SIGN,
                MYMINI_BNO_USE_MAG ? "Rotation Vector" : "Game Rotation Vector");
  printLine();
}

// ---------- ทดสอบ 1: I2C + สถานะ ----------

void test1() {
  printLine();
  Serial.println("[1] สแกน I2C + สถานะ IMU");
  I2C_Scan();
  bool at4A = probeI2C(0x4A), at4B = probeI2C(0x4B);
  Serial.printf("BNO08x 0x4A: %s | 0x4B: %s\n", at4A ? "OK" : "-", at4B ? "OK" : "-");
  Serial.printf("imuReady = %s, รีเซ็ตระหว่างทำงาน = %u ครั้ง, แบต = %.2f V\n",
                imuReady ? "true" : "false", imuResetCount, getBatteryVoltage());

  if (!at4A && !at4B) {
    Serial.println("=> ไม่เจอ BNO08x บนบัสเลย: ตรวจสาย SDA=GP4 / SCL=GP5 / 3V3 / GND, ตัว pull-up และขา PS0/PS1 (ต้องเป็นโหมด I2C)");
  } else if (!imuReady) {
    Serial.println("=> เจออุปกรณ์แต่เริ่มทำงานไม่ได้: ลองเริ่มใหม่...");
    if (gyro_Begin(3000)) Serial.println("=> เริ่มใหม่สำเร็จ (ตอนบูตอาจเปิดเร็วเกินไป / ไฟเลี้ยงยังไม่นิ่ง)");
    else Serial.println("=> ยังไม่ได้: สงสัยไฟเลี้ยง IMU หรือสายสัญญาณไม่ดี");
  } else {
    Serial.println("=> IMU พร้อม");
  }
}

// ---------- ทดสอบ 2: ทิศมุม (หมุนด้วยมือ) ----------

void test2() {
  printLine();
  Serial.println("[2] ทดสอบทิศมุม");
  if (!imuReady) {
    Serial.println("=> ข้าม: IMU ไม่พร้อม (ดูผลทดสอบ 1)");
    signResult = 0;
    return;
  }
  Serial.println("วางหุ่นนิ่ง ๆ ... (รีเซ็ตมุมเป็น 0)");
  delay(1000);
  resetAngles();
  Serial.println("ได้ยินเสียงบี๊บแล้ว ใช้มือหมุนหุ่นไปทาง \"ขวา\" (ตามเข็มนาฬิกา มองจากด้านบน) ประมาณ 90° ภายใน 6 วินาที");
  Beep(150);

  float peak = 0;
  unsigned long t0 = millis(), lastPrint = 0;
  while (millis() - t0 < 6000) {
    angleRead();
    if (fabsf(yawgyro) > fabsf(peak)) peak = yawgyro;
    if (millis() - lastPrint >= 500) {
      lastPrint = millis();
      Serial.printf("  มุม = %7.1f\n", yawgyro);
    }
  }
  Beep(150);
  Serial.printf("มุมที่หมุนได้มากที่สุด = %.1f°\n", peak);

  if (!imuFresh()) {
    signResult = 2;
    Serial.println("=> IMU ไม่ส่งค่ามุมใหม่ระหว่างทดสอบ (ค้าง): ดูสาย / ไฟเลี้ยง IMU");
  } else if (peak > 30) {
    signResult = 1;
    Serial.println("=> ทิศถูกต้อง (หมุนขวา = บวก)");
  } else if (peak < -30) {
    signResult = -1;
    Serial.println("=> ทิศกลับด้าน! นี่คือสาเหตุที่หุ่นหมุนไม่หยุด/วิ่งเบี้ยว");
    Serial.printf("   แก้: เพิ่ม  #define MYMINI_YAW_SIGN %d  ไว้บนสุดของ sketch หลัก ก่อน #include\n", -YAW_SIGN);
  } else {
    signResult = 2;
    Serial.println("=> มุมแทบไม่เปลี่ยน: ถ้าหมุนจริงแล้ว แปลว่า BNO086 ติดตั้งแนวตั้ง/เอียง (แกน yaw ไม่ตรงกับแกนหมุนหุ่น)");
    Serial.println("   หรือหมุนไม่ทัน 6 วินาที — ลองใหม่ด้วยคำสั่ง 2");
  }
}

// ---------- ทดสอบ 3: อัตราข้อมูล + มุมไหล (วางนิ่ง) ----------

void test3() {
  printLine();
  Serial.println("[3] วางหุ่นนิ่ง ๆ 5 วินาที: วัดอัตราข้อมูลและมุมไหล");
  if (!imuReady) {
    Serial.println("=> ข้าม: IMU ไม่พร้อม");
    return;
  }
  delay(500);
  angleReadFresh();
  float startYaw = yawgyro;
  uint16_t resetsBefore = imuResetCount;
  unsigned long count = 0, prevEvent = lastImuEventMs, maxGap = 0, lastSeen = millis();
  unsigned long t0 = millis();
  while (millis() - t0 < 5000) {
    angleRead();
    if (lastImuEventMs != prevEvent) {
      count++;
      unsigned long gap = lastImuEventMs - lastSeen;
      if (gap > maxGap) maxGap = gap;
      lastSeen = lastImuEventMs;
      prevEvent = lastImuEventMs;
    }
  }
  if (millis() - lastSeen > maxGap) maxGap = millis() - lastSeen;
  float rate = count / 5.0f;
  float drift = yawgyro - startYaw;
  Serial.printf("อัตราข้อมูล ≈ %.0f ครั้ง/วินาที (ควรใกล้ 200), ช่วงเงียบนานสุด = %lu ms\n", rate, maxGap);
  Serial.printf("มุมไหลใน 5 วินาที = %.2f°, รีเซ็ตระหว่างทดสอบ = %u ครั้ง\n", drift, imuResetCount - resetsBefore);

  bool ok = true;
  if (rate < 50) {
    ok = false;
    Serial.println("=> ข้อมูลมาช้าผิดปกติ: บัส I2C มีปัญหา (สายยาว/pull-up ไม่พอ) หรือ IMU รีเซ็ตบ่อย");
  }
  if (maxGap > IMU_STALE_MS) {
    ok = false;
    Serial.println("=> มีช่วงที่ IMU เงียบนานเกินไป: คำสั่งหมุนจะถูกยกเลิกด้วย 'gyro: no data'");
  }
  if (imuResetCount != resetsBefore) {
    ok = false;
    Serial.println("=> IMU รีเซ็ตตัวเองทั้งที่ยังไม่หมุนมอเตอร์: ไฟเลี้ยง IMU ไม่นิ่ง");
  }
  if (fabsf(drift) > 1.0f) {
    ok = false;
    Serial.println("=> มุมไหลมากกว่า 1° ขณะนิ่ง: ตรวจว่าหุ่นนิ่งจริง หรือ IMU ยังคาลิเบรตไม่เสร็จ (รอสักครู่แล้วลองใหม่)");
  }
  if (ok) Serial.println("=> ปกติ");
}

// ---------- ทดสอบ 4: หมุนจริงด้วยมอเตอร์ ----------

void test4() {
  printLine();
  Serial.println("[4] หุ่นจะหมุนขวา 90° อยู่กับที่ด้วยมอเตอร์");
  if (!imuReady) {
    Serial.println("=> ข้าม: IMU ไม่พร้อม");
    return;
  }
  readBatteryVoltage();
  if (getBatteryVoltage() < 6.0f) {
    Serial.println("=> ข้าม: ไม่ได้เสียบแบต (มอเตอร์จะไม่หมุน)");
    return;
  }
  Serial.println("วางหุ่นบนพื้น เว้นที่รอบตัว (ไม่ต้องยกล้อ เพราะหุ่นต้องหมุนจริงไจโรถึงจะเห็น)");
  Serial.println("พิมพ์ y เพื่อเริ่ม (อย่างอื่น = ยกเลิก)");
  flushInput();
  if (waitKey(30000) != 'y') {
    Serial.println("ยกเลิก");
    return;
  }
  delay(1000);
  resetAngles();
  uint16_t resetsBefore = imuResetCount;
  unsigned long t0 = millis();
  spinrg(90);
  unsigned long dt = millis() - t0;
  MotorStop();
  delay(300);
  angleReadFresh();
  float turned = yawgyro;
  bool timedOut = dt >= gyro_TurnTimeoutMs;
  bool stalled = !imuFresh();
  uint16_t resets = imuResetCount - resetsBefore;

  Serial.printf("ใช้เวลา %lu ms, หมุนได้ %.1f° (เป้า 90°), IMU รีเซ็ต %u ครั้ง\n", dt, turned, resets);

  if (resets > 0) {
    Serial.println("=> IMU รีเซ็ตตอนมอเตอร์ทำงาน: ไฟตกตอนมอเตอร์กระชาก");
    Serial.println("   โค้ดใหม่เปิดรายงานกลับให้เองแล้ว แต่ควรเพิ่มตัวเก็บประจุ (เช่น 10-100 uF) ที่ไฟเลี้ยงโมดูล IMU");
  }
  if (stalled) {
    Serial.println("=> IMU หยุดส่งค่ามุมหลังมอเตอร์ทำงาน: ปัญหาไฟเลี้ยง/สัญญาณรบกวนจากมอเตอร์");
  } else if (fabsf(turned - 90) <= 10) {
    Serial.printf("=> ผ่าน: คลาดเคลื่อน %.1f° (ถ้าเกินเป้าเยอะ ปรับ SetGyroSpin / set_brake_spin_gyro)\n", turned - 90);
  } else if (turned < -30) {
    if (signResult == 1) {
      Serial.println("=> ไจโรทิศถูก (ทดสอบ 2 ผ่าน) แต่หุ่นหมุนผิดทาง: ทิศมอเตอร์กลับ → แก้ set_MotorInvert()");
    } else {
      Serial.println("=> หุ่นหมุนผิดทาง: ทำทดสอบ 2 ก่อนเพื่อแยกว่าเป็นทิศไจโร หรือทิศมอเตอร์");
    }
  } else if (fabsf(turned) < 10) {
    Serial.println("=> ไจโรแทบไม่เห็นการหมุน: หุ่นไม่ได้หมุนจริง (มอเตอร์/แบต) หรือ BNO086 ติดตั้งแนวตั้ง/เอียง");
  } else if (timedOut) {
    Serial.println("=> หมุนไม่ถึงเป้าภายในเวลา: แรงมอเตอร์ไม่พอ (เพิ่ม min speed ใน SetGyroSpin) หรือทิศไจโรกลับ");
  } else {
    Serial.println("=> หมุนถึงแต่คลาดเคลื่อนมาก: ปรับค่า SetGyroSpin");
  }
}

// ---------- setup / loop ----------

void setup() {
  Serial.begin(115200);
  // รอเปิด Serial Monitor (ไม่เกิน 8 วินาที) จะได้เห็นข้อความตอนเริ่ม IMU
  unsigned long t0 = millis();
  while (!Serial && millis() - t0 < 8000) delay(10);
  delay(300);

  RobotSetup();

  // ตั้งค่ามอเตอร์ให้ตรงกับ Setting() ของ sketch หลัก (แก้ให้ตรงถ้าเปลี่ยนไว้)
  set_MotorDriver("DRV8874");
  set_Freq("Coreless_Motors");
  set_MotorInvert(false, false);
  set_BatteryCompensate(true);
  // ค่าหมุนตัวเดียวกับ Setting() ของ sketch หลัก
  SetGyroSpin(1.2, 0.25, 50, 8, 25.0, 1.0);
  set_brake_spin_gyro_setg(5);
  set_brake_spin_gyro(12, 5);
  MotorStop();

  printMenu();
}

void loop() {
  char c = waitKey(100);
  if (c == 0) return;
  flushInput();
  switch (c) {
    case 'a':
    case 'A':
      test1();
      test2();
      test3();
      printLine();
      Serial.println("เสร็จ 1-3 | ถ้าผ่าน ให้ลอง 4 (หุ่นหมุนจริงบนพื้น)");
      break;
    case '1': test1(); break;
    case '2': test2(); break;
    case '3': test3(); break;
    case '4': test4(); break;
    default: printMenu(); break;
  }
}
