#ifndef MYMINI_16PRO_I2C_H
#define MYMINI_16PRO_I2C_H

/*
 * MyMini_16Pro_BNO086_I2C.h
 * บัส I2C และอุปกรณ์บนบอร์ด: ADS1115, MCP3421, PCF8574, EEPROM CAT24C128
 *
 * สารบัญ:
 *   [1] ขา / แอดเดรส I2C
 *   [2] เริ่มต้นบัส / สแกนอุปกรณ์
 *   [3] ADS1115 (แบตเตอรี่ + เซนเซอร์กลาง CL / CR)
 *   [4] MCP3421 (ปุ่ม ADCcal สำหรับ Calibrate เซนเซอร์หลัง)
 *   [5] PCF8574 (ไฟ LED 8 ดวง)
 *   [6] EEPROM ภายนอก CAT24C128 (16 KB, หน้าละ 64 ไบต์)
 */

#include <Arduino.h>
#include <Wire.h>

// ======================================================================
// [1] ขา / แอดเดรส I2C
// ======================================================================

#define I2C_SDA_PIN 4
#define I2C_SCL_PIN 5
// ต้องใช้ 100 kHz: BNO086 ไม่ส่งข้อมูลมุมที่ 400 kHz (ทดสอบแล้วใน MyRP_Pico2_mini_BNO086)
#define I2C_CLOCK 100000

#define PCF8574_ADDR 0x20    // ไฟ LED 8 ดวง (HIGH = ติด)
#define ADS1115_ADDR 0x48    // AIN0 แบต, AIN1 CL, AIN2 CR, AIN3 สำรอง
#define EEPROM_ADDRESS 0x50  // CAT24C128 เก็บค่า Calibrate
#define MCP3421_ADDR 0x68    // ปุ่ม ADCcal (Calibrate เซนเซอร์หลัง)

#define ADS_BATTERY 0
#define ADS_CENTER_L 1
#define ADS_CENTER_R 2
#define ADS_AUX 3

#define EEPROM_MAX_ADDR 0x3FFF
#define EEPROM_PAGE_SIZE 64

// ======================================================================
// [2] เริ่มต้นบัส / สแกนอุปกรณ์
// ======================================================================

void I2C_Begin() {
  // ตั้งเป็น GPIO ชั่วคราวเพื่อ clock pulses
  pinMode(I2C_SDA_PIN, OUTPUT);
  pinMode(I2C_SCL_PIN, OUTPUT);
  digitalWrite(I2C_SDA_PIN, HIGH);
  digitalWrite(I2C_SCL_PIN, HIGH);
  delay(10);

  // ส่ง 10 clock pulses เพื่อเคลียร์ slave ที่อาจค้าง (เช่นรีเซ็ตกลางการอ่าน EEPROM)
  for (int i = 0; i < 10; i++) {
    digitalWrite(I2C_SCL_PIN, LOW);
    delayMicroseconds(5);
    digitalWrite(I2C_SCL_PIN, HIGH);
    delayMicroseconds(5);
  }

  // ปล่อย bus เป็น input (pull-up จะดึง HIGH)
  pinMode(I2C_SDA_PIN, INPUT);
  pinMode(I2C_SCL_PIN, INPUT);
  delay(10);

  Wire.setSDA(I2C_SDA_PIN);
  Wire.setSCL(I2C_SCL_PIN);
  Wire.begin();
  Wire.setClock(I2C_CLOCK);
  Wire.setTimeout(25, true);  // อุปกรณ์ค้าง → คืนค่าภายใน 25 ms และรีเซ็ตบัส
}

bool probeI2C(uint8_t address) {
  Wire.beginTransmission(address);
  return Wire.endTransmission() == 0;
}

void I2C_Scan() {
  Serial.println("I2C scan:");
  int found = 0;
  for (uint8_t address = 0x08; address <= 0x77; address++) {
    if (probeI2C(address)) {
      Serial.printf("  0x%02X\n", address);
      found++;
    }
  }
  Serial.printf("devices = %d\n", found);

  const uint8_t addresses[] = { PCF8574_ADDR, ADS1115_ADDR, EEPROM_ADDRESS, MCP3421_ADDR };
  const char *names[] = { "PCF8574 (LED)", "ADS1115 (BAT/CL/CR)", "CAT24C128 (EEPROM)", "MCP3421 (ADCcal)" };
  for (int i = 0; i < 4; i++) {
    Serial.printf("%-22s 0x%02X %s\n", names[i], addresses[i], probeI2C(addresses[i]) ? "OK" : "MISSING");
  }
}

// ======================================================================
// [3] ADS1115 (แบตเตอรี่ + เซนเซอร์กลาง CL / CR)
// ======================================================================

bool readADSRegister(uint8_t reg, uint16_t &value) {
  Wire.beginTransmission(ADS1115_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(static_cast<uint8_t>(ADS1115_ADDR), static_cast<uint8_t>(2)) != 2) return false;
  value = (uint16_t)Wire.read() << 8;
  value |= (uint8_t)Wire.read();
  return true;
}

// อ่านช่อง 0-3 แบบ single-shot: AINx-GND, ±4.096 V, 860 SPS (ใช้เวลาราว 1.5-2 ms)
// คืน false ถ้าอ่านไม่ได้ (raw คงค่าเดิม)
bool readADS1115(uint8_t channel, int16_t &raw) {
  if (channel > 3) return false;

  const uint16_t config = 0x8000U | ((0x04U + channel) << 12U) | 0x0200U | 0x0100U | 0x00E0U | 0x0003U;
  Wire.beginTransmission(ADS1115_ADDR);
  Wire.write((uint8_t)0x01);
  Wire.write((uint8_t)(config >> 8));
  Wire.write((uint8_t)(config & 0xFF));
  if (Wire.endTransmission() != 0) return false;

  // รอบิต OS = 1 (แปลงเสร็จ) ไม่เกิน 10 ms
  unsigned long started = millis();
  uint16_t current = 0;
  do {
    if (!readADSRegister(0x01, current)) return false;
    if (current & 0x8000U) break;
    delayMicroseconds(200);
  } while (millis() - started < 10);
  if ((current & 0x8000U) == 0) return false;

  uint16_t conversion = 0;
  if (!readADSRegister(0x00, conversion)) return false;
  raw = (int16_t)conversion;
  return true;
}

float adsToVolts(int16_t raw) {
  return raw * 4.096f / 32768.0f;
}

// ======================================================================
// [4] MCP3421 (ปุ่ม ADCcal สำหรับ Calibrate เซนเซอร์หลัง)
// ======================================================================

// 12-bit / 240 SPS / gain x1 → 1 LSB = 1 mV
bool readMCP3421(int16_t &raw) {
  Wire.beginTransmission(MCP3421_ADDR);
  Wire.write((uint8_t)0x10);  // continuous conversion
  if (Wire.endTransmission() != 0) return false;

  unsigned long started = millis();
  do {
    delayMicroseconds(500);
    if (Wire.requestFrom(static_cast<uint8_t>(MCP3421_ADDR), static_cast<uint8_t>(3)) != 3) return false;
    uint16_t data = ((uint16_t)Wire.read() << 8) | (uint8_t)Wire.read();
    uint8_t cfg = Wire.read();
    if ((cfg & 0x80U) == 0) {  // RDY = 0 → ได้ค่าใหม่
      raw = (int16_t)data;
      return true;
    }
  } while (millis() - started < 20);
  return false;
}

// อ่านค่าปุ่ม ADCcal — ใช้ในเมนู sw() (กดปุ่ม = ค่าต่ำกว่า CAL_B_THRESHOLD)
// คืนค่า 9999 หากไม่พบอุปกรณ์ เพื่อไม่ให้เข้าเงื่อนไขกดปุ่ม
int ADC_i2c() {
  int16_t raw = 0;
  if (!readMCP3421(raw)) return 9999;
  return raw;
}

// ======================================================================
// [5] PCF8574 (ไฟ LED 8 ดวง)
// ======================================================================

bool writePCF8574(uint8_t value) {
  Wire.beginTransmission(PCF8574_ADDR);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

// ======================================================================
// [6] EEPROM ภายนอก CAT24C128 (16 KB, หน้าละ 64 ไบต์)
// ======================================================================

// รอ EEPROM เขียนเสร็จ (ACK polling) ไม่เกิน timeoutMs
bool waitEEPROMReady(int deviceAddress, unsigned long timeoutMs) {
  unsigned long started = millis();
  do {
    if (probeI2C(deviceAddress)) return true;
  } while (millis() - started < timeoutMs);
  return false;
}

// เขียนทีละหน้า (64 ไบต์) ไม่ให้ข้อมูลวนทับต้นหน้า — คืน false ถ้าเขียนไม่สำเร็จ
bool writeEEPROM(int deviceAddress, unsigned int eeAddress, byte *data, int dataLength) {
  if (data == nullptr || dataLength < 0) return false;
  if (dataLength == 0) return true;
  if (eeAddress + dataLength - 1 > EEPROM_MAX_ADDR) return false;

  int offset = 0;
  while (offset < dataLength) {
    unsigned int addr = eeAddress + offset;
    int pageRemain = EEPROM_PAGE_SIZE - (addr % EEPROM_PAGE_SIZE);
    int chunk = min(dataLength - offset, pageRemain);

    Wire.beginTransmission(deviceAddress);
    Wire.write((uint8_t)(addr >> 8));
    Wire.write((uint8_t)(addr & 0xFF));
    if ((int)Wire.write(data + offset, chunk) != chunk) return false;
    if (Wire.endTransmission() != 0) return false;
    if (!waitEEPROMReady(deviceAddress, 20)) return false;
    offset += chunk;
  }
  return true;
}

// อ่านทีละ 32 ไบต์ — คืน false ถ้าอ่านไม่ครบ
bool readEEPROM(int deviceAddress, unsigned int eeAddress, byte *buffer, int dataLength) {
  if (buffer == nullptr || dataLength < 0) return false;
  if (dataLength == 0) return true;
  if (eeAddress + dataLength - 1 > EEPROM_MAX_ADDR) return false;

  int offset = 0;
  while (offset < dataLength) {
    unsigned int addr = eeAddress + offset;
    uint8_t chunk = (uint8_t)min(dataLength - offset, 32);

    Wire.beginTransmission(deviceAddress);
    Wire.write((uint8_t)(addr >> 8));
    Wire.write((uint8_t)(addr & 0xFF));
    if (Wire.endTransmission(false) != 0) return false;
    if (Wire.requestFrom(static_cast<uint8_t>(deviceAddress), chunk) != chunk) return false;
    for (int i = 0; i < chunk; i++) {
      buffer[offset + i] = Wire.read();
    }
    offset += chunk;
  }
  return true;
}

#endif  // MYMINI_16PRO_I2C_H
