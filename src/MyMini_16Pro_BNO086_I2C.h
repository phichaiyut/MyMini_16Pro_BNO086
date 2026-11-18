#ifndef MYMINI16PRO_BNO086_I2C_H
#define MYMINI16PRO_BNO086_I2C_H

/*
 * MyMini_16Pro_BNO086_I2C.h
 * บัส I2C (Wire บน GP4/GP5) และอุปกรณ์บนบอร์ด — ตรรกะจาก I2CDevices.cpp ของ MyMINI_16Pro
 *
 * แอดเดรสบนบัสเดียวกัน: 0x20 PCF8574, 0x48 ADS1115, 0x50 CAT24C128, 0x68 MCP3421,
 * 0x4B/0x4A BNO086 → ไม่มีแอดเดรสชนกัน
 * ห้ามใช้ Wire1: ต้นฉบับผูก Wire1 กับ GP26/GP27 ซึ่งเป็นขาสัญญาณ MUX หลัง/หน้า
 *
 * สารบัญ:
 *   [1] ขา / แอดเดรส / ช่อง ADS1115
 *   [2] เริ่มต้นบัส / สแกนอุปกรณ์
 *   [3] ADS1115 (แบต + เซนเซอร์กลาง CL / CR + AUX)
 *   [4] MCP3421 (ปุ่ม ADCcal สำหรับ Calibrate เซนเซอร์หลัง)
 *   [5] PCF8574 (ไฟ LED 8 ดวง)
 *   [6] EEPROM CAT24C128 (เก็บค่า Calibrate)
 */

#include <Arduino.h>
#include <Wire.h>

// ======================================================================
// [1] ขา / แอดเดรส / ช่อง ADS1115
// ======================================================================

#define I2C_SDA_PIN 4       // PIN_I2C_SDA
#define I2C_SCL_PIN 5       // PIN_I2C_SCL
#define I2C_CLOCK 100000    // I2C_CLOCK_HZ (BNO086 ต้องใช้ 100 kHz)
#define I2C_TIMEOUT_MS 25   // bus.setTimeout(25, true)

#define PCF8574_ADDR 0x20
#define ADS1115_ADDR 0x48
#define EEPROM_ADDR 0x50    // CAT24C128
#define MCP3421_ADDR 0x68

#define ADS_BATTERY 0       // AIN0 แบตผ่านตัวแบ่ง 20k / 6.8k
#define ADS_CENTER_L 1      // AIN1 เซนเซอร์กลางซ้าย
#define ADS_CENTER_R 2      // AIN2 เซนเซอร์กลางขวา
#define ADS_AUX 3           // AIN3 สำรอง (net adc_F)
#define ADS_FULL_SCALE 4.096f

#define EEPROM_MAX_SAFE_ADDR 0x3FFE  // 0x3FFF สงวนให้ทดสอบ EEPROM ของต้นฉบับ
#define EEPROM_PAGE_SIZE 64
#define EEPROM_READ_CHUNK 32

// ======================================================================
// [2] เริ่มต้นบัส / สแกนอุปกรณ์
// ======================================================================

void I2C_Begin() {
  Wire.setSDA(I2C_SDA_PIN);
  Wire.setSCL(I2C_SCL_PIN);
  Wire.begin();
  Wire.setClock(I2C_CLOCK);
  Wire.setTimeout(I2C_TIMEOUT_MS, true);
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
  Serial.printf("devices=%d\n", found);
}

// ======================================================================
// [3] ADS1115 (แบต + เซนเซอร์กลาง CL / CR + AUX)
// ======================================================================

bool readADSRegister(uint8_t reg, uint16_t &value) {
  Wire.beginTransmission(ADS1115_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((uint8_t)ADS1115_ADDR, (uint8_t)2) != 2) return false;
  value = (uint16_t)Wire.read() << 8;
  value |= (uint8_t)Wire.read();
  return true;
}

// single-shot, AINx-GND, ±4.096 V, 860 SPS, ปิด comparator — รอแปลงเสร็จไม่เกิน 10 ms
bool readADS1115(uint8_t channel, int16_t &raw) {
  if (channel > 3) return false;
  const uint16_t config = 0x8000U | ((0x04U + channel) << 12U) | 0x0200U | 0x0100U | 0x00E0U | 0x0003U;

  Wire.beginTransmission(ADS1115_ADDR);
  Wire.write((uint8_t)0x01);
  Wire.write((uint8_t)(config >> 8));
  Wire.write((uint8_t)(config & 0xFF));
  if (Wire.endTransmission() != 0) return false;

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
  return raw * ADS_FULL_SCALE / 32768.0f;
}

// ======================================================================
// [4] MCP3421 (ปุ่ม ADCcal สำหรับ Calibrate เซนเซอร์หลัง)
// ======================================================================

// continuous, 12 บิต / 240 SPS, gain x1 (1 LSB = 1 mV) — รอค่าใหม่ไม่เกิน 20 ms
bool readMCP3421(int16_t &raw) {
  Wire.beginTransmission(MCP3421_ADDR);
  Wire.write((uint8_t)0x10);
  if (Wire.endTransmission() != 0) return false;

  unsigned long started = millis();
  do {
    delayMicroseconds(500);
    if (Wire.requestFrom((uint8_t)MCP3421_ADDR, (uint8_t)3) != 3) return false;
    uint16_t data = ((uint16_t)Wire.read() << 8) | (uint8_t)Wire.read();
    uint8_t cfg = Wire.read();
    if ((cfg & 0x80U) == 0) {
      raw = (int16_t)data;
      return true;
    }
  } while (millis() - started < 20);
  return false;
}

// ======================================================================
// [5] PCF8574 (ไฟ LED 8 ดวง — HIGH = ติด)
// ======================================================================

bool LED_Write(uint8_t mask) {
  Wire.beginTransmission(PCF8574_ADDR);
  Wire.write(mask);
  return Wire.endTransmission() == 0;
}

void LED_Off() {
  LED_Write(0x00);
}

// ======================================================================
// [6] EEPROM CAT24C128 (เก็บค่า Calibrate)
// ======================================================================

bool waitEEPROMReady(unsigned long timeoutMs) {
  unsigned long started = millis();
  do {
    if (probeI2C(EEPROM_ADDR)) return true;
  } while (millis() - started < timeoutMs);
  return false;
}

// อ่านทีละ 32 ไบต์
bool readEEPROM(uint16_t address, uint8_t *buffer, uint16_t length) {
  if (buffer == nullptr) return false;
  if (length == 0) return true;
  if ((uint32_t)address + length - 1U > EEPROM_MAX_SAFE_ADDR) return false;

  uint16_t offset = 0;
  while (offset < length) {
    uint16_t remaining = length - offset;
    uint8_t chunk = remaining > EEPROM_READ_CHUNK ? EEPROM_READ_CHUNK : remaining;
    uint16_t addr = address + offset;

    Wire.beginTransmission(EEPROM_ADDR);
    Wire.write((uint8_t)(addr >> 8));
    Wire.write((uint8_t)(addr & 0xFF));
    if (Wire.endTransmission(false) != 0) return false;
    if (Wire.requestFrom((uint8_t)EEPROM_ADDR, chunk) != chunk) return false;
    for (uint8_t i = 0; i < chunk; i++) buffer[offset + i] = Wire.read();
    offset += chunk;
  }
  return true;
}

// เขียนไม่ข้ามหน้า 64 ไบต์ + รอเขียนเสร็จ (ACK polling) หน้าละไม่เกิน 20 ms
bool writeEEPROM(uint16_t address, const uint8_t *buffer, uint16_t length) {
  if (buffer == nullptr) return false;
  if (length == 0) return true;
  if ((uint32_t)address + length - 1U > EEPROM_MAX_SAFE_ADDR) return false;

  uint16_t offset = 0;
  while (offset < length) {
    uint16_t addr = address + offset;
    uint8_t pageRemain = EEPROM_PAGE_SIZE - (addr & (EEPROM_PAGE_SIZE - 1));
    uint16_t remaining = length - offset;
    uint8_t chunk = remaining < pageRemain ? remaining : pageRemain;

    Wire.beginTransmission(EEPROM_ADDR);
    Wire.write((uint8_t)(addr >> 8));
    Wire.write((uint8_t)(addr & 0xFF));
    if (Wire.write(buffer + offset, chunk) != chunk) return false;
    if (Wire.endTransmission() != 0) return false;
    if (!waitEEPROMReady(20)) return false;
    offset += chunk;
  }
  return true;
}

#endif  // MYMINI16PRO_BNO086_I2C_H
