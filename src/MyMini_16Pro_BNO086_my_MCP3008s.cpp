
#include "MyMini_16Pro_BNO086_my_MCP3008s.h"
#include <SPI.h>
#if MCP3008S_FAST_GPIO
#include <hardware/gpio.h>
#include <hardware/clocks.h>

// แปลง ns → จำนวนรอบ clk_sys (ปัดขึ้น ให้ได้เวลาอย่างน้อยเท่าที่ขอ)
static uint32_t nsToCycles(uint32_t ns) {
  uint64_t hz = clock_get_hz(clk_sys);
  return (uint32_t)((hz * ns + 999999999ULL) / 1000000000ULL);
}
#endif

bool my_MCP3008s::begin(uint8_t cs_, SPIClass* theSPI) {
  hwSPI = true;

  this->cs = cs_;

  pinMode(this->cs, OUTPUT);
  digitalWrite(this->cs, HIGH);
  _spi = theSPI;
  _spi->begin();
  /* SPI.begin(); */
  return true;
}

bool my_MCP3008s::begin(uint8_t sck_, uint8_t mosi_, uint8_t miso_,
                        uint8_t cs_) {
  hwSPI = false;

  this->sck = sck_;
  this->mosi = mosi_;
  this->miso = miso_;
  this->cs = cs_;

  pinMode(this->sck, OUTPUT);
  pinMode(this->mosi, OUTPUT);
  pinMode(this->miso, INPUT);
  pinMode(this->cs, OUTPUT);

  digitalWrite(this->sck, LOW);
  digitalWrite(this->mosi, LOW);
  digitalWrite(this->cs, HIGH);

#if MCP3008S_FAST_GPIO
  cycSetup = nsToCycles(MCP3008S_SETUP_NS);
  cycHigh = nsToCycles(MCP3008S_HIGH_NS);
  cycSettle = nsToCycles(MCP3008S_SETTLE_NS);
  cycCsIdle = nsToCycles(MCP3008S_CS_IDLE_NS);
#endif

  return true;
}

// สลับชิป CS โดยไม่ตั้งค่า pinMode/digitalWrite ซ้ำ ใช้เมื่อ pin ถูก begin() ไว้แล้ว
// (ต้องตั้ง pinMode(cs, OUTPUT) และ digitalWrite(cs, HIGH) ให้ทุกขา CS ที่จะสลับใช้ไว้ล่วงหน้า)
void my_MCP3008s::setCS(uint8_t cs_) {
  this->cs = cs_;
}

int my_MCP3008s::readADC(uint8_t channel) {
  if ((channel < 0) || (channel > 7))
    return -1;
  return SPIxADC(channel, false);
}

int my_MCP3008s::readADCDifference(uint8_t differential) {
  if ((differential < 0) || (differential > 7))
    return -1;
  return SPIxADC(differential, true);
}

// SPI transfer for ADC read
int my_MCP3008s::SPIxADC(uint8_t channel, bool differential) {
  byte command, sgldiff;

  if (differential) {
    sgldiff = 0;
  } else {
    sgldiff = 1;
  }

  command = ((0x01 << 7) |              // start bit
             (sgldiff << 6) |           // single or differential
             ((channel & 0x07) << 3));  // channel number

  if (hwSPI) {
    byte b0, b1, b2;

    _spi->beginTransaction(
      SPISettings(MCP3008s_SPI_MAX, MCP3008s_SPI_ORDER, MCP3008s_SPI_MODE));
    digitalWrite(cs, LOW);

    b0 = _spi->transfer(command);
    b1 = _spi->transfer(0x00);
    b2 = _spi->transfer(0x00);

    digitalWrite(cs, HIGH);
    _spi->endTransaction();

    return 0x3FF & ((b0 & 0x01) << 9 | (b1 & 0xFF) << 1 | (b2 & 0x80) >> 7);

  } else {

    uint16_t outBuffer, inBuffer = 0;

    // 5 command bits + 1 null bit + 10 data bits = 16 bits
    outBuffer = command << 8;

#if MCP3008S_FAST_GPIO
    // ลำดับต่อบิตเหมือนแบบเดิมทุกอย่าง: ตั้ง MOSI → SCK ขึ้น → SCK ลง → อ่าน MISO
    // ต่างแค่เขียนขาตรง ๆ และหน่วงเวลาตามสเปกแทนการพึ่งความช้าของ digitalWrite()
    // MCP3008 เลื่อนบิตออกตอน SCK ขาลง จึงต้องรอ cycSettle ก่อนอ่าน (ไม่งั้นได้บิตก่อนหน้า)
    gpio_put(cs, 0);
    for (int c = 0; c < 16; c++) {
      gpio_put(mosi, (outBuffer >> (15 - c)) & 0x01);
      busy_wait_at_least_cycles(cycSetup);
      gpio_put(sck, 1);
      busy_wait_at_least_cycles(cycHigh);
      gpio_put(sck, 0);
      busy_wait_at_least_cycles(cycSettle);
      inBuffer <<= 1;
      if (gpio_get(miso))
        inBuffer |= 0x01;
    }
    gpio_put(cs, 1);
    busy_wait_at_least_cycles(cycCsIdle);  // เว้น CS HIGH ก่อนช่องถัดไป (tCSH)

    return inBuffer & 0x3FF;
#else
    digitalWrite(cs, LOW);

    for (int c = 0; c < 16; c++) {
      digitalWrite(mosi, (outBuffer >> (15 - c)) & 0x01);
      digitalWrite(sck, HIGH);
      digitalWrite(sck, LOW);
      inBuffer <<= 1;
      if (digitalRead(miso))
        inBuffer |= 0x01;
    }

    digitalWrite(cs, HIGH);

    return inBuffer & 0x3FF;
#endif
  }
}
