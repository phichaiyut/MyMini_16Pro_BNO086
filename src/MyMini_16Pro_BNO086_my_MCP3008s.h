#ifndef MYMINI16PRO_BNO086_MY_MCP3008S_H
#define MYMINI16PRO_BNO086_MY_MCP3008S_H

#include <Arduino.h>
#include <SPI.h>

#define MCP3008s_SPI_MAX_5V 3600000           ///< SPI MAX Value on 5V pin
#define MCP3008s_SPI_MAX_3V 1350000           ///< SPI MAX Value on 3V pin
#define MCP3008s_SPI_MAX MCP3008s_SPI_MAX_3V  ///< SPI MAX Value
#define MCP3008s_SPI_ORDER MSBFIRST           ///<  SPI ORDER
#define MCP3008s_SPI_MODE SPI_MODE0           ///< SPI MODE

// ---------- software SPI แบบเร็ว (RP2040 / RP2350) ----------
// 1 = ใช้ gpio_put()/gpio_get() ตรง ๆ + หน่วงเวลาตามสเปก MCP3008 (เร็วกว่าเดิม)
//     เดิม digitalWrite() ของ arduino-pico เรียก gpio_set_function() ทุกครั้ง จึงช้า
// 0 = กลับไปใช้ digitalWrite()/digitalRead() แบบเดิม (ถ้าค่าเซนเซอร์ผิดเพี้ยนหลังอัปเดต)
//     ปิดได้โดยแก้ค่าบรรทัด #define MCP3008S_FAST_GPIO ด้านล่างเป็น 0
#ifndef MCP3008S_FAST_GPIO
#if defined(ARDUINO_ARCH_RP2040)
#define MCP3008S_FAST_GPIO 1
#else
#define MCP3008S_FAST_GPIO 0
#endif
#endif

// เวลาแต่ละช่วงของ 1 บิต (ns) — รวม ~800ns ≈ 1.25 MHz (สเปก MCP3008 ที่ 2.7V = 1.35 MHz)
// ค่าเซนเซอร์กระโดด / ผิดเพี้ยน → เพิ่มค่า (เช่น 500) | ห้ามลดจนรอบรวมต่ำกว่า ~740ns
#define MCP3008S_SETUP_NS 100   // ตั้ง MOSI / CS ก่อน SCK ขาขึ้น (tSU, tSUCS)
#define MCP3008S_HIGH_NS 350    // SCK ค้าง HIGH (tHI)
#define MCP3008S_SETTLE_NS 350  // หลัง SCK ขาลง รอ MISO นิ่งก่อนอ่าน (tDO)
#define MCP3008S_CS_IDLE_NS 500 // CS HIGH ค้างก่อนแปลงค่าช่องถัดไป (tCSH ขั้นต่ำ 270ns)

class my_MCP3008s {
public:
  bool begin(uint8_t cs = SS, SPIClass* theSPI = &SPI);
  bool begin(uint8_t sck, uint8_t mosi, uint8_t miso, uint8_t cs);
  void setCS(uint8_t cs);
  int readADC(uint8_t channel);
  int readADCDifference(uint8_t differential);

private:
  uint8_t cs;
  uint8_t mosi;
  uint8_t miso;
  uint8_t sck;
  bool hwSPI;
  // จำนวนรอบนาฬิกา CPU ของแต่ละช่วง (คำนวณจาก clk_sys จริงใน begin())
  uint32_t cycSetup = 0, cycHigh = 0, cycSettle = 0, cycCsIdle = 0;
  int SPIxADC(uint8_t channel, bool differential);
  SPIClass* _spi;
};

#endif  // MYMINI16PRO_BNO086_MY_MCP3008S_H
