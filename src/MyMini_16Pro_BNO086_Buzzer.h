#ifndef MYMINI16PRO_BNO086_BUZZER_H
#define MYMINI16PRO_BNO086_BUZZER_H

/*
 * MyMini_16Pro_BNO086_Buzzer.h
 * เสียง buzzer (GP9) — ความถี่และจังหวะทั้งหมดยกมาจาก MyMINI_16Pro
 *
 * ใช้ tone() / noTone() แบบต้นฉบับเท่านั้น
 * ห้ามใช้ analogWrite() กับ buzzer: analogWriteFreq() / analogWriteRange()
 * ของ arduino-pico เป็นค่ารวมทั้งชิป จะไปเปลี่ยนความถี่ / ความละเอียด PWM ของมอเตอร์
 * (tone() ของ arduino-pico ใช้ PIO ไม่แตะ PWM slice ของมอเตอร์)
 *
 * สารบัญ:
 *   [1] ขาและความถี่เสียง
 *   [2] ฟังก์ชันเสียงพื้นฐาน (Beep, Beep2, BZon, BZoff, beep, BeepScanner, blink)
 *   [3] เสียงสำเร็จรูปจากต้นฉบับ
 */

#include <Arduino.h>

// ======================================================================
// [1] ขาและความถี่เสียง
// ======================================================================

#define BUZZER_PIN 9      // MyMINIConfig::PIN_BUZZER
#define ToNe 3500         // เสียง START ของ wait_button() ต้นฉบับ
#define TONE_CAL_TICK 3000  // CALIBRATION_TONE_HZ
#define TONE_COMPLETE 3500  // COMPLETE_TONE_HZ

// ======================================================================
// [2] ฟังก์ชันเสียงพื้นฐาน (Beep, Beep2, BZon, BZoff)
// ======================================================================

void BuzzerBegin() {
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);
}

void Beep(int delayb) {
  tone(BUZZER_PIN, ToNe);
  delay(delayb);
  noTone(BUZZER_PIN);
}

void Beep2(int freq, int delayb) {
  tone(BUZZER_PIN, freq);
  delay(delayb);
  noTone(BUZZER_PIN);
}

void BZon() {
  tone(BUZZER_PIN, ToNe);
}

void BZon(int freq) {
  tone(BUZZER_PIN, freq);
}

void BZoff() {
  noTone(BUZZER_PIN);
}

// ---------- ชื่อจาก MyRP_Pico2_mini_BNO086_TEST (ขา buzzer เดียวกัน GP9) ----------

#define buzzer_pin BUZZER_PIN

// เสียงความถี่ freq ยาว dur ms แล้วรอให้จบ (+50 ms)
void beep(int freq, int dur) {
  tone(BUZZER_PIN, freq, dur);
  delay(dur + 50);
}

void BeepScanner() {
  beep(ToNe, 80);
}

// กระพริบ LED บนบอร์ด (GP25 / LED_BUILTIN) times ครั้ง
void blink(int times) {
  pinMode(LED_BUILTIN, OUTPUT);
  for (int i = 0; i < times; i++) {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(150);
    digitalWrite(LED_BUILTIN, LOW);
    delay(150);
  }
}

// ======================================================================
// [3] เสียงสำเร็จรูปจากต้นฉบับ
// ======================================================================

// เปิดเครื่อง โด-มี-ซอล-โด (robot_begin)
void SoundStartup() {
  const int notes[] = { 1047, 1319, 1568, 2093 };
  const int durs[] = { 90, 90, 90, 150 };
  for (int i = 0; i < 4; i++) {
    tone(BUZZER_PIN, notes[i]);
    delay(durs[i]);
    noTone(BUZZER_PIN);
    if (i < 3) delay(30);
  }
}

// เข้าเมนูรอปุ่ม: 3500 Hz 120 ms × 2 เว้น 100 ms (startWaitButtonSignal)
void SoundWait() {
  Beep2(TONE_COMPLETE, 120);
  delay(100);
  Beep2(TONE_COMPLETE, 120);
}

// กด START: 3500 Hz 400 ms แล้วรอให้ครบ 500 ms (wait_button)
void SoundStart() {
  noTone(BUZZER_PIN);
  tone(BUZZER_PIN, ToNe, 400);
  delay(500);
}

// Calibrate สำเร็จ: 3500 Hz 250 ms × 2 เว้น 150 ms
void SoundSuccess() {
  Beep2(TONE_COMPLETE, 250);
  delay(150);
  Beep2(TONE_COMPLETE, 250);
}

// ไจโรพร้อม: 2093 / 2637 / 3136 Hz ยาว 65 ms ทุก 80 ms (playGyroReadyMelody)
void SoundGyroReady() {
  const int notes[] = { 2093, 2637, 3136 };
  for (int i = 0; i < 3; i++) {
    tone(BUZZER_PIN, notes[i], 65);
    delay(80);
  }
  noTone(BUZZER_PIN);
}

#endif  // MYMINI16PRO_BNO086_BUZZER_H
