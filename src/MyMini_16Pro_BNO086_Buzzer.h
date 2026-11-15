#ifndef MYMINI_16PRO_BUZZER_H
#define MYMINI_16PRO_BUZZER_H

/*
 * MyMini_16Pro_BNO086_Buzzer.h
 * เสียง buzzer และไฟกระพริบ LED บนบอร์ด Pico
 *
 * สารบัญ:
 *   [1] ขาและความถี่เสียง
 *   [2] ฟังก์ชันเสียง / ไฟ (Beep, BZon, BZoff, blink)
 *   [3] เมโลดี้สำเร็จรูป (เปิดเครื่อง / พร้อม / สำเร็จ / ผิดพลาด)
 */

#include <Arduino.h>

// ======================================================================
// [1] ขาและความถี่เสียง
// ======================================================================

#define ToNe 3000
#define buzzer_pin 9

// ======================================================================
// [2] ฟังก์ชันเสียง / ไฟ (Beep, BZon, BZoff, blink)
// ======================================================================

void Beep(int delayb) {
  tone(buzzer_pin, ToNe);
  delay(delayb);
  noTone(buzzer_pin);
}

void Beep2(int freq, int delayb) {
  tone(buzzer_pin, freq);
  delay(delayb);
  noTone(buzzer_pin);
}

void BZon() {
  tone(buzzer_pin, ToNe);
}

void BZoff() {
  noTone(buzzer_pin);
}

void beep(int freq, int dur) {
  tone(buzzer_pin, freq, dur);
  delay(dur + 50);
}

// เสียง "ปี๊บ" สั้น แหลม คล้ายเครื่องยิงบาร์โค้ดใน supermarket
void BeepScanner() {
  beep(ToNe, 80);
}

// กระพริบ LED บนบอร์ด Pico (LED_BUILTIN) — ไฟ 8 ดวงบน PCF8574 ดูที่ LED.h
void blink(int times) {
  for (int i = 0; i < times; i++) {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(150);
    digitalWrite(LED_BUILTIN, LOW);
    delay(150);
  }
}

// ======================================================================
// [3] เมโลดี้สำเร็จรูป (เปิดเครื่อง / พร้อม / สำเร็จ / ผิดพลาด)
// ======================================================================

// เสียงเปิดเครื่อง โด-มี-ซอล-โด (จาก robot_begin() ของ MyMINI_16Pro)
void SoundStartup() {
  const int notes[] = { 1047, 1319, 1568, 2093 };
  const int durs[] = { 90, 90, 90, 150 };
  for (int i = 0; i < 4; i++) {
    tone(buzzer_pin, notes[i]);
    delay(durs[i]);
    noTone(buzzer_pin);
    delay(30);
  }
}

// เสียงไจโรพร้อมใช้งาน (สามโน้ตขึ้น)
void SoundReady() {
  const int notes[] = { 2093, 2637, 3136 };
  for (int i = 0; i < 3; i++) {
    tone(buzzer_pin, notes[i], 65);
    delay(80);
  }
  noTone(buzzer_pin);
}

// บันทึก / คาลิเบรตสำเร็จ: ปี๊บยาวสองครั้ง
void SoundSuccess() {
  beep(3500, 250);
  delay(100);
  beep(3500, 250);
}

// ผิดพลาด: เสียงต่ำยาว
void SoundError() {
  Beep2(800, 200);
  delay(80);
  Beep2(500, 400);
}

#endif  // MYMINI_16PRO_BUZZER_H
