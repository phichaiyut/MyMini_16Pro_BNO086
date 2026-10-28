//----------------------------------------------------------------------------->> ตั้งค่ามือจับ
// ขา: GP28 = แขนขึ้น-ลง, GP18 = มือซ้าย, GP22 = มือขวา (ค่าเริ่มต้นจาก arm.ino ของ MyMINI_16Pro)
int servo_down = 40;    //-------------------->> ตั้งค่า มือจับลงขนานกับพื้น
int servoL_open = 160;  //-------------------->> ตั้งค่า กางฝ่ามือด้านซ้าย
int servoR_open = 160;  //-------------------->> ตั้งค่า กางฝ่ามือด้านขวา
//----------------------------------------------------------------------------->> ตั้งค่ามือจับ

int readyL = 60, readyR = 60;          // ฟังก์ชันองศาของ arm_ready()  //--------->> แขนลง กางฝ่ามือออกเตรียมเข้าไปคีบ
int behindL = 15, behindR = 15;        // ฟังก์ชันองศาของ arm_behihd() //--------->> แขนลง เอาฝ่ามือมาข้างหลัง
int up = 90;                           // ยกแขนขึ้น
int closeL = 130, closeR = 130;        // หุบลูกเล็ก close
int closeBigL = 100, closeBigR = 100;  // หุบลูกใหญ่ big_box
int up45 = 45;

void arm_ready() {  //--------->> แขนลง กางฝ่ามือออกเตรียมเข้าไปคีบ
  MotorStop();
  armupdown(servo_down);
  arm_left_right(servoL_open - readyL, servoR_open - readyR);
}

void arm_open_down() {  //--------->>  กางฝ่ามือออก  และเอาแขนลง
  MotorStop();
  arm_left_right(servoL_open, servoR_open);
  delay(200);
  armupdown(servo_down);
  delay(50);
}

void arm_down_open() {  //--------->>  เอาแขนลง  และ กางฝ่ามือออก
  MotorStop();
  armupdown(servo_down);
  delay(200);
  arm_left_right(servoL_open, servoR_open);
  delay(50);
}

void arm_open_up() {  //--------->>  กางฝ่ามือออก  และยกแขนขึ้น
  MotorStop();
  arm_left_right(servoL_open, servoR_open);
  delay(200);
  armupdown(servo_down + up);
  delay(50);
}

void arm_up_open() {  //--------->>  เอาแขนขึ้น  และ กางฝ่ามือออก
  MotorStop();
  armupdown(servo_down + up);
  delay(200);
  arm_left_right(servoL_open, servoR_open);
  delay(50);
}

void arm_down_close() {  //--------->>  เอาแขนลง  และ หุบมือเข้า
  MotorStop();
  armupdown(servo_down);
  delay(200);
  arm_left_right(servoL_open - closeL, servoR_open - closeR);
  delay(50);
}

void arm_close_down() {  //--------->>  หุบมือเข้า  และ เอาแขนลง
  MotorStop();
  arm_left_right(servoL_open - closeL, servoR_open - closeR);
  delay(200);
  armupdown(servo_down);
  delay(50);
}

void arm_up_close() {  //--------->>  ยกแขนขึ้น  และ หุบมือเข้า
  MotorStop();
  armupdown(servo_down + up);
  delay(100);
  arm_left_right(servoL_open - closeL, servoR_open - closeR);
  delay(50);
}

void arm_close_up() {  //--------->>  หุบมือเข้า  และ ยกแขนขึ้น
  MotorStop();
  arm_left_right(servoL_open - closeL, servoR_open - closeR);
  delay(200);
  armupdown(servo_down + up);
  delay(100);
}

void arm_big_box() {  //--------->>  คีบกล่องใหญ่
  MotorStop();
  armupdown(servo_down);
  arm_left_right(servoL_open - closeBigL, servoR_open - closeBigR);
  delay(100);
}

void arm_big_box_up() {  //--------->>  คีบกล่องใหญ่ แล้วยกขึ้น
  MotorStop();
  arm_left_right(servoL_open - closeBigL, servoR_open - closeBigR);
  delay(200);
  armupdown(servo_down + up);
  delay(100);
}

void arm_behihd() {
  MotorStop();
  arm_left_right(servoL_open + behindL, servoR_open + behindR);
}

void arm_up() {
  MotorStop();
  armupdown(servo_down + up);
  delay(100);
}

void arm_up45() {
  MotorStop();
  armupdown(servo_down + up45);
  delay(100);
}

void arm_down() {
  MotorStop();
  armupdown(servo_down);
  delay(100);
}

void arm_open() {
  MotorStop();
  arm_left_right(servoL_open, servoR_open);
  delay(100);
}

void arm_close() {
  MotorStop();
  arm_left_right(servoL_open - closeL, servoR_open - closeR);
  delay(100);
}

void arm_big() {
  MotorStop();
  arm_left_right(servoL_open - closeBigL, servoR_open - closeBigR);
  delay(100);
}

void arm_open_l() {
  MotorStop();
  arm_left_right(servoL_open, servoR_open - closeR);
  delay(100);
}

void arm_open_r() {
  MotorStop();
  arm_left_right(servoL_open - closeL, servoR_open);
  delay(100);
}

// ปรับความเร็วเซอร์โว: spd = หน่วงกี่ ms ต่อ 1 องศา (เช่น arm_close(2);)

void arm_ready(int spd) {  //--------->> แขนลง กางฝ่ามือออกเตรียมเข้าไปคีบ
  armupdown(servo_down, spd);
  arm_left_right(servoL_open - readyL, servoR_open - readyR, spd);
}

void arm_open_down(int spd) {  //--------->>  กางฝ่ามือออก  และเอาแขนลง
  arm_left_right(servoL_open, servoR_open, spd);
  armupdown(servo_down, spd);
}

void arm_down_open(int spd) {  //--------->>  เอาแขนลง  และ กางฝ่ามือออก
  armupdown(servo_down, spd);
  arm_left_right(servoL_open, servoR_open, spd);
}

void arm_open_up(int spd) {  //--------->>  กางฝ่ามือออก  และยกแขนขึ้น
  arm_left_right(servoL_open, servoR_open, spd);
  armupdown(servo_down + up, spd);
}

void arm_up_open(int spd) {  //--------->>  เอาแขนขึ้น  และ กางฝ่ามือออก
  armupdown(servo_down + up, spd);
  arm_left_right(servoL_open, servoR_open, spd);
}

void arm_down_close(int spd) {  //--------->>  เอาแขนลง  และ หุบมือเข้า
  armupdown(servo_down, spd);
  arm_left_right(servoL_open - closeL, servoR_open - closeR, spd);
}

void arm_close_down(int spd) {  //--------->>  หุบมือเข้า  และ เอาแขนลง
  arm_left_right(servoL_open - closeL, servoR_open - closeR, spd);
  armupdown(servo_down, spd);
}

void arm_up_close(int spd) {  //--------->>  ยกแขนขึ้น  และ หุบมือเข้า
  armupdown(servo_down + up, spd);
  arm_left_right(servoL_open - closeL, servoR_open - closeR, spd);
}

void arm_close_up(int spd) {  //--------->>  หุบมือเข้า  และ ยกแขนขึ้น
  arm_left_right(servoL_open - closeL, servoR_open - closeR, spd);
  armupdown(servo_down + up, spd);
}

void arm_big_box(int spd) {  //--------->>  คีบกล่องใหญ่
  armupdown(servo_down, spd);
  arm_left_right(servoL_open - closeBigL, servoR_open - closeBigR, spd);
}

void arm_big_box_up(int spd) {  //--------->>  คีบกล่องใหญ่ แล้วยกขึ้น
  arm_left_right(servoL_open - closeBigL, servoR_open - closeBigR, spd);
  armupdown(servo_down + up, spd);
}

void arm_behihd(int spd) {
  arm_left_right(servoL_open + behindL, servoR_open + behindR, spd);
}

void arm_up(int spd) {
  armupdown(servo_down + up, spd);
}

void arm_up45(int spd) {
  armupdown(servo_down + up45, spd);
}

void arm_down(int spd) {
  armupdown(servo_down, spd);
}

void arm_open(int spd) {
  arm_left_right(servoL_open, servoR_open, spd);
}

void arm_close(int spd) {
  arm_left_right(servoL_open - closeL, servoR_open - closeR, spd);
}

void arm_big(int spd) {
  arm_left_right(servoL_open - closeBigL, servoR_open - closeBigR, spd);
}

void arm_open_l(int spd) {
  arm_left_right(servoL_open, servoR_open - closeR, spd);
}

void arm_open_r(int spd) {
  arm_left_right(servoL_open - closeL, servoR_open, spd);
}
