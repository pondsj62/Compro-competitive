// Maze SideGap - วิ่งต่อเนื่อง เจอช่องข้างแล้วเลี้ยวเข้า (ตามไอเดีย: ข้างว่างเกิน 1 บล็อก -> เลี้ยว
//                แล้วเดินไป 1 บล็อกโดยไม่สนข้าง สนแค่หน้า)
//
// การทำงาน:
//   วิ่งตรงด้วย IMU (PD) ไปเรื่อยๆ (ผนังสองข้าง = ทางเดินปกติ)
//   "จุดตัดสินใจ" = ข้างใดข้างหนึ่งว่าง (วิ่งเข้ากลางช่องก่อน) หรือ หน้าตัน
//   ที่จุดตัดสินใจ: ใช้ ROUTE (ฮาร์ดโค้ด) ถ้ามี ไม่งั้นใช้กฎ (เกาะซ้าย):
//     หน้าโล่ง: ซ้ายว่างข้างเดียว -> เลี้ยวซ้าย | ว่างทั้งสองข้าง หรือ ขวาว่างข้างเดียว -> ตรงไป
//     หน้าตัน : ซ้ายว่าง -> ซ้าย | ขวาว่าง -> ขวา | ตันหมด -> กลับรถ
//   ทำแล้วเดิน 1 บล็อก "ไม่สนข้าง" ผ่านช่องนั้นไปก่อน
//   IR เจอเส้นดำ -> วิ่งเข้าไปอีกนิดแล้วหยุดถาวร
//
// ปัญหาที่แก้ไว้แล้ว:
//   - เซนเซอร์ข้างเห็นช่องก่อนตัวรถถึงกลางช่อง -> มี ADVANCE ก่อนหมุน (ไม่งั้นชนมุม)
//   - "เดิน 1 บล็อก" วัดจากระยะหน้าที่ลดลง ไม่ใช่เวลา (แบตอ่อนก็ไม่เพี้ยน) หน้าไกลเกินค่อยใช้เวลา
//   - หลังเลี้ยวไม่สนข้าง 1 บล็อก -> ไม่เห็นช่องเดิม/ทางที่เพิ่งมาแล้วเลี้ยวซ้ำ
//   - ค่ากระโดด: median 3 ครั้ง + ต้องเจอติดกันหลายครั้ง
//   - หมุน/เดินมี timeout, รถติด (ระยะหน้าไม่ลด) -> ถอยแล้วตัดสินใจใหม่
//
// เลือกโหมดตอนเปิดเครื่อง (ไม่ต้องใช้สวิตช์) ภายใน 3 วิแรก:
//   บังเซนเซอร์ซ้าย = กฎมือซ้าย (ล้อกระตุก 1 ครั้ง) | บังขวา = กฎมือขวา (กระตุก 2 ครั้ง)
//   บังหน้า = โหมดวัดระยะ (มอเตอร์ไม่หมุน พิมพ์ค่าใน Serial) | ไม่บัง = DEFAULT_FOLLOW_LEFT
#include <Wire.h>

// ---------- ขา ----------
const int ENA = 10, IN1 = 9,  IN2 = 8;    // มอเตอร์ซ้าย (ต่อจริงที่ ENB/IN3/IN4 ของไดรเวอร์)
const int ENB = 11, IN3 = 12, IN4 = 13;   // มอเตอร์ขวา (ต่อจริงที่ ENA/IN1/IN2 ของไดรเวอร์)

const int TRIG_F = 4, ECHO_F = 5;   // หน้า
const int TRIG_L = 6, ECHO_L = 7;   // ซ้าย
const int TRIG_R = 2, ECHO_R = 3;   // ขวา

const int IR_PIN = A0;
const int LINE = HIGH;              // เส้นดำบนพื้นขาว = HIGH

// ---------- ความเร็ว (ค่าเดียวกับ Maze_Grid) ----------
const float SPEED_SCALE = 0.45;     // ความเร็ววิ่งตรงรวม (ลดตรงนี้ที่เดียว ถ้ายังเร็วไป)
const int LEFT_BASE  = 165 * SPEED_SCALE;
const int RIGHT_BASE = 115 * SPEED_SCALE;
const float SLOW_RATIO = 0.85;      // ชะลอตอนใกล้ผนังหน้า (ต่ำไปล้อขวาจะไม่หมุน)
const float GAP_RATIO  = 0.85;      // ชะลอตอนเห็นช่องข้าง/วิ่งเข้ากลางช่อง

// ---------- หมุนตัวเอง (ล้อหนึ่งหน้า อีกล้อถอย) ----------
//   ล้อซ้ายแรงน้อยกว่า -> คูณ TURN_BALANCE ให้หมุนรอบจุดกลางรถ ไม่ใช่รอบล้อข้างเดียว
//   ถ้าหมุนแล้วตัวรถเลื่อนไปทางซ้าย/ขวา (ไม่อยู่กับที่) ให้ปรับ TURN_BALANCE
const int   TURN_FAST = 80;        // ความเร็วล้อขวาตอนหมุน (ไกลเป้า)
const int   TURN_SLOW = 80;         // ความเร็วล้อขวาตอนใกล้เป้า (ต่ำไปจะหมุนไม่ไหว)
const float TURN_BALANCE = 1.43;    // ล้อซ้าย = ล้อขวา x ค่านี้ (165/115)
const float TURN_SLOW_DEG = 30;     // เหลืออีกกี่องศาเริ่มหมุนช้า
const float TURN_TOL_DEG = 2.5;     // หมุนถึงแค่นี้ถือว่าครบ
const int   BRAKE_MS = 40;          // เบรกด้วยการหมุนสวนสั้นๆ กันเลยมุม
// มุมที่สั่งหมุน: ถ้าเลี้ยวเกินทุกครั้งให้ลด (เช่น 90 -> 85), เลี้ยวไม่ถึงให้เพิ่ม (จูนซ้าย/ขวาแยกกัน)
const float TURN_DEG_LEFT  = 80;
const float TURN_DEG_RIGHT = 80;
const float TURN_DEG_BACK  = 180;
const int   STOP_BRAKE_MS = 30;     // เบรกด้วยการถอยสั้นๆ ตอนหยุด กันไหลเลยช่อง (0 = ปิด)

// ---------- เส้นทางฮาร์ดโค้ด (ผสมกับกฎ) ----------
// ทุกครั้งที่รถเจอ "จุดตัดสินใจ" (ข้างใดข้างหนึ่งว่าง หรือ หน้าตัน) จะใช้ตัวอักษรถัดไปในนี้
//   'L' = เลี้ยวซ้าย  'R' = เลี้ยวขวา  'S' = ตรงไป  'U' = กลับรถ  'A' = ใช้กฎอัตโนมัติ
// เกินความยาว หรือ สั่งไปทางที่ตัน -> ใช้กฎอัตโนมัติแทน
// ดูลำดับจุดได้จาก Serial (J0, J1, ...) ตอนลองวิ่ง แล้วค่อยแก้เฉพาะจุดที่รถเลือกผิด
// ตัวอย่าง: "AALAS"  -> จุด 0,1 ใช้กฎ, จุด 2 เลี้ยวซ้าย, จุด 3 ใช้กฎ, จุด 4 ตรงไป
const char ROUTE[] = "";

// ---------- เส้นทางทั้งสนาม ทีละช่อง (โหมด PATH: ไม่บังเซนเซอร์ตอนเปิดเครื่อง) ----------
// ทิศตามแผนที่ (มองจากด้านบน): N = ขึ้น  E = ขวา  S = ลง  W = ซ้าย   1 ตัวอักษร = 1 ช่อง
// สนามน้ำเงิน Start1(A) มุมขวาบน -> Parking น้ำเงิน (วางรถหันหน้าไปทาง W = ไปทาง Start2)
//   (c=คอลัมน์จากซ้ายของฝั่งน้ำเงิน 0-4, r=แถวจากบน 0-4)
//   A(c4,r0) W W -> (c2,r0) S -> (c2,r1) E -> (c3,r1) S -> (c3,r2) W -> (c2,r2) S -> (c2,r3)
//   E -> (c3,r3) S -> (c3,r4) W W -> (c1,r4) N N -> (c1,r2) W -> (c0,r2) S S -> Parking (c0,r4)
const char PATH[] = "WWSESWSESWWNNWSS";
const char START_DIR = 'W';         // รถหันหน้าทิศไหนตอนวางที่จุดเริ่ม
const float GYRO_SIGN = 1.0;

// ---------- คุมให้วิ่งตรง ----------
const float KP = 6.0, KD = 0.4;
const int   MAX_FIX = 50;
const float DEADBAND_DEG = 1.5;     // เอียงน้อยกว่านี้ไม่แก้
const float BIG_TILT_DEG = 5.0;     // เอียงเกินนี้ช่วยดันแรงขึ้น
const float BIG_TILT_GAIN = 2.0;    // 0 = ปิด
const float KP_CENTER = 1.5;        // มีผนังสองข้าง -> ดันให้อยู่กลางทาง

// ---------- ระยะ (cm) *** วัดจากรถจริงด้วยโหมดวัดระยะ ----------
const int CELL_CM = 20;             // ขนาดบล็อก
const int SIDE_OPEN_CM  = 18;       // ข้างไกลกว่านี้ = ว่าง (เกิน ~1 บล็อก) ค่าตอนมีผนังมักแค่ 3-8
const int FRONT_CENTER_CM = 5;      // ระยะหน้าตอนรถอยู่กลางช่อง มีผนังในช่องเดียวกัน
const int FRONT_STOP_CM = 7;        // หน้าใกล้กว่านี้ = ตัน หยุดตัดสินใจ (FRONT_CENTER_CM + 2)
const int FRONT_OPEN_CM = 15;       // ตอนตัดสินใจ หน้าไกลกว่านี้ = ทางว่าง
const int SLOW_CM = 15;             // หน้าใกล้กว่านี้ = ชะลอ
const int ADVANCE_CM = 4;           // เห็นช่องข้างแล้ววิ่งต่อเท่านี้ก่อนหมุน *** จูน
                                    //   เลยช่องไป = ลด (0 ได้), เลี้ยวแล้วชนมุมด้านหลัง = เพิ่ม
const int FRONT_FAR_CM = 90;        // หน้าไกลกว่านี้วัดระยะไม่แม่น -> ใช้เวลาแทน
const int MIN_VALID_CM = 2;
const int GAP_CONFIRM = 2;          // เห็นช่องข้างติดกันกี่ครั้งถึงจะเชื่อ
const int STOP_CONFIRM = 2;         // เห็นหน้าตันติดกันกี่ครั้งถึงจะหยุด

// ---------- เวลา (ms) ----------
const unsigned long CELL_MS = 750;        // เวลาวิ่ง 1 บล็อก (ใช้เมื่อหน้าไกล) *** จูน
const unsigned long STUCK_MS = 1200;      // ระยะหน้าไม่ลดนานเท่านี้ = รถติด
const unsigned long FINISH_MS = 300;
const unsigned long START_DELAY = 2000;

const bool DEFAULT_FOLLOW_LEFT = true;

// ---------- ตัวแปร ----------
bool followLeft = DEFAULT_FOLLOW_LEFT;
bool usePath = false;
float yaw = 0, rate = 0, gyroBias = 0;
float targetYaw = 0;
unsigned long lastTime;
float front = 999, left = 999, right = 999;
float histF[3] = {999, 999, 999}, histL[3] = {999, 999, 999}, histR[3] = {999, 999, 999};
int histIdx = 0;
int irCount = 0, gapCount = 0, stopCount = 0;
int junction = 0;                   // นับจุดตัดสินใจ (ใช้กับ ROUTE)

// =====================================================================
//  มอเตอร์
// =====================================================================
void motor(int l, int r) {
  l = constrain(l, -255, 255);
  r = constrain(r, -255, 255);
  digitalWrite(IN1, l > 0); digitalWrite(IN2, l < 0);
  digitalWrite(IN3, r > 0); digitalWrite(IN4, r < 0);
  analogWrite(ENA, abs(l));
  analogWrite(ENB, abs(r));
}

void twitch(int times) {
  for (int i = 0; i < times; i++) {
    motor(LEFT_BASE, RIGHT_BASE);   delay(70);
    motor(-LEFT_BASE, -RIGHT_BASE); delay(70);
    motor(0, 0);                    delay(300);
  }
}

// =====================================================================
//  IMU
// =====================================================================
int readGyroZ() {
  Wire.beginTransmission(0x68);
  Wire.write(0x47);
  Wire.endTransmission(false);
  Wire.requestFrom(0x68, 2);
  int hi = Wire.read();
  int lo = Wire.read();
  return (int16_t)(hi << 8 | lo);
}

void updateYaw() {
  unsigned long now = micros();
  float dt = (now - lastTime) / 1000000.0;
  lastTime = now;
  rate = (readGyroZ() - gyroBias) / 65.5 * GYRO_SIGN;   // + = หมุนซ้าย
  yaw += rate * dt;
}

// หมุนตัวเองอยู่กับที่: ล้อซ้ายกับขวาหมุนสวนกัน ไกลเป้าหมุนเร็ว ใกล้เป้าหมุนช้า แล้วเบรก
void spin(int dir, int speedR) {           // dir +1 = หมุนซ้าย, -1 = หมุนขวา
  int speedL = speedR * TURN_BALANCE;
  motor(-dir * speedL, dir * speedR);
}

void turnTo(float target) {
  motor(0, 0);
  delay(150);                               // ให้รถหยุดสนิทก่อนหมุน
  unsigned long start = millis();
  int lastDir = 0;
  while (millis() - start < 4000) {         // กันหมุนค้างถ้า IMU มีปัญหา
    updateYaw();
    float error = target - yaw;
    if (abs(error) < TURN_TOL_DEG) break;
    int dir = (error > 0) ? 1 : -1;
    spin(dir, abs(error) > TURN_SLOW_DEG ? TURN_FAST : TURN_SLOW);
    lastDir = dir;
    delay(5);
  }
  if (lastDir != 0) { spin(-lastDir, TURN_FAST); delay(BRAKE_MS); }   // เบรก
  motor(0, 0);
  delay(150);
  updateYaw();

  // เลยมุม/ไม่ถึงมุม -> แก้ทีละนิด
  for (int i = 0; i < 10; i++) {
    updateYaw();
    float error = target - yaw;
    if (abs(error) < TURN_TOL_DEG) break;
    spin(error > 0 ? 1 : -1, TURN_SLOW);
    delay(40);
    motor(0, 0);
    delay(80);
  }
  motor(0, 0);
  targetYaw = target;
  delay(100);
  updateYaw();
}

// มีผนังหน้า -> ขยับเข้า/ออกให้ระยะหน้า = FRONT_CENTER_CM (ให้รถอยู่กลางช่องก่อนหมุน รถใหญ่จะไม่ชน)
void alignFront() {
  readSensorsStill();
  if (front > FRONT_OPEN_CM) return;
  unsigned long start = millis();
  while (millis() - start < 1000) {
    readSensors();
    float err = front - FRONT_CENTER_CM;
    if (abs(err) < 1.5) break;
    if (err > 0) motor(LEFT_BASE, RIGHT_BASE);       // ไกลไป -> เดินหน้า
    else         motor(-LEFT_BASE, -RIGHT_BASE);     // ใกล้ไป -> ถอย
    delay(25);
    motor(0, 0);
    delay(40);
  }
  motor(0, 0);
}

float headingFix() {
  float error = targetYaw - yaw;        // + = รถเอียงขวา ต้องแก้ไปซ้าย
  float fix = 0;
  if (abs(error) > DEADBAND_DEG) fix = KP * error - KD * rate;
  if (abs(error) > BIG_TILT_DEG) {
    fix += (error - (error > 0 ? BIG_TILT_DEG : -BIG_TILT_DEG)) * BIG_TILT_GAIN;
  }
  return fix;
}

// วิ่งตรง 1 จังหวะ (useSides = ใช้ผนังข้างช่วยจัดกลางทางด้วย)
void driveStep(float ratio, bool useSides) {
  float fix = headingFix();
  if (useSides && left < SIDE_OPEN_CM && right < SIDE_OPEN_CM) {
    fix += KP_CENTER * (left - right);  // ซ้ายไกลกว่า = ชิดขวา -> แก้ไปซ้าย
  }
  float maxFix = MAX_FIX * ratio;
  fix = constrain(fix, -maxFix, maxFix);
  motor(LEFT_BASE * ratio - fix, RIGHT_BASE * ratio + fix);
}

// =====================================================================
//  Ultrasonic
// =====================================================================
float readCM(int trig, int echo) {
  digitalWrite(trig, LOW);  delayMicroseconds(2);
  digitalWrite(trig, HIGH); delayMicroseconds(10);
  digitalWrite(trig, LOW);
  long us = pulseIn(echo, HIGH, 6000);   // ~100 cm
  if (us == 0) return 999;
  return us * 0.0343 / 2;
}

float median3(float a, float b, float c) {
  return max(min(a, b), min(max(a, b), c));
}

float filtered(float raw, float hist[]) {
  if (raw < MIN_VALID_CM) raw = hist[(histIdx + 2) % 3];
  hist[histIdx] = raw;
  return median3(hist[0], hist[1], hist[2]);
}

void readSensors() {
  front = filtered(readCM(TRIG_F, ECHO_F), histF);  updateYaw();
  left  = filtered(readCM(TRIG_L, ECHO_L), histL);  updateYaw();
  right = filtered(readCM(TRIG_R, ECHO_R), histR);  updateYaw();
  histIdx = (histIdx + 1) % 3;
}

void readSensorsStill() {
  for (int i = 0; i < 4; i++) { readSensors(); delay(15); }
}

void printSensors(const char *msg) {
  Serial.print("F:");    Serial.print(front, 1);
  Serial.print(" L:");   Serial.print(left, 1);
  Serial.print(" R:");   Serial.print(right, 1);
  Serial.print(" yaw:"); Serial.print(yaw, 1);
  Serial.print(" tgt:"); Serial.print(targetYaw, 0);
  Serial.print("  ");    Serial.println(msg);
}

// =====================================================================
//  IR เส้นชัย
// =====================================================================
void checkFinish() {
  irCount = (digitalRead(IR_PIN) == LINE) ? irCount + 1 : 0;
  if (irCount < 3) return;
  motor(LEFT_BASE, RIGHT_BASE);
  delay(FINISH_MS);
  motor(0, 0);
  Serial.println("เจอเส้นชัย หยุด!");
  while (true) { }
}

// =====================================================================
//  เดินตามระยะ "ไม่สนข้าง" (สนแค่หน้า) ใช้ตอน ADVANCE และเดิน 1 บล็อกหลังเลี้ยว
// =====================================================================
void driveDistance(int cm, float ratio) {
  readSensors();
  float startF = front;
  bool useFront = startF < FRONT_FAR_CM;            // วัดจากระยะหน้าที่ลดลง
  float goal = startF - cm;
  unsigned long maxMs = CELL_MS * cm / CELL_CM;     // หน้าไกล -> ใช้เวลาเทียบจาก 1 บล็อก
  unsigned long start = millis(), lastProgress = millis();
  float bestF = startF;

  while (true) {
    checkFinish();
    readSensors();
    unsigned long t = millis() - start;

    if (front <= FRONT_STOP_CM) break;              // หน้าตัน หยุดก่อนชน
    if (useFront && front <= goal) break;
    if (!useFront && t >= maxMs) break;
    if (t >= maxMs * 3 + 500) break;                // กันค้าง

    if (useFront) {                                 // รถติด
      if (front < bestF - 1) { bestF = front; lastProgress = millis(); }
      else if (millis() - lastProgress > STUCK_MS) {
        Serial.println("รถติด -> ถอย");
        motor(-LEFT_BASE, -RIGHT_BASE); delay(250);
        break;
      }
    }
    driveStep(front < SLOW_CM ? min(ratio, SLOW_RATIO) : ratio, false);   // ไม่สนข้าง
  }
  brakeStop();
}

// หยุดแบบเบรก: ถอยสั้นๆ ดับแรงไหล
void brakeStop() {
  if (STOP_BRAKE_MS > 0) { motor(-LEFT_BASE, -RIGHT_BASE); delay(STOP_BRAKE_MS); }
  motor(0, 0);
  delay(100);
}

// กฎอัตโนมัติ (เกาะซ้าย):
//   หน้าตัน: ซ้ายว่าง -> L, ขวาว่าง -> R, ตันหมด -> U
//   หน้าโล่ง: ซ้ายว่างข้างเดียว -> L, ว่างทั้งสองข้าง/ขวาว่างข้างเดียว -> S
// เกาะขวาสลับซ้าย-ขวา
char ruleAction(bool oL, bool oF, bool oR) {
  bool oFirst  = followLeft ? oL : oR;
  bool oSecond = followLeft ? oR : oL;
  char first   = followLeft ? 'L' : 'R';
  char second  = followLeft ? 'R' : 'L';
  if (!oF) {
    if (oFirst)  return first;
    if (oSecond) return second;
    return 'U';
  }
  if (oFirst && !oSecond) return first;
  return 'S';                                     // ว่างทั้งสอง / ข้างตรงข้ามว่าง -> ตรงไป
}

// จุดตัดสินใจ: เลือกทาง (ROUTE ก่อน ไม่งั้นใช้กฎ) แล้วเดิน 1 บล็อกไม่สนข้าง
void decide() {
  bool oL = left  > SIDE_OPEN_CM;
  bool oR = right > SIDE_OPEN_CM;
  bool oF = front > FRONT_OPEN_CM;
  char act = ruleAction(oL, oF, oR);

  int routeLen = sizeof(ROUTE) - 1;
  char planned = (junction < routeLen) ? ROUTE[junction] : 'A';
  if (planned != 'A') {
    bool ok = (planned == 'L' && oL) || (planned == 'R' && oR) ||
              (planned == 'S' && oF) || (planned == 'U');
    if (ok) act = planned;
    else    Serial.println("!! ROUTE สั่งไปทางที่ตัน ใช้กฎแทน");
  }

  Serial.print("J"); Serial.print(junction);
  Serial.print(" ว่าง[");
  Serial.print(oL ? "L" : "-"); Serial.print(oF ? "F" : "-"); Serial.print(oR ? "R" : "-");
  Serial.print("] -> "); Serial.print(act);
  Serial.println(planned != 'A' ? " (ROUTE)" : " (กฎ)");
  printSensors("");
  junction++;

  if      (act == 'L') turnTo(targetYaw + TURN_DEG_LEFT);
  else if (act == 'R') turnTo(targetYaw - TURN_DEG_RIGHT);
  else if (act == 'U') turnTo(targetYaw + TURN_DEG_BACK);
  driveDistance(CELL_CM, 1.0);                    // เดิน 1 บล็อก ไม่สนข้าง (ผ่านช่องนี้ไปก่อน)
  gapCount = 0;
  stopCount = 0;
}

// =====================================================================
//  เลือกโหมดตอนเปิดเครื่อง
// =====================================================================
int selectMode() {                       // 0 = ไม่บัง, 1 = ซ้าย, 2 = ขวา, 3 = หน้า
  int cL = 0, cR = 0, cF = 0;
  unsigned long start = millis();
  while (millis() - start < 3000) {
    cL = (readCM(TRIG_L, ECHO_L) < 5) ? cL + 1 : 0;  delay(10);
    cR = (readCM(TRIG_R, ECHO_R) < 5) ? cR + 1 : 0;  delay(10);
    cF = (readCM(TRIG_F, ECHO_F) < 5) ? cF + 1 : 0;  delay(10);
    if (cL >= 3) return 1;
    if (cR >= 3) return 2;
    if (cF >= 3) return 3;
  }
  return 0;
}

void measureMode() {
  Serial.println("โหมดวัดระยะ (มอเตอร์ไม่หมุน) กด reset เพื่อออก");
  delay(1500);
  while (true) {
    readSensors();
    Serial.print(digitalRead(IR_PIN) == LINE ? "IR:เส้นดำ " : "IR:พื้น ");
    printSensors("");
    delay(100);
  }
}

// =====================================================================
//  setup / loop
// =====================================================================
void setup() {
  Serial.begin(115200);
  pinMode(ENA, OUTPUT); pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT); pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  pinMode(TRIG_F, OUTPUT); pinMode(ECHO_F, INPUT);
  pinMode(TRIG_L, OUTPUT); pinMode(ECHO_L, INPUT);
  pinMode(TRIG_R, OUTPUT); pinMode(ECHO_R, INPUT);
  pinMode(IR_PIN, INPUT);
  motor(0, 0);

  Wire.begin();
  Wire.setWireTimeout(3000, true);
  Wire.beginTransmission(0x68); Wire.write(0x6B); Wire.write(0x00); Wire.endTransmission();
  Wire.beginTransmission(0x68); Wire.write(0x1A); Wire.write(0x03); Wire.endTransmission();
  Wire.beginTransmission(0x68); Wire.write(0x1B); Wire.write(0x08); Wire.endTransmission();
  lastTime = micros();

  Serial.println("บังเซนเซอร์: ซ้าย = มือซ้าย, ขวา = มือขวา, หน้า = วัดระยะ (3 วิ)");
  int mode = selectMode();
  if (mode == 3) measureMode();
  if (mode == 1) followLeft = true;
  if (mode == 2) followLeft = false;
  usePath = (mode == 0 && sizeof(PATH) > 1);   // ไม่บัง + มี PATH -> วิ่งตามเส้นทางฮาร์ดโค้ด
  if (usePath) {
    Serial.println("วิ่งตาม PATH");
    twitch(3);
  } else {
    Serial.println(followLeft ? "ใช้กฎมือซ้าย" : "ใช้กฎมือขวา");
    twitch(followLeft ? 1 : 2);
  }
  delay(1000);

  Serial.println("Calibrate... ห้ามขยับรถ");
  long sum = 0;
  for (int i = 0; i < 500; i++) { sum += readGyroZ(); delay(3); }
  gyroBias = sum / 500.0;

  delay(START_DELAY);
  yaw = 0;
  targetYaw = 0;
  lastTime = micros();
  readSensorsStill();
  Serial.println("เริ่ม!");
}

// =====================================================================
//  โหมด PATH: วิ่งตามเส้นทางทีละช่อง
// =====================================================================
int dirIndex(char c) {                      // เรียงตามเข็มนาฬิกา
  if (c == 'N') return 0;
  if (c == 'E') return 1;
  if (c == 'S') return 2;
  return 3;                                 // 'W'
}

void runPath() {
  int cur = dirIndex(START_DIR);
  int n = sizeof(PATH) - 1;
  int i = 0;
  while (i < n) {
    int d = dirIndex(PATH[i]);
    int run = 1;                            // ทิศเดียวกันติดกันกี่ช่อง -> วิ่งรวดเดียว
    while (i + run < n && PATH[i + run] == PATH[i]) run++;

    int diff = (cur - d + 4) % 4;           // 1 = ซ้าย, 3 = ขวา, 2 = กลับรถ
    Serial.print("ขั้น "); Serial.print(i); Serial.print(": ");
    Serial.print(PATH[i]); Serial.print(" x"); Serial.println(run);
    if      (diff == 1) turnTo(targetYaw + TURN_DEG_LEFT);
    else if (diff == 3) turnTo(targetYaw - TURN_DEG_RIGHT);
    else if (diff == 2) turnTo(targetYaw + TURN_DEG_BACK);
    cur = d;

    driveDistance(run * CELL_CM, 1.0);      // วัดจากระยะหน้าที่ลดลง (หน้าไกลใช้เวลา)
    alignFront();                           // มีผนังหน้า -> จัดให้อยู่กลางช่อง
    i += run;
  }
  // จบเส้นทางแล้วยังไม่เจอเส้นดำ -> วิ่งหน้าอีก 1 ช่องหา IR
  Serial.println("จบ PATH หาเส้นชัย");
  driveDistance(CELL_CM, SLOW_RATIO);
  motor(0, 0);
  while (true) { checkFinish(); }
}

void loop() {
  if (usePath) runPath();                   // ไม่กลับมา

  checkFinish();
  readSensors();

  // 1) ข้างใดข้างหนึ่งว่าง -> วิ่งเข้ากลางช่อง หยุด แล้วตัดสินใจ
  gapCount = (left > SIDE_OPEN_CM || right > SIDE_OPEN_CM) ? gapCount + 1 : 0;
  if (gapCount >= GAP_CONFIRM) {
    driveDistance(ADVANCE_CM, GAP_RATIO);           // วิ่งเข้ากลางช่องแบบช้า (มีเบรกในตัว)
    readSensorsStill();
    if (left > SIDE_OPEN_CM || right > SIDE_OPEN_CM || front < FRONT_OPEN_CM) {
      decide();
    } else {
      gapCount = 0;                                 // ดูใหม่แล้วไม่ใช่ช่อง (ค่ากระโดด) วิ่งต่อ
    }
    return;
  }

  // 2) หน้าตัน -> หยุด จัดให้อยู่กลางช่อง แล้วตัดสินใจ
  stopCount = (front < FRONT_STOP_CM) ? stopCount + 1 : 0;
  if (stopCount >= STOP_CONFIRM) {
    brakeStop();
    alignFront();                                   // ถอย/ขยับให้อยู่กลางช่องก่อนหมุน
    readSensorsStill();
    if (front > FRONT_OPEN_CM) { stopCount = 0; return; }   // ค่ากระโดด หน้าไม่ได้ตันจริง
    decide();
    return;
  }

  // 3) วิ่งตรง (ผนังสองข้าง = ทางเดินปกติ)
  float ratio = 1.0;
  if (gapCount > 0)    ratio = GAP_RATIO;            // เริ่มเห็นช่องข้าง -> ชะลอรอยืนยัน
  if (front < SLOW_CM) ratio = min(ratio, SLOW_RATIO);
  driveStep(ratio, true);
}
