/*
  Turn90: motors + gyro together. The car turns 90 degrees left, waits, turns back to 0, and repeats.
  The first sketch where the car uses a sensor to decide when to stop.

  Built from Wheel_Test (motor pins and functions) and IMU_Test (gyro reading and yaw).
  Needs the BATTERY ON (motors) and the car ON THE FLOOR (it has to actually turn).
  Serial Monitor at 115200 shows the target, where it stopped and how far it overshot.

  Safety: a turn gives up after 6 s, and stops at once if yaw moves the wrong way
  (that means YAW_SIGN is wrong: fix it in IMU_Test first).
*/

#include <Wire.h>

// ---- Motor pins (Elegoo shield, same as Wheel_Test) ----
const int PIN_RIGHT_SPEED = 5, PIN_RIGHT_DIR = 8;
const int PIN_LEFT_SPEED  = 6, PIN_LEFT_DIR  = 7;
const int PIN_STANDBY     = 3;

// ---- IMU (same as IMU_Test) ----
const uint8_t MPU_ADDR = 0x68;
const float GYRO_PER_DPS = 131.0;
const float YAW_SIGN = -1.0;          // use the value that gave +90 for a left turn in IMU_Test

// ---- Turn settings: tune these ----
const int FAST_SPEED = 150;           // PWM far from the target
const int SLOW_SPEED = 90;            // PWM for the last part (too low and the car won't move at all)
const float SLOW_ZONE_DEG = 25;       // switch to SLOW_SPEED when this close to the target
const float STOP_EARLY_DEG = 3;       // cut the motors this early: the car coasts the rest
const unsigned long TURN_TIMEOUT_MS = 6000;

float gyroZBias = 0, yaw = 0;
unsigned long lastMicros = 0;

// ---------- Motors ----------
void setRight(int speed, bool forward) { digitalWrite(PIN_RIGHT_DIR, forward ? LOW : HIGH); analogWrite(PIN_RIGHT_SPEED, speed); }
void setLeft(int speed, bool forward)  { digitalWrite(PIN_LEFT_DIR,  forward ? HIGH : LOW); analogWrite(PIN_LEFT_SPEED, speed); }
void stopAll() { analogWrite(PIN_RIGHT_SPEED, 0); analogWrite(PIN_LEFT_SPEED, 0); }

// Spin in place. left = true: right side forward, left side backward (counter-clockwise from above).
void spin(int speed, bool left) {
  setRight(speed, left);
  setLeft(speed, !left);
}

// ---------- IMU ----------
void writeReg(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(MPU_ADDR); Wire.write(reg); Wire.write(value); Wire.endTransmission();
}

bool readGyroZ(float &gz) {           // only the 2 bytes we need: gyro z at register 0x47
  Wire.beginTransmission(MPU_ADDR); Wire.write(0x47);
  if (Wire.endTransmission(false) != 0 || Wire.requestFrom(MPU_ADDR, (uint8_t)2) != 2) return false;
  int16_t raw = (int16_t)((Wire.read() << 8) | Wire.read());
  gz = raw / GYRO_PER_DPS;
  return true;
}

// Call this as often as possible: it adds the latest bit of rotation to yaw.
void updateYaw() {
  float gz;
  if (!readGyroZ(gz)) return;
  unsigned long now = micros();
  float dt = (now - lastMicros) / 1e6;
  lastMicros = now;
  float rate = YAW_SIGN * (gz - gyroZBias);
  if (fabs(rate) < 0.2) rate = 0;
  yaw += rate * dt;
}

void calibrate() {
  Serial.println("Calibrating gyro, keep the car still for 2 s...");
  float sum = 0, gz; int n = 0;
  unsigned long start = millis();
  while (millis() - start < 2000) { if (readGyroZ(gz)) { sum += gz; n++; } delay(5); }
  gyroZBias = sum / n;
  lastMicros = micros();
}

// Wait 'ms' milliseconds while still keeping yaw up to date (plain delay() would miss rotation).
void waitTracking(unsigned long ms) {
  unsigned long start = millis();
  while (millis() - start < ms) { updateYaw(); delay(2); }
}

// ---------- The behaviour ----------
// Turn until yaw reaches 'target' degrees. Returns false if it gave up.
bool turnTo(float target) {
  bool left = target > yaw;               // which way to go
  float startYaw = yaw;
  unsigned long start = millis();
  Serial.print("Turning "); Serial.print(left ? "LEFT" : "RIGHT"); Serial.print(" to "); Serial.println(target, 1);

  while (true) {
    updateYaw();
    float remaining = left ? (target - yaw) : (yaw - target);   // degrees still to go

    if (remaining <= STOP_EARLY_DEG) break;                       // close enough: let it coast
    if (millis() - start > TURN_TIMEOUT_MS) { stopAll(); Serial.println("  gave up: timeout (stuck, or speed too low?)"); return false; }
    float moved = left ? (yaw - startYaw) : (startYaw - yaw);
    if (moved < -10) { stopAll(); Serial.println("  STOP: yaw goes the wrong way, flip YAW_SIGN"); return false; }

    spin(remaining > SLOW_ZONE_DEG ? FAST_SPEED : SLOW_SPEED, left);
    delay(2);
  }

  stopAll();
  waitTracking(500);                      // let it coast to a stop while we keep measuring
  Serial.print("  stopped at "); Serial.print(yaw, 1);
  Serial.print("   error "); Serial.print(yaw - target, 1); Serial.println(" deg");
  return true;
}

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000) {}
  pinMode(PIN_RIGHT_SPEED, OUTPUT); pinMode(PIN_RIGHT_DIR, OUTPUT);
  pinMode(PIN_LEFT_SPEED, OUTPUT);  pinMode(PIN_LEFT_DIR, OUTPUT);
  pinMode(PIN_STANDBY, OUTPUT);
  stopAll();
  digitalWrite(PIN_STANDBY, HIGH);

  Wire.begin();
  writeReg(0x6B, 0x00);                   // wake the MPU-6050
  delay(100);
  calibrate();
  Serial.println("Ready. First turn in 3 s.");
  waitTracking(3000);
}

void loop() {
  if (!turnTo(90)) { stopAll(); while (true) {} }   // on any problem: stop for good (press reset to retry)
  waitTracking(3000);
  if (!turnTo(0))  { stopAll(); while (true) {} }
  waitTracking(3000);
}
