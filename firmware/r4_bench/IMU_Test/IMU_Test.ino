/*
  IMU_Test: read the MPU-6050 on the Elegoo shield with an Arduino UNO R4 WiFi. No library, just Wire.

  What it prints 10 times a second (Serial Monitor, 115200 baud):
    acc  = acceleration in g   (lying flat and still: x ~ 0, y ~ 0, z ~ +1 or -1)
    gyro = rotation speed in degrees per second (still: all ~ 0)
    yaw  = heading in degrees since start, made by adding up gyro z over time

  Try it: keep the car still for the 2 s calibration, then turn it by hand 90 degrees on the table.
  Yaw should read about 90 (left turn = positive, the ROS convention) and slowly drift a little.
  That drift is why the Scout design calibrates the gyro bias every time the robot stops.

  Motors are not used here, so USB power alone is enough.
*/

#include <Wire.h>

const uint8_t MPU_ADDR = 0x68;     // the MPU-6050's I2C address (Elegoo uses the default)
const float ACC_PER_G   = 16384.0; // raw counts per 1 g        (default range +-2 g)
const float GYRO_PER_DPS = 131.0;  // raw counts per 1 deg/s    (default range +-250 deg/s)
const float YAW_SIGN = -1.0;       // Elegoo's code flips gyro z (chip orientation on the shield).
                                   // If a LEFT turn gives a NEGATIVE yaw, change this to +1.0.

float gyroZBias = 0;               // what the gyro reads when standing still (measured in setup)
float yaw = 0;                     // heading in degrees
unsigned long lastMicros = 0;

// Write one value into one MPU-6050 register.
void writeReg(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

// Read 'count' bytes starting at register 'reg' into 'buf'. Returns false if the chip didn't answer.
bool readRegs(uint8_t reg, uint8_t *buf, uint8_t count) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;   // false = keep the bus for the read
  if (Wire.requestFrom(MPU_ADDR, count) != count) return false;
  for (uint8_t i = 0; i < count; i++) buf[i] = Wire.read();
  return true;
}

// The chip sends each value as 2 bytes, high byte first; this joins them into a signed number.
int16_t toInt16(uint8_t high, uint8_t low) { return (int16_t)((high << 8) | low); }

// Read all 14 data bytes at once: accel x,y,z, temperature, gyro x,y,z.
bool readAll(float acc[3], float gyro[3]) {
  uint8_t b[14];
  if (!readRegs(0x3B, b, 14)) return false;
  for (int i = 0; i < 3; i++) {
    acc[i]  = toInt16(b[2 * i], b[2 * i + 1]) / ACC_PER_G;
    gyro[i] = toInt16(b[8 + 2 * i], b[9 + 2 * i]) / GYRO_PER_DPS;
  }
  return true;
}

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000) {}   // wait up to 3 s for the Serial Monitor
  Wire.begin();

  // 1. Is anyone there? Register 0x75 (WHO_AM_I) holds the chip's ID.
  uint8_t id = 0;
  if (!readRegs(0x75, &id, 1)) {
    Serial.println("No answer from the MPU-6050 at 0x68. Check the shield is seated (SDA/SCL pins).");
    while (true) {}
  }
  Serial.print("WHO_AM_I = 0x"); Serial.println(id, HEX);   // 0x68 on a genuine chip; clones may differ

  // 2. Wake it up: it starts in sleep mode. Register 0x6B (PWR_MGMT_1) = 0 means 'awake'.
  writeReg(0x6B, 0x00);
  delay(100);

  // 3. Calibrate: average gyro z for 2 s while the car stands still.
  Serial.println("Calibrating, keep the car still for 2 s...");
  float acc[3], gyro[3], sum = 0;
  int n = 0;
  unsigned long start = millis();
  while (millis() - start < 2000) {
    if (readAll(acc, gyro)) { sum += gyro[2]; n++; }
    delay(5);
  }
  gyroZBias = sum / n;
  Serial.print("gyro z bias = "); Serial.print(gyroZBias, 3); Serial.println(" deg/s");
  lastMicros = micros();
}

void loop() {
  float acc[3], gyro[3];
  if (!readAll(acc, gyro)) { Serial.println("read failed"); delay(500); return; }

  // Yaw = add up (rotation speed x time since last reading). 'dt' is in seconds.
  unsigned long now = micros();
  float dt = (now - lastMicros) / 1e6;
  lastMicros = now;
  float gz = YAW_SIGN * (gyro[2] - gyroZBias);
  if (fabs(gz) < 0.2) gz = 0;      // ignore tiny noise when still (Elegoo does the same)
  yaw += gz * dt;

  static unsigned long lastPrint = 0;
  if (millis() - lastPrint >= 100) {   // print 10x per second, but integrate on every loop
    lastPrint = millis();
    Serial.print("acc g: ");    Serial.print(acc[0], 2);  Serial.print(" "); Serial.print(acc[1], 2);  Serial.print(" "); Serial.print(acc[2], 2);
    Serial.print("   gyro dps: "); Serial.print(gyro[0], 1); Serial.print(" "); Serial.print(gyro[1], 1); Serial.print(" "); Serial.print(gyro[2], 1);
    Serial.print("   yaw: ");   Serial.println(yaw, 1);
  }
  delay(5);
}
