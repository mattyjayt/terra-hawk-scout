/*
  Car_Base: one sketch for the Elegoo car on an Arduino UNO R4 WiFi.
  - takes motor commands over a serial link
  - streams IMU data (MPU-6050) and heading back, 20 times a second
  - stops the motors by itself if commands stop arriving (safety watchdog)
  - moves the servo (pin 10) and reports the battery voltage (A3)

  The same text protocol works from the Arduino Serial Monitor, from bench/serial_teleop.py on the Mac,
  and later from the Raspberry Pi. Every line ends with a newline. Settings: 115200 baud.

  COMMANDS (to the car)
    M <left> <right>          drive: PWM -255..255 per side, sign = direction. Must repeat at least
                              every 300 ms or the watchdog stops the motors (the teleop script sends 10x/s).
    M <left> <right> <ms>     drive for <ms> milliseconds (max 5000), then stop. Handy for typing by hand.
    S                         stop now
    Z                         zero the heading (yaw = 0)
    C                         recalibrate the gyro (keep the car still for 2 s; motors stop)
    V <angle>                 servo to <angle> degrees, 0..180 (90 = straight ahead)

  REPLIES (from the car)
    I <t_ms> <ax> <ay> <az> <gx> <gy> <gz> <yaw> <servo> <batt>
        accel in g, gyro in deg/s, yaw in deg (left = +), servo angle in deg, battery in volts
    A <command> ok | A err <reason>                 acknowledgement of each command
    W stop                                          the watchdog stopped the motors

  Motors need the battery switched on. USB alone runs everything else.
*/

#include <Wire.h>
#include <Servo.h>

// ---- Which serial port talks to the controller ----
// Serial  = the USB-C port (Mac, or a USB cable to the Pi: shows up as /dev/ttyACM0 there)
// Serial1 = pins D0 (RX) / D1 (TX). 5 V logic: the Pi's pins are 3.3 V, so they need a level shifter!
#define LINK Serial

// ---- Motor pins (Elegoo shield) ----
const int PIN_RIGHT_SPEED = 5, PIN_RIGHT_DIR = 8;
const int PIN_LEFT_SPEED  = 6, PIN_LEFT_DIR  = 7;
const int PIN_STANDBY     = 3;
const int PIN_SERVO       = 10;         // Elegoo "Servo_z" header
const int PIN_BATTERY     = A3;         // battery through a 10k/1.5k divider on the shield
const int SERVO_CENTRE    = 90;

// ---- IMU ----
const uint8_t MPU_ADDR = 0x68;
const float ACC_PER_G = 16384.0, GYRO_PER_DPS = 131.0;
const float YAW_SIGN = -1.0;            // the value that gave +90 for a left turn in IMU_Test

// ---- Timing ----
const unsigned long WATCHDOG_MS  = 300; // no fresh command for this long -> stop
const unsigned long TELEMETRY_MS = 50;  // send IMU data every 50 ms (20 Hz)
const unsigned long MAX_TIMED_MS = 5000;

float gyroBias[3] = {0, 0, 0};
float acc[3], gyro[3], yaw = 0;
unsigned long lastImuMicros = 0, lastTelemetry = 0;
unsigned long driveUntil = 0;           // motors run until this millis() time
bool driving = false;
bool imuOk = false;
Servo servo;
int servoAngle = SERVO_CENTRE;

// Battery volts: ADC (0..1023 = 0..5 V) x divider (10k + 1.5k) / 1.5k, plus Elegoo's +8 % correction.
float batteryVolts() { return analogRead(PIN_BATTERY) * (5.0 / 1023.0) * (11.5 / 1.5) * 1.08; }

// ================= Motors =================
void setSide(int speedPin, int dirPin, int pwm, bool forwardIsHigh) {
  bool forward = pwm >= 0;
  digitalWrite(dirPin, forward == forwardIsHigh ? HIGH : LOW);
  analogWrite(speedPin, constrain(abs(pwm), 0, 255));
}

void drive(int left, int right) {
  setSide(PIN_RIGHT_SPEED, PIN_RIGHT_DIR, right, false);  // right side: forward = LOW
  setSide(PIN_LEFT_SPEED,  PIN_LEFT_DIR,  left,  true);   // left side:  forward = HIGH
}

void stopAll() {
  analogWrite(PIN_RIGHT_SPEED, 0);
  analogWrite(PIN_LEFT_SPEED, 0);
  driving = false;
}

// ================= IMU =================
void writeReg(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(MPU_ADDR); Wire.write(reg); Wire.write(value); Wire.endTransmission();
}

bool readImu() {                        // accel x,y,z, temp, gyro x,y,z = 14 bytes from 0x3B
  Wire.beginTransmission(MPU_ADDR); Wire.write(0x3B);
  if (Wire.endTransmission(false) != 0 || Wire.requestFrom(MPU_ADDR, (uint8_t)14) != 14) return false;
  uint8_t b[14];
  for (int i = 0; i < 14; i++) b[i] = Wire.read();
  for (int i = 0; i < 3; i++) {
    acc[i]  = (int16_t)((b[2 * i] << 8) | b[2 * i + 1]) / ACC_PER_G;
    gyro[i] = (int16_t)((b[8 + 2 * i] << 8) | b[9 + 2 * i]) / GYRO_PER_DPS - gyroBias[i];
  }
  return true;
}

void calibrateGyro() {
  stopAll();
  float sum[3] = {0, 0, 0}; int n = 0;
  float savedBias[3] = {gyroBias[0], gyroBias[1], gyroBias[2]};
  gyroBias[0] = gyroBias[1] = gyroBias[2] = 0;
  unsigned long start = millis();
  while (millis() - start < 2000) {
    if (readImu()) { for (int i = 0; i < 3; i++) sum[i] += gyro[i]; n++; }
    delay(5);
  }
  if (n > 0) for (int i = 0; i < 3; i++) gyroBias[i] = sum[i] / n;
  else for (int i = 0; i < 3; i++) gyroBias[i] = savedBias[i];
  lastImuMicros = micros();
}

void updateImu() {
  if (!imuOk || !readImu()) return;
  unsigned long now = micros();
  float dt = (now - lastImuMicros) / 1e6;
  lastImuMicros = now;
  float rate = YAW_SIGN * gyro[2];
  if (fabs(rate) < 0.2) rate = 0;       // ignore tiny noise when still
  yaw += rate * dt;
}

void sendTelemetry() {
  LINK.print("I ");  LINK.print(millis());
  for (int i = 0; i < 3; i++) { LINK.print(' '); LINK.print(acc[i], 3); }
  for (int i = 0; i < 3; i++) { LINK.print(' '); LINK.print(gyro[i], 2); }
  LINK.print(' ');   LINK.print(yaw, 2);
  LINK.print(' ');   LINK.print(servoAngle);
  LINK.print(' ');   LINK.println(batteryVolts(), 2);
}

// ================= Commands =================
char line[48];
uint8_t lineLen = 0;

void handleCommand(char *cmd) {
  int l = 0, r = 0; long ms = 0;
  switch (cmd[0]) {
    case 'M': {
      int n = sscanf(cmd + 1, "%d %d %ld", &l, &r, &ms);
      if (n < 2) { LINK.println("A err M needs: M <left> <right> [ms]"); return; }
      unsigned long hold = (n == 3) ? (unsigned long)constrain(ms, 0L, (long)MAX_TIMED_MS) : WATCHDOG_MS;
      drive(constrain(l, -255, 255), constrain(r, -255, 255));
      driveUntil = millis() + hold;
      driving = true;
      if (n == 3) LINK.println("A M ok");   // no ack for streamed commands: they come 10x per second
      break;
    }
    case 'S': stopAll(); LINK.println("A S ok"); break;
    case 'Z': yaw = 0;   LINK.println("A Z ok"); break;
    case 'C': LINK.println("A C calibrating 2 s"); calibrateGyro(); LINK.println("A C ok"); break;
    case 'V': {
      int a;
      if (sscanf(cmd + 1, "%d", &a) != 1) { LINK.println("A err V needs: V <angle 0-180>"); return; }
      servoAngle = constrain(a, 0, 180);
      servo.write(servoAngle);
      LINK.println("A V ok");
      break;
    }
    default:  LINK.print("A err unknown: "); LINK.println(cmd);
  }
}

void readCommands() {                    // collect characters until a newline, never blocking
  while (LINK.available()) {
    char c = LINK.read();
    if (c == '\r') continue;
    if (c == '\n') {
      line[lineLen] = '\0';
      if (lineLen > 0) handleCommand(line);
      lineLen = 0;
    } else if (lineLen < sizeof(line) - 1) {
      line[lineLen++] = c;
    } else {
      lineLen = 0;                         // line too long: drop it
      LINK.println("A err line too long");
    }
  }
}

// ================= Main =================
void setup() {
  LINK.begin(115200);
  pinMode(PIN_RIGHT_SPEED, OUTPUT); pinMode(PIN_RIGHT_DIR, OUTPUT);
  pinMode(PIN_LEFT_SPEED, OUTPUT);  pinMode(PIN_LEFT_DIR, OUTPUT);
  pinMode(PIN_STANDBY, OUTPUT);
  stopAll();
  digitalWrite(PIN_STANDBY, HIGH);
  servo.attach(PIN_SERVO);
  servo.write(SERVO_CENTRE);             // start looking straight ahead

  Wire.begin();
  Wire.setClock(400000);                 // fast I2C: a full IMU read takes ~0.4 ms
  Wire.beginTransmission(MPU_ADDR);
  imuOk = (Wire.endTransmission() == 0);
  delay(1500);                           // time to open the Serial Monitor / let the car settle
  if (imuOk) {
    writeReg(0x6B, 0x00);                // wake the MPU-6050
    delay(100);
    LINK.println("A boot calibrating gyro, keep still 2 s");
    calibrateGyro();
    LINK.println("A boot ready");
  } else {
    LINK.println("A err no IMU at 0x68 (motors still work)");
  }
}

void loop() {
  readCommands();
  updateImu();

  if (driving && (long)(millis() - driveUntil) >= 0) {   // watchdog / timed drive finished
    stopAll();
    LINK.println("W stop");
  }

  if (millis() - lastTelemetry >= TELEMETRY_MS) {
    lastTelemetry = millis();
    if (imuOk) sendTelemetry();
  }
}
