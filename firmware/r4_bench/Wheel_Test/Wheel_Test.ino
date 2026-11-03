/*
  Wheel_Test: the simplest possible motor check for the Elegoo car on an Arduino UNO R4 WiFi.

  Every 10 seconds the next test starts and runs for 5 seconds, then 5 seconds of rest:
    1. RIGHT wheels forward   2. RIGHT wheels backward
    3. LEFT wheels forward    4. LEFT wheels backward      ...then it repeats.
  Open Serial Monitor at 115200 baud to see which test should be running.

  Why "wheels" and not "wheel": the shield's TB6612 driver has only 2 channels.
  The two right motors share channel A and the two left motors share channel B,
  so they always turn together.

  Pins are the ones from Elegoo's own code (DeviceDriverSet_xxx0.h in Basic_RobotCar_Drive).
*/

// ---- Pins on the Elegoo shield ----
const int PIN_RIGHT_SPEED = 5;   // PWMA: how fast the right side turns (0-255)
const int PIN_RIGHT_DIR   = 8;   // AIN1: which way the right side turns
const int PIN_LEFT_SPEED  = 6;   // PWMB: how fast the left side turns (0-255)
const int PIN_LEFT_DIR    = 7;   // BIN1: which way the left side turns
const int PIN_STANDBY     = 3;   // STBY: TB6612 must be HIGH to run (full Elegoo V4 code uses pin 3)

// ---- Settings ----
const int TEST_SPEED      = 120;    // about half power; safe on the bench
const unsigned long RUN_MS  = 5000; // spin for 5 s
const unsigned long REST_MS = 5000; // then rest 5 s (one test every 10 s)

// Set one side. speed: 0-255. forward: true/false.
// Elegoo wired the two sides mirror-imaged, so "forward" is LOW on the right and HIGH on the left.
void setRight(int speed, bool forward) {
  digitalWrite(PIN_RIGHT_DIR, forward ? HIGH : LOW);
  analogWrite(PIN_RIGHT_SPEED, speed);
}

void setLeft(int speed, bool forward) {
  digitalWrite(PIN_LEFT_DIR, forward ? HIGH : LOW);
  analogWrite(PIN_LEFT_SPEED, speed);
}

void stopAll() {
  analogWrite(PIN_RIGHT_SPEED, 0);
  analogWrite(PIN_LEFT_SPEED, 0);
}

void setup() {
  Serial.begin(115200);
  pinMode(PIN_RIGHT_SPEED, OUTPUT);
  pinMode(PIN_RIGHT_DIR, OUTPUT);
  pinMode(PIN_LEFT_SPEED, OUTPUT);
  pinMode(PIN_LEFT_DIR, OUTPUT);
  pinMode(PIN_STANDBY, OUTPUT);
  stopAll();                        // start with everything off
  digitalWrite(PIN_STANDBY, HIGH);  // wake the motor driver up
  delay(2000);                      // 2 s to put the car down / lift it up
  Serial.println("Wheel_Test ready");
}

void loop() {
  static int test = 0;  // which of the 4 tests runs next (remembered between loops)

  switch (test) {
    case 0: Serial.println("1/4 RIGHT forward");  setRight(TEST_SPEED, true);  break;
    case 1: Serial.println("2/4 RIGHT backward"); setRight(TEST_SPEED, false); break;
    case 2: Serial.println("3/4 LEFT forward");   setLeft(TEST_SPEED, true);   break;
    case 3: Serial.println("4/4 LEFT backward");  setLeft(TEST_SPEED, false);  break;
  }
  delay(RUN_MS);

  stopAll();
  Serial.println("    rest");
  delay(REST_MS);

  test = (test + 1) % 4;  // 0,1,2,3,0,1,...
}
