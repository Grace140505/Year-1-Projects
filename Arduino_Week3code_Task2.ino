#include <LiquidCrystal.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// ================= LCD pins =================
#define D4 4
#define D5 5
#define D6 6
#define D7 7
#define RS 8
#define EN 9

// ================= Motor pins =================
#define IN1 13
#define IN2 12
#define IN3 2
#define IN4 1
#define ENA 11
#define ENB 3

// ================= Speed & timing =================
const int SPEED_SLOW  = 120;      // flat / top / down ramp
const int SPEED_FAST  = 255;      // ONLY for going UP ramp
const int SPIN_SPEED  = 120;      // spin right

const unsigned long TOP_STOP_MS          = 4000;   // 4 s at top
const unsigned long MIN_TIME_BEFORE_RAMP = 1000;   // drive straight first
const unsigned long EXTRA_TOP_DRIVE_MS   = 200;    // small roll forward on top
const unsigned long DOWN_TIME_MS         = 2000;   // how long to drive down ramp

// ================= Angle thresholds (deg) =================
const float RAMP_ON_ANGLE = 7.0;   // > this => clearly on ramp
const float FLAT_ANGLE    = 3.0;   // < this => flat enough

LiquidCrystal lcd(RS, EN, D4, D5, D6, D7);
Adafruit_MPU6050 mpu;

float flatPitchDeg = 0.0;   // reference pitch on flat ground
float rampAngleDeg = 0.0;   // pitch relative to flat
float spinAngleDeg = 0.0;   // gyro spin angle (yaw)

// ================== STATE MACHINE ==================
enum {
  STATE_DRIVE_UP,
  STATE_WAIT_TOP,
  STATE_SPIN,
  STATE_DRIVE_DOWN,
  STATE_STOP
};

int  state           = STATE_DRIVE_UP;
bool rampUpStarted   = false;

unsigned long startTime;
unsigned long topTime;
unsigned long downStartTime;

// loop timing for gyro integration
unsigned long lastLoopTime = 0;

// ---------------- Motor helpers ----------------
void stopMotor() {
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
  digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
}

void driveForward(int sp) {
  sp = constrain(sp, 0, 255);
  analogWrite(ENA, sp);
  analogWrite(ENB, sp);
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);   // right forward
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);   // left forward
}

void spinRight(int sp) {
  sp = constrain(sp, 0, 255);
  analogWrite(ENA, sp);
  analogWrite(ENB, sp);
  digitalWrite(IN1, LOW);  digitalWrite(IN2, HIGH);  // right backward
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);   // left forward
}

// ---------------- LCD ----------------
void showStatus(unsigned long elapsed_ms) {
  lcd.setCursor(0,0);
  lcd.print("Ang:");
  lcd.print(rampAngleDeg, 1);
  lcd.print("   ");

  lcd.setCursor(0,1);
  lcd.print("t:");
  lcd.print(elapsed_ms / 1000.0, 1);
  lcd.print("s   ");
}

// ---------------- Calibrate flat pitch ----------------
void calibrateFlatPitch() {
  const int N = 80;
  float sum = 0;

  lcd.clear();
  lcd.setCursor(0,0); lcd.print("Calibrating");
  lcd.setCursor(0,1); lcd.print("Keep flat");

  // robot MUST be on flat ground and not moving here
  for (int i = 0; i < N; i++) {
    sensors_event_t a, g, t;
    mpu.getEvent(&a, &g, &t);
    float pitch = atan2(a.acceleration.x, a.acceleration.z) * 180.0 / PI;
    sum += pitch;
    delay(5);
  }

  flatPitchDeg = sum / N;

  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Flat:");
  lcd.print(flatPitchDeg,1);
  delay(800);
}

// ---------------- Setup ----------------
void setup() {
  pinMode(IN1,OUTPUT); pinMode(IN2,OUTPUT);
  pinMode(IN3,OUTPUT); pinMode(IN4,OUTPUT);
  pinMode(ENA,OUTPUT); pinMode(ENB,OUTPUT);

  lcd.begin(16,2);
  stopMotor();

  Wire.begin();
  if (!mpu.begin()) {
    lcd.clear();
    lcd.print("MPU ERROR");
    while(1);   // block forever if MPU not found
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  calibrateFlatPitch();

  startTime     = millis();
  lastLoopTime  = startTime;
}

// ---------------- Main loop ----------------
void loop() {
  unsigned long now     = millis();
  unsigned long elapsed = now - startTime;

  // dt for gyro integration (seconds)
  float dt = (now - lastLoopTime) / 1000.0;
  lastLoopTime = now;

  // read MPU
  sensors_event_t a, g, t;
  mpu.getEvent(&a, &g, &t);

  // pitch from accelerometer (for ramp angle)
  // NOTE: assumes MPU X-axis points forward
  float pitchDeg = atan2(a.acceleration.x, a.acceleration.z) * 180.0 / PI;
  rampAngleDeg   = pitchDeg - flatPitchDeg;
  float absAngle = fabs(rampAngleDeg);

  // yaw rate (deg/s) from gyro Z (for spin)
  float yawRateDegPerSec = g.gyro.z * 180.0 / PI;

  // LCD update every ~200 ms
  static unsigned long lastLcd = 0;
  if (now - lastLcd > 200) {
    showStatus(elapsed);
    lastLcd = now;
  }

  // helper to confirm flat top
  static int flatTopCount = 0;

  switch (state) {

    // ===== 1. DRIVE UP TO TOP =====
    case STATE_DRIVE_UP: {
      int speed = SPEED_SLOW;
      if (absAngle > RAMP_ON_ANGLE) {
        speed = SPEED_FAST;  // strong only on ramp
      }
      driveForward(speed);

      // mark ramp started (only after some straight time)
      if (!rampUpStarted &&
          (now - startTime > MIN_TIME_BEFORE_RAMP) &&
          (absAngle > RAMP_ON_ANGLE)) {
        rampUpStarted = true;
      }

      // count how many loops we see "flat" after ramp
      if (rampUpStarted && absAngle < FLAT_ANGLE) {
        flatTopCount++;
      } else {
        flatTopCount = 0;
      }

      // when flat for a few cycles, assume we reached the top
      if (rampUpStarted && flatTopCount >= 5) { // ~5 loops of flat
        driveForward(SPEED_SLOW);
        delay(EXTRA_TOP_DRIVE_MS);  // roll forward fully onto top

        stopMotor();
        topTime = millis();
        state   = STATE_WAIT_TOP;
      }
      break;
    }

    // ===== 2. WAIT 4 SECONDS AT TOP =====
    case STATE_WAIT_TOP:
      stopMotor();
      if (now - topTime > TOP_STOP_MS) {
        spinAngleDeg = 0.0;     // reset gyro spin angle
        state        = STATE_SPIN;
      }
      break;

    // ===== 3. 360° SPIN USING GYRO =====
    case STATE_SPIN: {
      // integrate yaw angle only while spinning
      spinAngleDeg += yawRateDegPerSec * dt;   // degrees

      if (fabs(spinAngleDeg) < 360.0) {
        spinRight(SPIN_SPEED);
      } else {
        stopMotor();
        // prepare to go down
        downStartTime = millis();
        state         = STATE_DRIVE_DOWN;
      }
      break;
    }

    // ===== 4. DRIVE DOWN RAMP (TIME-BASED, ALWAYS SLOW) =====
    case STATE_DRIVE_DOWN: {
      driveForward(SPEED_SLOW);
      if (now - downStartTime >= DOWN_TIME_MS) {
        stopMotor();
        state = STATE_STOP;

        lcd.clear();
        lcd.setCursor(0,0); lcd.print("Task 2 done");
      }
      break;
    }

    // ===== 5. FINAL STOP =====
    case STATE_STOP:
      stopMotor();
      break;
  }
}