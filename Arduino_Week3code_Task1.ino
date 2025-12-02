#include <PinChangeInterrupt.h>
#include <LiquidCrystal.h>

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

// ================= Encoder pins =================
#define L_ENCODER A5
#define R_ENCODER A1

// ================= IR sensors =================
// Center module: Black = 1, White = 0
#define IR_CENTER A2
// Left & Right custom sensors: Black = 0, White = 1
#define IR_LEFT   A3
#define IR_RIGHT  A4

// ================= Base speed =================
const int BASE_SPEED_R    = 100;   // stronger base speed
const int BASE_SPEED_L    = 100;
const int LOST_TURN_SPEED = 120;   // for hard spins

// ================= Distance =================
const float TICKS_PER_REV = 22.0;
const float WHEEL_CIRC    = 21.3;
const float CM_PER_TICK   = (WHEEL_CIRC / TICKS_PER_REV);

// ================= GROUP SETTINGS (Group 39) =================
const int   GROUP_ID            = 39;
const float TARGET_DISTANCE_CM  = GROUP_ID * 10.0;      // 390 cm
const unsigned long MID_STOP_MS = 2000;                 // 2 s

// After mid-stop, only allow final stop after this extra distance
const float EXTRA_AFTER_MID_FOR_FINAL = 50.0;   // cm

// ================= PD CONSTANTS =================
// Positive error = line on RIGHT, Negative = line on LEFT
const float Kp = 18.0;
const float Kd = 10.0;

// ================= Global variables =================
volatile long leftCount  = 0;
volatile long rightCount = 0;

long start_time;
bool midStopDone     = false;
bool allowFinalStop  = false;
float distAtMidStop  = 0.0;

float lastError = 0.0;

LiquidCrystal lcd(RS, EN, D4, D5, D6, D7);

// =========================================================
// Motor Functions
// =========================================================
void stopMotor()
{
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
  digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
}

void driveForward(int spR, int spL)
{
  spR = constrain(spR, 0, 255);
  spL = constrain(spL, 0, 255);

  analogWrite(ENA, spR);
  analogWrite(ENB, spL);

  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
}

// HARD LEFT: right forward, left backward (spin)
void hardTurnLeft()
{
  analogWrite(ENA, LOST_TURN_SPEED); // right
  analogWrite(ENB, LOST_TURN_SPEED); // left

  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);   // right forward
  digitalWrite(IN3, LOW);  digitalWrite(IN4, HIGH);  // left backward
}

// HARD RIGHT: left forward, right backward
void hardTurnRight()
{
  analogWrite(ENA, LOST_TURN_SPEED); // right
  analogWrite(ENB, LOST_TURN_SPEED); // left

  digitalWrite(IN1, LOW);  digitalWrite(IN2, HIGH);  // right backward
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);   // left forward
}

// =========================================================
// Encoders
// =========================================================
void LeftEncoderISR()  { leftCount++; }
void RightEncoderISR() { rightCount++; }

// =========================================================
// Helpers
// =========================================================
float getDistanceCm()
{
  noInterrupts();
  long l = leftCount;
  long r = rightCount;
  interrupts();

  float avgTicks = (l + r) / 2.0;
  return avgTicks * CM_PER_TICK;
}

void printLiveStatus(long elapsed_ms)
{
  float dist_cm = getDistanceCm();
  float t_s     = elapsed_ms / 1000.0;

  lcd.setCursor(0, 0);
  lcd.print("D:");
  lcd.print(dist_cm, 1);
  lcd.print("cm   ");

  lcd.setCursor(0, 1);
  lcd.print("T:");
  lcd.print(t_s, 1);
  lcd.print("s    ");
}

// Error from sensors
// Positive error = line on RIGHT, Negative = line on LEFT
float computeLineError(bool lB, bool cB, bool rB)
{
  int pattern = (lB ? 4 : 0) | (cB ? 2 : 0) | (rB ? 1 : 0);

  switch (pattern)
  {
    case 0b010: return  0.0;  // center only
    case 0b011: return  1.0;  // center + right
    case 0b001: return  3.0;  // right only → strong right error
    case 0b110: return -1.0;  // left + center
    case 0b100: return -3.0;  // left only → strong left error
    default:    return lastError; // keep previous direction
  }
}

// =========================================================
// Setup
// =========================================================
void setup()
{
  pinMode(IN1,OUTPUT); pinMode(IN2,OUTPUT);
  pinMode(IN3,OUTPUT); pinMode(IN4,OUTPUT);
  pinMode(ENA,OUTPUT); pinMode(ENB,OUTPUT);

  pinMode(IR_CENTER,INPUT);
  pinMode(IR_LEFT,INPUT);
  pinMode(IR_RIGHT,INPUT);

  pinMode(L_ENCODER,INPUT);
  pinMode(R_ENCODER,INPUT);

  attachPCINT(digitalPinToPinChangeInterrupt(L_ENCODER), LeftEncoderISR, RISING);
  attachPCINT(digitalPinToPinChangeInterrupt(R_ENCODER), RightEncoderISR, RISING);

  lcd.begin(16,2);
  stopMotor();

  start_time = millis();
}

// =========================================================
// Main Loop
// =========================================================
void loop()
{
  unsigned long now = millis();
  long elapsed      = now - start_time;

  // ----- LIVE LCD UPDATE every 200 ms -----
  static unsigned long lastLcdUpdate = 0;
  if (now - lastLcdUpdate >= 200)
  {
    printLiveStatus(elapsed);
    lastLcdUpdate = now;
  }

  float dist_cm = getDistanceCm();

  // ====================== MID STOP AT 390 cm ======================
  if (!midStopDone && dist_cm >= TARGET_DISTANCE_CM)
  {
    stopMotor();

    distAtMidStop = dist_cm;
    allowFinalStop = false;   // reset final-stop permission

    lcd.clear();
    lcd.setCursor(0,0); lcd.print("MID D:"); lcd.print(dist_cm,1);
    lcd.setCursor(0,1); lcd.print("MID T:"); lcd.print(elapsed/1000.0,1);

    delay(MID_STOP_MS);      // 2 s stop

    // small forward "kick" so it clearly starts moving again
    driveForward(90, 90);
    delay(200);
    lastError = 0.0;

    midStopDone = true;
    return;
  }

  // ================= AFTER MID: ALLOW FINAL STOP ONLY AFTER EXTRA DISTANCE =================
  if (midStopDone && !allowFinalStop)
  {
    if (dist_cm >= distAtMidStop + EXTRA_AFTER_MID_FOR_FINAL)
    {
      allowFinalStop = true;
    }
  }

  // ====================== READ SENSORS ======================
  bool cB = (digitalRead(IR_CENTER) == 1); // center: 1 = black
  bool lB = (digitalRead(IR_LEFT)   == 0); // left:   0 = black
  bool rB = (digitalRead(IR_RIGHT)  == 0); // right:  0 = black

  bool cW = !cB;
  bool lW = !lB;
  bool rW = !rB;

  // =========================== FINAL FINISH LINE (all black) ===========================
  if (allowFinalStop && cB && lB && rB)
  {
    stopMotor();

    float finalDist = getDistanceCm();
    lcd.clear();
    lcd.setCursor(0,0); lcd.print("STOP D:"); lcd.print(finalDist,1);
    lcd.setCursor(0,1); lcd.print("T:"); lcd.print(elapsed/1000.0,1);

    while(1);   // freeze
  }

  // =========================== LOST LINE / FULL WHITE ===========================
  if (cW && lW && rW)
  {
    // spin back toward last known side of line
    if (lastError >= 0)
      hardTurnRight();
    else
      hardTurnLeft();

    delay(40);     // short spin
    return;
  }

  // =========================== SHARP CORNER ===========================
  // Left 90° corner: center white, only left black
  if (cW && lB && rW)
  {
    unsigned long t0 = millis();
    while (millis() - t0 < 300)  // ~0.3 s burst
    {
      hardTurnLeft();
      if (digitalRead(IR_CENTER) == 1) break;
    }
    lastError = -3.0;
    return;
  }

  // Right 90° corner: center white, only right black
  if (cW && rB && lW)
  {
    unsigned long t0 = millis();
    while (millis() - t0 < 300)
    {
      hardTurnRight();
      if (digitalRead(IR_CENTER) == 1) break;
    }
    lastError = 3.0;
    return;
  }

  // =========================== PD LINE FOLLOWING (wiggle) ===========================
  float error = computeLineError(lB, cB, rB);

  float P = error;
  float D = error - lastError;
  float output = Kp * P + Kd * D;

  float maxSteer = 70.0;
  if (output >  maxSteer) output =  maxSteer;
  if (output < -maxSteer) output = -maxSteer;

  int spR = (int)(BASE_SPEED_R - output);
  int spL = (int)(BASE_SPEED_L + output);

  driveForward(spR, spL);

  lastError = error;
}