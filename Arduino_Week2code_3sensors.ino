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
#define IR_CENTER A2   // Black = 1, White = 0
#define IR_LEFT   A3   // Black = 0, White = 1
#define IR_RIGHT  A4   // Black = 0, White = 1

// ================= Speed settings =================
const int FAST_R  = 110;
const int FAST_L  = 110;

const int SLOW_R  = 70;
const int SLOW_L  = 70;

const int TURN_SPEED = 200;

// ================= Finish line =================
//const int FINISH_COUNT = 5;
//const long MIN_STOP_TIME_MS = 5000;

// ================= Distance =================
const float TICKS_PER_REV = 22.0;
const float WHEEL_CIRC = 21.3;
const float CM_PER_TICK = (WHEEL_CIRC / TICKS_PER_REV);

// ================= Variables =================
volatile long leftCount = 0;
volatile long rightCount = 0;

long start_time;

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

void forward(int spR, int spL)
{
  analogWrite(ENA, spR);
  analogWrite(ENB, spL);
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
}

void turnLeft()
{
  analogWrite(ENA, TURN_SPEED);
  analogWrite(ENB, 0);
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);  digitalWrite(IN4, LOW);
}

void turnRight()
{
  analogWrite(ENA, 0);
  analogWrite(ENB, TURN_SPEED);
  digitalWrite(IN1, LOW);  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
}

// =========================================================
// Encoders
// =========================================================
void LeftEncoderISR()  { leftCount++; }
void RightEncoderISR() { rightCount++; }

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
  long now = millis();
  long elapsed = now - start_time;

  // ===== STOP AFTER 60 SECONDS =====
  if (elapsed >= 60000)
  {
    stopMotor();

    float dist = ((leftCount + rightCount) / 2.0) * CM_PER_TICK;
    lcd.clear();
    lcd.setCursor(0,0); lcd.print("D:"); lcd.print(dist,1);
    lcd.setCursor(0,1); lcd.print("T:"); lcd.print(elapsed/1000.0,1);

    while(1);
  }

  // Read sensors
  bool cB = (digitalRead(IR_CENTER) == 1);
  bool lB = (digitalRead(IR_LEFT)   == 0);
  bool rB = (digitalRead(IR_RIGHT)  == 0);

  bool cW = !cB;
  bool lW = !lB;
  bool rW = !rB;

  // =========================== FINISH LINE (instant) ===========================
if (cB && lB && rB)
{
  stopMotor();

  float dist = ((leftCount + rightCount) / 2.0) * CM_PER_TICK;
  lcd.clear();
  lcd.setCursor(0,0); lcd.print("STOP D:"); lcd.print(dist,1);
  lcd.setCursor(0,1); lcd.print("T:"); lcd.print(elapsed/1000.0,1);

  while(1);  // freeze here
}

  // ========================= SMART LEFT TURN =========================
  if (lB && cW)
  {
    while (true)
    {
      turnLeft();

      bool centerBlack = (digitalRead(IR_CENTER) == 1);
      bool rightBlack  = (digitalRead(IR_RIGHT) == 0);

      if (centerBlack) break;

      // If opposite sensor becomes black → switch direction
      if (rightBlack)
      {
        while (true)
        {
          turnRight();
          bool c2 = (digitalRead(IR_CENTER) == 1);
          if (c2) break;
          delay(10);
        }
        break;
      }

      delay(10);
    }
    return;
  }

  // ========================= SMART RIGHT TURN =========================
  if (rB && cW)
  {
    while (true)
    {
      turnRight();

      bool centerBlack = (digitalRead(IR_CENTER) == 1);
      bool leftBlack   = (digitalRead(IR_LEFT) == 0);

      if (centerBlack) break;

      // If opposite side detects black → switch to opposite turn
      if (leftBlack)
      {
        while (true)
        {
          turnLeft();
          bool c2 = (digitalRead(IR_CENTER) == 1);
          if (c2) break;
          delay(10);
        }
        break;
      }

      delay(10);
    }
    return;
  }

  // ====================== ONLY CENTER BLACK → FAST FORWARD ======================
  if (cB && lW && rW)
  {
    forward(FAST_R, FAST_L);
    delay(20);
    return;
  }

  // ====================== CENTER WHITE → SLOW FORWARD ==========================
  if (cW)
  {
    forward(SLOW_R, SLOW_L);
    delay(20);
    return;
  }

  // ====================== FALLBACK = SLOW FORWARD ======================
  forward(SLOW_R, SLOW_L);
  delay(20);
}

