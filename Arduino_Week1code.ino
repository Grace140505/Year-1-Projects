#include <LiquidCrystal.h>

// ================= LCD PINS =================
// RW is grounded -> write only
#define D4 4
#define D5 5
#define D6 6
#define D7 7
#define RS 8   // Command or data mode
#define EN 9   // Signal LCD to read

// ================= MOTOR PINS =================
// “Head” of robot is where ultrasonic sensor would be
#define IN1 13   // Right wheel direction
#define IN2 12   // Right wheel direction
#define IN3 2    // Left wheel direction
#define IN4 1    // Left wheel direction
#define ENA 11   // PWM – Right wheels
#define ENB 3    // PWM – Left wheels

// ================= SPEED SETTINGS =================
const int SPEED_RIGHT = 200;   // 0–255 (PWM)
const int SPEED_LEFT  = 200;   // 0–255 (PWM)

// Move duration (10 seconds)
const unsigned long MOVE_TIME_MS = 10000;

// ================= GLOBALS =================
LiquidCrystal lcd(RS, EN, D4, D5, D6, D7);
unsigned long start_time;

// ================= SETUP =================
void setup() {
  // Motor pins
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);

  func_motorInit();

  // LCD
  lcd.begin(16, 2);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Week 1: Move");
  lcd.setCursor(0, 1);
  lcd.print("Time: ");

  // Start timer
  start_time = millis();
}

// ================= MAIN LOOP =================
void loop() {
  unsigned long current_time = millis();
  unsigned long elapsed_time = current_time - start_time;

  // --- Task 1: Move for 10 seconds in straight line ---
  if (elapsed_time < MOVE_TIME_MS) {
    func_moveForward(SPEED_LEFT, SPEED_RIGHT);
    // --- Task 2: Display time on LCD ---
    func_print(elapsed_time);      // shows elapsed time while moving
  } else {
    // After 10 seconds: stop and show final message & time
    func_stop();

    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Movement Done!");

    lcd.setCursor(0, 1);
    lcd.print("Time: ");
    func_print(MOVE_TIME_MS);      // show 10.000s as final time

    // Freeze here
    while (true) { }
  }
}

// ================= MOTOR FUNCTIONS =================

// Reset initial state
void func_motorInit() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
}

// Stop robot car
void func_stop() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
  delay(10);
}

// Move robot car forward
void func_moveForward(const int SPEED_LEFT, const int SPEED_RIGHT) {
  analogWrite(ENA, SPEED_RIGHT);
  analogWrite(ENB, SPEED_LEFT);

  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

// ================= LCD TIME PRINT =================
void func_print(unsigned long elapsed_time) {
  unsigned int seconds = elapsed_time / 1000;
  unsigned int milliseconds = elapsed_time % 1000;

  // LCD second line: "Time: SS:MMM"
  lcd.setCursor(6, 1);   // after "Time: "

  // Format SS
  if (seconds < 10) lcd.print("0");
  lcd.print(seconds);
  lcd.print(":");

  // Format MMM
  if (milliseconds < 100) lcd.print("0");
  if (milliseconds < 10)  lcd.print("0");
  lcd.print(milliseconds);
}
