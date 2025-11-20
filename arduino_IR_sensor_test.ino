#include <LiquidCrystal.h>

// LCD pin definitions
#define D4 4
#define D5 5
#define D6 6
#define D7 7
#define RS 8
#define EN 9

// IR sensor pin
#define IR A4

// Initialize LCD
LiquidCrystal lcd(RS, EN, D4, D5, D6, D7);

void setup() {
  pinMode(IR, INPUT);
  lcd.begin(16, 2);
  lcd.setCursor(0, 0);
  lcd.print("IR Sensor Test");
  delay(1000);
}

void loop() {
  int irValue = digitalRead(IR);

  lcd.setCursor(0, 1);
  lcd.print("Reading: ");
  lcd.print(irValue);
  lcd.print("   "); // Clear leftover digits

  delay(200);
}