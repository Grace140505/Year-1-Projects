#include <LiquidCrystal.h>

// LCD pins
LiquidCrystal lcd(8, 9, 4, 5, 6, 7);

// Encoder pin
const int ENC = A1;   // right encoder D0 -> A1
int lastState = HIGH;
unsigned long ticks = 0;

void setup() {
  lcd.begin(16, 2);
  lcd.print("Rotate wheel");

  pinMode(ENC, INPUT_PULLUP);
}

void loop() {
  int state = digitalRead(ENC);

  // Detect rising edge = one tick
  if (lastState == LOW && state == HIGH) {
    ticks++;
  }
  lastState = state;

  lcd.setCursor(0,1);
  lcd.print("Ticks: ");
  lcd.print(ticks);
  lcd.print("    "); // clear old digits
}
