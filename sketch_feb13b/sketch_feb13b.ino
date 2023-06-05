#include <LiquidCrystal.h>

LiquidCrystal lcd(12,11,5,4,3,2);

void setup(){
  lcd.begin(16,2);
  lcd.print("hello");
}
void loop(){
  lcd.print("ITS NOT A BOMB");
  delay(500);
  lcd.clear();
  delay(500);
}