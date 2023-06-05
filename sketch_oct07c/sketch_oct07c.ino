int PWMOutPin =  10 ;
int PreviousPotValue = 0;
int Threshold = 0;

void setup() {
  Serial.begin(9600);
}

void loop() {
  int PotValue =  analogRead(A0);
  int PWMValue = PotValue / 4;
  analogWrite(PWMOutPin, PWMValue);
  Threshold = abs(PotValue - PreviousPotValue);
  if (Threshold >= 10) {
  int PWM_DutyCycle = ((float)PWMValue / 255.0) * 100.0;
  Serial.print("PWM Duty Cycle = ");
  Serial.print(PWM_DutyCycle);
  Serial.println("%");
  PreviousPotValue = PotValue;
  }

  delay(2);
}
