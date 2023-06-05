int yellow_LED = 2;
int white_LED = 4;
int green_LED = 7;
void setup() {
  // put your setup code here, to run once:
  pinMode(2, OUTPUT);
  
}

void loop() {
  // put your main code here, to run repeatedly:
  digitalWrite(yellow_LED, HIGH);
  delay(1000);
  digitalWrite(yellow_LED, LOW);
  digitalWrite(white_LED,HIGH);
  delay(1000);
  digitalWrite(white_LED,LOW);
  digitalWrite(green_LED,HIGH);
  delay(1000);
  digitalWrite(green_LED,LOW);
}
