
int LEDPin = 8;
int PBPin= 5;
 
byte lastButtonState = LOW; // byte stores an 8-bit unsigned number from 0 to 255
byte LEDState = LOW;
 
void setup() {
pinMode(A0, INPUT);
  pinMode(LEDPin, OUTPUT);
  pinMode(PBPin, INPUT);
}
 
void loop() {
double tempReading  = analogRead(A0);
  Serial.begin(9600);
double tempK = log(10000.0 * ((1024.0 / tempReading - 1)));
tempK = 1 / (0.001129148 + (0.000234125 + (0.0000000876741 * tempK * tempK )) * tempK );     //  Temp Kelvin
float tempC = tempK - 273.15;    // Convert Kelvin to Celcius
float tempF = (tempC * 9.0)/ 5.0 + 32.0; // Convert Celcius to Fahrenheit
  byte buttonState = digitalRead(PBpin);
  if (buttonState != lastButtonState) {
    lastButtonState = buttonState;
    if (buttonState == LOW) {
      ledState = (ledState == HIGH) ? LOW: HIGH;
     if (ledState){
     Serial.println("Tempature (Celcisus)");
     Serial.println(tempC);
Else{
Serial.println("Tempature (Farenheit)");
Serial.println(tempF);
}
    }
  }
delay(1500);
}
 
