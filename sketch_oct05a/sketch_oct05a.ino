void setup() {
  // put your setup code here, to run once:
 pinMode(A0, INPUT);
 pinMode(7, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  double tempReading  = analogRead(A0);
  Serial.begin(9600);

Serial.println("Raw Temp Value:");
Serial.println(tempReading);

double tempK = log(10000.0 * ((1024.0 / tempReading - 1)));
tempK = 1 / (0.001129148 + (0.000234125 + (0.0000000876741 * tempK * tempK )) * tempK );     //  Temp Kelvin
float tempC = tempK - 273.15;    // Convert Kelvin to Celcius
float tempF = (tempC * 9.0)/ 5.0 + 32.0; // Convert Celcius to Fahrenheit

if(tempF >80){
  digitalWrite(7,HIGH);
}
else{
  digitalWrite(7,LOW);
}
Serial.println("Tempature (Farenheit)");
Serial.println(tempF);
delay(1500);   //make it easier to read
}
