int trigPin =3;
int echoPin= 2;
int fanPin= 7;
long duration; // variable for the duration of sound wave travel
int distance; // variable for the distance measurement
char sendnum;
int sendint;

void setup() {
  pinMode(trigPin, OUTPUT); // Sets the trigPin as an OUTPUT
  pinMode(echoPin, INPUT); // Sets the echoPin as an INPUT
  pinMode(fanPin, OUTPUT);
  Serial.begin(9600); // // Serial Communication is starting with 9600 of baudrate speed
}
void loop() {
  // Clears the trigPin condition
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  // Sets the trigPin HIGH (ACTIVE) for 10 microseconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(echoPin, HIGH);
  // Calculating the distance
  distance = duration * 0.034 / 2; // Speed of sound wave divided by 2 (go and back)
  if(distance>10){
    sendnum='0';
    sendint=0;
  }
  else{
    sendnum='1';
    sendint=0;
  }
  delay(200);
  if(Serial.availableForWrite());
  Serial.write(sendnum);  
   if(sendint==1){
    digitalWrite(fanPin,HIGH);  
  }
  else{
    digitalWrite(fanPin,LOW);
  }
  }
