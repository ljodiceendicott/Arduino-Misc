const int trigleft = 7;
const int echoleft = 6;
const int greenleft = 34;
const int redleft = 32;
long leftduration = 0;
int leftdist = 0;

const int trigcenter = 24;
const int echocenter = 22;
const int greencenter = 38;
const int redcenter = 36;
long centerduration;
int centerdist;

const int trigright = 28;
const int echoright = 26;
const int greenright = 42;
const int redright = 40;
long rightduration;
int rightdist;

void setup() {
  // put your setup code here, to run once:
Serial.begin(9600);

  pinMode(greenleft, OUTPUT);
  pinMode(greencenter, OUTPUT);
  pinMode(greenright, OUTPUT);

  pinMode(redleft, OUTPUT);
  pinMode(redcenter, OUTPUT);
  pinMode(redright, OUTPUT);

  pinMode(trigleft, OUTPUT);
  pinMode(trigcenter, OUTPUT);
  pinMode(trigright, OUTPUT);

  pinMode(echoleft,  INPUT);
  pinMode(echocenter, INPUT);
  pinMode(echoright, INPUT);
}

void loop() {
//   // put your main code here, to run repeatedly:
 // Clears the trigPin
  digitalWrite(trigleft, LOW);
  delayMicroseconds(2);
  // Sets the trigleft on HIGH state for 10 micro seconds
  digitalWrite(trigleft, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigleft, LOW);
  leftduration = pulseIn(echoleft, HIGH);
  // Calculating the distance
  leftdist = leftduration * 0.034 / 2;


 // Clears the trigPin
  digitalWrite(trigright, LOW);
  delayMicroseconds(2);
  // Sets the trigright on HIGH state for 10 micro seconds
  digitalWrite(trigright, HIGH);
  delayMicroseconds(10);
  rightduration = pulseIn(echoright, HIGH);
  // Calculating the distance
  rightdist = rightduration * 0.034 / 2;

 // Clears the trigPin
digitalWrite(trigcenter, LOW);
delayMicroseconds(2);
// Sets the trigcenter on HIGH state for 10 micro seconds
digitalWrite(trigcenter, HIGH);
delayMicroseconds(10);
centerduration = pulseIn(echocenter, HIGH);
// Calculating the distance
centerdist = centerduration * 0.034 / 2;

  if(leftdist<30){//object in the way
  digitalWrite(redleft,HIGH);
  digitalWrite(greenleft,LOW);
  }
  else{//clear
  digitalWrite(redleft,LOW);
  digitalWrite(greenleft,HIGH);
  }
 if(centerdist<30){//object in the way
  digitalWrite(redcenter,HIGH);
  digitalWrite(greencenter,LOW);
  }
  else{//Clear
  digitalWrite(redcenter,LOW);
  digitalWrite(greencenter,HIGH);
  }

 if(rightdist<30){//object in the way
  digitalWrite(redright,HIGH);
  digitalWrite(greenright,LOW);}
  else{//Clear
  digitalWrite(redright,LOW);
  digitalWrite(greenright,HIGH);
  }
  // Serial.println("Left");
  // Serial.println(leftdist);
  // // Serial.println("center");
  // Serial.println(centerdist);
  Serial.print("center ");
  Serial.println(centerdist);
}



