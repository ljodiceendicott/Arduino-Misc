const int trigleft = 24;
const int echoleft = 25;
const int greenleft = 34;
const int redleft = 32;
long leftduration;
int leftdist;

void setup() {
  // put your setup code here, to run once:
Serial.begin(9600);
pinMode(echoleft,  INPUT);
pinMode(trigleft, OUTPUT);
pinMode(redleft, OUTPUT);
pinMode(greenleft, OUTPUT);

}

void loop() {
  // put your main code here, to run repeatedly:
digitalWrite(trigleft, LOW);
  delayMicroseconds(2);
  // Sets the trigleft on HIGH state for 10 micro seconds
  digitalWrite(trigleft, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigleft, LOW);
  leftduration = pulseIn(echoleft, HIGH);
  // Calculating the distance
  leftdist = leftduration * 0.034 / 2;
  Serial.println(leftdist);
}
