int redLEDPin=9; //Declare redLEDPin an int set to pin 9
int yellowLEDPin=10; //Declare yellowLEDPin an int set to pin 10
int redOnTime=250; //Declare redOnTime an int set to 250 mseconds
int redOffTime=250; //Declare redOffTime an int set to 250
int yellowOnTime=250; //Declare yellowOnTime an int set to 250
int yellowOffTime=250; //Declare yellowOffTime an int set to 250


void setup() {
  pinMode(redLEDPin, OUTPUT);  // Set redLEDPin as output
  pinMode(yellowLEDPin, OUTPUT);  //Set YellowLEDPin as output
   // initialize serial communication at 9600 bits per second:
  Serial.begin(9600);
  // set your pushbutton's pin as an input:
  pinMode(5, INPUT);
}

void loop() {
  
  if(digitalRead(5)==HIGH){
 for ( int i = 0; i<5; i++){
  if(i<2){
    digitalWrite(redLEDPin,HIGH); //Turn red LED on
    delay(redOnTime*.5);           //Leave on for redOnTime
    digitalWrite(redLEDPin,LOW);  //Turn red LED off
    delay(redOffTime*.5);          //Leave off for redOffTime
  }
  else{
digitalWrite(redLEDPin,HIGH); //Turn red LED on
delay(redOnTime);           //Leave on for redOnTime
digitalWrite(redLEDPin,LOW);  //Turn red LED off
delay(redOffTime);          //Leave off for redOffTime
  }
 }


for(int i =0; i<5; i++){
if(i<2){
digitalWrite(yellowLEDPin,HIGH); //Turn yellow LED on
delay(yellowOnTime/2);          //Leave on for yellowOnTime
digitalWrite(yellowLEDPin,LOW);  //Turn yellow LED off
delay(yellowOffTime/2);   //Leave off for yellowOffTime
}

 else{
digitalWrite(yellowLEDPin,HIGH); //Turn yellow LED on
delay(yellowOnTime);          //Leave on for yellowOnTime
digitalWrite(yellowLEDPin,LOW);  //Turn yellow LED off
delay(yellowOffTime);   //Leave off for yellowOffTime
 }
}
  }
}
