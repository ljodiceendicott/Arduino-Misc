// Include the Servo library 
#include <Servo.h> 
// Declare the Servo pin 
int servoPin = 3; 
// Create a servo object 
Servo Servo1; 
void setup() { 
   // We need to attach the servo to the used pin number 
   pinMode(A0, INPUT);//X
   pinMode(A1, INPUT);//Y
   Servo1.attach(servoPin); 
    Serial.begin(9600);
}
void loop(){
  int stickRead = analogRead(A1); 
  Serial.println(stickRead);
   if(stickRead >520){
    Servo1.write(0);
   }
   else if(stickRead < 500){
    Servo1.write(180);
   }
   else{
    Servo1.write(90);
   }
   // Make servo go to 0 degrees 
   //Servo1.write(0);  
   // Make servo go to 90 degrees 
   //Servo1.write(90); 
   // Make servo go to 180 degrees 
   //Servo1.write(180); 
   delay(1000); 
} 
