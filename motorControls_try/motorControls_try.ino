// Luke Jodice 4/3/2023

int dirRPin = 1; //IN1 Dir Right Wheel
int pwmRPin = 2; //an1 speed Right Wheel
int dirLPin = 3; //In2 Dir Left Wheel
int pwmLPin = 4; //AN2 speed Left Wheel

const int rt=512;
const int lt=512;

void setup() {
  // put your setup code here, to run once:
  pinMode(dirRPin, OUTPUT);
  pinMode(pwmRPin, OUTPUT);
  pinMode(dirLPin, OUTPUT);
  pinMode(pwmLPin, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:

}



void forward(){//L-low R-high
  digitalWrite(dirLPin, low);
  digitalWrite(dirRPin, HIGH);

  
  //Ramp down
  for(int i=map(rt,512,1024,0,255); i>0; i--){
    analogWrite(pwmLPin,i);
    analogWrite(pwmRPin,i);
  }
  //Ramp up
  for(int i=0; i<map(rt,512,1024,0,255); i++){
    analogWrite(pwmLPin,i);
    analogWrite(pwmRPin,i);
  }
  //Ramp down
  for(int i=map(rt,512,1024,0,255); i>0; i--){
    analogWrite(pwmLPin,i);
    analogWrite(pwmRPin,i);
  }
}

void reverse(){//L-high R low
//  digitalWrite(dirLPin, HIGH);
//   digitalWrite(dirRPin, LOW);
//   //Ramp down
//   for(int i=map(lt,512,1024,0,255); i>0; i--){
//     analogWrite(pwmLPin,i);
//     analogWrite(pwmRPin,i);
//   }
//   //Ramp up
//   for(int i=0; i<map(lt,512,1024,0,255); i++){
//     analogWrite(pwmLPin,i);
//     analogWrite(pwmRPin,i);
//   }
//   //Ramp down
//   for(int i=map(lt,512,1024,0,255); i>0; i--){
//     analogWrite(pwmLPin,i);
//     analogWrite(pwmRPin,i);
//   }
}
void spinLeft(){//L-high R-high

}
void spinRight(){

}
void skidLeft(){

}
void skidRight(){

}
void braking(){

}
