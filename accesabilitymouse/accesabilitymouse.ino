//#include <Mouse.h>

int mbLeft = 2;
//int mbMiddle = 2;
int mbRight = 4;

//char scroll = "A1";

//char mx = "A5";
//char my = "A4";

int currMx = 0;
int currMy = 0;

int range =12; 
int responseDelay = 5;
int threshold = range /4;
int center = range/ 2;

bool mouseisActive = false;
int lastSwitchState = LOW; 

void setup() {
  Serial.begin(9600);
 //This is the mouse pins
  //LEFT-MOUSE CLICK
  pinMode(mbLeft,INPUT);
  //MIDDLE MOUSE-CLICK
//  pinMode(mbMiddle,INPUT);
//  //RIGHT MOUSE-CLICK
  pinMode(mbRight,INPUT);

 //this is the scroll "wheel"
  //SCROLL-UP  
  pinMode(A1,INPUT);

 //this is the Mouse coords
  //MOUSE-X
  pinMode(A5,INPUT);
  //MOUSE-Y
  pinMode(A4,INPUT);
}

void loop() {

int prevbutton= 0; 
int mbLeft = 4;
int mbMiddle = 2;
int mbRight = 5;

int leftval = 0;
leftval = digitalRead(mbLeft);

int middleval = 0;
middleval = digitalRead(mbMiddle);

int rightval = 0;
rightval = digitalRead(mbRight);

//Mouse buttons
if(leftval > 0){
  //leftclick
  Serial.println("left Click");
}
else if(middleval > 0){
  //middleclick
}
else if(rightval){
  //rightclick
  Serial.println("right Click");
}
else{
  leftval = 0;
  middleval = 0;
  rightval = 0;
}
// Mouse Movement
  int stickX = analogRead(A5);
  int stickY = analogRead(A4);
//  //higher->down
//  //510-525 is the deadzone for x and Y

//diagnals
if (stickX > 525 && stickY >525){
  //up right
  Serial.println("mouse up right");
  Serial.print(stickX);
  Serial.println(stickY);
}
else if(stickX > 525 && stickY <510){
 //down left
  Serial.println("mouse up right"); 
  Serial.print(stickX);
  Serial.println(stickY);}
else if(stickX < 510 && stickY > 525){
  //up right
  Serial.print(stickX);
  Serial.println(stickY);}
else if(stickX >510 && stickY <510){
  //up left
  Serial.print(stickX);
  Serial.println(stickY);
}

//Basic Directions
if (stickX > 525){
  Serial.println("mouse to the right");
}
else if(stickX <510){
  //move to the left
  Serial.println("mouse to the left");
}

if(stickY >525){
  //move up
  Serial.println("mouse up");
}
else if(stickY <510){
  //move down
  Serial.println("mouse down");
}
//Scrolling 
  int scroll = analogRead(A1);
  //495-510 is the deadzone

  if(scroll >510){
    //scroll up
    Serial.println("Scroll up");
  }
  else if(scroll <495){
    //scroll down
    Serial.println("Scroll down");
  }

}
