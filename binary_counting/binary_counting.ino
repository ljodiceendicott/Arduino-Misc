 int counter=0;
void setup() {
  // put your setup code here, to run once:
 //4s place
 pinMode(13, OUTPUT);
 //1s place
 pinMode(12, OUTPUT);
 //2s place
 pinMode(11, OUTPUT);


}

void loop() {
  // put your main code here, to run repeatedly:
if(counter==0){
  //4s
  digitalWrite(13,0);
  //2s
  digitalWrite(12,0);
  //1s
  digitalWrite(11,0);
}
if(counter/4==1){
  digitalWrite(13,1);
}
else{
  digitalWrite(13,0);
}
if(counter/2==1 || counter/2==3){
  digitalWrite(12,1);
}
else{
  digitalWrite(12,0);
}
if(counter%2==1){
  digitalWrite(11,1);
}
else{
  digitalWrite(11,0);
}
delay(500);
counter++;
if(counter==8){
  counter=0;
}
}
