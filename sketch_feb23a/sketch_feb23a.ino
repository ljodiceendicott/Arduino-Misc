String input;
void dot(){
  digitalWrite(8,HIGH);
  delay(500);
  digitalWrite(8,LOW);
}

void dash(){
  digitalWrite(8,HIGH);
  delay(1000);
  digitalWrite(8,LOW);
}

void setup() {
 Serial.begin(115200);
 Serial.setTimeout(1);
 pinMode(8,OUTPUT);
}
void loop() {
 while (!Serial.available());
 input = Serial.readString();
 input.toLowerCase();
 for(int i = 0; i< input.length();i++){
  char x = input.charAt(i);
        switch(x) {
            case 'a':
               dot();
               delay(200);
               dash();
               delay(1500);
                break;
            case 'b':
                dash();
                delay(200);
                dot();
                delay(200);
                dot();
                delay(200);
                dot();
                delay(1500);
                 break;
            case 'c':
                dash();
                delay(200);
                dot();
                delay(200);
                dash();
                delay(200);
                dot();
                delay(1500);
                break;
            case 'd':
                dash();
                delay(200);
                dot();
                delay(200);
                dot();
                delay(1500);
                break;
            case 'e':
                dot();
                delay(1500);
                break;
                
            case 'f':
                dot();
                delay(200);
                dot();
                delay(200);
                dash();
                delay(1500);
                break;
             case 'g':
                dash();
                delay(200);
                dash();
                delay(200);
                dot();
                delay(1500);
                break;
             case 'h':
                dot();
                delay(200);
                dot();
                delay(200);
                dot();
                delay(200);
                dot();
                delay(1500);
                break;
             case 'i':
                dot();
                delay(200);
                dot();
                delay(1500);
                break;
             case 'j':
                dot();
                delay(200);
                dash();
                delay(200);
                dash();
                delay(200);
                dash();
                delay(1500);
                break;
              case 'k':
                dash();
                delay(200);
                dot();
                delay(200);
                dash();
                delay(1500);  
                break;
              case 'l':
                dot();
                delay(200);
                dash();
                delay(200);
                dot();
                delay(200);
                dot();
                delay(1500);
                break;
              case 'm':
                dash();
                delay(200);
                dash();
                delay(1500);
                break;
              case 'n':
                dash();
                delay(200);
                dot();
                delay(1500);
                break;
              case 'o':
                dash();
                delay(200);
                dash();
                delay(200);
                dash();
                delay(1500);
                break;
              case 'p':
                dot();
                delay(200);
                dash();
                delay(200);
                dash();
                delay(200);
                dot();
                delay(1500);
                break;
              case 'q':
                dash();
                delay(200);
                dash();
                delay(200);
                dot();
                delay(200);
                dash();
                break;
              case 'r':
                dot();
                delay(200);
                dash();
                delay(200);
                dot();
                delay(1500);
                break;
              case 's':
                dot();
                delay(200);
                dot();
                delay(200);
                dot();
                delay(1500);
                break;
              case 't':
                dash();
                delay(1500);
                break;
              case 'u':
                dot();
                delay(200);
                dot();
                delay(200);
                dash();
                delay(1500);
                break;
              case 'v':
                dot();
                delay(200);
                dot();
                delay(200);
                dot();
                delay(200);
                dash();
                delay(1500);
                break;
              case 'w':
                dot();
                delay(200);
                dash();
                delay(200);
                dash();
                delay(1500);  
                break;
              case 'y':
                dash();
                delay(200);
                dot();
                delay(200);
                dash();
                delay(200);
                dash();
                delay(1500);
                break;
              case 'z':
                dash();
                delay(200);
                dash();
                delay(200);
                dot();
                delay(200);
                dot();
                delay(1500);
                break;
              case ' ':
                delay(3000);
                break;
            default:
                delay(4000);
                break;
            }
 }

 
}
