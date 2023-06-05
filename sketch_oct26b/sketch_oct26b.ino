int ledArray[] = {12, 9, 5, 2};// ledArray[0] is green, ledArray[1] is red, ledArray[2] is yellow, and ledArray[3] is blue.
int buttonArray[] = {13, 10, 6, 3};// buttonArray[0] is green, buttonArray[1] is red, buttonArray[2] is yellow, and buttonArray[3] is blue.
int pinCount = 4;
int passPin = 7;
bool isPassing = true;
int gameState = 0;
int currLevel = 1;
void setLed(int lednum);
void gameOver();
void setup() {
  Serial.begin(9600);
  // put your setup code here, to run once:
pinMode(passPin, OUTPUT);

 for(int pin = 0; pin<pinCount; pin++){
      pinMode(ledArray[pin], OUTPUT);
      pinMode(buttonArray[pin],INPUT);
      }
}

void loop() {
  if(gameState ==0){
  digitalWrite(passPin, HIGH);
  // put your main code here, to run repeatedly:
   //green
   if (digitalRead(13)==1){
    Serial.println("its a 1");
   }
   else if(digitalRead(13)==0){
    Serial.println("its a 0");
    digitalWrite(12, HIGH);
    delay(500);
    digitalWrite(12, LOW);
   }
   //Red
      if (digitalRead(10)==0){
    Serial.println("its a 1");
   }
   else if(digitalRead(10)==1){
    Serial.println("its a 0");
    digitalWrite(9, HIGH);
    delay(500);
    digitalWrite(9, LOW);
   }
   //Yellow
      if (digitalRead(6)==0){
    Serial.println("its a 1");
   }
   else if(digitalRead(6)==1){
    Serial.println("its a 0");
    digitalWrite(5, HIGH);
    delay(500);
    digitalWrite(5, LOW);
   }
   //blue
      if (digitalRead(3)==0){
    Serial.println("its a 1");
   }
   else if(digitalRead(3)==1){
    Serial.println("its a 0");
    digitalWrite(2, HIGH);
    delay(500);
    digitalWrite(2, LOW);
    gameState = gameState + 1;
    delay(2000);
    digitalWrite(9,HIGH);
    delay(1500);
    digitalWrite(9,LOW);
   }
  }
  if(gameState == 1){
    if(digitalRead(10)==1){
      digitalWrite(9,HIGH);
    delay(500);
    digitalWrite(9,LOW);
    delay(1500);
    gameState++;
    digitalWrite(5,HIGH);
    delay(2000);
    digitalWrite(5,LOW);
    digitalWrite(12,HIGH);
    delay(2000);
    digitalWrite(12,LOW); 
    }
    else if(digitalRead(6)==1){
      digitalWrite(passPin,LOW);
    }
    else if(digitalRead(3)==1){
      digitalWrite(passPin,LOW);
    }
    else if(digitalRead(13)==0){
      digitalWrite(passPin,LOW);
    }
  }
  if(gameState == 2){
    if(digitalRead(6)==1){
      digitalWrite(5,HIGH);
      delay(1500);
      digitalWrite(5,LOW);
      gameState++;
    }
    else if(digitalRead(10)==1){
      digitalWrite(passPin,LOW);
    }
    else if(digitalRead(3)==1){
      digitalWrite(passPin,LOW);
    }
    else if(digitalRead(13)==0){
      digitalWrite(passPin,LOW);
    }
  }
  if(gameState == 3){
      if(digitalRead(13)==0){
      digitalWrite(12,HIGH);
      delay(1500);
      digitalWrite(12,LOW);
      gameState++;
      digitalWrite(9,HIGH);
      delay(2000);
      digitalWrite(9,LOW);
      digitalWrite(2,HIGH);
      delay(2000);
      digitalWrite(2,LOW);
      digitalWrite(12,HIGH);
      delay(2000);
      digitalWrite(12,LOW); 
    }
    else if(digitalRead(10)==1){
      digitalWrite(passPin,LOW);
    }
    else if(digitalRead(3)==1){
      digitalWrite(passPin,LOW);
    }
    else if(digitalRead(6)==1){
      digitalWrite(passPin,LOW);
    }
  }
  if(gameState == 4){
   if(digitalRead(10)==1){
      digitalWrite(9,HIGH);
    delay(2000);
    digitalWrite(9,LOW);
    gameState++;
    delay(200);
  }
     else if(digitalRead(13)==0){
      digitalWrite(passPin,LOW);
    }
    else if(digitalRead(3)==1){
      digitalWrite(passPin,LOW);
    }
    else if(digitalRead(6)==1){
      digitalWrite(passPin,LOW);
    }
  }
    if(gameState == 5){
   if(digitalRead(3)==1){
      digitalWrite(2,HIGH);
    delay(2000);
    digitalWrite(2,LOW);
    gameState++;
    delay(200);
  }
     else if(digitalRead(13)==0){
      digitalWrite(passPin,LOW);
    }
    else if(digitalRead(10)==1){
      digitalWrite(passPin,LOW);
    }
    else if(digitalRead(6)==1){
      digitalWrite(passPin,LOW);
    }
  }
  if(gameState == 6){
   if(digitalRead(13)==0){
      digitalWrite(12,HIGH);
    delay(2000);
    digitalWrite(12,LOW);
    gameState++;
    delay(200);
  }
     else if(digitalRead(3)==1){
      digitalWrite(passPin,LOW);
    }
    else if(digitalRead(10)==1){
      digitalWrite(passPin,LOW);
    }
    else if(digitalRead(6)==1){
      digitalWrite(passPin,LOW);
    }
  }
}
