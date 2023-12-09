#include <Arduino.h>


void resetAll() 
{
  delay(2);
  digitalWrite(2, LOW);
  digitalWrite(3, LOW);
  digitalWrite(4, LOW);
  digitalWrite(5, LOW);
  delay(2);
}


void setup(){
  Serial.begin(115200);  //link to PC
  
  pinMode(2, OUTPUT);
  pinMode(3, OUTPUT);
  pinMode(4, OUTPUT);
  pinMode(5, OUTPUT);

  digitalWrite(2, LOW);
  digitalWrite(3, LOW);
  digitalWrite(4, LOW);
  digitalWrite(5, LOW);

}


void loop() {


  resetAll();
  digitalWrite(2, HIGH);

  int a = map(-80,1023,0,1023,analogRead(A0));
  
  delay(10);

  resetAll();
  digitalWrite(3, HIGH);
  int b = map(-80,1023,0,1023,analogRead(A0));

  delay(10);

  resetAll();
  digitalWrite(4, HIGH);

  int c = map(-80,1023,0,1023,analogRead(A0));
  
  delay(10);

  resetAll();
  digitalWrite(5, HIGH);
  int d = map(-80,1023,0,1023,analogRead(A0));

  Serial.println(">Analogread a(2):" + (String)a);
  Serial.println(">Analogread b(3):" + (String)b);
  Serial.println(">Analogread c(4):" + (String)c);
  Serial.println(">Analogread d(5):" + (String)d);

  delay(10);


}

