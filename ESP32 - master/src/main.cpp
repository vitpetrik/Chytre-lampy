//Vít Petřík@2018
#include <Arduino.h>
#include <Wire.h>

boolean turnOn = false;
long millisOn = -1000;
int threshold = 700;
int interval = 1000;
int value;
int lastValue;
long millisFPS = 0;
int loopcounter = 0;
long ref = 0;
byte PWMvalue = 0;

int readI2C(int address, int bytes){
  int x = 0;
  Wire.requestFrom(address, bytes);    //zažádá 2 byty na adrese 0x04
  x = Wire.read();      
  x = x << 8;
  x += Wire.read();         //uloží hodnotu do INTu po 2 bytech ( bitový posun dolevo o 8 )
  return x;
}
 
void setup() {
  delay(500);
  Wire.begin(22,23);
  Serial.begin(230400);
  Wire.beginTransmission(0x04);
  Wire.write(0);       
  Wire.endTransmission();   
}

void loop() {

    value = readI2C(0x04, 2);
  Serial.println(value);        //pošle hodnotu na serial... jen tak pro srandu 
  if(value > threshold){        //pokud hodnota přesáhne threshold pošle 255 na zařízení a uloží čas ( millisOn )
         Wire.beginTransmission(0x04);
     Wire.write(255);       
     Wire.endTransmission();   }
     else{
            Wire.beginTransmission(0x04);
     Wire.write(0);       
     Wire.endTransmission();   
     }

}