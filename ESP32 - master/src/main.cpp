//Vít Petřík@2018
/*
Mám úžasný a vysoce funkční kódy, omluvte  prehlednost :(
*/
#include <Arduino.h>
#include <Wire.h>
#include <PubSubClient.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include "Adafruit_SI1145.h"
#include <lamp.h>

#define SEALEVELPRESSURE_HPA (1013.25)

Adafruit_BME280 bme;
Adafruit_SI1145 uv = Adafruit_SI1145();

unsigned long sensorMillis = 0;
unsigned long onMillis = 0;
boolean turnOn = false;

void lamp(void *parameters)
{
  int address = int(parameters);
  delay(20);
  uint8_t *p = readLocation(address);
  uint8_t value = 0;
  uint8_t lastValue = 0;
  Serial.println("--------------");
  Serial.print("Lampa nalezena na I2C adrese: ");
  Serial.println(address);
  Serial.println("Se souřadnicemi: ");
  Serial.print("X: ");
  Serial.println(String(p[0], HEX).c_str());
  Serial.print("Y: ");
  Serial.println(String(p[1], HEX).c_str());
  Serial.println("--------------");
  Serial.println("");
  while (true)
  {
    value = readTouch(address);
    if (value != lastValue)
    {
      Serial.println("--------------");
      Serial.println("Lampa se souřadnicemi: ");
      Serial.print("X: ");
      Serial.println(String(p[0], HEX).c_str());
      Serial.print("Y: ");
      Serial.println(String(p[1], HEX).c_str());
      if (value == 0)
      {
        Serial.println("Byla vypnuta!");
      }
      else if (value == 1)
      {
        Serial.println("je zapnuta bez dotyku!");
      }
      else if (value == 2)
      {
        Serial.println("Někdo se dotknul lampy!");
      }
      Serial.println("--------------");
      Serial.println("");
    }
    lastValue = value;
    delay(20);
  }
}

//začekuje všechny adresy a pokud objeví lampu, tak jí vypíše na displey
void i2cscanner(void *parameters)
{
  for (uint8_t i = 1; i < 128; i++)
  {
    Wire.beginTransmission(i);
    //pokud je přenos úspěšný, a zároveň to je vážně lampa, tak přečteme souřadnice a vypíšeme je na display
    if (Wire.endTransmission() == 0 && i != 60 && i != 96 && i != 118)
    {
      xTaskCreatePinnedToCore(lamp, "blinky", 10000, (void *)i, 1, NULL, 1);
    }
    delayMicroseconds(1);
  }
  vTaskDelete(NULL);
}

void setup()
{
  //Nastavíme I2C sběrnici

  delay(700);
  //Wire.begin(4, 15); //ESP32 s LoRou
  Wire.begin(22, 23, 100000L); //ESP32 bez LoRa
  bme.begin(0x76);
  uv.begin();

  pinMode(2, OUTPUT);

  //inicializujeme display

  Serial.begin(115200);
  xTaskCreatePinnedToCore(i2cscanner, "scanner", 10000, (void *)1, 1, NULL, 1);
}

void loop()
{
  vTaskDelete(NULL);
}