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

void koko(void *parameters)
{
  uint8_t address = int(parameters);
  uint8_t value = 0;
  uint8_t lastValue = 0;

  Wire.beginTransmission(address);
  if (Wire.endTransmission() != 0)
  {
    vTaskDelete(NULL);
  }

  Serial.println("");
  Serial.print("Lampa nalezena na I2C adrese: ");
  Serial.println(address);
  Serial.println("");

  delay(100);

  uint8_t *p = readLocation(address);
  Serial.println(p[0]);
  Serial.println(p[1]);
  delay(100);
  writePWM(address, 255);
  while (true)
  {

    value = readTouch(address);
    Serial.println(value);
    delay(20);
  }
}

void setup()
{
  //Nastavíme I2C sběrnici

  delay(700);
  //Wire.begin(4, 15); //ESP32 s LoRou
  Wire.begin(22, 23); //ESP32 bez LoRa
  bme.begin(0x76);
  uv.begin();
  pinMode(2, OUTPUT);

  //inicializujeme display

  Serial.begin(115200);
  Serial.println("");

  for (int i = 1; i < 50; i++)
  {
    xTaskCreatePinnedToCore(koko, "lamp", 10000, (void *)i, 3, NULL, 1);
    delayMicroseconds(200);
  }
  vTaskDelete(NULL);
}

void loop()
{
  vTaskDelete(NULL);
}