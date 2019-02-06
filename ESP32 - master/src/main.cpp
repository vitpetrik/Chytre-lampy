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
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <lamp.h>

#define SEALEVELPRESSURE_HPA (1013.25)

Adafruit_BME280 bme;
Adafruit_SI1145 uv = Adafruit_SI1145();

unsigned long sensorMillis = 0;
unsigned long onMillis = 0;
boolean turnOn = false;

Adafruit_SSD1306 display(128, 64, &Wire, 16);

void lamp(void *parameters)
{

  int address = int(parameters);
  unsigned long onMillis = 0;
  boolean turnOn = false;
  int value;
  delay(2000);
  while (true)
  {
    value = readTouch(address);
    display.clearDisplay();
    display.setTextSize(4);
    display.setCursor(0, 0);
    display.println(value);
    display.display();
    delay(20);
  }
}

//začekuje všechny adresy a pokud objeví lampu, tak jí vypíše na displey
void i2cscanner(void *parameters)
{
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("I2C nalezeno na:");
  display.display();
  for (uint8_t i = 1; i < 128; i++)
  {
    Wire.beginTransmission(i);
    //pokud je přenos úspěšný, a zároveň to je vážně lampa, tak přečteme souřadnice a vypíšeme je na display
    if (Wire.endTransmission() == 0 && i != 60 && i != 96 && i != 118)
    {
      uint8_t *p = readLocation(i);
      display.print("0x");
      display.print(String(i, HEX).c_str());
      display.print(" X: 0x");
      display.print(String(p[0], HEX).c_str());
      display.print(" Y: 0x");
      display.println(String(p[1], HEX).c_str());
      display.display();
      xTaskCreatePinnedToCore(lamp, "blinky", 10000, (void *)i, 1, NULL, 1);
    }
    delayMicroseconds(10);
  }
  vTaskDelete(NULL);
}

void setup()
{
  //Nastavíme I2C sběrnici
  //Wire.begin(22, 23); //ESP32 bez LoRa
  delay(700);
  Wire.begin(4, 15); //ESP32 s LoRou
  bme.begin(0x76);
  uv.begin();

  pinMode(2, OUTPUT);

  //inicializujeme display
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.clearDisplay();
  display.setCursor(0, 0);
  display.setTextColor(WHITE);
  display.setTextSize(1);
  display.display();

  autonomusLow(0x04, 5);

  Serial.begin(115200);
  xTaskCreatePinnedToCore(i2cscanner, "scanner", 10000, (void *)1, 1, NULL, 1);
}

void loop()
{
  vTaskDelete(NULL);
}