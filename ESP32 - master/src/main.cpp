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
#include <math.h>

#define SEALEVELPRESSURE_HPA (1013.25)

Adafruit_BME280 bme;
Adafruit_SI1145 uv = Adafruit_SI1145();

SemaphoreHandle_t i2c_mutex = xSemaphoreCreateMutex();

uint8_t lampCount = 0;
uint8_t triggerY = 0;
uint8_t triggerX = 0;
uint8_t triggerCount = 0;
uint8_t low = 5;
uint8_t high = 255;
uint16_t interval = 5000;
uint8_t radius = 1;

void lampTrigger(void *parameters)
{
}

void lamp(void *parameters)
{
  uint8_t address = int(parameters);
  uint8_t value = 0;
  uint8_t lastValue = 0;
  uint8_t pos[2];
  unsigned long onMillis = 0;
  bool on = false;

  while (true)
  {
    if (xSemaphoreTake(i2c_mutex, 1 / portTICK_PERIOD_MS))
    {
      Wire.beginTransmission(address);
      if (Wire.endTransmission() != 0)
      {
        xSemaphoreGive(i2c_mutex);
        vTaskDelete(NULL);
      }
      xSemaphoreGive(i2c_mutex);
      break;
    }
  }

  lampCount++;

  delay(10);

  while (true)
  {
    if (xSemaphoreTake(i2c_mutex, 1 / portTICK_PERIOD_MS))
    {
      writeMode(address, 1);
      delayMicroseconds(1000);
      writeSpeed(address, 15);
      delayMicroseconds(1000);
      writeSample(address, 20);
      delayMicroseconds(1000);
      writeThreshold(address, 80);
      delayMicroseconds(1000);
      autonomusHigh(address, 255);
      delayMicroseconds(1000);
      autonomusLow(address, 5);
      delayMicroseconds(1000);
      autonomusInterval(address, 5000);
      xSemaphoreGive(i2c_mutex);
      break;
    }
  }

  delay(10);

  while (true)
  {
    if (xSemaphoreTake(i2c_mutex, 1 / portTICK_PERIOD_MS))
    {
      writePWM(address, 255);
      delayMicroseconds(500);
      xSemaphoreGive(i2c_mutex);
      break;
    }
  }

  delay(1000);

  while (true)
  {
    if (xSemaphoreTake(i2c_mutex, 1 / portTICK_PERIOD_MS))
    {
      writePWM(address, 5);
      delayMicroseconds(500);
      xSemaphoreGive(i2c_mutex);
      break;
    }
  }

  delay(100);

  while (true)
  {
    if (xSemaphoreTake(i2c_mutex, 50 / portTICK_PERIOD_MS))
    {
      uint8_t *p = readPosition(address);
      pos[0] = p[0];
      pos[1] = p[1];
      xSemaphoreGive(i2c_mutex);
      Serial.print("Adresa: ");
      Serial.print(address);
      Serial.print(": X - ");
      Serial.print(pos[0]);
      Serial.print(" Y - ");
      Serial.println(pos[1]);
      break;
    }
  }

  Serial.println("");
  Serial.println(lampCount);

  while (true)
  {
    if (xSemaphoreTake(i2c_mutex, 1))
    {
      value = readTouch(address);
      xSemaphoreGive(i2c_mutex);
      if (value == 1 && ((lastValue != value) || ((millis() - onMillis) > (interval - 100))))
      {
        Serial.print(address);
        Serial.print(": ");
        Serial.println("dotyk!");
        triggerCount = 0;
        triggerX = pos[0];
        triggerY = pos[1];
        lastValue = value;
      }
      else if (value == 0 && lastValue != value)
      {
        Serial.print(address);
        Serial.print(": ");
        Serial.println("Konec dotyku!");
        triggerCount = 0;
        triggerX = pos[0];
        triggerY = pos[1];
        lastValue = value;
      }
    }

    if (triggerCount != lampCount)
    {
      triggerCount++;
      double rad = ((triggerX - pos[0]) * (triggerX - pos[0])) + ((triggerY - pos[1]) * (triggerY - pos[1]));
      rad = sqrt(rad);
      Serial.print(address);
      Serial.print(" radius : ");
      Serial.println(String(rad).c_str());

      if (rad <= radius)
      {
        if (!on)
        {
          on = true;
          if (xSemaphoreTake(i2c_mutex, 100 / portTICK_PERIOD_MS))
          {
            writePWM(address, high);
            xSemaphoreGive(i2c_mutex);
          }
        }
        onMillis = millis();
      }
    }

    if (on && (millis() - onMillis) > interval)
    {
      if (xSemaphoreTake(i2c_mutex, 100 / portTICK_PERIOD_MS))
      {
        writePWM(address, low);
        xSemaphoreGive(i2c_mutex);
      }
      on = false;
    }
  }
  vTaskDelete(NULL);
}

void sensors(void *parameters)
{
  while (true)
  {
    if (xSemaphoreTake(i2c_mutex, (50 / portTICK_PERIOD_MS)))
    {
      Serial.println("");
      Serial.print("Teplota: ");
      Serial.println(String(bme.readTemperature()).c_str());
      Serial.print("Tlak: ");
      Serial.println(String(bme.readPressure()).c_str());
      Serial.print("Vlhkost: ");
      Serial.println(String(bme.readHumidity()).c_str());
      Serial.print("Viditelné světlo: ");
      Serial.println(uv.readVisible());
      Serial.print("Infra-red: ");
      Serial.println(uv.readIR());
      xSemaphoreGive(i2c_mutex);
      delay(1000);
    }
  }
}

void setup()
{
  // put your setup code here, to run once:
  delay(700);

  Serial.begin(230400);
  Wire.begin(22, 23); //ESP32 bez LoRa
  bme.begin(0x76);
  uv.begin();

  Serial.println("");
  Serial.println("");

  for (int i = 4; i < 20; i++)
  {
    xTaskCreatePinnedToCore(lamp, "lamp", 10000, (void *)i, 3, NULL, 1);
  }

  xTaskCreatePinnedToCore(sensors, "sensory", 10000, NULL, 3, NULL, 1);

  vTaskDelete(NULL);
}

void loop()
{
  vTaskDelete(NULL);
}