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
SemaphoreHandle_t trigger_mutex = xSemaphoreCreateMutex();

uint8_t lampCount = 0;
uint8_t triggerPos[2] = {0, 0};
uint8_t triggerCount = 0;
unsigned long triggerNum = 0;
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
  uint8_t pos[2] = {0, 0};
  unsigned long lastTriggerNum = 1000;
  unsigned long onMillis = 0;
  bool on = false;
  double rad;

  if (xSemaphoreTake(i2c_mutex, 1000 / portTICK_PERIOD_MS) == pdTRUE)
  {
    Wire.beginTransmission(address);
    if (Wire.endTransmission() != 0)
    {
      xSemaphoreGive(i2c_mutex);
      vTaskDelete(NULL);
    }
    lampCount++;
    triggerCount++;
    xSemaphoreGive(i2c_mutex);
  }
  else
  {
    ESP.restart();
  }

  delay(100);

  if (xSemaphoreTake(i2c_mutex, 1000 / portTICK_PERIOD_MS) == pdTRUE)
  {
    writeMode(address, 1);
    writeSpeed(address, 15);
    writeSample(address, 20);
    writeThreshold(address, 80);
    autonomusHigh(address, 255);
    autonomusLow(address, 5);
    autonomusInterval(address, 5000);
    xSemaphoreGive(i2c_mutex);
  }
  else
  {
    ESP.restart();
  }

  delay(100);

  if (xSemaphoreTake(i2c_mutex, 1000 / portTICK_PERIOD_MS))
  {
    writePWM(address, 255);
    xSemaphoreGive(i2c_mutex);
  }
  else
  {
    ESP.restart();
  }

  delay(1000);

  if (xSemaphoreTake(i2c_mutex, 1000 / portTICK_PERIOD_MS))
  {
    writePWM(address, 5);
    xSemaphoreGive(i2c_mutex);
  }
  else
  {
    ESP.restart();
  }

  delay(100);

  if (xSemaphoreTake(i2c_mutex, 1000 / portTICK_PERIOD_MS) == pdTRUE)
  {
    uint8_t *p = readPosition(address);
    xSemaphoreGive(i2c_mutex);
    pos[0] = p[0];
    pos[1] = p[1];
    Serial.print("Adresa: ");
    Serial.print(address);
    Serial.print(": X - ");
    Serial.print(pos[0]);
    Serial.print(" Y - ");
    Serial.println(pos[1]);
  }
  else
  {
    ESP.restart();
  }

  while (true)
  {
    if (xSemaphoreTake(i2c_mutex, 1))
    {
      value = readTouch(address);
      xSemaphoreGive(i2c_mutex);
      if (value == 1 && ((lastValue != value) || ((millis() - onMillis) > 500)) && triggerCount == lampCount && xSemaphoreTake(trigger_mutex, 1000 / portTICK_PERIOD_MS))
      {
        triggerCount = 0;
        triggerNum++;
        triggerPos[0] = pos[0];
        triggerPos[1] = pos[1];
        xSemaphoreGive(trigger_mutex);
        Serial.print(address);
        Serial.print(": ");
        Serial.println("dotyk!");
        lastValue = value;
      }
      else if (value == 0 && lastValue != value && triggerCount == lampCount && xSemaphoreTake(trigger_mutex, 1000 / portTICK_PERIOD_MS))
      {
        triggerCount = 0;
        triggerNum++;
        triggerPos[0] = pos[0];
        triggerPos[1] = pos[1];
        xSemaphoreGive(trigger_mutex);
        Serial.print(address);
        Serial.print(": ");
        Serial.println("Konec dotyku!");
        lastValue = value;
      }
    }

    if (lastTriggerNum != triggerNum && xSemaphoreTake(trigger_mutex, 1000 / portTICK_PERIOD_MS))
    {
      lastTriggerNum = triggerNum;
      triggerCount++;
      rad = sqrt(((triggerPos[0] - pos[0]) * (triggerPos[0] - pos[0])) + ((triggerPos[1] - pos[1]) * (triggerPos[1] - pos[1])));
      xSemaphoreGive(trigger_mutex);
      if (rad <= radius)
      {
        if (!on)
        {
          on = true;
          if (xSemaphoreTake(i2c_mutex, 1000 / portTICK_PERIOD_MS))
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
      if (xSemaphoreTake(i2c_mutex, 1000 / portTICK_PERIOD_MS))
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
  delay(400);

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