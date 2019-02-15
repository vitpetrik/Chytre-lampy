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

SemaphoreHandle_t i2c_mutex = xSemaphoreCreateMutex();

void koko(void *parameters)
{
  uint8_t address = int(parameters);
  uint8_t value = 0;
  uint8_t lastValue = 0;
  while (true) {
    if (xSemaphoreTake(i2c_mutex, (1 / portTICK_PERIOD_MS))) {
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

  delay(10);

  while (true) {
    if (xSemaphoreTake(i2c_mutex, (1 / portTICK_PERIOD_MS))) {
      writePWM(address, 255);
      xSemaphoreGive(i2c_mutex);
      break;
    }
  }

  delay(1000);

  while (true) {
    if (xSemaphoreTake(i2c_mutex, (1 / portTICK_PERIOD_MS))) {
      writePWM(address, 5);
      xSemaphoreGive(i2c_mutex);
      break;
    }
  }

  while (true) {
    if (xSemaphoreTake(i2c_mutex, (1 / portTICK_PERIOD_MS))) {
      uint8_t *pos = readPosition(address);
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

  while (true) {
    if (xSemaphoreTake(i2c_mutex, (1 / portTICK_PERIOD_MS))) {
      writeMode(address, 1);
      writeSpeed(address, 25);
      writeSample(address, 20);
      writeThreshold(address, 40);
      xSemaphoreGive(i2c_mutex);
      break;
    }
  }

  while (true) {
    if (xSemaphoreTake(i2c_mutex, (1 / portTICK_PERIOD_MS))) {
      value = readTouch(address);
      xSemaphoreGive(i2c_mutex);
      if (value == 1 && lastValue != value) {
        Serial.print(address);
        Serial.print(": ");
        Serial.println("dotyk!");
      }
      else if (value == 0 && lastValue != value) {
        Serial.print(address);
        Serial.print(": ");
        Serial.println("Konec dotyku!");
      }
      lastValue = value;
      delay(10);
    }
  }
  vTaskDelete(NULL);
}

void setup() {
  // put your setup code here, to run once:
  delay(700);
  Serial.begin(230400);
  Wire.begin(22, 23); //ESP32 bez LoRa

  Serial.println("");
  Serial.println("");

  for (int i = 4; i < 20; i++) {
    xTaskCreatePinnedToCore(koko, "lamp", 10000, (void *)i, 3, NULL, 1);
  }

  vTaskDelete(NULL);
}

void loop()
{
  vTaskDelete(NULL);
}