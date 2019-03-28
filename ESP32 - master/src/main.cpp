//Vít Petřík@2018
/*
Mám úžasný a vysoce funkční kódy, omluvte  prehlednost :(
*/
#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <math.h>
#include <ESPmDNS.h>
#include <Update.h>
#include <WebServer.h>
#include <ota.h>
#include <telnet.h>
#include <lamp.h>

SemaphoreHandle_t trigger_mutex = xSemaphoreCreateMutex();

uint8_t triggerPos[2] = {0, 0};
uint8_t triggerCount = 0;
uint8_t lampCount = 0;
unsigned long triggerNum = 0;
uint8_t low = 5;
uint8_t high = 255;
uint16_t interval = 100;
uint8_t radius = 25;

void lamp(void *parameters)
{
  uint8_t address = int(parameters);
  uint8_t value = 0;
  uint8_t lastValue = 0;
  uint8_t pos[2] = {0, 0};
  unsigned long lastTriggerNum = 0;
  unsigned long onMillis = 0;
  bool on = false;
  double rad;

  writeMode(address, 1);
  if (true)
  {
    commonAnode(address, false);
    writeSpeed(address, 15);
    writeSample(address, 20);
    writeThreshold(address, 50);
    autonomusHigh(address, 255);
    autonomusLow(address, 5);
    autonomusInterval(address, 5000);
  }

  uint8_t *p = readPosition(address);
  pos[0] = p[0];
  pos[1] = p[1];
  writeStringTelnet("Adresa: ");
  writeStringTelnet(String(address));
  writeStringTelnet(": X - ");
  writeStringTelnet(String(pos[0]));
  writeStringTelnet(" Y - ");
  writeStringTelnetln(String(pos[1]));

  writePWM(address, high);
  delay(1000);
  writePWM(address, low);

  while (true)
  {
    value = readTouch(address);
    if (value == 1)
    {
      writeStringTelnet("Adresa: ");
      writeStringTelnet(String(address));
      writeStringTelnet(": X - ");
      writeStringTelnet(String(pos[0]));
      writeStringTelnet(" Y - ");
      writeStringTelnet(String(pos[1]));
      writeStringTelnetln(" dotyk");
    }
    if (value == 1 && ((lastValue != value) || ((millis() - onMillis) > 500)) && triggerCount == lampCount && xSemaphoreTake(trigger_mutex, 1000 / portTICK_PERIOD_MS))
    {
      triggerCount = 0;
      triggerNum++;
      triggerPos[0] = pos[0];
      triggerPos[1] = pos[1];
      xSemaphoreGive(trigger_mutex);
      writeStringTelnet(String(address));
      writeStringTelnet(": ");
      writeStringTelnetln("dotyk!");
      lastValue = value;
    }
    else if (value == 0 && lastValue != value && triggerCount == lampCount && xSemaphoreTake(trigger_mutex, 1000 / portTICK_PERIOD_MS))
    {
      triggerCount = 0;
      triggerNum++;
      triggerPos[0] = pos[0];
      triggerPos[1] = pos[1];
      xSemaphoreGive(trigger_mutex);
      writeStringTelnet(String(address));
      writeStringTelnet(": ");
      writeStringTelnetln("Konec dotyku!");
      lastValue = value;
    }

    if (lastTriggerNum != triggerNum && xSemaphoreTake(trigger_mutex, 1 / portTICK_PERIOD_MS))
    {
      lastTriggerNum = triggerNum;
      triggerCount++;
      rad = sqrt(pow(triggerPos[0] - pos[0], 2) + pow(triggerPos[1] - pos[1], 2));
      xSemaphoreGive(trigger_mutex);
      if (rad <= radius)
      {
        if (!on)
        {
          on = true;
          writePWM(address, high);
        }
        onMillis = millis();
      }
    }

    if (on && (millis() - onMillis) > interval)
    {
      on = false;
      writePWM(address, low);
    }
    yield();
  }
  vTaskDelete(NULL);
}

void scanner(void *parameters)
{
  while (true)
  {
    for (int i = 0; i < MAX_SRV_CLIENTS; i++)
    {
      if (serverClients[i] && serverClients[i].connected())
      {
        goto escapeLoop;
      }
    }
    delay(1000);
  }

escapeLoop:
  for (int i = 4; i < 50; i++)
  {
    if (isLampHere(i))
    {
      lampCount++;
      triggerCount = lampCount;
      xTaskCreatePinnedToCore(lamp, "lamp", 5000, (void *)i, 3, NULL, 1);
    }
  }
  vTaskDelete(NULL);
}

void setup()
{
  // put your setup code here, to run once:
  delay(400);
  Serial.begin(115200);

  Wire.begin(22, 23, 60000); //ESP32 bez LoRa
  pinMode(22, INPUT);
  pinMode(23, INPUT);

  xSemaphoreGive(i2c_mutex);
  xSemaphoreGive(telnet_mutex);

  Serial.println("");
  Serial.println("");

  WiFi.softAP("ChytreLampy", "");
  //WiFi.begin("💩💩💩🦄😵🏳‍🌈", "un1corn666");

  xTaskCreatePinnedToCore(serverHandle, "server", 20000, (void *)1, 3, NULL, 1);
  xTaskCreatePinnedToCore(OTA, "OTA", 20000, (void *)1, 3, NULL, 1);
  xTaskCreatePinnedToCore(scanner, "scanner", 2000, (void *)1, 3, NULL, 1);

  vTaskDelete(NULL);
}

void loop()
{
  vTaskDelete(NULL);
}