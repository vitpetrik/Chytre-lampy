#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <math.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>

// FreeRTOS Semaphore pro zamezeni konfliktu při přistupování triggrovacích proměnných
SemaphoreHandle_t trigger_mutex = xSemaphoreCreateMutex();
SemaphoreHandle_t lamp_mutex = xSemaphoreCreateMutex();

// init globalnich promennych
uint8_t triggerPos[2] = {0, 0};
uint8_t triggerCount = 0;
unsigned long triggerNum = 0;
uint8_t lampCount = 0;
uint8_t low = 5;
uint8_t high = 255;
uint16_t interval = 1000;
uint8_t radius = 25;
bool easterEgg = false;

#include <ota.h>
#include <telnet.h>
#include <lamp.h>

//Task pro ošéfení poloměru
void lampTrigger(void *parameters)
{
  //inicializace
  uint8_t address = int(parameters);
  uint8_t pos[2] = {0, 0};

  unsigned long onMillis = 0;
  unsigned long lastTrigger = 0;
  bool on = false;
  double rad;

  uint8_t *p = readPosition(address);
  pos[0] = p[0];
  pos[1] = p[1];

  while (true)
  {
    if (xSemaphoreTake(trigger_mutex, 20) == pdTRUE)
    {
      if (triggerNum != lastTrigger)
      {
        rad = sqrt(pow(pos[0] - triggerPos[0], 2) + pow(pos[1] - triggerPos[1], 2));
        lastTrigger = triggerNum;
        triggerCount--;

        if (triggerCount == 0)
        {
          xSemaphoreGive(lamp_mutex);
        }
        xSemaphoreGive(trigger_mutex);

        if (rad <= radius)
        {
          if (!on)
          {
            on = true;
            writePWM(address, high);
            writeStringTelnetln("Z-X-" + String(pos[0], HEX) + "-Y-" + String(pos[1], HEX));
          }
          onMillis = millis();
        }
      }
      else
      {
        xSemaphoreGive(trigger_mutex);
      }
    }

    if (on && (millis() - onMillis) > interval)
    {
      on = false;
      writePWM(address, low);
      writeStringTelnetln("V-X-" + String(pos[0], HEX) + "-Y-" + String(pos[1], HEX));
    }
    taskYIELD();
  }
}

// task kazde lampy
void lamp(void *parameters)
{
  // init lampy
  uint8_t address = int(parameters);
  uint8_t pos[2] = {0, 0};

  uint8_t *p = readPosition(address);
  pos[0] = p[0];
  pos[1] = p[1];

  // smycka tasku lampy 💡
  while (true)
  {
    if (!easterEgg)
    {
      if (xSemaphoreTake(lamp_mutex, 20) == pdTRUE)
      {
        if (readTouch(address) == 1)
        {
          triggerPos[0] = pos[0];
          triggerPos[1] = pos[1];
          triggerCount = lampCount;
          triggerNum++;
          writeStringTelnetln("T-X-" + String(pos[0], HEX) + "-Y-" + String(pos[1], HEX));
          delay(50);
        }
        else
        {
          xSemaphoreGive(lamp_mutex);
        }
      }
    }
    else
    {
      easterEggMode(address); //🦄🦄🦄🦄🦄🦄
    }
    taskYIELD();
  }
  vTaskDelete(NULL);
}

void lampInit(void *parameters)
{
  uint8_t address = int(parameters);
  uint8_t pos[2] = {0, 0};

  while (true)
  {
    if (xSemaphoreTake(trigger_mutex, 5) == pdTRUE)
    {
      lampCount++;
      Serial.println("počet lamp: " + String(lampCount));
      xSemaphoreGive(trigger_mutex);
      break;
    }
  }

  writeMode(address, 1);
  delay(1);
  if (true)
  {
    commonAnode(address, false);
    delay(1);
    writeSpeed(address, 1);
    delay(1);
    writeSample(address, 20);
    delay(1);
    writeThreshold(address, 150);
    delay(1);
    autonomusHigh(address, 255);
    delay(1);
    autonomusLow(address, 5);
    delay(1);
    autonomusInterval(address, 5000);
  }

  // odeslani informace o poloze lampy pri jejim nalezeni na Telnet
  uint8_t *p = readPosition(address);
  pos[0] = p[0];
  pos[1] = p[1];
  writeStringTelnetln("L-X-" + String(pos[0], HEX) + "-Y-" + String(pos[1], HEX));

  for (int i = 0; i < 3; i++)
  {
    writePWM(address, high);
    delay(1000);
    writePWM(address, low);
    delay(1000);
  }
  writePWM(address, low);
  delay(10);
  writePWM(address, low);

  xTaskCreatePinnedToCore(lamp, "lamp", 1500, (void *)address, 3, NULL, 1);
  xTaskCreatePinnedToCore(lampTrigger, "lampTrigger", 1500, (void *)address, 3, NULL, 1);

  vTaskDelete(NULL);
}

// vyhledani lampy na sbernici
void scanner(void *parameters)
{
  for (int i = 4; i < 50; i++)
  {
    if (isLampHere(i))
    {
      xTaskCreatePinnedToCore(lampInit, "lamp", 1000, (void *)i, 5, NULL, 1);
    }
    delay(1);
  }
  vTaskDelete(NULL);
}

void setup()
{
  // put your setup code here, to run once:
  delay(800);
  Serial.begin(115200);
  Wire.begin(22, 23); //ESP32 bez LoRa
  pinMode(22, INPUT);
  pinMode(23, INPUT);

  Serial.println("");
  Serial.println("");

  WiFi.softAP("ChytreLampy", "");

  MDNS.begin("chytrelampy");
  MDNS.addService("http", "tcp", 80);

  xTaskCreatePinnedToCore(serverHandle, "server", 2000, (void *)1, 3, NULL, 1);
  xTaskCreatePinnedToCore(OTA, "OTA", 2000, (void *)1, 3, NULL, 1);
  xTaskCreatePinnedToCore(scanner, "scanner", 2000, (void *)1, 5, NULL, 1);
  vTaskDelete(NULL);
}

void loop()
{
  vTaskDelete(NULL);
}