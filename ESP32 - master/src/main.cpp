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
#include <WiFiUdp.h>
#include <ArduinoOTA.h>

// FreeRTOS Semaphore pro zamezeni konfliktu při přistupování triggrovacích proměnných
SemaphoreHandle_t triggerRead_mutex = xSemaphoreCreateMutex();
SemaphoreHandle_t triggerWrite_mutex = xSemaphoreCreateMutex();

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

void lampTrigger(void *parameters)
{
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
    if (lastTrigger != triggerNum)
    {
      rad = sqrt(pow(pos[0] - triggerPos[0], 2) + pow(pos[1] - triggerPos[1], 2));
      Serial.println("Zaznamenán nový trigger 😈");
      if (rad <= radius)
      {
        Serial.println("Jsme v rádiu, nebo rádiusu... na češtinu jsem blbej 😭");
        if (!on)
        {
          Serial.println("Zapínám vypnutou lampu 💡");
          writePWM(address, high);
          on = true;
        }
        onMillis = millis();
      }
      lastTrigger = triggerNum;
      triggerCount--;
    }

    if (on && (millis() - onMillis) > interval)
    {
      Serial.println("Vypínám zapnutou lampu 😒");
      on = false;
      writePWM(address, low);
    }
  }
}

// task kazde lampy
void lamp(void *parameters)
{
  // init lampy
  uint8_t address = int(parameters);
  uint8_t pos[2] = {0, 0};

  delay(100);
  uint8_t *p = readPosition(address);
  pos[0] = p[0];
  pos[1] = p[1];

  // smycka tasku lampy 💡
  while (true)
  {
    if (!easterEgg)
    {
      value = readTouch(address);
      if (readTouch(address) == 1)
      {
        while (true)
        {
          if (triggerCount == 0)
          {
            if (xSemaphoreTake(triggerWrite_mutex, 0))
            {
              triggerPos[0] = pos[0];
              triggerPos[1] = pos[1];
              triggerCount = lampCount;
              triggerNum++;
              xSemaphoreGive(triggerWrite_mutex);
              Serial.println("Dotyk nahrán 🤓🤓");
              break;
            }
          }
        }
      }
      delay(10);
    }
    else
    {
      easterEggMode(address); //🦄🦄🦄🦄🦄🦄
    }
  }
  vTaskDelete(NULL);
}

void lampInit(void *parameters)
{
  while (true)
  {
    if (xSemaphoreTake(triggerWrite_mutex, 5))
    {
      lampCount++;
      xSemaphoreGive(triggerWrite_mutex);
      break;
    }
  }

  uint8_t address = int(parameters);
  uint8_t pos[2] = {0, 0};

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
  writeStringTelnet("Adresa: ");
  writeStringTelnet(String(address));
  writeStringTelnet(": X - ");
  writeStringTelnet(String(pos[0]));
  writeStringTelnet(" Y - ");
  writeStringTelnetln(String(pos[1]));

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

  xTaskCreatePinnedToCore(lamp, "lamp", 1000, (void *)address, 5, NULL, 1);
  xTaskCreatePinnedToCore(lampTrigger, "lampTrigger", 1000, (void *)address, 5, NULL, 1);

  vTaskDelete(NULL);
}

// vyhledani lampy na sbernici
void scanner(void *parameters)
{
  while (false)
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
// vytvoreni tasku prave nalezene lampy
escapeLoop:
  for (int i = 4; i < 50; i++)
  {
    if (isLampHere(i))
    {
      xTaskCreatePinnedToCore(lampInit, "lamp", 1000, (void *)i, 5, NULL, 1);
    }
    delay(1);
  }
  delay(1000);
  Serial.print("počet lamp: ");
  Serial.println(lampCount);
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
  //WiFi.begin("💩💩💩🦄😵🏳‍🌈", "un1corn666");

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
