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

#include <ota.h>
#include <lamp.h>

//Task pro ošéfení poloměru
void lampTrigger(void *parameters)
{
  //inicializace
  uint8_t address = int(parameters);

  uint8_t pos[2] = {0, 0};
  uint8_t *p = readPosition(address);
  pos[0] = p[0];
  pos[1] = p[1];

  unsigned long onMillis = 0;
  unsigned long lastTrigger = 0;
  bool on = false;
  double rad;

  while (true)
  {
    if (xSemaphoreTake(trigger_mutex, 20) == pdTRUE) //požádá o semafor
    {
      if (triggerNum != lastTrigger) //pokud je zaznamenán nový trigger
      {
        rad = sqrt(pow(pos[0] - triggerPos[0], 2) + pow(pos[1] - triggerPos[1], 2)); //výpočet poloměru
        lastTrigger = triggerNum;
        triggerCount--; //dekrementace

        if (triggerCount < 1) //pokud trigger zpracovali všechny tasky uvolníme semafor pro čtení lamp
        {
          xSemaphoreGive(lamp_mutex);
        }
        xSemaphoreGive(trigger_mutex); //uvolníme semafor pro poloměr

        if (rad <= radius) //pokud jsme v poloměru
        {
          if (!on) //pokud je lampa zhasnutá
          {
            on = true;
            writePWM(address, high); //zapneme lampu
            Serial.println("Z-X-" + String(pos[0], HEX) + "-Y-" + String(pos[1], HEX));
          }
          onMillis = millis(); //nastavíme čas pro výpočet intervalu
        }
      }
      else
      {
        xSemaphoreGive(trigger_mutex); //vrátíme semafor pro poloměr
      }
    }

    //pokud je lampa rozsvícena a zároveň jsme mimo interval
    if (on && (millis() - onMillis) > interval)
    {
      on = false;
      writePWM(address, low); //vypneme lampu
      Serial.println("V-X-" + String(pos[0], HEX) + "-Y-" + String(pos[1], HEX));
    }
    taskYIELD();
  }
}

// task kazde lampy
void lamp(void *parameters)
{
  //inicializace
  uint8_t address = int(parameters);
  bool discoMode = false;

  uint8_t pos[2] = {0, 0};
  uint8_t *p = readPosition(address);
  pos[0] = p[0];
  pos[1] = p[1];

  // smycka tasku lampy 💡
  while (true)
  {
    //disco mód :)
    if (!discoMode)
    {
      if (xSemaphoreTake(lamp_mutex, 20) == pdTRUE) //požádáme o semafor pro čtení lamp
      {
        if (readTouch(address) == 1) //pokud máme dotyk
        {
          //zapíšeme souřadnice triggeru
          triggerPos[0] = pos[0];
          triggerPos[1] = pos[1];
          triggerCount = lampCount;
          triggerNum++;
          Serial.println("T-X-" + String(pos[0], HEX) + "-Y-" + String(pos[1], HEX));
          delay(50); //tato delay zde nemusí nutně být, ale malinko odlehčí sběrnici :)
        }
        else
        {
          //pokud není dotyk vrátíme semafor, jinak ho nevracíme!!!
          xSemaphoreGive(lamp_mutex);
        }
      }
    }
    else
    {
      disco(address); //🦄🦄🦄🦄🦄🦄
    }
    taskYIELD();
  }
  vTaskDelete(NULL);
}

//nastavení lampy
void lampInit(void *parameters)
{
  uint8_t address = int(parameters);

  uint8_t pos[2] = {0, 0};
  uint8_t *p = readPosition(address);
  pos[0] = p[0];
  pos[1] = p[1];

  //odeslani informace o poloze lampy pri jejim nalezeni na Telnet
  Serial.println("L-I2C-" + String(address) + "-X-" + String(pos[0], HEX) + "-Y-" + String(pos[1], HEX));

  //inkrementace počtu lamp
  while (true)
  {
    if (xSemaphoreTake(trigger_mutex, 20) == pdTRUE)
    {
      lampCount++;
      xSemaphoreGive(trigger_mutex);
      break;
    }
  }

  //nastavení módu lampy
  writeMode(address, 1);
  if (true)
  {
    writeSpeed(address, 5);
    autonomusHigh(address, high);
    autonomusLow(address, low);
    autonomusInterval(address, 5000);
    writeFade(address, true);
  }

  //3x zablikání lampy
  for (int i = 0; i < 3; i++)
  {
    writePWM(address, high);
    delay(1000);
    writePWM(address, low);
    delay(1000);
  }
  writePWM(address, low);

  //vytvoření tasků nutných pro správné pracování
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
      //pokud jsme našli lampu vytvoříme pro ni task
      xTaskCreatePinnedToCore(lampInit, "lamp", 1000, (void *)i, 5, NULL, 1);
    }
    delay(1);
  }
  vTaskDelete(NULL);
}

void draznilampa(void *parameters)
{
  for (int i = 0; i < 3; i++)
  {
    writePWM(13, 255);
    delay(1000);
    writePWM(13, 5);
    delay(1000);
  }
  delay(10);
  while (true)
  {
    Serial.println(readTouch(13));
    delay(100);
  }
}

void setup()
{
  //inicializace ESP
  delay(500); //"bezpečnostní" zpoždění¨

  Serial.begin(115200);
  Wire.begin(22, 23);
  pinMode(22, INPUT);
  pinMode(23, INPUT);

  WiFi.softAP("ChytreLampy", "");
  //Wifi.begin("💩💩💩🦄😵🏳‍🌈", "un1corn666");

  MDNS.begin("chytrelampy");
  MDNS.addService("http", "tcp", 80);

  xTaskCreatePinnedToCore(OTA, "OTA", 2000, (void *)1, 3, NULL, 1);
  xTaskCreatePinnedToCore(scanner, "scanner", 2000, (void *)1, 5, NULL, 1);
  //xTaskCreatePinnedToCore(draznilampa, "scanner", 2000, (void *)1, 5, NULL, 1);
  vTaskDelete(NULL);
}

void loop()
{
  vTaskDelete(NULL);
}