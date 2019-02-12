//Vít Petřík@2018 + Vojta Ullmann@2019
/*
Máme úžasný a vysoce funkční kódy, omluvte  prehlednost :(
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
// TODO FreeRTOS knihovny? 
//#include <queue.h>
#define SEALEVELPRESSURE_HPA (1013.25)
// TODO definovat jak velke okoli lampa ovlivnuje
#define PERIMETR
// Inicializace fronty pro ukladani souradnic
QueueHandle_t queueX = xQueueCreate(127, sizeof(uint8_t));
QueueHandle_t queueY = xQueueCreate(127, sizeof(uint8_t));

Adafruit_BME280 bme;
Adafruit_SI1145 uv = Adafruit_SI1145();

unsigned long sensorMillis = 0;
unsigned long onMillis = 0;
boolean turnOn = false;
 
 // Radius + QueueClean
void lightradius(uint8_t x, uint8_t y, uint8_t lastValue, int address) 
{
    // TODO implementace semaforu? (=> jistota ze check probehne kompletne ALE docela ztrata vyhody multitasku)
    
    // Podminka zda ma vubec smysl neco delat
    if((uxQueueMessagesWaiting(queueX) != 0) && (uxQueueMessagesWaiting(queueY) != 0) && (uxQueueMessagesWaiting(queueX) == uxQueueMessagesWaiting(queueY)))
    {
        uint8_t maxindex = uxQueueMessagesWaiting(queueX); // aktualni mnozstvi dat ve fronte
        double radius;
        // Souradnice lampy ve stavu 2
        uint8_t xs;
        uint8_t ys;
        
        for(uint8_t i=0; i < maxindex; i++) 
        {
            if(xQueueReceive(queueX, &xs, (TickType_t) 5)) 
            {
                Serial.println("xs se uspesne nahralo.");
            }
            else
            {
                Serial.println("xs se nenahralo.");
            }
            if(xQueueReceive(queueY, &ys, (TickType_t) 5)) 
            {
                Serial.println("ys se uspesne nahralo.");
            }
            else
            {
                Serial.println("ys se nenahralo.");
            }
            radius = ((xs-x) * (xs-x)) + ((ys-y) * (ys-y));
            radius = sqrt(radius);
            Serial.println("Radius = ");
            Serial.print(radius);
            // Vraceni stale aktivni lampy do fronty pokud tam ma byt
            if (!(lastValue == 2 && radius == 0.0)) 
            {
                if(xQueueSend(queueX, &xs, (TickType_t) 0))
                {
                    System.println("Souradnice X uspesne ulozena.");            
                } 
                else
                {
                    System.println("Chyba, X neulozeno.");
                }
                if(xQueueSend(queueY, &ys, (TickType_t) 0))
                {
                    System.println("Souradnice Y uspesne ulozena.");            
                } 
                else
                {
                    System.println("Chyba, Y neulozeno.");
                } 
            }
            if(radius <= PERIMETR) 
            {
                // ROZSVICENI LAMPY
                writePWM(address, 0xFF)
            }
        }
    }
    else if (uxQueueMessagesWaiting(queueX) != uxQueueMessagesWaiting(queueY)) 
    {
        Serial.println("Something went wrong in the queue :(");
    }
}

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
        
        // Ulozeni souradnice lampy pri dotyku/pohybu
        if(xQueueSend(queueX, &p[0], (TickType_t) 0))
        {
            System.println("Souradnice X uspesne ulozena.");            
        } 
        else
        {
            System.println("Chyba, X neulozeno.");
        }
        if(xQueueSend(queueY, &p[1], (TickType_t) 0))
        { 
            System.println("Souradnice Y uspesne ulozena.");
        } 
        else 
        {
            System.println("Chyba, Y neulozeno.");
        }

      }
      Serial.println("--------------");
      Serial.println("");
      lightradius(p[0], p[1], lastValue, address);
    }
    lastValue = value;

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
