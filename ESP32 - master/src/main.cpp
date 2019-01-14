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

//mám dobrou wifinu co? 😂
#define SSID "💩💩💩🦄😵🏳‍🌈"
#define PASS "un1corn666"
#define MQTT "10.10.10.19"
#define MQTTport 1883
#define SEALEVELPRESSURE_HPA (1013.25)

Adafruit_BME280 bme;
Adafruit_SI1145 uv = Adafruit_SI1145();

unsigned long sensorMillis = 0;
unsigned long onMillis = 0;
boolean turnOn = false;

Adafruit_SSD1306 display(128, 64, &Wire, 16);

//přečte hodnotu z dotykového čidla
int readTouch(int address)
{
  static uint8_t x = 0;
  Wire.requestFrom(address, 1);
  x = Wire.read();
  //x *= 4;
  // x = x << 8;
  // x += Wire.read();
  //Serial.println(x);
  return x;
}

//zapíše PWM hodnotu na I2C
void writePWM(uint8_t address, uint8_t PWM)
{
  Wire.beginTransmission(address);
  Wire.write(0x00);
  Wire.write(PWM);
  Wire.endTransmission();
}

//zapíše, jak rychle se má rozsvicet lampa
void writeSpeed(uint8_t address, uint8_t speed)
{
  Wire.beginTransmission(address);
  Wire.write(0x01);
  Wire.write(speed);
  Wire.endTransmission();
}

//zapíše, jestli má být plynulá změna úrovně osvětlení
void writeFade(uint8_t address, boolean fade)
{
  Wire.beginTransmission(address);
  Wire.write(0x02);
  if (fade)
  {
    Wire.write(0xFF);
  }
  else
  {
    Wire.write(0x00);
  }
  Wire.endTransmission();
}

//zapíše novou I2C adresu
void writeAddress(uint8_t address, uint8_t newAddress)
{
  if (newAddress < 128)
  {
    Wire.beginTransmission(address);
    Wire.write(0x03);
    Wire.write(newAddress);
    Wire.endTransmission();
  }
}

//zapíše souřadnice X a Y do ATtiny
void writeGPS(uint8_t address, uint8_t X, uint8_t Y)
{
  Wire.beginTransmission(address);
  Wire.write((uint8_t)0x04);
  Wire.write(X);
  Wire.write(Y);
  Wire.endTransmission();
}

//přečte souřadnice a vrátí je v poli [ X, Y ]
uint8_t *readLocation(uint8_t address)
{
  static uint8_t data[2];
  Wire.beginTransmission(address);
  Wire.write(0x05);
  Wire.endTransmission();
  Wire.requestFrom(address, 2);
  data[0] = Wire.read();
  data[1] = Wire.read();
  return data;
}

void autonomus(uint8_t address, boolean autonomus)
{
  Wire.beginTransmission(address);
  Wire.write(0x06);
  if (autonomus)
  {
    Wire.write(0xFF);
  }
  else
  {
    Wire.write(0x00);
  }
  Wire.endTransmission();
}

void autonomusHigh(uint8_t address, uint8_t PWM)
{
  Wire.beginTransmission(address);
  Wire.write(0x07);
  Wire.write(PWM);
  Wire.endTransmission();
}

void autonomusLow(uint8_t address, uint8_t PWM)
{
  Wire.beginTransmission(address);
  Wire.write(0x08);
  Wire.write(PWM);
  Wire.endTransmission();
}

void autonomusThreshold(uint8_t address, uint8_t thres)
{
  Wire.beginTransmission(address);
  Wire.write(0x09);
  Wire.write(thres);
  Wire.endTransmission();
}

void autonomusInterval(uint8_t address, int inter)
{
  Wire.beginTransmission(address);
  Wire.write(0x0A);
  Wire.write(highByte(inter));
  Wire.write(lowByte(inter));
  Wire.endTransmission();
}

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
    if(value == 255){

    }
    if (((millis() - onMillis) > 1000) && turnOn)
    {
      delay(1);
      writePWM(address, 0);
      turnOn = false;
    }

    //kontrolujeme čidlo doteku

    if (value > 60)
    {
      //pokud lampa není zapnutá, tak jí zapneme
      if (!turnOn)
      {
        delay(1);
        writePWM(address, 255);
        turnOn = true;
      }
      //pokud je dotyk, tak vždy restartujeme počítadlo
      onMillis = millis();
    }
    delay(20);*
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
  }
  vTaskDelete(NULL);
}

void setup()
{
  //Nastavíme I2C sběrnici
  //Wire.begin(22, 23); //ESP32 bez LoRa
  delay(500);
  Wire.begin(4, 15, 100000L); //ESP32 s LoRou
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
  
  //autonomusThreshold(0x04, 8);
  
  Serial.begin(115200);
  xTaskCreatePinnedToCore(i2cscanner, "scanner", 10000, (void *)1, 1, NULL, 1);
}

void loop()
{
  vTaskDelete(NULL);
}