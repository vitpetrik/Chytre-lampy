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

WiFiClient espClient;
PubSubClient client(espClient);

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

void autonomusLamp(uint8_t address, boolean foo, uint8_t high, uint8_t low, uint8_t interval2, uint8_t threshold2)
{
  Wire.beginTransmission(address);
  Wire.write(0x06);
  if (foo)
  {
    Wire.write(0xFF);
  }
  else
  {
    Wire.write(0x00);
  }
  Wire.endTransmission();

  Wire.beginTransmission(address);
  Wire.write(0x07);
  Wire.write(high);
  Wire.endTransmission();

  Wire.beginTransmission(address);
  Wire.write(0x08);
  Wire.write(low);
  Wire.endTransmission();

  Wire.beginTransmission(address);
  Wire.write(0x09);
  Wire.write(threshold2);
  Wire.endTransmission();

  Wire.beginTransmission(address);
  Wire.write(0x0A);
  Wire.write(interval2);
  Wire.endTransmission();
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

void autonomusThreshold(uint8_t address, int thres)
{
  Wire.beginTransmission(address);
  Wire.write(0x09);
  Wire.write(highByte(thres));
  Wire.write(lowByte(thres));
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
  //display.setTextSize(7);
  while (true)
  {
    //digitalWrite(2, HIGH);
    if (((millis() - onMillis) > 1000) && turnOn)
    {
      //vTaskPrioritySet(NULL, 10);
      writePWM(address, 0x00);
      delay(1);
      //vTaskPrioritySet(NULL, 1);
      turnOn = false;
      //client.publish("0x04/onoff", "false");
    }
    //vTaskPrioritySet(NULL, 10);
    value = readTouch(address);
    delay(1);
    //Serial.println(value);
    //vTaskPrioritySet(NULL, 1);
    /*display.clearDisplay();
    display.setCursor(0, 0);
    display.println(value);
    display.display();*/
    //kontrolujeme čidlo doteku
    /*if (!turnOn)
    {
      display.clearDisplay();
      display.setCursor(0, 0);
      display.setTextSize(2);
      display.println("Vase IQ je");
      display.print(random(60, 200));
      display.display();
    }*/

    if (value > 170)
    {
      //pokud lampa není zapnutá, tak jí zapneme
      if (!turnOn)
      {
        //client.publish("0x04/onoff", "true");
        //vTaskPrioritySet(NULL, 10);
        writePWM(address, 0xFF);
        delay(1);
        //vTaskPrioritySet(NULL, 1);
        turnOn = true;
      }
      //pokud je dotyk, tak vždy restartujeme počítadlo
      onMillis = millis();
    }
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

//funkce se vyvolá, pokud přijeme MQTT zprávu
void callback(char *t, uint8_t *payload, unsigned int length)
{
  vTaskPrioritySet(NULL, 10);
  String topic(t);
  String s = "";
  for (int i = 0; i < length; i++)
  {
    s += (char)payload[i];
  }
  Serial.println(s);
  Serial.println(topic);
  if (topic == "0x04/pwm")
  {
    writePWM(0x04, lowByte(s.toInt()));
    readTouch(0x04);
    String out = "Poslána hodnota PWM: " + s;
    client.publish("0x04/debug", out.c_str());
  }

  if (topic == "0x04/fade")
  {
    if (s == "true")
      delayMicroseconds(1000);
    writeFade(0x04, true);
    readTouch(0x04);
    client.publish("0x04/debug", "Plynulá změna zapnuta");
    if (s == "false")
    {
      delayMicroseconds(1000);
      writeFade(0x04, false);
      readTouch(0x04);
      client.publish("0x04/debug", "Plynulá změna vypnuta");
    }
  }

  if (topic == "0x04/speed")
  {
    writeSpeed(0x04, lowByte(s.toInt()));
    readTouch(0x04);
    String out = "Rychlost změny změněna na: " + s;
    client.publish("0x04/debug", out.c_str());
  }
  vTaskPrioritySet(NULL, 1);
}

void MQTTsensors(void *parameters)
{
  while (true)
  {
    client.publish("0x04/temp", String(bme.readTemperature()).c_str());
    client.publish("0x04/press", String(bme.readPressure()).c_str());
    client.publish("0x04/hum", String(bme.readHumidity()).c_str());
    client.publish("0x04/vis", String(uv.readVisible()).c_str());
    client.publish("0x04/ir", String(uv.readIR()).c_str());
    delay(2000);
  }
}

void MQTThandle(void *parameters)
{
  client.setServer(MQTT, MQTTport);
  client.setCallback(callback);
  boolean sensors = false;
  Serial.println("AHOJ0");
  while (true)
  {
    if (!client.connected())
    {
      Serial.println("AHOJ2");
      String clientId = "0x04";
      clientId += String(random(0xffff), HEX);

      if (client.connect(clientId.c_str()))
      {
        Serial.println("AHOJ3");
        client.subscribe("0x04/pwm");
        client.subscribe("0x04/fade");
        client.subscribe("0x04/speed");
        client.publish("0x04/debug", "0x04 připojeno k MQTT");
        if (!sensors)
        {
          //xTaskCreatePinnedToCore(MQTTsensors, "MQTTsensors", 10000, (void *)1, 3, NULL, 1);
          sensors = true;
        }
      }
    }
    else
    {
      client.loop();
      delay(1);
    }
  }
}

void WiFiconnection(void *parameters)
{

  if (WiFi.status() != WL_CONNECTED)
  {
    Serial.println("pripojuji se");
    WiFi.begin(SSID, PASS);
  }
  vTaskDelete(NULL);
}

void setup()
{
  //Nastavíme I2C sběrnici
  //Wire.begin(22, 23); //ESP32 bez LoRa
  delay(500);
  Wire.begin(4, 15); //ESP32 s LoRou
  //bme.begin(0x76);
  //uv.begin();

  pinMode(2, OUTPUT);

  //inicializujeme display
  /*display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.clearDisplay();
  display.setCursor(0, 0);
  display.setTextColor(WHITE);
  display.setTextSize(3);
  display.println("Vitecek");
  display.print("je buh!");
  display.display();*/

  autonomus(0x04, false);

 /* display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0, 0);
  Serial.begin(115200);*/
  //WiFi.begin(SSID, PASS);
  //přes I2C scanner najdeme všechny lampy na I2C sběrnici a přidáme je do třídy 'lamp'
  //xTaskCreatePinnedToCore(i2cscanner, "scanner", 10000, (void *)1, 1, NULL, 1);
  xTaskCreatePinnedToCore(lamp, "blinky", 10000, (void *)0x0B, 1, NULL, 1);
  //xTaskCreatePinnedToCore(MQTThandle, "MQTT", 100000, (void *)1, 1, NULL, 1);
  //xTaskCreatePinnedToCore(WiFiconnection, "WiFiconnection", 100000, (void *)1, 1, NULL, 0);

  //xTaskCreatePinnedToCore(MQTTsensors, "MQTTsensors", 10000, (void *)1, 3, NULL, 1);
}

void loop()
{
  vTaskDelete(NULL);
}