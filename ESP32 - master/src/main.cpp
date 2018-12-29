//Vít Petřík@2018
#include <Arduino.h>
#define TWI_FREQ 400000L
#include <Wire.h>
#include <PubSubClient.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <Update.h>
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
WebServer server(80);

Adafruit_BME280 bme;
Adafruit_SI1145 uv = Adafruit_SI1145();

unsigned long sensorMillis = 0;
unsigned long onMillis = 0;
boolean turnOn = false;

Adafruit_SSD1306 display(128, 64, &Wire, 16);

//přečte INT z attiny
int readTouch(int address)
{
  int x = 0;
  Wire.requestFrom(address, 2);
  x = Wire.read();
  x = x << 8;
  x += Wire.read();
  return x;
}

uint8_t *readLocation(uint8_t address)
{
  static uint8_t data[2];
  delay(1);
  Wire.beginTransmission(address);
  Wire.write(0x05);
  Wire.endTransmission();
  delay(1);
  Wire.requestFrom(address, 2);
  data[0] = Wire.read();
  data[1] = Wire.read();
  readTouch(address);
  return data;
}

void i2cscanner()
{
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("I2C nalezeno na:");
  for (uint8_t i = 1; i < 128; i++)
  {
    Wire.beginTransmission(i);
    if (Wire.endTransmission() == 0 && i != 60 && i != 96 && i != 118)
    {
      uint8_t *p = readLocation(i);
      display.print("Adr: ");
      display.print(String(i, HEX).c_str());
      display.print(" X: ");
      display.print(String(p[0], HEX).c_str());
      display.print(" Y: ");
      display.println(String(p[1], HEX).c_str());
      display.display();
      //delay(1500);
    }
  }
}

//zapíše PWM hodnotu na I2C
void writePWM(uint8_t address, uint8_t data)
{
  Wire.beginTransmission(address);
  Wire.write(0x00);
  Wire.write(data);
  Wire.endTransmission();
  readTouch(address);
}

void writeGPS(uint8_t address, uint8_t X, uint8_t Y)
{
  Wire.beginTransmission(address);
  Wire.write((uint8_t)0x04);
  Wire.write(X);
  Wire.write(Y);
  Wire.endTransmission();
  readTouch(address);
}

void writeAddress(uint8_t address, uint8_t addrr)
{
  if (addrr < 128)
  {
    Wire.beginTransmission(address);
    Wire.write(0x03);
    Wire.write(addrr);
    Wire.endTransmission();
    readTouch(address);
  }
}

void writeSpeed(uint8_t address, uint8_t data)
{
  Wire.beginTransmission(address);
  Wire.write(0x01);
  Wire.write(data);
  Wire.endTransmission();
  readTouch(address);
}

void writeFade(uint8_t address, boolean foo)
{
  Wire.beginTransmission(address);
  Wire.write(0x02);
  if (foo)
  {
    Wire.write(0xFF);
  }
  else
  {
    Wire.write(0x00);
  }
  Wire.endTransmission();
  readTouch(address);
}

void callback(char *t, uint8_t *payload, unsigned int length)
{
  String topic(t);
  String s = "";
  for (int i = 0; i < length; i++)
  {
    s += (char)payload[i];
  }
  if (topic == "0x04/pwm")
  {
    writePWM(0x04, lowByte(s.toInt()));
    String out = "Poslána hodnota PWM: " + s;
    client.publish("0x04/debug", out.c_str());
  }

  if (topic == "0x04/fade")
  {
    if (s == "true")
      writeFade(0x04, true);
    client.publish("0x04/debug", "Plynulá změna zapnuta");
    if (s == "false")
    {
      writeFade(0x04, false);
      client.publish("0x04/debug", "Plynulá změna vypnuta");
    }
  }

  if (topic == "0x04/speed")
  {
    writeSpeed(0x04, lowByte(s.toInt()));
    String out = "Rychlost změny změněna na: " + s;
    client.publish("0x04/debug", out.c_str());
  }
}

void setup()
{
  delay(500);
  //Wire.begin(22, 23); //ESP32 bez LoRa
  Wire.begin(4, 15); //ESP32 s LoRou
  bme.begin(0x76);
  uv.begin();
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0, 0);
  Serial.begin(9600);
  WiFi.begin(SSID, PASS);
  display.println("Connecting to WiFi");
  display.println("");
  display.display();
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
    display.print(".");
    display.display();
  }
  display.println("");
  display.println("");
  display.println("Connected at IP: ");
  display.println("");
  display.println(WiFi.localIP());
  display.display();
  delay(1000);
  display.clearDisplay();
  display.setCursor(0, 0);
  client.setServer(MQTT, MQTTport);
  client.setCallback(callback);
  //delay(10);
  //writeGPS(0x04, 0x96, 0x66);
  //writeAddress(0x04, 0x66);
  i2cscanner();
}

void loop()
{
  //pokud nejsme připojeni, tak se připojíme 🙃
  if (!client.connected())
  {
    String clientId = "0x04";
    clientId += String(random(0xffff), HEX);

    if (client.connect(clientId.c_str()))
    {
      client.subscribe("0x04/pwm");
      client.subscribe("0x04/fade");
      client.subscribe("0x04/speed");
      client.publish("0x04/debug", "0x04 připojeno k MQTT");
    }
  }
  else
  {
    client.loop();
  }

  //každou sekundu pošleme data ze senzorů na MQTT
  if ((millis() - sensorMillis) > 1000)
  {
    client.publish("0x04/temp", String(bme.readTemperature()).c_str());
    client.publish("0x04/press", String(bme.readPressure()).c_str());
    client.publish("0x04/hum", String(bme.readHumidity()).c_str());
    client.publish("0x04/vis", String(uv.readVisible()).c_str());
    client.publish("0x04/ir", String(uv.readIR()).c_str());
    sensorMillis = millis(); //resetujeme odpočítávač
  }

  //pokud je lampa zapnutá a zároveň uběhl předem daný interval od zapnutí tak vypneme lampu
  if (((millis() - onMillis) > 5000) && turnOn)
  {
    writePWM(0x04, 0x00);
    turnOn = false;
    client.publish("0x04/onoff", "false");
    client.publish("0x04/debug", "LEDka vypnuta");
  }

  //pokud je lampa vypnutá kontrolujeme čidlo doteku
  if (readTouch(0x04) > 700)
  {
    if (!turnOn)
    {
      writePWM(0x04, 0xFF);
      turnOn = true;
      client.publish("0x04/onoff", "true");
      client.publish("0x04/debug", "Zaznamenán dotek a zapnuta LEDka");
    }
    onMillis = millis();
  }
}