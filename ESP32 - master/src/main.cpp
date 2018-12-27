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

int * readLocation (int address){
  static int data[3];
  Wire.beginTransmission(address);
  Wire.write(0xFF);
  Wire.endTransmission();
  delay(10);
  Wire.requestFrom(address, 3);
  data[0] = Wire.read();
  data[1] = Wire.read();
  data[2] = Wire.read();
  return data;
}

//zapíše PWM hodnotu na I2C
void writePWM(byte address, byte data)
{
  Wire.beginTransmission(address);
  Wire.write((byte)0x00);
  Wire.write(data);
  Wire.endTransmission();
  readTouch(0x04);
}

void writeSpeed(byte address, byte data)
{
  Wire.beginTransmission(address);
  Wire.write(0x01);
  Wire.write(data);
  Wire.endTransmission();
  readTouch(0x04);
}

void writeFade(byte address, boolean foo)
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
  readTouch(0x04);
}

void callback(char *t, byte *payload, unsigned int length)
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
  Wire.begin(22, 23);
  bme.begin(0x76);
  uv.begin();
  Serial.begin(9600);
  WiFi.begin(SSID, PASS);

  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }

  client.setServer(MQTT, MQTTport);
  client.setCallback(callback);
  writeFade(0x04, true);
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
    int * location;
    location = readLocation(0x04);
    client.publish("0x04/debug", String(location[0]).c_str());
    client.publish("0x04/X", String(location[1]).c_str());
    client.publish("0x04/Y", String(location[2]).c_str());
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