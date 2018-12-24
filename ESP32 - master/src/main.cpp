//Vít Petřík@2018
#include <Arduino.h>
#include <Wire.h>
#include <PubSubClient.h>
#include <WiFi.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include "Adafruit_SI1145.h"

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

int readI2C(int address, int bytes)
{
  int x = 0;
  Wire.requestFrom(address, bytes); //zažádá 2 byty na adrese 0x04
  x = Wire.read();
  x = x << 8;
  x += Wire.read(); //uloží hodnotu do INTu po 2 bytech ( bitový posun dolevo o 8 )
  return x;
}

void writeI2C(int address, byte data)
{
  Wire.beginTransmission(address);
  Wire.write(data);
  Wire.endTransmission();
}

void callback(char *topic, byte *payload, unsigned int length)
{
  if (length == 1)
  {
    writeI2C(0x04, payload[0]);
  }
}

void reconnect()
{
  while (!client.connected())
  {
    String clientId = "0x04";
    clientId += String(random(0xffff), HEX);

    if (client.connect(clientId.c_str()))
    {
      client.subscribe("0x04/pwm");
    }
    else
    {
      delay(5000);
    }
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
}

void loop()
{
  if (!client.connected())
  {
    reconnect();
  }
  client.loop();
  if ((millis() - sensorMillis) > 1000)
  {
    client.publish("0x04/temp", String(bme.readTemperature()).c_str());
    client.publish("0x04/press", String(bme.readPressure()).c_str());
    client.publish("0x04/hum", String(bme.readHumidity()).c_str());
    client.publish("0x04/vis", String(uv.readVisible()).c_str());
    client.publish("0x04/ir", String(uv.readIR()).c_str());
    sensorMillis = millis();
  }

  if (((millis() - onMillis) > 1000) && turnOn)
  {
    if (readI2C(0x04, 2) > 700)
    {
      onMillis = millis();
    }
    else
    {
      writeI2C(0x04, 0);
      turnOn = false;
    }
  }
  if (!turnOn)
  {
    if (readI2C(0x04, 2) > 700)
    {
      writeI2C(0x04, (byte) 255);
      readI2C(0x04, 2);
      onMillis = millis();
      turnOn = true;
    }
  }
}