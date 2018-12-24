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

//přečte INT z attiny
int readI2Cint(int address)
{
  int x = 0;
  Wire.requestFrom(address, 2);
  x = Wire.read();
  x = x << 8;
  x += Wire.read();
  return x;
}

//zapíše 1 byte do attiny
void writeI2C(int address, byte data)
{
  Wire.beginTransmission(address);
  Wire.write(data);
  Wire.endTransmission();
}

void callback(char *topic, byte *payload, unsigned int length)
{
    String s = "";
    for(int i = 0; i < length; i++){
        s += (char) payload[i];
    }
//pošle PWM hodnotu z MQTT serveru na attiny
    writeI2C(0x04, lowByte(s.toInt()));
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
//pokud nejsme připojeni, tak se připojíme 🙃
  if (!client.connected())
  {
    reconnect();
  }
  client.loop();
//každou sekundu pošleme data ze senzorů na MQTT
  if ((millis() - sensorMillis) > 1000)
  {
    client.publish("0x04/temp", String(bme.readTemperature()).c_str());
    client.publish("0x04/press", String(bme.readPressure()).c_str());
    client.publish("0x04/hum", String(bme.readHumidity()).c_str());
    client.publish("0x04/vis", String(uv.readVisible()).c_str());
    client.publish("0x04/ir", String(uv.readIR()).c_str());
    sensorMillis = millis();
  }
//pokud je lampa zapnutá a zároveň uběhl předem daný interval od zapnutí tak vypneme lampu
  if (((millis() - onMillis) > 1000) && turnOn)
  {
//ale pokud se někdo dotýká čidla, vypínat nebudeme a zrestartujeme počítadlo 😉
    if (readI2Cint(0x04) > 700)
    {
      onMillis = millis();
    }
    else
    {
      writeI2C(0x04, 0);
      turnOn = false;
      client.publish("0x04/onoff", "false");
    }
  }
//pokud je lampa vypnutá kontrolujeme čidlo doteku
  if (!turnOn)
  {
    if (readI2Cint(0x04) > 700)
    {
      writeI2C(0x04, (byte) 255);
      readI2Cint(0x04);
      onMillis = millis();
      turnOn = true;
      client.publish("0x04/onoff", "true");
    }
  }
}