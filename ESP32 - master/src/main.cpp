//Vít Petřík@2018
#include <Arduino.h>
#include <Wire.h>
#include <PubSubClient.h>
#include <WiFi.h>

#define SSID "💩💩💩🦄😵🏳‍🌈"
#define PASS "un1corn666"
#define MQTT "10.10.10.19"
#define MQTTport "1883"

WiFiClient espClient;
PubSubClient client(espClient);

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

void setup()
{
  delay(500);
  Wire.begin(22, 23);
  Serial.begin(230400);
  writeI2C(0x04, 0);
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
  value = readI2C(0x04, 2);
  client.publish("0x04/touch", value);
}