#define MAX_SRV_CLIENTS 10
WiFiServer server(23);
WiFiClient serverClients[MAX_SRV_CLIENTS];

SemaphoreHandle_t telnet_mutex = xSemaphoreCreateMutex();

void writeStringTelnetln(String s)
{
  Serial.println(s);
  for (int i = 0; i < MAX_SRV_CLIENTS; i++)
  {
    if (serverClients[i] && serverClients[i].connected())
    {
      serverClients[i].println(s);
    }
  }
}

void writeStringTelnet(String s)
{
  Serial.print(s);
  for (int i = 0; i < MAX_SRV_CLIENTS; i++)
  {
    if (serverClients[i] && serverClients[i].connected())
    {
      serverClients[i].print(s);
    }
  }
}

void decodeString(String s)
{
  s.toLowerCase();
  if (s.indexOf("easteregg") >= 0)
  {
    easterEgg = !easterEgg;
    writeStringTelnetln("Prepnuti eastereggu 💡"); //juchuuu 🎆🎆🎆🎆🎆🎇🎈🧨✨🎉🎊🎃
  }
}

void serverHandle(void *parameters)
{
  server.begin();
  server.setNoDelay(true);
  while (true)
  {
    uint8_t i;
    if (xSemaphoreTake(telnet_mutex, 0))
    {
      if (WiFi.status() == WL_CONNECTED || WiFi.status() == WL_NO_SHIELD)
      {
        if (server.hasClient())
        {
          for (i = 0; i < MAX_SRV_CLIENTS; i++)
          {
            if (!serverClients[i] || !serverClients[i].connected())
            {
              if (serverClients[i])
                serverClients[i].stop();
              serverClients[i] = server.available();
              break;
            }
          }
          if (i >= MAX_SRV_CLIENTS)
          {
            server.available().stop();
          }
        }
        for (i = 0; i < MAX_SRV_CLIENTS; i++)
        {
          if (serverClients[i] && serverClients[i].connected())
          {
            if (serverClients[i].available())
            {
              String data = "";
              while (serverClients[i].available())
              {
                data += (char)serverClients[i].read();
              }
              decodeString(data);
            }
          }
          else
          {
            if (serverClients[i])
            {
              serverClients[i].stop();
            }
          }
        }
      }
      else
      {
        Serial.println("WiFi not connected!");
        for (i = 0; i < MAX_SRV_CLIENTS; i++)
        {
          if (serverClients[i])
            serverClients[i].stop();
        }
        delay(1000);
      }
      xSemaphoreGive(telnet_mutex);
    }
    delay(1);
  }
}