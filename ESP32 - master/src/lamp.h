// parametr address je adresa lampy na i2c sbernici
SemaphoreHandle_t i2c_mutex = xSemaphoreCreateMutex();
int tickCount = 20;
// otestuje zda na dane adrese existuje zarizeni
bool isLampHere(uint8_t address)
{
  bool presence = false;
  while (true)
  {
    if (xSemaphoreTake(i2c_mutex, tickCount) == pdTRUE)
    {
      for (int i = 0; i < 3; i++)
      {
        Wire.beginTransmission(address);
        if (Wire.endTransmission() == 0)
        {
          xSemaphoreGive(i2c_mutex);
          return true;
        }
      }
      xSemaphoreGive(i2c_mutex);
      return false;
    }
  }
  return false;
}
//přečte hodnotu z dotykového čidla
uint8_t readTouch(int address)
{
  uint8_t msg = 0;
  if (xSemaphoreTake(i2c_mutex, 1) == pdTRUE)
  {
    Wire.requestFrom(address, 1);
    msg = Wire.read();
    xSemaphoreGive(i2c_mutex);
  }
  return msg;
}
//přečte souřadnice a vrátí je v poli [ X, Y ]
uint8_t *readPosition(uint8_t address)
{
  static uint8_t data[2];
  while (true)
  {
    if (xSemaphoreTake(i2c_mutex, tickCount) == pdTRUE)
    {
      Wire.beginTransmission(address);
      Wire.write(0x05);
      Wire.endTransmission();
      Wire.requestFrom(address, 2);
      data[0] = Wire.read();
      data[1] = Wire.read();
      xSemaphoreGive(i2c_mutex);
      return data;
    }
  }
  return data;
}
//zapíše PWM hodnotu na I2C
void writePWM(uint8_t address, uint8_t PWM)
{
  while (true)
  {
    if (xSemaphoreTake(i2c_mutex, tickCount) == pdTRUE)
    {
      Wire.beginTransmission(address);
      Wire.write(0x00);
      Wire.write(PWM);
      Wire.endTransmission();
      xSemaphoreGive(i2c_mutex);
      return;
    }
  }
}
//zapíše, jak rychle se má rozsvicet lampa
void writeSpeed(uint8_t address, uint8_t speed)
{
  while (true)
  {
    if (xSemaphoreTake(i2c_mutex, tickCount) == pdTRUE)
    {
      Wire.beginTransmission(address);
      Wire.write(0x01);
      Wire.write(speed);
      Wire.endTransmission();
      xSemaphoreGive(i2c_mutex);
      return;
    }
  }
}
//zapíše, jestli má být plynulá změna úrovně osvětlení zapnuta/vypnuta
void writeFade(uint8_t address, boolean fade)
{
  while (true)
  {
    if (xSemaphoreTake(i2c_mutex, tickCount) == pdTRUE)
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
      xSemaphoreGive(i2c_mutex);
      return;
    }
  }
}
//zapíše novou I2C adresu
void writeI2CAddress(uint8_t address, uint8_t newAddress)
{
  if (newAddress < 128)
  {
    while (true)
    {
      if (xSemaphoreTake(i2c_mutex, tickCount) == pdTRUE)
      {
        Wire.beginTransmission(address);
        Wire.write(0x03);
        Wire.write(newAddress);
        Wire.endTransmission();
        xSemaphoreGive(i2c_mutex);
        return;
      }
    }
  }
}
//zapíše souřadnice X a Y do ATtiny
void writePosition(uint8_t address, uint8_t X, uint8_t Y)
{
  while (true)
  {
    if (xSemaphoreTake(i2c_mutex, tickCount) == pdTRUE)
    {
      Wire.beginTransmission(address);
      Wire.write((uint8_t)0x04);
      Wire.write(X);
      Wire.write(Y);
      Wire.endTransmission();
      xSemaphoreGive(i2c_mutex);
      return;
    }
  }
}

void writeSample(uint8_t address, uint8_t sample)
{
  while (true)
  {
    if (xSemaphoreTake(i2c_mutex, tickCount) == pdTRUE)
    {
      Wire.beginTransmission(address);
      Wire.write(0x0B);
      Wire.write(sample);
      Wire.endTransmission();
      xSemaphoreGive(i2c_mutex);
      return;
    }
  }
}
// volba rezimu
void writeMode(uint8_t address, uint8_t Mode)
{
  while (true)
  {
    if (xSemaphoreTake(i2c_mutex, tickCount) == pdTRUE)
    {
      Wire.beginTransmission(address);
      Wire.write(0x0C);
      Wire.write(Mode);
      Wire.endTransmission();
      xSemaphoreGive(i2c_mutex);
      return;
    }
  }
}
// hodnota kapacitniho cidla ktera sepne lampu
void writeThreshold(uint8_t address, uint8_t thres)
{
  while (true)
  {
    if (xSemaphoreTake(i2c_mutex, tickCount) == pdTRUE)
    {
      Wire.beginTransmission(address);
      Wire.write(0x09);
      Wire.write(thres);
      Wire.endTransmission();
      xSemaphoreGive(i2c_mutex);
      return;
    }
  }
}
// nastavi dobu zapnuto pri dotyku v autonomnim rezimu
void autonomusInterval(uint8_t address, int inter)
{
  while (true)
  {
    if (xSemaphoreTake(i2c_mutex, tickCount) == pdTRUE)
    {
      Wire.beginTransmission(address);
      Wire.write(0x0A);
      Wire.write(highByte(inter));
      Wire.write(lowByte(inter));
      Wire.endTransmission();
      xSemaphoreGive(i2c_mutex);
      return;
    }
  }
}
// nastavi intenzitu osvetleni ve stavu zapnuto v autonomnim rezimu
void autonomusHigh(uint8_t address, uint8_t PWM)
{
  while (true)
  {
    if (xSemaphoreTake(i2c_mutex, tickCount) == pdTRUE)
    {
      Wire.beginTransmission(address);
      Wire.write(0x07);
      Wire.write(PWM);
      Wire.endTransmission();
      xSemaphoreGive(i2c_mutex);
      return;
    }
  }
}
// nastavi intenzitu osvetleni ve stavu vypnuto v autonomnim rezimu
void autonomusLow(uint8_t address, uint8_t PWM)
{
  while (true)
  {
    if (xSemaphoreTake(i2c_mutex, tickCount) == pdTRUE)
    {
      Wire.beginTransmission(address);
      Wire.write(0x08);
      Wire.write(PWM);
      Wire.endTransmission();
      xSemaphoreGive(i2c_mutex);
      return;
    }
  }
}
// zapne/vypne spolecnou anodu
void commonAnode(uint8_t address, bool commonAnode)
{
  while (true)
  {
    if (xSemaphoreTake(i2c_mutex, tickCount) == pdTRUE)
    {
      Wire.beginTransmission(address);
      Wire.write(0x0D);
      if (commonAnode)
      {
        Wire.write(0xFF);
      }
      else
      {
        Wire.write(0);
      }
      Wire.endTransmission();
      xSemaphoreGive(i2c_mutex);
      return;
    }
  }
}

void easterEggMode(uint8_t address)
{
  if (random(0, 5) == 0)
  {
    writePWM(address, 255);
  }
  else
  {
    writePWM(address, 5);
  }
  delay(800);
}