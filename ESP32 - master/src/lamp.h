#include <Arduino.h>
#include <Wire.h>

//přečte hodnotu z dotykového čidla
uint8_t readTouch(int address)
{
  Wire.requestFrom(address, 1);
  return Wire.read();
}

//přečte souřadnice a vrátí je v poli [ X, Y ]
uint8_t *readPosition(uint8_t address)
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
void writeI2CAddress(uint8_t address, uint8_t newAddress)
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
void writePosition(uint8_t address, uint8_t X, uint8_t Y)
{
  Wire.beginTransmission(address);
  Wire.write((uint8_t)0x04);
  Wire.write(X);
  Wire.write(Y);
  Wire.endTransmission();
}

void writeSample(uint8_t address, uint8_t sample){
  Wire.beginTransmission(address);
  Wire.write(0x0B);
  Wire.write(sample);
  Wire.endTransmission();
}

void writeMode(uint8_t address, uint8_t Mode) {
  Wire.beginTransmission(address);
  Wire.write(0x0C);
  Wire.write(Mode);
  Wire.endTransmission();
}

void writeThreshold(uint8_t address, uint8_t thres)
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