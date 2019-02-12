#include <QTouchADCTiny.h>
#include <Wire.h>
#include <EEPROM.h>

#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))

#define refPin PB5 //piny
#define sensePin PB4
#define ledPin PB1

uint8_t value = 0;
int correction = 0;
int threshold = 0;
int interval = 0;
unsigned long onMillis = 0;
uint8_t address = 0;
uint8_t pwmValue = 0;
uint8_t prevPwmValue = 0;
uint8_t rychlost = 0;
uint8_t X = 0;
uint8_t Y = 0;
uint8_t foo = 0;
int bar = 0;
uint8_t autonomusHigh = 0;
uint8_t autonomusLow = 0;
uint8_t sample = 1;
uint8_t modeSelect = 3;
uint8_t outputValue = 0;
boolean fade = true;
boolean location = false;
boolean turnOn = false;
unsigned long fadeMicros = 0;
unsigned long milliRead = 0;

//EEPROM
//0x10 - address, 0x11 - X souřadnice, 0x12 - Y souřadnice, 0x13 - rychlost, 0x14 - fade

void setPWM(uint8_t pwm)
{
  if (pwm == 0)
  {
    cbi(TCCR0A, COM0B1);
    PORTB = PORTB & B11111101;
  }
  else if (pwm == 255)
  {
    cbi(TCCR0A, COM0B1);
    PORTB = PORTB | B00000010;
  }
  else
  {
    sbi(TCCR0A, COM0B1);
    OCR0B = pwm;
  }
}

void fading()
{
  if ((millis() - fadeMicros) > ( rychlost) ) {
    fadeMicros = millis();
    if (prevPwmValue > pwmValue) {
      prevPwmValue--;
      setPWM(prevPwmValue);
    }
    else if ( prevPwmValue < pwmValue) {
      prevPwmValue++;
      setPWM(prevPwmValue);
    }
  }
}

//když master pošle data, tak je hodíme na LEDku jako PWM
void receiveEvent(uint8_t num)
{
  switch (Wire.read())
  {
    case 0x00:
      pwmValue = Wire.read();
      EEPROM.write(0x00, pwmValue);
      if ( pwmValue > prevPwmValue) {
        turnOn = true;
      }
      else if ( pwmValue < prevPwmValue) {
        turnOn = false;
      }
      goto out;
      break;
    case 0x01:
      rychlost = Wire.read();
      EEPROM.write(0x01, rychlost);
      goto out;
      break;
    case 0x02:
      foo = Wire.read();
      if (foo == 0xFF)
      {
        fade = true;
        EEPROM.write(0x02, 0xFF);
        goto out;
      }
      else if (foo == 0x00)
      {
        fade = false;
        EEPROM.write(0x02, 0x00);
        goto out;
      }
      break;
    case 0x03:
      address = Wire.read();
      EEPROM.write(0x03, address);
      goto out;
      break;
    case 0x04:
      X = Wire.read();
      Y = Wire.read();
      EEPROM.write(0x04, X);
      EEPROM.write(0x05, Y);
      goto out;
      break;
    case 0x05:
      location = true;
      goto out;
      break;
    case 0x07:
      autonomusHigh = Wire.read();
      EEPROM.write(0x07, autonomusHigh);
      goto out;
      break;
    case 0x08:
      autonomusLow = Wire.read();
      EEPROM.write(0x08, autonomusLow);
      goto out;
      break;
    case 0x09:
      threshold = Wire.read();

      EEPROM.write(0x09, threshold);
      goto out;
      break;
    case 0x0A:
      interval = Wire.read();
      foo = Wire.read();
      interval = interval << 8;
      interval += foo;

      EEPROM.write(0x0B, highByte(interval));
      EEPROM.write(0x0C, lowByte(interval));
      goto out;
      break;
    case 0x0B:
      sample = Wire.read();
      EEPROM.write(0x0D, sample);
      break;
    case 0x0C:
      modeSelect = Wire.read();
      turnOn = false;
      onMillis = 0;
      break;
    default:
      break;
  }
out:
  __asm__("nop\n\t");
}

void loadEEPROM()
{

  rychlost = EEPROM.read(0x01);

  if (EEPROM.read(0x02) == 255)
  {
    fade = true;
  }
  else if (EEPROM.read(0x02) == 0)
  {
    fade = false;
  }

  address = EEPROM.read(0x03);
  if (address == 0x00 || address == 0xFF)
  {
    address = 0x04;
  }

  X = EEPROM.read(0x04);
  Y = EEPROM.read(0x05);

  autonomusHigh = EEPROM.read(0x07);
  autonomusLow = EEPROM.read(0x08);
  pwmValue = autonomusLow;
  if (autonomusHigh == autonomusLow)
  {
    autonomusLow = 0x00;
  }

  threshold = EEPROM.read(0x09);

  interval = EEPROM.read(0x0B);
  interval = interval << 8;
  interval += EEPROM.read(0x0C);
  sample = EEPROM.read(0x0D);
}

//když si master vyžádá data, tak mu pošleme číslo value jako dva byty
void requestEvent()
{
  if (location)
  {
    Wire.write(X);
    Wire.write(Y);
    location = false;
  }
  else
  {
    Wire.write(outputValue);
  }
}

void beginI2C()
{
  Wire.begin(address);
  Wire.onReceive(receiveEvent);
  Wire.onRequest(requestEvent);
}

void setup()
{
  delay(300);
  MCUSR = 0;
  DDRB = DDRB | B00000010;
  PORTB = PORTB & B11111101;
  QTouchADCTiny.init();
  loadEEPROM();
  delay(1);
  beginI2C();
  delay(1);
  correction = QTouchADCTiny.sense(sensePin, refPin, 128) - 10;
}



void loop()
{
  if (((millis() - milliRead) > 10) && millis() > 1000) {
    milliRead = millis();
    bar = QTouchADCTiny.sense(sensePin, refPin, sample) - correction;

    if (bar > 255)
    {
      value = 255;
    }
    else if (bar < 0)
    {
      value = 0;
    }
    else
    {
      value = bar;
    }

    switch (modeSelect) {
      case 0:
        outputValue = value;
        break;
      case 1:
        if (value > threshold) {
          outputValue = 1;
        }
        else {
          outputValue = 0;
        }
        break;
      case 2:
        if (value > threshold)
        {
          if (!turnOn)
          {
            pwmValue = autonomusHigh;
            turnOn = true;
          }
        }
        break;
      case 3:
        if (value > threshold)
        {
          if (!turnOn)
          {
            pwmValue = autonomusHigh;
            turnOn = true;
          }
          //pokud je dotyk, tak vždy restartujeme počítadlo
          onMillis = millis();
        }

        if (((millis() - onMillis) > interval) && turnOn)
        {
          pwmValue = autonomusLow;
          turnOn = false;
        }
        break;
      default:
        break;
    }
  }

  if (prevPwmValue != pwmValue)
  {
    if (!fade)
    {
      setPWM(pwmValue);
      prevPwmValue = pwmValue;
    }
    else
    {
      fading();
    }
  }
}