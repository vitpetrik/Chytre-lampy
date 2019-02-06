#include <Arduino.h>
#include <QTouchADCTiny.h>
#include <TinyWireS.h>
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
boolean fade = true;
boolean location = false;
boolean autonomus = true;
boolean turnOn = false;

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

void fading(uint8_t now, uint8_t prev)
{
  if (now > prev)
  {
    for (byte i = prev; i <= now; i++)
    {
      setPWM(i);
      delayMicroseconds(rychlost * 40);
      if (i == 255)
        break;
    }
  }
  else
  {
    for (byte i = prev; i >= now; i--)
    {
      setPWM(i);
      delayMicroseconds(rychlost * 40);
      if (i == 0)
        break;
    }
  }
}

//když master pošle data, tak je hodíme na LEDku jako PWM
void receiveEvent(uint8_t num)
{
  switch (TinyWireS.receive())
  {
  case 0x00:
    pwmValue = TinyWireS.receive();
    EEPROM.write(0x00, pwmValue);
    goto out;
    break;
  case 0x01:
    rychlost = TinyWireS.receive();
    EEPROM.write(0x01, rychlost);
    goto out;
    break;
  case 0x02:
    foo = TinyWireS.receive();
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
    address = TinyWireS.receive();
    EEPROM.write(0x03, address);
    goto out;
    break;
  case 0x04:
    X = TinyWireS.receive();
    Y = TinyWireS.receive();
    EEPROM.write(0x04, X);
    EEPROM.write(0x05, Y);
    goto out;
    break;
  case 0x05:
    location = true;
    goto out;
    break;
  case 0x06:
    foo = TinyWireS.receive();
    if (foo == 0xFF)
    {
      autonomus = true;
      goto out;
    }
    else if (foo == 0x00)
    {
      autonomus = false;
      goto out;
    }
    break;
  case 0x07:
    autonomusHigh = TinyWireS.receive();
    EEPROM.write(0x07, autonomusHigh);
    goto out;
    break;
  case 0x08:
    autonomusLow = TinyWireS.receive();
    EEPROM.write(0x08, autonomusLow);
    goto out;
    break;
  case 0x09:
    threshold = TinyWireS.receive();

    EEPROM.write(0x09, threshold);
    goto out;
    break;
  case 0x0A:
    interval = TinyWireS.receive();
    foo = TinyWireS.receive();
    interval = interval << 8;
    interval += foo;

    EEPROM.write(0x0B, highByte(interval));
    EEPROM.write(0x0C, lowByte(interval));
    goto out;
    break;
  case 0x0B:
    sample = TinyWireS.receive();
    EEPROM.write(0x0D, sample);
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
    TinyWireS.send(X);
    TinyWireS.send(Y);
    location = false;
  }
  else if (autonomus)
  {
    if (turnOn && bar > threshold)
    {
      TinyWireS.send(0x02);
    }
    else if(turnOn){
      TinyWireS.send(0x01);
    }
    else
    {
      TinyWireS.send(0);
    }
  }
  else
  {
    TinyWireS.send(value);
  }
}

void beginI2C()
{
  TinyWireS.begin(address);
  TinyWireS.onReceive(receiveEvent);
  TinyWireS.onRequest(requestEvent);
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
  correction = QTouchADCTiny.sense(sensePin, refPin, 128)-10;
}

void loop()
{
  TinyWireS_stop_check();
  tws_delay(2);
  bar = QTouchADCTiny.sense(sensePin, refPin, sample) - correction;
  tws_delay(2);

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

  if (autonomus)
  {
    if (bar > threshold)
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
  }

  if (prevPwmValue != pwmValue)
  {
    if (!fade)
    {
      setPWM(pwmValue);
    }
    else
    {
      fading(pwmValue, prevPwmValue);
    }
    prevPwmValue = pwmValue;
  }
}