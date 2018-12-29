//Vít Petřík@2018
//SCL = pin 7
//SDA = pin 5
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))

#include <QTouchADCTiny.h>
#include <TinyWireS.h>
#include <EEPROM.h>

#define refPin PB5              //piny
#define sensePin PB4
#define ledPin PB1

int value;
uint8_t address;
uint8_t pwmValue = 0;
uint8_t prevPwmValue = 0;
uint8_t rychlost = 60;
uint8_t X = 0x69;
uint8_t Y = 0x88;
uint8_t foo = 0x00;
uint8_t bar = 0x00;
boolean fade = true;
boolean location = false;

//EEPROM
//0x10 - address, 0x11 - X souřadnice, 0x12 - Y souřadnice, 0x13 - rychlost, 0x14 - fade
void loadEEPROM() {
  address = EEPROM.read(0xF0);
  if (address == 0x00 || address == 0xFF) {
    address = 0x04;
  }
  delay(10);
  X = EEPROM.read(0xF1);
  delay(10);
  Y = EEPROM.read(0xF2);
  delay(10);
  rychlost = EEPROM.read(0xF3);
  delay(10);
  if (EEPROM.read(0xF4) == 255) {
    fade = true;
  }
  else if (EEPROM.read(0xF4) == 0) {
    fade = false;
  }
}

void beginI2C() {
  TinyWireS.begin(address);
  TinyWireS.onReceive(receiveEvent);
  TinyWireS.onRequest(requestEvent);
}

void fading (uint8_t now, uint8_t prev) {
  if ( now > prev ) {
    for ( byte i = prev; i <= now; i++) {
      setPWM(i);
      delayMicroseconds(rychlost * 40 );
      if (i == 255)
        break;
    }
  }
  else {
    for ( byte i = prev; i >= now; i--) {
      setPWM(i);
      delayMicroseconds(rychlost * 40);
      if (i == 0)
        break;
    }
  }
}

void setPWM(uint8_t value) {
  if ( value == 0 ) {
    cbi(TCCR0A, COM0B1);
    PORTB = PORTB & B11111101;
  }
  else if ( value == 255 ) {
    cbi(TCCR0A, COM0B1);
    PORTB = PORTB | B00000010;
  }
  else {
    sbi(TCCR0A, COM0B1);
    OCR0B = value;
  }
}

//když master pošle data, tak je hodíme na LEDku jako PWM
void receiveEvent(uint8_t num)
{
  switch (TinyWireS.receive()) {
    case 0x00:
      pwmValue = TinyWireS.receive();
      break;
    case 0x01:
      rychlost = TinyWireS.receive();
      EEPROM.write(0xF3, rychlost);
      break;
    case 0x02:
      foo = TinyWireS.receive();
      if (foo == 0xFF) {
        fade = true;
        EEPROM.write(0xF4, 0xFF);
      }
      else if (foo == 0x00) {
        fade = false;
        EEPROM.write(0xF4, 0x00);
      }
      break;
    case 0x03:
      address = TinyWireS.receive();
      EEPROM.write(0xF0, address);
      break;
    case 0x04:
      X = TinyWireS.receive();
      Y = TinyWireS.receive();
      EEPROM.write(0xF1, X);
      EEPROM.write(0xF2, Y);
      break;
    case 0x05:
      location = true;
      delay(1);
      break;
    default:
      break;
  }
}

//když si master vyžádá data, tak mu pošleme číslo value jako dva byty
void requestEvent()
{
  if (location) {
    TinyWireS.send(X);
    TinyWireS.send(Y);
    location = false;
  }
  else {
    value = QTouchADCTiny.sense(sensePin, refPin, 1);
    TinyWireS.send(highByte(value));
    TinyWireS.send(lowByte(value));
  }
}

void setup() {
  delay(50);
  MCUSR = 0;
  DDRB = DDRB | B00000010;
  PORTB = PORTB & B11111101;
  QTouchADCTiny.init();
  loadEEPROM();
  beginI2C();
}

void loop() {
  if (prevPwmValue != pwmValue) {
    if (!fade) {
      setPWM(pwmValue);
    }
    else {
      fading(pwmValue, prevPwmValue);
    }
    prevPwmValue = pwmValue;
  }
}
