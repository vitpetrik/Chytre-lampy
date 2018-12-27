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
byte address = 0x04;
byte pwmValue = 0;
byte prevPwmValue = 0;
byte rychlost = 60;
byte X = 22;
byte Y = 120;
boolean fade = true;
boolean location = false;

//EEPROM
//0x00 - address, 0x01 - X souřadnice, 0x02 - Y souřadnice, 0x03 - rychlost, 0x04 - fade
void loadEEPROM() {
  if (EEPROM.read(0x00) != 0x00) {
    address = EEPROM.read(0x00);
  }
  X = EEPROM.read(0x01);
  Y = EEPROM.read(0x02);
  rychlost = EEPROM.read(0x03);
  if (EEPROM.read(0x04) == 255) {
    fade = true;
  }
  else if (EEPROM.read(0x04) == 0) {
    fade = false;
  }
}

void beginI2C() {
  TinyWireS.begin(address);
  TinyWireS.onReceive(receiveEvent);
  TinyWireS.onRequest(requestEvent);
}

void fading (byte now, byte prev) {
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

void setPWM(byte value) {
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
void receiveEvent(uint8_t byty)
{
  if (byty == 1) {
    if ( TinyWireS.receive() == 0xFF ) {
      location = true;
    }
  }
  if (byty == 2) {
    switch (TinyWireS.receive()) {
      case 0x00:
        pwmValue = TinyWireS.receive();
        break;
      case 0x01:
        rychlost = TinyWireS.receive();
        EEPROM.write(0x03, rychlost);
        break;
      case 0x02:
        byte foo = TinyWireS.receive();
        if (foo == 0xFF) {
          fade = true;
          EEPROM.write(0x04, 0xFF);
        }
        else if (foo == 0x00) {
          fade = false;
          EEPROM.write(0x04, 0x00);
        }
        break;
      case 0x03:
        address = TinyWireS.receive();
        EEPROM.write(0x00, address);
        break;
      default:
        break;
    }
  }
}

//když si master vyžádá data, tak mu pošleme číslo value jako dva byty
void requestEvent()
{
  if (location) {
    TinyWireS.send(address);
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
