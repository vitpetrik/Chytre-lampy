#include <QTouchADCTiny.h>
#include <Wire.h>
#include <EEPROM.h>

#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))

//nastavení pinů
#define refPin PB5
#define sensePin PB4
#define ledPin PB1

//globální proměnné
uint8_t value = 0;
int correction = 0;           //korekce, která se udělá po startu programu
int threshold = 0;            //hodnota, nad kterou je sensor aktivní
int interval = 0;             //interval pro autonomní řízení
unsigned long onMillis = 0;   //millis pro autonomní řízení
uint8_t address = 0;          //I2C adresa
uint8_t pwmValue = 0;         //PWM hodnota na LEDce
uint8_t prevPwmValue = 0;     //proměnná pro fading
uint8_t rychlost = 0;         //rychlost fadingu
uint8_t X = 0;                //X souřadnice
uint8_t Y = 0;                //Y souřadnice
uint8_t foo = 0;            
int bar = 0;
uint8_t autonomusHigh = 0;    //PWM hodnota pro rozsvícení pro autonomní režim
uint8_t autonomusLow = 0;     //PWM hodnota pro pohasnutí pro autonmní režim
uint8_t sample = 1;           //Z kolika měření se má zprůměrovat výsledná hodnota
uint8_t modeSelect = 3;       //Mód řízení - zprvu je autonomní - 3
uint8_t outputValue = 0;      //hodnota, která se pošle přes I2C
boolean fade = true;          //zapnutí plynulé změny svitu
boolean location = false;     //Rozhoduje, zda se pošlou souřadnice, nebo ne
boolean turnOn = false;       //Jestli jsme high nebo low
boolean commonAnode = false;  //pokud by došlo ke špatnému zapojení LEDky, dalo by se to ještě spravit
unsigned long fadeMicros = 0; //millis pro fading
unsigned long milliRead = 0;  //ze senzoru čteme každých 10 miliSekund - není důvod po menších intervalech

//nastaví aktuální hodnotu PWM
void setPWM(uint8_t pwm)
{
  if (commonAnode)
    pwm = ~pwm;
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

//plynulá změna svitu
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

//Master pošle data
void receiveEvent(uint8_t num)
{
  //Co se stane se rozhoduje dle prvního zaslaného bytu
  switch (Wire.read())
  {
    //Zápis PWM hodnoty
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
    //Zápis rychlosti fadingu
    case 0x01:
      rychlost = Wire.read();
      EEPROM.write(0x01, rychlost);
      goto out;
      break;
    //Zápis, zdali budeme plynule měnit svit
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
    //Zápis I2C adresy
    case 0x03:
      address = Wire.read();
      EEPROM.write(0x03, address);
      goto out;
      break;
    //Zápis souřadnic
    case 0x04:
      X = Wire.read();
      Y = Wire.read();
      EEPROM.write(0x04, X);
      EEPROM.write(0x05, Y);
      goto out;
      break;
    //Pokud chceme číst pozici
    case 0x05:
      location = true;
      goto out;
      break;
    //Nastavení high hodnoty autonomního režimu
    case 0x07:
      autonomusHigh = Wire.read();
      EEPROM.write(0x07, autonomusHigh);
      goto out;
      break;
    //Nastavení low hodnoty autonomního režimu
    case 0x08:
      autonomusLow = Wire.read();
      EEPROM.write(0x08, autonomusLow);
      goto out;
      break;
    //hodnota, nad kterou je sensor aktivní
    case 0x09:
      threshold = Wire.read();
      EEPROM.write(0x09, threshold);
      goto out;
      break;
    //Doba pro svícení lampy po dotyku - vyžaduje 16-bitové číslo
    case 0x0A:
      interval = Wire.read();
      foo = Wire.read();
      interval = interval << 8;
      interval += foo;

      EEPROM.write(0x0B, highByte(interval));
      EEPROM.write(0x0C, lowByte(interval));
      goto out;
      break;
    //počet samplů, ze kterých se zprůměruje výsledná hodnots
    case 0x0B:
      sample = Wire.read();
      EEPROM.write(0x0D, sample);
      break;
    //Výběr módu
    case 0x0C:
      modeSelect = Wire.read();
      turnOn = false;
      onMillis = 0;
      break;
    //rezím commonAnode
    case 0x0D:
      foo = Wire.read();
      if (foo == 0xFF)
      {
        commonAnode = true;
        EEPROM.write(0x0E, 0xFF);
        goto out;
      }
      else if (foo == 0x00)
      {
        commonAnode = false;
        EEPROM.write(0x0E, 0x00);
        goto out;
      }
      break;
    default:
      break;
  }
out:
  __asm__("nop\n\t");
}

void loadEEPROM()
{
  //načtení rychlosti fadingu z EEPROM
  rychlost = EEPROM.read(0x01);

  //Jestli budeme plynule měnit PWM hodnotu
  if (EEPROM.read(0x02) == 255)
  {
    fade = true;
  }
  else if (EEPROM.read(0x02) == 0)
  {
    fade = false;
  }

  //přečtení I2C adresy
  //Pokud přečteme hodnotu 0xFF nastavíme jako adresu 0x04
  address = EEPROM.read(0x03);
  if (address == 0x00 || address == 0xFF)
  {
    address = 0x04;
  }

  //Načtení pozice
  X = EEPROM.read(0x04);
  Y = EEPROM.read(0x05);

  //Načtení hodnot low a high pro autonomní režim
  autonomusHigh = EEPROM.read(0x07);
  autonomusLow = EEPROM.read(0x08);
  pwmValue = autonomusLow;
  if (autonomusHigh == autonomusLow)
  {
    autonomusLow = 0x00;
  }

  //hodnota, nad kterou je sensor aktivní
  threshold = EEPROM.read(0x09);

  //přečtení intervalu pro autonomní režim
  interval = EEPROM.read(0x0B);
  interval = interval << 8;
  interval += EEPROM.read(0x0C);
  sample = EEPROM.read(0x0D);

  //režim commonAnode
  if (EEPROM.read(0x0E) == 255)
  {
    commonAnode = true;
  }
  else if (EEPROM.read(0x0E) == 0)
  {
    commonAnode = false;
  }
}

//Master si vyžádá data
void requestEvent()
{
  //Pokud je aktivní proměnná location pošleme dva byty
  //jenž reprezentují X a Y souřadnice lampy
  if (location)
  {
    Wire.write(X);
    Wire.write(Y);
    location = false;
  }
  //Pošleme hdonotu outputValue - její hodnota
  //je závislá na vybraném módu
  else
  {
    Wire.write(outputValue);
  }
}

//inicializace I2C sběrnice
void beginI2C()
{
  Wire.begin(address);
  Wire.onReceive(receiveEvent);
  Wire.onRequest(requestEvent);
}

void setup()
{
  delay(300);
  //Nastavení PWM pinu
  MCUSR = 0;
  DDRB = DDRB | B00000010;
  PORTB = PORTB & B11111101;
  //nastavení dotykového sensoru
  QTouchADCTiny.init();
  //načtení z EEPROM
  loadEEPROM();
  delay(1);
  //Inicializace I2C
  beginI2C();
  delay(1);
  //Hodnota pro korekci výstupu ze sensoru
  correction = QTouchADCTiny.sense(sensePin, refPin, 128) - 10;
}



void loop()
{
  if (((millis() - milliRead) > 10) && millis() > 1000) {
    milliRead = millis();

    //přečtení sensoru
    bar = QTouchADCTiny.sense(sensePin, refPin, sample) - correction;

    //Ořežeme číslo na 1 byte
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

    //Různé módy
    switch (modeSelect) {

      //Mód 0
      //Masterovi pošleme rovnou číslo ze sensoru
      //komparace se musí dělat na straně mastera
      case 0:
        outputValue = value;
        break;

      //Mód 1
      //podobný jako mód 0
      //komparace se vykonává na straně slave
      //neaktivní sensor - 0
      //aktivní sensor - 1
      case 1:
        if (value > threshold) {
          outputValue = 1;
        }
        else {
          outputValue = 0;
        }
        break;
      
      //Mód 2
      //Pokud je sensor aktivní
      //zapneme LEDku
      //ale už jí sami nevypneme
      //Výstup je podobný jako u módu 1
      case 2:
        if (value > threshold)
        {
          outputValue = 1;
          if (!turnOn)
          {  
            pwmValue = autonomusHigh;
            turnOn = true;
          }
        }
        else
        {
          outputValue = 0;
        }
        break;

      //Mód 3
      //plně autonomní režim řízení
      //doba rozsvícení je řízena pomocí proměnné interval
      case 3:
        if (value > threshold)
        {
          outputValue = 1;
          if (!turnOn)
          {
            pwmValue = autonomusHigh;
            turnOn = true;
          }
          //pokud je dotyk, tak vždy restartujeme počítadlo
          onMillis = millis();
        }
        else
        {
          outputValue = 0;
        }
        
        //Pokud vypršel interval vypneme LEDku
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

  //Pokud se změnila PWM hodnota, budeme s tím něco dělat
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