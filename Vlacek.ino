//Zvýšení frekvence PWM na maximální možnou hodnotu
#define MICROSECONDS_PER_TIMER0_OVERFLOW (clockCyclesToMicroseconds(1*510))

// ------------- Nastavení -------------
int maxSpeed = 50 ;               // Maximální rychlost
int slowSpeed = 35;               // Dojezdová rychlost
int accSpeed = 2;                 // Rychlost akcelerace
int breakSpeed = 2;               // Rychlost brždění
int dobaStani = 1;                // Doba stání v sekundách
int smer = 1;                     // Počáteční směr vlakového modelu

int pinOutOne = 5;                // Pin na kterém je první Input H-můstku
int pinOutTwo = 6;                // Pin na kterém je druhý Input H-můstku
int pinKoncakOne = 9;             // Pin koncáku na prvním konci
int pinKoncakTwo = 8;             // Pin koncáku na druhém konci
int pinSwitch = 3;                // Pin centrálního spouštěcího a vypínacího tlačítka
// ------------- Konec Nastavení -------

// Globální proměnné (NEZASAHOVAT!)
bool pohyb = false;
int stateOne = 0;
int stateTwo = 0;
int pocetSepnuti = 0;

void setup()
{
  Serial.begin(9600);
  pinMode(pinOutOne, OUTPUT);
  pinMode(pinOutTwo, OUTPUT);
  pinMode(pinKoncakOne, INPUT);
  pinMode(pinKoncakTwo, INPUT);
  pinMode(pinSwitch, INPUT);
  TCCR0A = _BV(COM0A1) | _BV(COM0B1) | _BV(WGM01) | _BV(WGM00);
  TCCR0B = _BV(CS00);
}

void loop()
{
  int stavKoncakOne = digitalRead(pinKoncakOne);
  int stavKoncakTwo = digitalRead(pinKoncakTwo);
  int stavSwitch = digitalRead(pinSwitch);

  // Ovládání pohybu vláčku pomocí tlačítka vypnout/zapnout
  if(stavSwitch){
    if(!pohyb){
      Accelerate();
      pohyb = true;
    } else {
      Stop();
      pocetSepnuti = 0;
      pohyb = false;
    }
    delay(300*64);            // Ochrana proti několikánásobným stisknutím
  }

  // Reakce na první přejetí čidla
  if(pocetSepnuti == 0){
    if(stavKoncakOne && stavKoncakOne != stateOne){
      Break();
      pocetSepnuti++;
    }
    if(stavKoncakTwo && stavKoncakTwo != stateTwo){
      Break();
      pocetSepnuti++;
    }
    goto here;
  }

  // Reakce na druhý přejetí čidla
  if(pocetSepnuti == 1){
    if(stavKoncakOne && stavKoncakOne != stateOne){
      Stop();
      smer = 2;
      for(int i = 0; i <= (dobaStani-1); i++){
        delay(500*64);
        delay(500*64);
      }
      Accelerate();
      pocetSepnuti++;
    }
    if(stavKoncakTwo && stavKoncakTwo != stateTwo){
      Stop();
      smer = 1;
      for(int i = 0; i <= (dobaStani-1); i++){
        delay(500*64);
        delay(500*64);
      }
      Accelerate();
      pocetSepnuti++;
    }
    delay(230*64);                  // Vyřešení nesymetrie magnetů na vláčku
    goto here;
  }

  // Reakce na třetí přejetí čidla
  if(pocetSepnuti == 2){
    if(stavKoncakOne && stavKoncakOne != stateOne){
      pocetSepnuti=0;
    }
    if(stavKoncakTwo && stavKoncakTwo != stateTwo){
      pocetSepnuti=0;
    }
  }

here:
  stateOne = stavKoncakOne;
  stateTwo = stavKoncakTwo;
}

// Zastavení vláčku
void Stop(){
  analogWrite(pinOutOne, 0);
  analogWrite(pinOutTwo, 0);
}

// Rozjezd vláčku
void Accelerate(){
  if(smer == 1 ){
    for(int i = 0; i <= maxSpeed; i++){
      analogWrite(pinOutOne, i);
      analogWrite(pinOutTwo, 0);
      delay(accSpeed*64);
    }
  }
  if(smer == 2){
    for(int i = 0; i <= maxSpeed; i++){
      analogWrite(pinOutOne, 0);
      analogWrite(pinOutTwo, i);
      delay(accSpeed*64);
    }
  }
}

// Zpomalení vláčku
void Break(){
  if(smer == 1){
    for(int i = maxSpeed; i >= slowSpeed; i--){
      analogWrite(pinOutOne, i);
      analogWrite(pinOutTwo, 0);
      delay(breakSpeed*64);
    }
  }
  if(smer == 2){
    for(int i = maxSpeed; i >= slowSpeed; i--){
      analogWrite(pinOutOne, 0);
      analogWrite(pinOutTwo, i);
      delay(breakSpeed*64);
    }
  }
}
