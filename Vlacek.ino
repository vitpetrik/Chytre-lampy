#define MICROSECONDS_PER_TIMER0_OVERFLOW (clockCyclesToMicroseconds(1*510))
int maxSpeed = 50 ;
int accSpeed = 10;
int breakSpeed = 5;

int pinOutOne = 5;
int pinOutTwo = 6;

int pinKoncakOne = 9;
int pinKoncakTwo = 8;
int pinSwitch = 3;

bool pohyb = false;
int smer = 1;
int stateOne = 0;
int stateTwo = 0;
int pocetSepnuti = 0;
int dobaStani = 1;     //Sekundy

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

  if(stavSwitch){
    if(!pohyb){
      Accelerate();
      pohyb = true;
    } else {
      Stop();
      pocetSepnuti = 0;
      pohyb = false;
    }
    delay(300*64);
  }

  if(pocetSepnuti == 0){
    if(stavKoncakOne && stavKoncakOne != stateOne){
      Break();
      Serial.print("Pocet sepntui ");
      Serial.println(pocetSepnuti);
      Serial.print(". Brzdím! pičo\n");
      pocetSepnuti++;
    }
    if(stavKoncakTwo && stavKoncakTwo != stateTwo){
      Break();
      Serial.print("Pocet sepntui ");
      Serial.println(pocetSepnuti);
      Serial.print(". Brzdím! pičo\n");
      pocetSepnuti++;
    }
    goto here;
  }

  if(pocetSepnuti == 1){
    if(stavKoncakOne && stavKoncakOne != stateOne){
      Stop();
      Serial.print("Pocet sepntui ");
      Serial.println(pocetSepnuti);
      Serial.print(". Měním směr! pičo\n");
      smer = 2;
      Accelerate();
      pocetSepnuti++;
    }
    if(stavKoncakTwo && stavKoncakTwo != stateTwo){
      Stop();
      Serial.print("Pocet sepntui ");
      Serial.println(pocetSepnuti);
      Serial.print(". Měním směr! pičo\n");
      smer = 1;
      Accelerate();
      pocetSepnuti++;
    }
    delay(230*64);
    goto here;
  }

  if(pocetSepnuti == 2){
    if(stavKoncakOne && stavKoncakOne != stateOne){
      pocetSepnuti=0;
      Serial.print("Pocet sepntui zase ");
      Serial.println(pocetSepnuti);
      Serial.print(". Připraven na další otočku ! pičo\n");
    }
    if(stavKoncakTwo && stavKoncakTwo != stateTwo){
      pocetSepnuti=0;
      Serial.print("Pocet sepntui zase ");
      Serial.println(pocetSepnuti);
      Serial.print(". Připraven na další otočku ! pičo\n");
    }
  }

here:
  stateOne = stavKoncakOne;
  stateTwo = stavKoncakTwo;
}

void Stop(){
  analogWrite(pinOutOne, 0);
  analogWrite(pinOutTwo, 0);
}

void Accelerate(){
  if(smer == 1 ){
    for(int i = 0; i <= maxSpeed; i = i + 2){
      analogWrite(pinOutOne, i);
      analogWrite(pinOutTwo, 0);
      delay(accSpeed*64);
    }
  }
  if(smer == 2){
    for(int i = 0; i <= maxSpeed; i = i + 2){
      analogWrite(pinOutOne, 0);
      analogWrite(pinOutTwo, i);
      delay(accSpeed*64);
    }
  }
}

void Break(){
  if(smer == 1){
    for(int i = maxSpeed; i >= 35; i--){
      analogWrite(pinOutOne, i);
      analogWrite(pinOutTwo, 0);
      delay(breakSpeed*64);
    }
  }
  if(smer == 2){
    for(int i = maxSpeed; i >= 35; i--){
      analogWrite(pinOutOne, 0);
      analogWrite(pinOutTwo, i);
      delay(breakSpeed*64);
    }
  }
}
