#include "RotaryPendulum.h"

unsigned int perioda = 70;    // perioda opakavania cyklu v ms
unsigned long int t;    // premenna do ktorej sa uklada aktualny cas
unsigned long int dalsi_krok;   // premenna, ktora definuje, kedy nastane dalsi cyklus (krok)
bool pretecenie = false;    // premenna oznacujuca pretecenie


void timer1extend()
{
    STEP_HIGH
    STEP_LOW

  //  skracovanie delayu pri zrychlovani
  if ( RotaryPendulum.rampUpStepCount == 0 ) { // ramp up phase  
    RotaryPendulum.n++;
    RotaryPendulum.d = RotaryPendulum.d - (2 * RotaryPendulum.d) / (4 * RotaryPendulum.n + 1);
    if ( RotaryPendulum.d <= RotaryPendulum.maxDelay ) { // reached max speed
      RotaryPendulum.d = RotaryPendulum.maxDelay;
      RotaryPendulum.rampUpStepCount = 1;
    }
  }

  // predlzovanie delayu pri spomalovani
  else if ( RotaryPendulum.rampUpStepCount == 2 ) { // ramp down phase
    RotaryPendulum.n++;
    RotaryPendulum.d = RotaryPendulum.d*(4*RotaryPendulum.n+1)/(4*RotaryPendulum.n-1);
    if ( RotaryPendulum.d >= RotaryPendulum.maxDelay ) { // reached max speed
      RotaryPendulum.d = RotaryPendulum.maxDelay;
      RotaryPendulum.rampUpStepCount = 1;
    }
  }

  OCR1A = RotaryPendulum.d;  // pocet ms po kolkych sa zapne interrupt
}

void setup() {
  Serial.begin(115200);
  RotaryPendulum.begin();
  RotaryPendulum.calibration();
  RotaryPendulum.timer1Hook(timer1extend);
  dalsi_krok = millis() + perioda;    // zadefinovanie casu, v ktorom nastane prvy cyklus
}

void loop() {
  t = millis();   //  odcitanie hodnoty aktualneho casu
  if(t >= dalsi_krok) {   // overenie podmienky, ci uz nastal cas pre dalsi krok
    if(!pretecenie){    // podmienka, ktora overuje pretecenie
      RotaryPendulum.krok();     // vykonanie pokynov v cykle
      dalsi_krok += perioda;    // zadefinovanie casu pre novy cyklus
      if(dalsi_krok < t){     // uprava v pripade pretecenia premennej "dalsi_krok"
        pretecenie = true;
      }
    }
    else{
      if(t < perioda){    // podmienka, ktora je aktivna len po preteceni premennej "t"
        pretecenie = false;
      }
    }
  }
}
