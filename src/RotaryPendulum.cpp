#include "RotaryPendulum.h"         // Include header file

#include "Arduino.h"

void PendulumClass::begin(void) {                                      // Board initialisation
  Wire.begin();
    // Serial.println(">>>>>>>>>>>>>>>>>>>>>>>>>>> ");
  if(RotaryPendulum.detectMagnet() == 0 ){           // overenie, ci je pripojeny magneticky encoder
    while(1){
        if(RotaryPendulum.detectMagnet() == 1 ){
           // Serial.print("Current Magnitude: ");
            //Serial.println(ams5600.getMagnitude());
            break;
        }
        else{
            // Serial.println("Can not detect magnet");
        }
        delay(1000);
    }
  }
  // inicializacia motora
  pinMode(STEP_PIN,   OUTPUT);
  pinMode(DIR_PIN,    OUTPUT);
  pinMode(ENABLE_PIN, OUTPUT);
  noInterrupts();
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1  = 0;
  OCR1A = 1000;                             
  TCCR1B |= (1 << WGM12);
  TCCR1B |= ((1 << CS11) | (1 << CS10));
  interrupts();
  c0 = 3500; // was 2000 * sqrt( 2 * angle / accel )
}

void PendulumClass::calibration(void) {         // Board initialisation
  Serial.println(">>>>>>>>>>>>>>>>>>>>>>>>>");
  Serial.println("Calibration running");
  int e = 0;
  delay(1500);
  while (e < 1500){
  float a_min = RotaryPendulum.convertRawAngleToDegrees(RotaryPendulum.getRawAngle());
  if (a_min < minAngle){
    minAngle = a_min;
    }
  e++;
  }
  min_hod = minAngle;
  Serial.print("Minimalna hodnota kalibracie: ");
  Serial.println(RotaryPendulum.min_hod);
  e = 0;
  delay(1500);
  while (e < 1500){
  float a_max = RotaryPendulum.convertRawAngleToDegrees(RotaryPendulum.getRawAngle());
  if (a_max > maxAngle){
    maxAngle = a_max;
    }
  e++;
  }
  max_hod = maxAngle;
  Serial.print("Maximalna hodnota kalibracie: ");
  Serial.println(RotaryPendulum.max_hod);
  e = 0;
  delay(5000);
  while (e < 1500){
    float angle = RotaryPendulum.convertRawAngleToDegrees(RotaryPendulum.getRawAngle());
    float mapAngle = mapFloat(angle, min_hod, max_hod, 0.00, 360.00);
    bott_hod_sum += mapAngle;        // treba urobit ako sumu
    e++;
    }
  bott_hod = bott_hod_sum/1500.0;
  Serial.print("Spodna hodnota kalibracie: ");
  Serial.println(RotaryPendulum.bott_hod);
}

float PendulumClass::convertRawAngleToDegrees(word newAngle) // funkcia, ktora premeni pocet segmentov na stupne
{
  /* Raw data reports 0 - 4095 segments, which is 0.087 of a degree */
  float retVal = newAngle * 0.087;
  return retVal;
}

float PendulumClass::getRealAngle() // funkcia, ktora vrati uhol po uspesnej kalibracii
{
  float angle = RotaryPendulum.convertRawAngleToDegrees(RotaryPendulum.getRawAngle());
  float mapAngle = mapFloat(angle, min_hod, max_hod, 0.00, 360.00);
  mapAngle = mapAngle - bott_hod;
  if(mapAngle<0.0){
    mapAngle = 360.0 + mapAngle;
    }
  return mapAngle;
}

word PendulumClass::getRawAngle()                                                             // Function for getting raw pendulum angle data 0-4096
{
  return readTwoBytes(_raw_ang_hi, _raw_ang_lo);                                           // Another function for communication with senzor, called from this library 
}

int PendulumClass::detectMagnet()                                                             // Function for detecting presence of magnet 
{
  int magStatus;                                                                           // Auxiliary variable
  int retVal = 0;                                                                          // Auxiliary variable
  magStatus = readOneByte(_stat);                                                          // Another function for communication with senzor, called from this library                         

  if (magStatus & 0x20)
    retVal = 1;
  return retVal;                                                                           // Return value 
}

int PendulumClass::getMagnetStrength()                 // Function for getting the strength of magnet 
{
  int magStatus;                                    // Auxiliary variable
  int retVal = 0;                                   // Auxiliary variable
  magStatus = readOneByte(_stat);                   // Another function for communication with senzor, called from this library     

  if (detectMagnet() == 1)                          // Return 0 if no magnet is detected
  {
    retVal = 2;                                     // Return 2 if magnet is just right
    if (magStatus & 0x10)
      retVal = 1;                                   // Return 1 if magnet is too weak
    else if (magStatus & 0x08)
      retVal = 3;                                   // Return 3 if magnet is too strong
  }

  return retVal;                                    // Return value 
}

int PendulumClass::readOneByte(int in_adr)             // Function for communicating with the senzor using 1 Byte 
{
  int retVal = -1;
  Wire.beginTransmission(_ams5600_Address);         // Initialise wire transmission 
  Wire.write(in_adr);                               // Write 4 bits 
  Wire.endTransmission();                           // End wire transmission 
  Wire.requestFrom(_ams5600_Address, 1);            // Request answer  
  while (Wire.available() == 0);                    // Wait for returning bits 

  retVal = Wire.read();                             // Store returning bits 

  return retVal;                                    // Return stored bits 
}

word PendulumClass::readTwoBytes(int in_adr_hi, int in_adr_lo)          // Function for communicating with the senzor using 2 Bytes 
{
  word retVal = -1;

  /* Read Low Byte */
  Wire.beginTransmission(_ams5600_Address);        // Initialise wire transmission 
  Wire.write(in_adr_lo);                           // Write 4 bits 
  Wire.endTransmission();                          // End wire transmission 
  Wire.requestFrom(_ams5600_Address, 1);           // Request answer  
  while (Wire.available() == 0);                    // Wait for returning bits 

  int low = Wire.read();                           // Store first returning bits 

  /* Read High Byte */
  Wire.beginTransmission(_ams5600_Address);        // Initialise wire transmission 
  Wire.write(in_adr_hi);                           // Write 4 bits   
  Wire.endTransmission();                          // End wire transmission 
  Wire.requestFrom(_ams5600_Address, 1);           // Request answer  
  while (Wire.available() == 0);                    // Wait for returning bits 

  word high = Wire.read();                         // Store second returning bits 

  high = high << 8;                                // bitwise left shift
  retVal = high | low;

  return retVal;                                   // Return stored bits 
}

// funkcia, do ktorej zadame ziadanu hodnotu rychlosti
void PendulumClass::linear_speed(float velocity) {
  digitalWrite(DIR_PIN, velocity < 0 ? LOW : HIGH);
  abs_velocity = abs(velocity);
  beforeDelay = maxDelay;
  if(abs_velocity == 0.0){
    TIMER1_INTERRUPTS_OFF;
    }
  else{
    maxDelay = 1000000.0*rozlisenie/(abs_velocity*4.0);
  }
  if(beforeDelay < 3500 && beforeDelay > maxDelay){
    d = beforeDelay;
    }
  else{  
    d = c0;
  }
  OCR1A = d;
  n = 0;
  rampUpStepCount = (beforeDelay > maxDelay) ? 0 : 2;
  if(abs_velocity > 0.0){
    TIMER1_INTERRUPTS_ON 
    }
  
}

void PendulumClass::timer1Hook(void (*function)(void))
{
  __timer1Hook = function;
}

// funkcia, v ktrej sa napocitava skracovanie a predlzovanie delayu
ISR(TIMER1_COMPA_vect)
{
  __timer1Hook();
}

double PendulumClass::computePID(double inp){     
        currentTime = millis();                //get current time
        elapsedTime = (double)(currentTime - previousTime)/1000.0;        //compute time elapsed from previous computation
        
        error = SetPoint - inp;                                // determine error
        cumError += error * elapsedTime;                // compute integral
        rateError = (error - lastError)/elapsedTime;   // compute derivative

        double out = kp*error + ki*cumError + kd*rateError;                //PID output               

        lastError = error;                                //remember current error
        previousTime = currentTime;                        //remember current time

        if(out > 250.0){
          out = 250.0;
          }
        else if(out < -250.0){
          out = -250.0;
          }
        return out;                                        //have function return the PID output
}

void PendulumClass::krok(){
  input = RotaryPendulum.getRealAngle();    // nacitanie vstupu zo snimaca
  if(SetPoint == 0.0){  // rozdelenie pola vstupu na cas od -180-0 stupnov a 0-180 stupnov
    if(input > 180.0 && input < 360.0){
      input = input - 360.0;
      }
  }
  output = RotaryPendulum.computePID(input);   // vypocet vystupu z regulatora
  linear_speed((float)output);    // aplikovanie akcneho zasahu na ovladanie motora
//  Serial.print(input);
//  Serial.print(",");
//  Serial.print(SetPoint-input);
//  Serial.print(",");
//  Serial.print(SetPoint);
//  Serial.print(",");
//  Serial.println(output);
}

PendulumClass RotaryPendulum;
