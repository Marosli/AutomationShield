<<<<<<< Updated upstream
/*
  API for the AeroShield hardware.
  
  The file is a part of the application programming interface for
  the AeroShield didactic tool for control engineering and 
  mechatronics education.
  
  This code is part of the AutomationShield hardware and software
  ecosystem. Visit http://www.automationshield.com for more
  details. This code is licensed under a Creative Commons
  Attribution-NonCommercial 4.0 International License.

  Created by .... 
  Last update: 05.02.2022.
*/


#include "AeroShield.h"         // Include header file

// Initializes adreeses needed for communication with as5600
AeroShield::AeroShield()
{
  _ams5600_Address = 0x36;
  _raw_ang_hi = 0x0c;
  _raw_ang_lo = 0x0d;
  _stat = 0x0b;
}



// Initializes hardware pins
float AeroShield::begin(bool isDetected){                             // Board initialisation
    pinMode(AERO_UPIN,OUTPUT);  		                                  // Actuator pin

  #ifdef ARDUINO_ARCH_AVR                                             // For AVR architecture boards
    Wire.begin();                                                     // Use Wire object
  #elif ARDUINO_ARCH_SAM                                              // For SAM architecture boards
    Wire1.begin();                                                    // Use Wire1 object
  #elif ARDUINO_ARCH_SAMD                                             // For SAMD architecture boards
    Wire.begin();                                                     // Use Wire object
  #endif

        if(isDetected == 0 ){                                           // If magnet not detected go on
    while(1){                                                           // Go forever until magnet detected 
        if(isDetected == 1 ){                                           // If magnet detected
            AutomationShield.serialPrint("Magnet detected \n");         // Print information then break
            break;
        }
        else{                                                           // If magnet not detected 
            AutomationShield.serialPrint("Can not detect magnet \n");   // Print information then go back to check while statement
      }
    }
  }                                                                                       
} 


float AeroShield::convertRawAngleToDegrees(word newAngle) {        // Function for converting raw angle(0-4096) to degrees(0-360°) 
  float retVal = newAngle * 0.087;                                      // 360°/4096=0.087° times the raw value
  ang = retVal;                               
  return ang;                                                           // Return angle value in degrees 
}


float AeroShield::calibration(word RawAngle) {                     // Calibration 
  AutomationShield.serialPrint("Calibration running...\n");             // Print info 
  startangle=0;                                                         // Zero out Variable(precaution)
  analogWrite(AERO_UPIN,50);                                            // Power the actuator, swing the pendulum 
  delay(250);                                                           // Wait for 0.25s 
  analogWrite(AERO_UPIN,0);                                             // Actuator powered off, pendulum goes to zero position
  delay(4000);                                                          // Wait for pendulum to stop oscillating 
  
  startangle = RawAngle;                                                // Save the value of zero pozition in raw format 
  analogWrite(AERO_UPIN,0);                                             // Actuator powered off(precaution)
    for(int i=0;i<3;i++){                                               // Simple sound indication of successful calibration 3 beeps
      analogWrite(AERO_UPIN,1);                                         // Actuator powereded just a bit so the rotor doesn't turn just beep 
      delay(200);                                                       // wait 
      analogWrite(AERO_UPIN,0);                                         // Actuator powered off
      delay(200);                                                       // wait 
      }

  AutomationShield.serialPrint("Calibration done");
    return startangle;
}


  float AeroShield::referenceRead(void) {                                             // Reference read
  referenceValue = (float)analogRead(AERO_RPIN);                                           // Reads the actual analog value of potentiometer runner
  referencePercent = AutomationShield.mapFloat(referenceValue, 0.0, 1024.0, 0.0, 100.0);   // Remapps the analog value from original range 0.0-1023 to percentual range 0.0-100.0
  return referencePercent;                                                                 // Returns the percentual position of potentiometer runner
}

void AeroShield::actuatorWrite(float PotPercent) {                                    // Actuator write
  float mappedValue = AutomationShield.mapFloat(PotPercent, 0.0, 100.0, 0.0, 255.0);       // Takes the float type percentual value 0.0-100.0 and remapps it to range 0.0-255.0
  mappedValue = AutomationShield.constrainFloat(mappedValue, 0.0, 255.0);                  // Constrains the remapped value to fit the range 0.0-255.0 - safety precaution
  analogWrite(AERO_UPIN, (int)mappedValue);                                                // Write remapped value to actuator pin 
}

float AeroShield::currentMeasure(void){                                               // Measuring current drawn by DC motor 
  for(int i=0 ; i<repeatTimes ; i++){                                                      // Function for callculating mean current value 
     voltageValue= analogRead(VOLTAGE_SENSOR_PIN);                                         // Read a value from the INA169 
     voltageValue= (voltageValue * voltageReference) / 1024;                               // Remap the ADC value into a voltage number (5V reference)
     current= current + correction1-(voltageValue / (10 * ShuntRes));                      // Equation given by the INA169 datasheet to
                                                                                           // determine the current flowing through ShuntRes. RL = 10k
     }                                                                                     // Is = (Vout x 1k) / (RS x RL)

   float currentMean= current/repeatTimes;                                                 // Callculating mean current value 
   currentMean= currentMean-correction2;                                                   // Small correction of current value(determined by ampermeter)
   if(currentMean < 0.000){                                                                // Correction for occasional bug causing the value to be negative. 
      currentMean= 0.000;                                                                  // When it so happens, zero out the value. 
      }

  current= 0;                                                                              // Zero out current value        
  voltageValue= 0;                                                                         // Zero out voltage value  
  return currentMean;                                                                      // Return mean current value 
}

word AeroShield::getRawAngle()                                                             // Function for getting raw pendulum angle data 0-4096
{
  return readTwoBytes(_raw_ang_hi, _raw_ang_lo);                                           // Another function for communication with senzor, called from this library 
}


int AeroShield::detectMagnet()                                                             // Function for detecting presence of magnet 
{
  int magStatus;                                                                           // Auxiliary variable
  int retVal = 0;                                                                          // Auxiliary variable
  magStatus = readOneByte(_stat);                                                          // Another function for communication with senzor, called from this library                         

  if (magStatus & 0x20)
    retVal = 1;

  return retVal;                                                                           // Return value 
}


int AeroShield::getMagnetStrength()                                                  // Function for getting the strength of magnet 
{
  int magStatus;                                                                     // Auxiliary variable
  int retVal = 0;                                                                    // Auxiliary variable
  magStatus = readOneByte(_stat);                                                    // Another function for communication with senzor, called from this library     

  if (detectMagnet() == 1)                                                           // Return 0 if no magnet is detected
  {
    retVal = 2;                                                                      // Return 2 if magnet is just right
    if (magStatus & 0x10)
      retVal = 1;                                                                    // Return 1 if magnet is too weak
    else if (magStatus & 0x08)
      retVal = 3;                                                                    // Return 3 if magnet is too strong
  }

  return retVal;                                                                     // Return value 
}


int AeroShield::readOneByte(int in_adr)             // Function for communicating with the senzor using 1 bite 
{
  int retVal = -1;
  Wire.beginTransmission(_ams5600_Address);         // Initialise wire transmission 
  Wire.write(in_adr);                               // Write 1bite 
  Wire.endTransmission();                           // End wire transmission 
  Wire.requestFrom(_ams5600_Address, 1);            // Request answer  
  while (Wire.available() == 0)                     // Wait for returning bite 
    ;
  retVal = Wire.read();                             // Store returning bite 

  return retVal;                                    // Return stored bite 
}


word AeroShield::readTwoBytes(int in_adr_hi, int in_adr_lo)          // Function for communicating with the senzor using 2 bites 
{
  word retVal = -1;

  /* Read Low Byte */
  Wire.beginTransmission(_ams5600_Address);        // Initialise wire transmission 
  Wire.write(in_adr_lo);                           // Write 1bite 
  Wire.endTransmission();                          // End wire transmission 
  Wire.requestFrom(_ams5600_Address, 1);           // Request answer  
  while (Wire.available() == 0)                    // Wait for returning bite 
    ;
  int low = Wire.read();                           // Store first returning bite 

  /* Read High Byte */
  Wire.beginTransmission(_ams5600_Address);        // Initialise wire transmission 
  Wire.write(in_adr_hi);                           // Write 1bite 
  Wire.endTransmission();                          // End wire transmission 
  Wire.requestFrom(_ams5600_Address, 1);           // Request answer  
  while (Wire.available() == 0)                    // Wait for returning bite 
    ;

  word high = Wire.read();                         // Store second returning bite 

  high = high << 8;                                // bitwise left shift
  retVal = high | low;

  return retVal;                                   // Return stored bites 
}
=======
/*
  API for the AeroShield hardware.
  
  The file is a part of the application programming interface for
  the AeroShield didactic tool for control engineering and 
  mechatronics education.
  
  This code is part of the AutomationShield hardware and software
  ecosystem. Visit http://www.automationshield.com for more
  details. This code is licensed under a Creative Commons
  Attribution-NonCommercial 4.0 International License.

  Created by .... 
  Last update: 02.03.2022.
*/


#include "AeroShield.h"         // Include header file

// Initializes adreeses needed for communication with as5600
AeroShield::AeroShield()
{
 
}



// Initializes hardware pins
float AeroShield::begin(bool isDetected){                             // Board initialisation
    pinMode(AERO_UPIN,OUTPUT);  		                                  // Actuator pin

  #ifdef ARDUINO_ARCH_AVR                                             // For AVR architecture boards
    Wire.begin();                                                     // Use Wire object
  #elif ARDUINO_ARCH_SAM                                              // For SAM architecture boards
    Wire1.begin();                                                    // Use Wire1 object
  #elif ARDUINO_ARCH_SAMD                                             // For SAMD architecture boards
    Wire.begin();                                                     // Use Wire object
  #endif
  
  
        if(isDetected == 0 ){                                           // If magnet not detected go on
    while(1){                                                           // Go forever until magnet detected 
        if(isDetected == 1 ){                                           // If magnet detected
            AutomationShield.serialPrint("Magnet detected \n");         // Print information then break
            break;
        }
        else{                                                           // If magnet not detected 
            AutomationShield.serialPrint("Can not detect magnet \n");   // Print information then go back to check while statement
      }
    }
  }       
} 


float AeroShield::convertRawAngleToDegrees(word newAngle) {             // Function for converting raw angle(0-4096) to degrees(0-360°) 
  float retVal = newAngle * 0.087;                                      // 360°/4096=0.087° times the raw value
  ang = retVal;                               
  return ang;                                                           // Return angle value in degrees 
}


float AeroShield::calibration(word RawAngle) {                          // Calibration 
  AutomationShield.serialPrint("Calibration running...\n");             // Print info 
  startangle=0;                                                         // Zero out Variable(precaution)
  analogWrite(AERO_UPIN,50);                                            // Power the actuator, swing the pendulum 
  delay(250);                                                           // Wait for 0.25s 
  analogWrite(AERO_UPIN,0);                                             // Actuator powered off, pendulum goes to zero position
  delay(4000);                                                          // Wait for pendulum to stop oscilating 
  
  startangle = RawAngle;                                                // Save the value of zero pozition in raw format 
  analogWrite(AERO_UPIN,0);                                             // Actuator powered off(precaution)
    for(int i=0;i<3;i++){                                               // Simple sound indication of successful calibration 3 beeps
      analogWrite(AERO_UPIN,1);                                         // Actuator powereded just a bit so the rotor doesn't turn just beep 
      delay(200);                                                       // wait 
      analogWrite(AERO_UPIN,0);                                         // Actuator powered off
      delay(200);                                                       // wait 
      }

  AutomationShield.serialPrint("Calibration done");
    return startangle;                                                  // Return start angle
}


  float AeroShield::referenceRead(void) {                                                  // Reference read
  referencePercent = AutomationShield.mapFloat(analogRead(AERO_RPIN), 0.0, 1024.0, 0.0, 100.0);   // Remapps the analog value from original range 0.0-1023 to percentual range 0.0-100.0
  return referencePercent;                                                                 // Returns the percentual position of potentiometer runner
}

void AeroShield::actuatorWrite(float PotPercent) {                                         // Actuator write
  float mappedValue = AutomationShield.mapFloat(PotPercent, 0.0, 100.0, 0.0, 255.0);       // Takes the float type percentual value 0.0-100.0 and remapps it to range 0.0-255.0
  mappedValue = AutomationShield.constrainFloat(mappedValue, 0.0, 255.0);                  // Constrains the remapped value to fit the range 0.0-255.0 - safety precaution
  analogWrite(AERO_UPIN, (int)mappedValue);                                                // Write remapped value to actuator pin 
}

float AeroShield::currentMeasure(void){                                                    // Measuring current drawn by DC motor 
  for(int i=0 ; i<repeatTimes ; i++){                                                      // Function for callculating mean current value 
     voltageValue= analogRead(VOLTAGE_SENSOR_PIN);                                         // Read a value from the INA169 
     voltageValue= (voltageValue * voltageReference) / 1024;                               // Remap the ADC value into a voltage number (5V reference)
     current= current + correction1-(voltageValue / (10 * ShuntRes));                      // Equation given by the INA169 datasheet to
                                                                                           // determine the current flowing through ShuntRes. RL = 10k
     }                                                                                     // Is = (Vout x 1k) / (RS x RL)

   float currentMean= current/repeatTimes;                                                 // Callculating mean current value 
   currentMean= currentMean-correction2;                                                   // Small correction of current value(determined by multimeter)
   if(currentMean < 0.000){                                                                // Correction for occasional bug causing the value to be negative. 
      currentMean= 0.000;                                                                  // When it so happens, zero out the value. 
      }

  current= 0;                                                                              // Zero out current value        
  voltageValue= 0;                                                                         // Zero out voltage value  
  return currentMean;                                                                      // Return mean current value 

}

word AeroShield::getRawAngle()                                                             // Function for getting raw pendulum angle data 0-4096
{
  return readTwoBytes(_raw_ang_hi, _raw_ang_lo);                                           // Another function for communication with senzor, called from this library 
}


int AeroShield::detectMagnet()                                                             // Function for detecting presence of magnet 
{
  int magStatus;                                                                           // Auxiliary variable
  int retVal = 0;                                                                          // Auxiliary variable
  magStatus = readOneByte(_stat);                                                          // Another function for communication with senzor, called from this library                         

  if (magStatus & 0x20)
    retVal = 1;

  return retVal;                                                                           // Return value 
}


int AeroShield::getMagnetStrength()                 // Function for getting the strength of magnet 
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


int AeroShield::readOneByte(int in_adr)             // Function for communicating with the senzor using 1 Byte 
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


word AeroShield::readTwoBytes(int in_adr_hi, int in_adr_lo)          // Function for communicating with the senzor using 2 Bytes 
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
>>>>>>> Stashed changes
