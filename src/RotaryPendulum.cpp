#include "RotaryPendulum.h"         // Include header file

void PendulumClass::begin(void) {                                      // Board initialisation
  Wire.begin();
    // Serial.println(">>>>>>>>>>>>>>>>>>>>>>>>>>> ");
  if(ams5600.detectMagnet() == 0 ){           // overenie, ci je pripojeny magneticky encoder
    while(1){
        if(ams5600.detectMagnet() == 1 ){
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
}

void PendulumClass::calibration(void) {                                      // Board initialisation
  int e = 0;
  delay(1000);
  while (e < 1500){
  float a_min = RotaryPendulum.convertRawAngleToDegrees(ams5600.getRawAngle());
  if (a_min < minAngle){
    minAngle = a_min;
    }
  e++;
  }
  min_hod = minAngle;
  e = 0;
  delay(1000);
  while (e < 1500){
  float a_max = RotaryPendulum.convertRawAngleToDegrees(ams5600.getRawAngle());
  if (a_max > maxAngle){
    maxAngle = a_max;
    }
  e++;
  }
  max_hod = maxAngle;
  e = 0;
  delay(1000);
  while (e < 1000){
    float angle = RotaryPendulum.convertRawAngleToDegrees(ams5600.getRawAngle());
    float mapAngle = mapFloat(angle, min_hod, max_hod, 0.00, 360.00);
    float bottom_calibration_sum = mapAngle;        // treba urobit ako sumu
    bottom_calibration = bottom_calibration_sum;
    e++;
    }
  
}

float PendulumClass::convertRawAngleToDegrees(word newAngle) // funkcia, ktora premeni pocet segmentov na stupne
{
  /* Raw data reports 0 - 4095 segments, which is 0.087 of a degree */
  float retVal = newAngle * 0.087;
  return retVal;
}

float PendulumClass::mapAngle() // funkcia, ktora premeni pocet segmentov na stupne
{
  float angle = RotaryPendulum.convertRawAngleToDegrees(ams5600.getRawAngle());
  float mapAngle = mapFloat(angle, min_hod, max_hod, 0.00, 360.00);
  mapAngle = mapAngle - bottom_calibration;
  if(mapAngle<0.0){
    mapAngle = 360.0 + mapAngle;
    }
  return mapAngle;
}
