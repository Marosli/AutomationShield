// Defining c++ library 
#ifndef ROTARYPENDULUM_H
#define ROTARYPENDULUM_H 

// #include <Arduino.h> ?? 

#include <Wire.h> //kniznica pre I2C komunikaciu
#include <AS5600.h> // kniznica pre magneticky snimac uhla
#include <MapFloat.h> // kniznica pre mapovanie premennej typu float

#define DIR_PIN          2
#define STEP_PIN         3
#define ENABLE_PIN       4

// zapinanie a vypinanie 
#define STEP_HIGH        PORTD |=  0b00001000;
#define STEP_LOW         PORTD &= ~0b00001000;

#define TIMER1_INTERRUPTS_ON    TIMSK1 |=  (1 << OCIE1A);
#define TIMER1_INTERRUPTS_OFF   TIMSK1 &= ~(1 << OCIE1A);

AMS_5600 ams5600;

class PendulumClass{                                     // Class for the RotaryPendulum device
 public:
    void begin(void);
    void calibration(void);            // Kalibracia
    float convertRawAngleToDegrees(word newAngle); // funkcia, ktora premeni pocet segmentov na stupne
    float mapAngle(); // funkcia, ktora premeni pocet segmentov na stupne
 private:
    // premenne uhlov, ktore by mali byt namerane pred kalibraciou
    float maxAngle = 0.0;
    float minAngle = 360.0;
    float bottomAngle = 0.0;
    
    // premenne uhlov, ktore sa pouzivaju pri kalibracii 
    float min_hod;
    float max_hod;
    float bottom_calibration;
};
extern PendulumClass RotaryPendulum; 

#endif
