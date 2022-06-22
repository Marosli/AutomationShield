// Defining c++ library used by the AeroShield 
#ifndef ROTARYPENUDLUM_H
#define ROTARYPENUDLUM_H

#include "Arduino.h"

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

static void (*__timer1Hook)(void);

class PendulumClass{                                     // Class for the AeroShield device
 public:
    void begin(void);
    void calibration(void);            // Kalibracia
    float convertRawAngleToDegrees(word newAngle); // funkcia, ktora premeni pocet segmentov na stupne
    float getRealAngle(); // funkcia, ktora premeni pocet segmentov na stupne
    int detectMagnet();                         // AS5600 detect magnet 
    int getMagnetStrength();                     // AS5600 magnet strength
    word getRawAngle();
    void linear_speed(float velocity);  // funkcia, ktora ovlada rychlost motora

    void timer1Hook(void (*function)(void));
    double computePID(double inp);
    void krok();
        
        // premenne, ktore sa pouzivaju pri praci s motorm
    unsigned int c0;
    // premenne pre rychlost a rozlisenie
    float velocity, abs_velocity;
    float rozlisenie = 0.1125;  // LOW | LOW | HIGH na driveri

    volatile int dir = 0;
    volatile unsigned int beforeDelay = 3500;
    volatile unsigned int maxDelay = 3500;
    volatile unsigned long n = 0;
    volatile float d;
    volatile unsigned long rampUpStepCount = 0;
    
    double SetPoint = 0.0;   // ziadana hodnota
    //PID konstanty
    double kp = 2.0;
    double ki = 1.0;
    double kd = 0.1;

    // premenne uhlov, ktore sa pouzivaju pri kalibracii 
    float min_hod;
    float max_hod;
    float bott_hod;
    
    
 private:
    // premenne uhlov, ktore by mali byt namerane pred kalibraciou
    float maxAngle = 0.0;
    float minAngle = 360.0;
    float bottomAngle = 0.0;
    
    float bott_hod_sum = 0.0;
    
    // premenne, ktore sa pouzivaju pri I2C komunikacii s encoderom
    int _ams5600_Address = 0x36;                  // AS5600 address
    int _stat = 0x0b;                      // AS5600 communication variable 
    int _raw_ang_hi = 0x0c;                  // AS5600 communication variable 
    int _raw_ang_lo = 0x0d;                  // AS5600 communication variable 
    int readOneByte(int in_adr);                // AS5600 one byte communication
    word readTwoBytes(int in_adr_hi, int in_adr_lo);  // AS5600 two bytes communication

    // premenne potrebne pre vypocet akcneho zasahu PID regulatora
    unsigned long currentTime, previousTime;
    double elapsedTime;
    double error;
    double lastError;
    double input, output;
    double cumError, rateError;
};
extern PendulumClass RotaryPendulum; 

#endif
