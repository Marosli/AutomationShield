/*
  OptoShield manual PID response example

  PID feedback control of brightness on the OptoShield.
  
  This example initializes the sampling and PID control 
  subsystems from the AutomationShield library and allows the
  user to manually create a reference trajectory for LED 
  brightness. Upload the code to your board, then open the Serial
  Plotter function in your Arduino IDE. Adjust the reference by
  turning the potentiometer knob.
  
  This code is part of the AutomationShield hardware and software
  ecosystem. Visit http://www.automationshield.com for more
  details. This code is licensed under a Creative Commons
  Attribution-NonCommercial 4.0 International License.

  Created by Gergely Takács and Tibor Konkoly. 
  Last update: 28.09.2018.
*/

#include <OptoShield.h> 		// Include the library

unsigned long Ts = 10;               // Sampling in milliseconds
bool enable=false;                  // Flag for sampling 

float r = 0.0;						// Reference
float y = 0.0;						// Output
float u = 0.0;						// Input					

#define KP 0.1						// PID Kp
#define TI 0.015                    // PID Ti
#define TD 0.000001                // PID Td

void setup() {
  Serial.begin(9600);               // Initialize serial
  
  // Initialize and calibrate board
  OptoShield.begin();               // Define hardware pins
  OptoShield.calibration();         // Calibration percentages
  
  // Initialize sampling function
  Sampling.period(Ts * 1000);   // Sampling init.
  Sampling.interrupt(stepEnable); // Interrupt fcn.

 // Set the PID constants
 PIDAbs.setKp(KP);
 PIDAbs.setTi(TI);
 PIDAbs.setTd(TD); 
}

// Main loop launches a single step at each enable time
void loop() {
  if (enable) { 							// If ISR enables
    step();									// Algorithm step
    enable=false;  							// Then disable
  }  
}

void stepEnable(){  						// ISR 
  enable=true;							    // Change flag
}

// A signle algoritm step

void step(){ 
	r = OptoShield.referenceRead();  	   // Read reference
	y = OptoShield.sensorRead();           // Read sensor 
	u = PIDAbs.compute(r-y,0,100,0,100);   // PID
	OptoShield.actuatorWrite(u);           // Actuate

Serial.print(r);						// Print reference
Serial.print(", ");						 
Serial.print(y);						// Print output  
Serial.print(", ");
Serial.println(u);						// Print input
}
