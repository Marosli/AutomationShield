%   TugShield system identification experiment
%   Creates a pulse train to extract input-output data.
  
%   This example initializes the sampling subsystem from the 
%   AutomationShield library and starts an open-loop experiment
%   with the intention of extracting useful data for a system
%   identification procedure. Upload the code to your board, then
%   open the Serial Monitor or log the data to an external program.
  
%   This code is part of the AutomationShield hardware and software
%   ecosystem. Visit http://www.automationshield.com for more
%   details. This code is licensed under a Creative Commons
%   Attribution-NonCommercial 4.0 International License.
%
%   Created by Eva Vargov�.
%   Last update: 23.11.2020.

startScript;

TugShield = TugShield;          % Create TugShield object from TugShield class
TugShield.begin('COM3', 'UNO'); % Initialise shield with used Port and Board type
TugShield.calibrate();          % Calibrate TugShield

disp('Testovanie flexi sn�ma�a...poloha serva: 0 ')
TugShield.actuatorWrite(0);
pause(1);
y = TugShield.sensorRead();
if (y >= 0.0 && y <= 15.0)
    disp("Hodnoty flexi sn�ma�a s� v poriadku");
    disp(y);
else 
    disp("Hodnoty flexi s� nevyhovuj�ce");
    disp(y);
end
pause(0.5);

disp('Testovanie flexi sn�ma�a...poloha serva: 180 ')
TugShield.actuatorWrite(180);
pause(1);
y = TugShield.sensorRead();
if (y >= 85.0 && y <= 100.0) 
    disp("Hodnoty flexi sn�ma�a s� v poriadku");
    disp(y);
else 
    disp("Hodnoty flexi s� nevyhovuj�ce");
    disp(y);
end

disp('Testovanie ukon�en�')