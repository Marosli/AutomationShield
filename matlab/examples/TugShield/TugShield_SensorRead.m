%   TugShield sensor read example
%
%   Flexi sensor feedback in the TugShield.
%
%   This example initialises and calibrates the board then set 
%   servo to 8 diferent position and watch the response of flexi
%   senzor.
%
%   This code is part of the AutomationShield hardware and software
%   ecosystem. Visit http://www.automationshield.com for more
%   details. This code is licensed under a Creative Commons
%   Attribution-NonCommercial 4.0 International License.
%
%   Created by Eva Vargov�.
%   Last update: 23.11.2020.

startScript;

TugShield = TugShield;          % Create TugShield object from TugShield class
TugShield.begin('COM5', 'UNO'); % Initialise shield with used Port and Board type
TugShield.calibrate();          % Calibrate TugShield
 

u = [0,30,90,60,120,50,180,0];  % Definition of servo arm trajectory
n = lenght(u); 

for i=1:n
    TugShield.actuatorWrite(u(i));
    y(i) = TugShield.sensorRead();
    pause(1);
end

t = 1:n;
plot(t,u)
hold on
plot(t,y)
title('V�stupn� sign�l flexi sn�ma�a pri predefinovanej trajekt�ri� servomotora v �ase');
legend('Vstupn� sign�l zo servo motora','V�stupn� sign�l z flexi sn�ma�a');
xlabel('�as t(s)');
ylabel('Hodnoty u a y');

