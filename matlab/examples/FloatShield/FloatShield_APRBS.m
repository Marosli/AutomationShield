%   Generates an APRBS sequence for the FloatShield and saves it as a
%   C header for Arduino
%
%   This function generates a amplitude modulated pseudo-random binary
%   sequence to test the FloatShield
%
%   This code is part of the AutomationShield hardware and software
%   ecosystem. Visit http://www.automationshield.com for more
%   details. This code is licensed under a Creative Commons
%   Attribution-NonCommercial 4.0 International License.
%
%   Created by:      Gergely Tak�cs and Peter Chmurciak.
%   Last updated by: Peter Chmurciak.
%   Last update on:  29.8.2019.

N = 4800;                 % Length (samples)
seed = 100;               % Seed value for generating pseudorandom values
minu = -1;                % Minimum input
maxu = 1;                 % Maximum input
B = 1 / 50;               % Upper passband (Unit sample / slow-down)

aprbsGenerate('aprbsU', N, seed, minu, maxu, B) % Generate header "aprbsU.h"