%   Installs AutomationShield MATLAB API.
%  
%   This code is part of the AutomationShield hardware and software
%   ecosystem. Visit http://www.automationshield.com for more
%   details. This code is licensed under a Creative Commons
%   Attribution-NonCommercial 4.0 International License.
% 
%   Created by Gergely Tak�cs. 
%   Last update: 22.10.2018.

function installForMATLAB()
    thisdir=pwd;
    addpath(genpath(thisdir));
    savepath
    disp('AutomationShield MATLAB API added to MATLAB path.')
    if ~exist('tbxmanager','dir') % If the tbxmanager is not installed (checks directory)
        disp('AutomationShield MATLAB API added to MATLAB path.')
        mkdir('tbxmanager')
        cd tbxmanager
        urlwrite('http://www.tbxmanager.com/tbxmanager.m', 'tbxmanager.m');
        tbxmanager
        savepath
        tbxmanager install mpt mptdoc cddmex fourier glpkmex hysdel lcp yalmip sedumi espresso
        tbxmanager restorepath
        mpt_init
        cd ..
    end
end