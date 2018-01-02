%Dynamic Analysis of Current and Velocity Loops
%Using the control systems toolbox
close all;
 
% Nomenclature
% pi-cr = PI Current Regulator
% pc = Power Controller
% cr = Current Regulator
% cs = Current Sensor
 
%% Motor Properties
Um_max = 30;    % Max applied voltage           Units: volts
Wmax = 300;     % Max velocity                  Units: rad/sec
Ra=15;          % Electrical resistance         Units: ohms
La= 0.15;       % Electrical inductance         Units: henry
J=0.12E-5;      % Inertia                       Units: Kg*m^2
 
Ta = La/Ra;     % Inductance Resistance Ratio
Ke=Um_max/Wmax; % Electromotive force constant  Units: volts / (rad/sec)
Km=Ke;          % Torque constant               Units: Nm / (Work)^2
 
%% Feedback Control Loop Properties
% Power Controller
Kpc=3;
Tpc=0.001;
 
% Current Sensor
Kcs=3.33;
 
% Current Regulator
Kcr=25;
Tcr=Ta;
 
%% Current Loop
% Create transfer functions for changing Kcr
i = 0;
for Kcr=[4.0, 7.5, 10]
    i = i + 1;
    %Store Transfer Functions
    Plant_CL= tf(Kpc, [Tpc 1]) * tf(1/Ra, [Ta 1]) *Kcs;
    PI_CR = Kcr*tf([Tcr, 1], [Tcr, 0]);
    CL_Open(i) = PI_CR*Plant_CL;
    CL_Closed(i) = feedback(CL_Open(i), 1);
    
    %Store legend entries
    theLegend(i) = {['Kcr=' num2str(Kcr)]};
end
L = i;
 
%Plot: Open Loop
f1 = figure(1); hold;
movegui(f1,'northeast');
P = bodeoptions;
P.FreqUnits = 'Hz';
for i = 1:L
     bode(CL_Open(i), {10,100000}, P); grid on
end
title('Current Loop Open - Bode Diagram');
legend(theLegend);
 
%Plot: Closed Loop
f2 = figure(2); hold;
movegui(f2,'southeast');
for i = 1:L
     step(CL_Closed(i), 0:0.0001:0.015); grid on
end
title('Current Loop Closed - Step Diagram');
legend(theLegend);
set(legend, 'location', 'southeast');
 
%% Velocity Loop
Tvr=8*Tpc;
Kvs=10/600;
Kvr=1;
 
%Create transfer functions for changing Kcr
i = 0;
for Kvr=[0.6, 1.3, 2.0]
    i = i + 1;
    %Store Transfer Functions
    Plant_VL= tf(1/Kcs, [2*Tpc 1]) * Km * tf(1, [J 0]) * Kvs;
    PI_VR = Kvr*tf([Tvr, 1], [Tvr, 0]);
    VL_Open(i) = PI_VR*Plant_VL;
    VL_Closed(i) = feedback(VL_Open(i), 1);
    
    %Store legend entries
    theLegend(i) = {['Kvr=' num2str(Kvr)]};
    
end
L = i;
 
%Plot: Open Loop
f1 = figure(3); hold;
movegui(f1,'northeast');
P = bodeoptions;
P.FreqUnits = 'Hz';
for i = 1:L
     bode(VL_Open(i), {10,10000}, P); grid on
end
title('Velocity Loop Open - Bode Diagram');
legend(theLegend);
 
%Plot: Closed Loop
f2 = figure(4); hold;
movegui(f2,'southeast');
for i = 1:L
     step(VL_Closed(i), 0:0.0001:0.04); grid on
end
title('Velocity Loop Closed - Step Diagram');
legend(theLegend);
set(legend, 'location', 'southeast');
