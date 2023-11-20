clear; clc;
load("Analytical_Model_5_SORI_softx4.mat")
load("Analytical_Model_5_SORI_hardx4.mat")
load("Analytical_Model_5_SORI_heartx4.mat")

%SORI Contact Area Simple
windowSize =10;

%F_array = movmean(soft.Force,windowSize);
%pressure = movmean(soft.PouchPressure,windowSize); 

%F_array = movmean(hard.Force,windowSize);
%pressure = movmean(hard.PouchPressure,windowSize);

%F_array = movmean(heart.Force,windowSize);
%pressure = movmean(heart.PouchPressure,windowSize); 


%F_array = movmean(soft.ForceSetpoint,windowSize);
%pressure = movmean(soft.SetpointPressure,windowSize); 

%F_array = movmean(hard.ForceSetpoint,windowSize);
%pressure = movmean(hard.SetpointPressure,windowSize);

F_array = movmean(heart.ForceSetpoint,windowSize);
pressure = movmean(heart.SetpointPressure,windowSize); 



% User1's finger!
R = 9; E_f0 = 0.013;   beta = 0.9;
% Rigid indenter
%R = 9; E_f0 = 400;   beta = 0;

E_finger_array  = E_f0 * (1 + beta.*F_array);
A_flat = pi*(0.75*R.*F_array.*(1./1.32 + 1./E_finger_array)).^(2/3);        % [mm^2] %spherical cap area! 
c1=200; b1= 0.13;
h_array = c1*(pressure/1000).*((1-b1.*F_array.^(1/3)).^2./(1+b1.*F_array.^(1/3)).^2);
A_sides = 2*pi*R*h_array;
A_contact = A_flat + A_sides;
plot(A_contact)
hold on
plot(hard.Force)
plot(hard.PouchPressure)
