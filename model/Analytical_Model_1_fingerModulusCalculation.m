%% finger modulus calibration. Touch on the hard surface. Calculate the finger radius.
clear; clc; close all;
E_material_PS = 400; %PLA

%  --- finger values ----
%[mm, N/mm^2, -]
R_User1 = 9;         E_f0_User1 = 0.013;      beta_User1 = 0.9;
R_User2 = 7.75;    E_f0_User2 = 0.02;    beta_User2 = 0.7; 

R_indenter = R_User1;                                                       % [mm]
E_f0 = E_f0_User1; beta = beta_User1;

% --- finger values  ----
P_indenter = [0.5,1,2,3,4];         % [N]
% force dependent
E_finger_PS = E_f0 * (1 + beta.*P_indenter);                                % [N/mm^2]

E_finger_PS_User1 = E_f0_User1 * (1 + beta_User1.*P_indenter);              % [N/mm^2]
E_finger_PS_User2 = E_f0_User2 * (1 + beta_User2.*P_indenter);              % [N/mm^2]

E_total_PS = 1./(1./E_material_PS + 1./E_finger_PS);                        % [MPa or N/mm^2]

E_total_PS_User1 = 1./(1./E_material_PS + 1./E_finger_PS_User1);            % [MPa or N/mm^2]
E_total_PS_User2 = 1./(1./E_material_PS + 1./E_finger_PS_User2);            % [MPa or N/mm^2]
    
R_indenter_plus = R_indenter * (1 + E_material_PS./E_finger_PS);

R_indenter_plus_User1 = R_indenter * (1 + E_material_PS./E_finger_PS_User1);
R_indenter_plus_User2 = R_indenter * (1 + E_material_PS./E_finger_PS_User2);


%h_indenter = (3*P_indenter./(4*sqrt(R_indenter).*E_total_PS)).^(2/3);
h_indenter_plus = (3*P_indenter./(4*sqrt(R_indenter_plus).*E_material_PS)).^(2/3);

h_indenter_plus_User1    = (3*P_indenter./(4*sqrt(R_indenter_plus_User1).*E_material_PS)).^(2/3);
h_indenter_plus_User2  = (3*P_indenter./(4*sqrt(R_indenter_plus_User2).* E_material_PS)).^(2/3);

% a from the herz model. assumption  h<< R - same results
% a_indenter = sqrt(R_indenter).*(h_indenter).^(1/2);
% a_indenter1 = sqrt(R_indenter_plus).*(h_indenter_plus).^(1/2);

% gives same results
%k = (E_finger_PS./(E_finger_PS + E_material_PS));
%ContactAreaindenter = 2*pi*R_indenter_plus.*(k.*h_indenter/2) % [mm^2]     % spherical cap area!
%ContactAreaindenter = 2*pi*R_indenter_plus.*(h_indenter_plus/2) % [mm^2]   %spherical cap area! 
ContactAreaindenter = pi*(0.75*R_indenter.*P_indenter.*(1./E_material_PS + 1./E_finger_PS)).^(2/3);     % [mm^2] %spherical cap area! 

ContactAreaUser1     = pi*(0.75*R_User1.*P_indenter.*(1./E_material_PS + 1./E_finger_PS_User1)).^(2/3); % [mm^2] %spherical cap area! 
ContactAreaUser2   = pi*(0.75*R_User2.*P_indenter.*(1./E_material_PS + 1./E_finger_PS_User2)).^(2/3);   % [mm^2] %spherical cap area! 

% c= 130 and 100 for User1 and User2
% A_flat = c * P_indenter.^(1/3);                                           % [mm^2]

toughPLAContactAreaUser1 = [99.21933333, 131.6793333, 151.7163333, 182.5186667, 184.5766667];
toughPLAContactAreaUser2 = [73.138, 99.40733333, 125.1376667, 136.348, 142.5866667];

figure
set(groot,'defaultLineLineWidth',3.0)
plot(P_indenter,toughPLAContactAreaUser1,'r--',P_indenter,toughPLAContactAreaUser2, 'g--')
hold on
plot(P_indenter,ContactAreaUser1,'r', P_indenter,ContactAreaUser2,'g')
%plot(P_indenter,A_flat)
hold off
xlim([0,4])
ylim([50,300])
xlabel("Force (N)")
ylabel("Area (mm^2)")
legend('Finger 1: Experiment', 'Finger 2: Experiment', ...
    'Finger 1: Model 1', 'Finger 1: Model 2')
legend('Location','northwest')
fontsize(gca,20,"pixels")

toughPLAContactAreaUser1 = [99.21933333, 131.6793333, 151.7163333, 182.5186667, 184.5766667];
toughPLAContactAreaUser2 = [73.138, 99.40733333, 125.1376667, 136.348, 142.5866667];

toughPLAContactAreaUser1Normalized = toughPLAContactAreaUser1.*E_finger_PS_User1.^(2/3)/R_User1^(2/3);
toughPLAContactAreaUser2Normalized = toughPLAContactAreaUser2.*E_finger_PS_User2.^(2/3)/R_User2^(2/3);

ContactArea = pi*(0.75*P_indenter).^(2/3);                                  % [mm^2] %spherical cap area! 

figure
set(groot,'defaultLineLineWidth',3.0)
plot(P_indenter,toughPLAContactAreaUser1Normalized,'r--',P_indenter,toughPLAContactAreaUser2Normalized, 'g--')
hold on
plot(P_indenter,ContactArea,'k')
%plot(P_indenter,A_flat)
hold off
xlim([0,4])
%ylim([50,300])
xlabel("Force (N)")
ylabel("Area (mm^2)")
legend('Finger 1', 'Finger 2', 'Model')
legend('Location','northwest')
fontsize(gca,20,"pixels")
