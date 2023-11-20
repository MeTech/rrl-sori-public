%% Soft finger to Soft Material Contact Model - Hertzian Model
%clear; clc; close all;

% EF00-30, EF00-50, DS10, DS30, PLA
E_material_PS = [0.1, 0.17,0.45, 1.3, 400];

[m,numberOfMaterial] = size(E_material_PS);

for i=1:numberOfMaterial
    % User1
    %R_indenter = 9; E_f0 = 0.013;   beta = 0.9;  

    %User2
    R_indenter = 7.75;    E_f0 = 0.020;    beta = 0.7;


    P_indenter = [0.5,1,2,3,4];                                             % [N]
    
    % force dependent
    E_finger_PS = E_f0 * (1 + beta.*P_indenter);                            % [N/mm^2]
    
    E_total_PS = 1./(1./E_material_PS(i) + 1./E_finger_PS);                 % [MPa or N/mm^2]
    
    R_indenter_plus = R_indenter * (1 + E_material_PS(i)./E_finger_PS);
    
    h_indenter = (3*P_indenter./(4*sqrt(R_indenter).*E_total_PS)).^(2/3);
    h_indenter_plus = (3*P_indenter./(4*sqrt(R_indenter_plus).*E_material_PS(i))).^(2/3);
    
    % a from the herz model - same results
    %a_indenter = sqrt(R_indenter).*(h_indenter).^(1/2);
    %a_indenter2 = sqrt(R_indenter_plus).*(h_indenter_plus).^(1/2);

    % gives similar results
    %k=(E_finger_PS./(E_finger_PS + E_material_PS(i)));
    %ContactAreaindenter = 2*pi*R_indenter_plus.*(k.*h_indenter/2) % [mm^2] % spherical cap area!
    %ContactAreaindenter = 2*pi*R_indenter_plus.*(h_indenter_plus/2) % [mm^2] %spherical cap area! 
    
    ContactAreaindenter(i,:) = pi*(0.75*R_indenter.*P_indenter.*(1./E_material_PS(i) + 1./E_finger_PS)).^(2/3); % [mm^2] %spherical cap area!
end


% c=130, d=150;
% A_flat = c * P_indenter.^(1/3);
% A_flat1 = d * P_indenter.^(2/3);

% User1
% toughPLAContactArea = [99.21933333, 131.6793333, 151.7163333, 182.5186667, 184.5766667];
% ecoflex50ContactArea = [118.9573333, 159.967, 193.9183333, 224.4313333, 242.2033333];
% ecoflex30ContactArea = [125.8526667,150.8636667, 195.1493333, 235.127, 271.2866667];
% dragonSkin30ContactArea = [112.881,	136.889, 170.4963333, 185.4146667, 198.6936667];
% dragonSkin10ContactArea = [113.23,144.0263333,178.0976667,205.4863333,	221.3976667]

% User2
toughPLAContactArea = [73.138	99.40733333	125.1376667	136.348	142.5866667];
ecoflex50ContactArea = [95.58833333	116.4106667	154.6026667	175.677	182.6893333];
ecoflex30ContactArea = [98.55533333	128.1526667	158.019	180.7653333	201.6056667];
dragonSkin30ContactArea = [77.36033333	110.9993333	126.3366667	146.573	157.632];
dragonSkin10ContactArea =[82.81033333,	119.2536667,134.502,149.9813333,174.656];

figure
set(groot,'defaultLineLineWidth',3.0)
%plot(P_indenter,ecoflex30ContactArea)
hold on
plot(P_indenter,ecoflex30ContactArea,'r--',P_indenter,ecoflex50ContactArea,'g--',P_indenter,dragonSkin10ContactArea,'b--',P_indenter,dragonSkin30ContactArea,'c--')
plot(P_indenter,ContactAreaindenter(1,:),'r',P_indenter,ContactAreaindenter(2,:),'g',P_indenter,ContactAreaindenter(3,:),'b',P_indenter,ContactAreaindenter(4,:),'c')
hold off
xlim([0,4])
ylim([50,400])
xlabel("Force (N)")
ylabel("Area (mm^2)")
legend('EF 00-30 Exp', 'EF 00-50 Exp', 'DS 10 Exp', 'DS 30 Exp', ...
    'EF 00-30 Model', 'EF 00-50 Model', 'DS 10 Model', 'DS 30 Model')
legend('Location','northwest')
fontsize(gca,20,"pixels")
