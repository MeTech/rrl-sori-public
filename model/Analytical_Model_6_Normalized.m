clear; clc; close all;

R_User1 = 9;         E_f0_User1 = 0.013;      beta_User1 = 0.9;
R_User2 = 7.75;    E_f0_User2 = 0.020;    beta_User2 = 0.7;

P_indenter = [0.5,1,2,3,4];                                                 % [N]

% User1 User2
E_finger_User1 = E_f0_User1 * (1 + beta_User1.*P_indenter);                 % [N/mm^2]
E_finger_User2 = E_f0_User2 * (1 + beta_User2.*P_indenter);                 % [N/mm^2]

% EF00-30, EF00-50, DS10, DS30, PLA
E_material = [0.1, 0.17, 0.9, 1.3, 400]; 

%User1
ecoflex30ContactAreaUser1 = [125.8526667,150.8636667, 195.1493333, 235.127, 271.2866667]     .*((1/E_material(1) + 1)./(1/E_material(1) + 1./E_finger_User1)/R_User1).^(2/3);
ecoflex50ContactAreaUser1 = [118.9573333, 159.967, 193.9183333, 224.4313333, 242.2033333]    .*((1/E_material(2) + 1)./(1/E_material(2) + 1./E_finger_User1)/R_User1).^(2/3);
dragonSkin10ContactAreaUser1 = [113.23,144.0263333,178.0976667,205.4863333,	221.3976667]    .*((1/E_material(3) + 1)./(1/E_material(3) + 1./E_finger_User1)/R_User1).^(2/3);
dragonSkin30ContactAreaUser1 = [112.881,	136.889, 170.4963333, 185.4146667, 198.6936667]     .*((1/E_material(4) + 1)./(1/E_material(4) + 1./E_finger_User1)/R_User1).^(2/3);
toughPLAContactAreaUser1 = [99.21933333, 131.6793333, 151.7163333, 182.5186667, 184.5766667] .*((1/E_material(5) + 1)./(1/E_material(5) + 1./E_finger_User1)/R_User1).^(2/3);

%User2
ecoflex30ContactAreaUser2 = [98.55533333	128.1526667	158.019	180.7653333	201.6056667]    .*((1/E_material(1) + 1)./(1/E_material(1) + 1./E_finger_User2)/R_User2).^(2/3);
ecoflex50ContactAreaUser2 = [95.58833333	116.4106667	154.6026667	175.677	182.6893333]    .*((1/E_material(2) + 1)./(1/E_material(2) + 1./E_finger_User2)/R_User2).^(2/3);
dragonSkin10ContactAreaUser2 =[82.81033333,	119.2536667,134.502,149.9813333,174.656]    .*((1/E_material(3) + 1)./(1/E_material(3) + 1./E_finger_User2)/R_User2).^(2/3);
dragonSkin30ContactAreaUser2 = [77.36033333	110.9993333	126.3366667	146.573	157.632]    .*((1/E_material(4) + 1)./(1/E_material(4) + 1./E_finger_User2)/R_User2).^(2/3);
toughPLAContactAreaUser2 = [73.138	99.40733333	125.1376667	136.348	142.5866667]            .*((1/E_material(5) + 1)./(1/E_material(5) + 1./E_finger_User2)/R_User2).^(2/3);

ContactArea1= pi*(0.75*P_indenter*(1+1/E_material(1))).^(2/3);
ContactArea2= pi*(0.75*P_indenter*(1+1/E_material(2))).^(2/3);
ContactArea3= pi*(0.75*P_indenter*(1+1/E_material(3))).^(2/3);
ContactArea4= pi*(0.75*P_indenter*(1+1/E_material(4))).^(2/3);
ContactArea5= pi*(0.75*P_indenter*(1+1/E_material(5))).^(2/3);

figure
set(groot,'defaultLineLineWidth',2.0)
plot(P_indenter,toughPLAContactAreaUser1,'r:',P_indenter,toughPLAContactAreaUser2, 'g:')
hold on
plot(P_indenter,ContactArea2,'k', P_indenter,ContactArea4,'k', P_indenter,ContactArea5,'k')
plot(P_indenter,ecoflex50ContactAreaUser1,'r:',P_indenter,ecoflex50ContactAreaUser2, 'g:')
plot(P_indenter,dragonSkin30ContactAreaUser1,'r:',P_indenter,dragonSkin30ContactAreaUser2, 'g:')
hold off
xlim([0,4])
xlabel("Force (N)")
ylabel("Normalized Contact Area (mm^2)")
legend('Finger 1', 'Finger 2', 'Model')
legend('Location','northwest')
fontsize(gca,20,"pixels")