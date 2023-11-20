%clear; clc; close all;

% EF00-30, EF00-50, DS10, DS30
E_material = [0.1, 0.17, 0.9, 1.3]; 
[m,numberOfMaterial] = size(E_material);

% User1
% R = 9; E_f0 = 0.013;   beta = 0.9;
%User2
R = 7.75; E_f0 = 0.020;   beta = 0.7;

F_array = [0.5,1,2,3,4];                                                        % [N]

for i=1:numberOfMaterial
    E_finger_array  = E_f0 * (1 + beta.*F_array);                               % [N/mm^2]
    E_total_array   = 1./(1./E_material(i) + 1./E_finger_array);                % [MPa or N/mm^2]
    
    R_plus_array = R * (1 + E_material(i)./E_finger_array);                     % [mm]
    h_plus_array = (3*F_array./(4*sqrt(R_plus_array).*E_material(i))).^(2/3);   % [mm]
    
    % gives similar results
    ContactAreaIndenter_array = pi*(0.75*R.*F_array.*(1./E_material(i) + 1./E_finger_array)).^(2/3); % [mm^2] %spherical cap area! 
    ContactAreaIndenterFlat_array = pi*(0.75*R.*F_array.*(1./1.32 + 1./E_finger_array)).^(2/3); % [mm^2] %spherical cap area! 
    
    a_array = sqrt(R_plus_array).*(h_plus_array).^(1/2);
    eps=0.4;
    a_array(a_array>(R-eps))=(R-eps);
    
    [a_x a_array_size] = size(a_array);
    
    %% Donut contact
    ro      = 11;           % mm % total rendering radius of softness display
    b_array       = (ro-a_array)/2;     % ellipse radius
    d_array       = sqrt(R^2-a_array.^2); % mm % center of finger to the surface
    
    % calcalate desired Side Contact Area
    DesiredContactSideArea = (ContactAreaIndenter_array - ContactAreaIndenterFlat_array);
    if(DesiredContactSideArea<0)
        DesiredContactSideArea=0.*ContactAreaIndenter_array;
    end
    
    % calculate desired torus h
    desired_h_c = DesiredContactSideArea/(2*pi*R);
    M = desired_h_c - sqrt(R^2 - a_array.^2);
    x0 = sqrt(R^2 - M.^2) - a_array - b_array; % -b=<x0<=0
    
    if ((x0./b_array).^2 < 0.99) % add some margin
        desired_h = desired_h_c./(sqrt(1 - (x0./b_array).^2));
    else 
        desired_h = desired_h_c;
    end
  
    
    %% find the control input P    
    % used model
    c1=200; b1= 0.13;
    K = c1.*((1-b1.*F_array.^(1/3)).^2./(1+b1.*F_array.^(1/3)).^2); % the last model
    if(K>0)
        P_desired = desired_h./K;
        P_desired(P_desired>0.024)=0.024;
        P(i,:) = P_desired;
    end
    
    h_array = desired_h;
    A_sides = 2*pi*R*h_array;
    
%% sum both contact areas
A_flat = ContactAreaIndenterFlat_array
A_contact(i,:) = A_flat + A_sides;

end
User1SDContactArea(:,1) = User1SD(P(:,1),1);
User1SDContactArea(:,2) = User1SD(P(:,2),2);
User1SDContactArea(:,3) = User1SD(P(:,3),3);
User1SDContactArea(:,4) = User1SD(P(:,4),4);
User1SDContactArea(:,5) = User1SD(P(:,5),5);

User2SDContactArea(:,1) = User2SD(P(:,1),1);
User2SDContactArea(:,2) = User2SD(P(:,2),2);
User2SDContactArea(:,3) = User2SD(P(:,3),3);
User2SDContactArea(:,4) = User2SD(P(:,4),4);
User2SDContactArea(:,5) = User2SD(P(:,5),5);


% User1
% toughPLAContactArea = [99.21933333, 131.6793333, 151.7163333, 182.5186667, 184.5766667];
% ecoflex50ContactArea = [118.9573333, 159.967, 193.9183333, 224.4313333, 242.2033333];
% ecoflex30ContactArea = [125.8526667,150.8636667, 195.1493333, 235.127, 271.2866667];
% dragonSkin30ContactArea = [112.881,	136.889, 170.4963333, 185.4146667, 198.6936667];
% dragonSkin10ContactArea = [113.23,144.0263333,178.0976667,205.4863333,	221.3976667]
% stdToughPLAContactArea =[4.064379698 4.334472209	6.91770022	4.874643098	3.645515647];
% stdEcoflex50ContactArea = [5.35034787	6.539433309	7.646798175	14.72222882	7.881267305];
% stdDragonSkin30ContactArea = [5.296397644	1.075419918	11.66737474	17.91867714	16.98826319];
% stdDragonSkin10ContactArea = [4.521095885	7.072669675	4.876358922	4.866581689	10.20193277];

%User2
toughPLAContactArea = [73.138	99.40733333	125.1376667	136.348	142.5866667];
ecoflex50ContactArea = [95.58833333	116.4106667	154.6026667	175.677	182.6893333];
ecoflex30ContactArea = [98.55533333	128.1526667	158.019	180.7653333	201.6056667];
dragonSkin30ContactArea = [77.36033333	110.9993333	126.3366667	146.573	157.632];
dragonSkin10ContactArea =[82.81033333,	119.2536667,134.502,149.9813333,174.656];

% stdDragonSkin10ContactArea =[4.148758529	1.602248524	8.451540629	10.97125819	6.055402299];
% stdEcoflex50ContactArea = [10.34719355	5.155570321	4.064775681	1.111369875	1.294882363];
% stdDragonSkin30ContactArea = [3.664322357	6.633439631	2.829016319	8.523118971	9.954538312];

figure
%plot(F_array,ecoflex30ContactArea, "r--")
plot(F_array,ecoflex50ContactArea, "g--")
hold on
%plot(F_array,dragonSkin10ContactArea, "b--")
plot(F_array,dragonSkin30ContactArea, "r--")
%plot(F_array,toughPLAContactArea, "k--")


plot(F_array,User2SDContactArea(2,:), "g",F_array,User2SDContactArea(4,:), "r")

plot(F_array,A_contact(2,:), "g:", F_array,A_contact(4,:), "r:")

legend('E00-50 Mat.', 'DS30 Mat.', 'E00-50 SD', 'DS30 SD', 'E00-50 Mode;', 'DS30 Model');

title("Contact area vs Force");
xlabel("Force (N)")
ylabel("Area (mm^2)")
xlim([0,4])
ylim([50,400])
set(groot,'defaultLineLineWidth',3.0)
legend('Location','northwest')
fontsize(gca,20,"pixels")
hold off

function out = User1SD(x,id)
    x=x*1000; % N/mm^2 to kPa
    if(id==1) %0.5N
        out = 0.0083.*x.^3 - 0.4021.*x.^2 + 6.1737.*x + 114.09; % N/mm^2 to kPa
    elseif(id==2) %1N
        out = 0.0148.*x.^3 - 0.6336.*x.^2 + 7.8927.*x + 144.8;
    elseif(id==3) %2N
        out = 0.0194.*x.^3 - 0.8218.*x.^2 + 9.6732.*x + 175.7;
    elseif(id==4) %3N
        out = 0.0272.*x.^3 - 1.1158.*x.^2 + 12.453.*x + 195.69;
    elseif(id==5) %4N
        out = 0.0183.*x.^3 - 0.8481.*x.^2 + 10.585.*x + 212.06;
    else
        out = 0;
    end
end

function out = User2SD(x,id)
    x=x*1000; % N/mm^2 to kPa
    if(id==1) %0.5N
        out = 0.0182.*x.^3 - 0.7935.*x.^2 + 10.483.*x + 55.011; % N/mm^2 to kPa
    elseif(id==2) %1N
        out = 0.0177.*x.^3 - 0.8255.*x.^2 + 11.688.*x + 67.836;
    elseif(id==3)
        out = 0.0111.*x.^3 - 0.5545.*x.^2 + 9.3224.*x + 84.08;
    elseif(id==4)
        out = 0.0121.*x.^3 - 0.6377.*x.^2 + 10.507.*x + 95.906;
    elseif(id==5)
        out = 0.0077.*x.^3 - 0.449.*x.^2 +  8.3344.*x + 107.94;
    else
        out = 0;
    end
end

