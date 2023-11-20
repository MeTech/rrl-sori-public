clear; clc; close all;
h0= 1.71; %[mm]
P0=0.012; %[N/mm^2]
l0=5.2; %[mm]
F0=0.5; %[N]
t0=0.75; ; %[mm]

E = 12*P0*l0^4/(384*h0*t0^3);

P=0:0.001:0.016;
l00 = [8, 5.2,3.6,2.8]';


pressure    = [	0	4	8	12	16];
f0          = [0	1.27	1.75	2.1	    2.51];
f0_5        = [0	0.59	1.38	1.71	2.04];
f2          = [0	0.29	0.77	1.44	2.09];
f8          = [0	0.33	0.6	    0.67    1];

force = [0 0.5 2 8];
p0          = [0 0 0 0];
p4          = [1.27     0.59    0.29    0.33];
p8          = [1.75     1.38    0.77    0.6];
p12         = [2.1      1.71    1.44    0.67];
p16         = [2.51     2.04    2.09    1];

c=200; b=0.35; % the coefficients!!
c1=200; b1= 0.13;

c2=180; b2= 0.35;
beta = 0.9;

force = [0, 0.5, 2, 8];                                                     %[N]
h_appx = c*(pressure/1000).*(1-b.*force.^(1/3))';
AA=(1-b1.*force.^(1/3)).^2./(1+b1.*force.^(1/3)).^2;
%c1*(pressure/1000);
h_appx_energy = c1*(pressure/1000).*((1-b1.*force.^(1/3)).^2./(1+b1.*force.^(1/3)).^2)' ;% used

f_term_num = (1-b2.*(force./(1+beta.*force)).^(1/3)).^2;
f_term_denum =1+b2.*(force./(1+beta.*force)).^(1/3).^2;

figure
plot(pressure,f0,"--",pressure,f0_5,"--", pressure, f2,"--", pressure,f8,"--")
hold on
plot( pressure,h_appx)
xlabel("Pressure (kPa)")
ylabel("Torus SPA Height (mm^2)")
legend('Experiment F=0 N','Experiment F=0.5 N','Experiment F=2 N','Experiment F=8 N', 'Model F=0 N', ...
    'Model F=0.5 N','Model F=2 N','Model F=8 N')
legend('Location','northwest')
fontsize(gca,16,"pixels");
set(groot,'defaultLineLineWidth',5.0)
