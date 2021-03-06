clear; clc; %close all;
cla = 4.752798721;
cda = 0.6417112299;
cma = -1.8;

cla_stall = -3.85;
cda_stall = -0.9233984055;
cma_stall = 0;

alpha_stall = 0.3391428111; % 19.4315


alp_bs = linspace(0,alpha_stall,10);
alp_as = linspace(alpha_stall, deg2rad(45),10);

cl_bs = cla*alp_bs;
cl_as = cla*alpha_stall + cla_stall*(alp_as-alpha_stall);

cd_bs = cda*alp_bs;
cd_as = cda*alpha_stall + cda_stall*(alp_as-alpha_stall);

cm_bs = cma*alp_bs;
cm_as = cma*alpha_stall + cma_stall*(alp_as-alpha_stall);

Alp = [alp_bs, alp_as];
CL = [cl_bs, cl_as];
CD = [cd_bs, cd_as];
CM = [cm_bs, cm_as];

% figure(3);
% hold on;
% plot(rad2deg(Alp), CL);
% plot(rad2deg(Alp), CD);
% plot(rad2deg(Alp), CM);
% grid on;
% hold off;

% Lets assume
v = 90; % m/s or just barely mach 0.2
rho = 1.2041;
S = 1.4575;
% calculate dynamic pressure
q = 0.5*rho*v^2;

alp = 4;
alp = deg2rad(alp);

i = find(Alp>=alp,1,'first');

Lift = CL * q * S;
Drag = CD * q * S;
Moment = CM * q * S;

figure(2);
hold on;
plot(rad2deg(Alp), Lift);
plot(rad2deg(Alp), Drag);
plot(rad2deg(Alp), Moment);
grid on;
hold off;

