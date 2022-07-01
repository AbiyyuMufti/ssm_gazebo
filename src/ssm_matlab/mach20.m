clear; clc; close all;

Alpha = [     0,      4,      8,     12,     16,     20,     24,     28];
CL    = [     0,  1.017,  2.067,  3.218,  4.176,  5.066,  5.836,  6.576];
CD    = [ 0.266,  0.321,  0.487,  0.835,  1.337,  2.014,  2.738,  3.553];
CM    = [ 0.000, -0.222, -0.289, -0.558, -0.080,  0.688,  1.595,  2.038];

CN    = [ 0.000,  1.037,  2.115,  3.322,  4.383,  5.449,  6.445,  7.475];
CA    = [ 0.266,  0.249,  0.194,  0.148,  0.134,  0.160,  0.128,  0.050];

X_CP  = [-0.295, -0.214, -0.136, -0.168, -0.018,  0.126,  0.247,  0.273];

Table = [Alpha' CN' CM' CA' CL' CD' X_CP'];

figure;
hold on;
plot(Alpha, CL);
plot(Alpha, CD);
plot(Alpha, CM);
% plot(Alpha, CA);
% plot(Alpha, CN);
% legend('CL', 'CD', 'CM', 'CA', 'CN');
legend('CL', 'CD', 'CM');
grid on;
hold off;

% Lets assume
v = 20; % m/s or just barely mach 0.2
rho = 1.2041;
S = 1.4575;

% calculate dynamic pressure
q = 0.5*rho*v^2;

alp = 4;

i = find(Alpha>=alp,1,'first');

Lift = CL(i) * q * S;
Drag = CD(i) * q * S;
Moment = CM(i) * q * S;

Normal = CN(i) * q * S;
Axial = CA(i) * q * S;

LiftFromAN = Normal*cos(alp) - Axial*sin(alp);
DragFromAN = Normal*sin(alp) + Axial*cos(alp);
NormalFromLD = Lift*cos(alp) - Drag*sin(alp);
AxialFromLD = -1*Lift*sin(alp) + Drag*cos(alp);


% [Lift, LiftFromAN]
% [Drag, DragFromAN]
% [Normal, NormalFromLD]
% [Axial, AxialFromLD]




