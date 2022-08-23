clear;
load('OutputFile1.mat');
Z = out.simout.position.z.Data;
Y = out.simout.position.y.Data;
X = out.simout.position.x.Data;

figure(1);
plot3(X,Y,Z);
grid on;
axis equal;

R = out.simout.orientation.x.Data;
P = out.simout.orientation.y.Data;
Y = out.simout.orientation.z.Data;

figure(2);
plot(out.tout, P);
grid on;

ref = out.simout.RefSpeed.Data;
speedX = out.simout.LinearVelocity.x.Data;
figure(3)
hold on;
plot(out.tout, ref);
plot(out.tout, speedX);
hold off;
grid on;
