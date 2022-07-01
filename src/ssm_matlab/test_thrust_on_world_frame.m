clear; clc; close all;

rot = [0 -0.349029 0];
rad2deg(rot)
fwd = [1 0 0];
dir = [0.939705 0 0.341986];
force = [9397.05 0 3419.86];

rotang = acos(dot(fwd,dir)/(norm(fwd)*norm(dir)));
rad2deg(rotang)

hold on; grid on;
quiver3(0,0,0, fwd(1),fwd(2),fwd(3));
quiver3(0,0,0, dir(1),dir(2),dir(3));
hold off;





