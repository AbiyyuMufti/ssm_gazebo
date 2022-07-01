clc;
clear;
close all;

pos_rot = [0 1 0 deg2rad(20)];
pos_quat = axang2quat(pos_rot);

orig_tgt = [1 0 0];

z_v = [0 0 1];

tgt_v = quatrotate(pos_quat, orig_tgt);

tgt_v = tgt_v/norm(tgt_v);

azimth = 30;
elevat = -5;

d_x = cos(deg2rad(azimth))*sin(deg2rad(elevat));
d_y = sin(deg2rad(azimth))*sin(deg2rad(elevat));
d_z = cos(deg2rad(elevat));


[x, y, z] = sph2cart(deg2rad(azimth),deg2rad(90-elevat),1);

dir_v = [d_x, d_y, d_z];
% dir_v = dir_v/norm(dir_v)
% comp = [x, y, z]

dir_v = dir_v/norm(dir_v);

angle_to_rotate = acos(dot(z_v, tgt_v)/norm(z_v)*norm(tgt_v));

axis_rotation = cross(tgt_v, z_v);

axang = [axis_rotation angle_to_rotate];
quat = axang2quat(axang);

goal = quatrotate(quat, dir_v);
goal = goal/norm(goal);

pos_rot_rev = [0 -1 0 deg2rad(20)];
pos_quat_rev = axang2quat(pos_rot_rev);
goal_orig = quatrotate(pos_quat_rev, goal);

hold on; grid on;
quiver3(0,0,0, 0, 1, 0,'--'); 
quiver3(0,0,0, z_v(1), z_v(2), z_v(3), '--');

% tgt in original
quiver3(0,0,0, orig_tgt(1), orig_tgt(2), orig_tgt(3), 'g');
% tgt in pose
quiver3(0,0,0, tgt_v(1), tgt_v(2), tgt_v(3), '--', 'LineWidth',2);

quiver3(0,0,0, dir_v(1), dir_v(2), dir_v(3), 'r');
quiver3(0,0,0, goal(1), goal(2), goal(3), 'b','LineWidth',2);
quiver3(0,0,0, goal_orig(1), goal_orig(2), goal_orig(3),'LineWidth',2);

legend('y', 'z', 'orig', 'target', 'ORIGINAL', 'RESULT', 'goal_orig');
axis equal;
hold off;