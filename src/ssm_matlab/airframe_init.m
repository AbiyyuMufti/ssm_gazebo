clear; clc;
%% Simulation Parameter
SimTime = 60;

%% Initial Pose and Velocity
pose_ini = [0, 0, 0]; % m
rot_ini = [0, deg2rad(20), 0]; % radian

vel_ini = [0, 0, 0]; % m/s
rotvel_ini = [0, 0, 0]; % rad/s

%% Mass and Inertia

ixx = 13.5604;
ixy = 0.0;
ixz = 0.0;
iyx = 0.0;
iyy = 1088.29;
iyz = 0.0;
izx = 0.0;
izy = 0.0;
izz = 1088.29;
inertia = [ixx, ixy, ixz;
           iyx, iyy, iyz;
           izx, izy, izz;];
mass = 780; % Kg

%% Thrust 
thrust_magnitude = 78*1e+3; % 78000; % N
boosting_time = 5;

%% Thrust Vectoring
ArmLength = 0.820 + 1.540;

