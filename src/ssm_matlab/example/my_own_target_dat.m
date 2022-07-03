%==================================================================
% Useful Constants
%==================================================================

d2r     = pi/180;                 % Conversion Deg to Rad
g       = 9.81;                   % Gravity [m/s/s]
m2ft    = 3.28084;                % metre to feet
Kg2slug = 0.0685218;              % Kg to slug

%==================================================================
% Define Initial Conditions
%==================================================================
x_ini      = 0;		        % Initial downrange position [m]
h_ini      = 10000/m2ft;        % Initial altitude [m]
% v_ini      = 3*328;		% Initial velocity [m/s]
% alpha_ini  = 0*d2r;		% Initial incidence [rad]
% theta_ini  = 0*d2r;		% Initial Body Attitude [rad]
% q_ini      = 0*d2r;		% Initial pitch rate [rad/sec]

%==================================================================
%==================================================================
% Define Target 
%==================================================================
pos_tgt   = [4500+x_ini -h_ini-500]; % Initial Target position [m]
v_tgt     = 328;		% Target Velocity [m/s]
theta_tgt = 180*d2r;		% Target Direction [rad]
