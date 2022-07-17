clear; clc; close all;
mach_table = [0.2, 0.4, 0.5, 0.6, 0.7, 0.75, 0.8, 0.93];
alpha_table = [.00, 4.00, 8.00, 12.00, 16.00, 20.00, 24.00, 28.00];

% mach 0.2
cn_table(1,:) = [0.000, 1.037, 2.115, 3.322, 4.383, 5.449, 6.445, 7.475];
cm_table(1,:) = [0.000, -0.222, -0.289, -0.558, -0.080, 0.688, 1.595, 2.038];
ca_table(1,:) = [0.266, 0.249, 0.194, 0.148, 0.134, 0.160, 0.128, 0.050];
cp_table(1,:) = [-0.295, -0.214, -0.136, -0.168, -0.018, 0.126, 0.247, 0.273];
cl_table(1,:) = [.000, 1.017, 2.067, 3.218, 4.176, 5.066, 5.836, 6.576];
cd_table(1,:) = [.266, .321, .487, .835, 1.337, 2.014, 2.738, 3.553];

% mach 0.4
cn_table(2,:) = [0.000, 1.051, 2.134, 3.337, 4.373, 5.448, 6.500, 7.626];
cm_table(2,:) = [0.000, -0.250, -0.333, -0.604, -0.055, 0.655, 1.426, 1.835];
ca_table(2,:) = [0.263, 0.246, 0.193, 0.147, 0.133, 0.157, 0.121, 0.037];
cp_table(2,:) = [-0.323, -0.238, -0.156, -0.181, -0.013, 0.120, 0.219, 0.241];
cl_table(2,:) = [.000, 1.031, 2.087, 3.233, 4.167, 5.066, 5.889, 6.716];
cd_table(2,:) = [.263, .318, .488, .838, 1.333, 2.011, 2.754, 3.613];

% mach 0.5
cn_table(3,:) = [0.000, 1.070, 2.169, 3.390, 4.433, 5.517, 6.603, 7.866];
cm_table(3,:) = [0.000, -0.276, -0.389, -0.727, -0.210, 0.446, 1.204, 1.690];
ca_table(3,:) = [0.261, 0.243, 0.187, 0.141, 0.127, 0.152, 0.116, 0.025];
cp_table(3,:) = [-0.339, -0.258, -0.180, -0.214, -0.047, 0.081, 0.182, 0.215];
cl_table(3,:) = [.000, 1.051, 2.121, 3.286, 4.227, 5.132, 5.986, 6.934];
cd_table(3,:) = [.261, .317, .487, .842, 1.344, 2.030, 2.791, 3.715];

% mach 0.6
cn_table(4,:) = [0.000, 1.093, 2.208, 3.444, 4.494, 5.607, 6.812, 8.206];
cm_table(4,:) = [0.000, -0.305, -0.444, -0.825, -0.337, 0.307, 1.051, 1.581];
ca_table(4,:) = [0.259, 0.240, 0.183, 0.135, 0.121, 0.148, 0.108, 0.011];
cp_table(4,:) = [-0.359, -0.279, -0.201, -0.239, -0.075, 0.055, 0.154, .193];
cl_table(4,:) = [.000, 1.074, 2.161, 3.341, 4.287, 5.219, 6.179, 7.240];
cd_table(4,:) = [.259, .316, .488, .848, 1.355, 2.056, 2.869, 3.862];

% mach 0.7
cn_table(5,:) = [0.000, 1.101, 2.218, 3.456, 4.531, 5.736, 7.029, 8.570];
cm_table(5,:) = [0.000, -0.320, -0.476, -0.877, -0.474, 0.203, 0.845, 1.353];
ca_table(5,:) = [0.257, 0.238, 0.180, 0.132, 0.119, 0.144, 0.101, -0.002];
cp_table(5,:) = [-0.369, -0.291, -0.214, -0.254, -0.105, 0.035, 0.120, 0.158];
cl_table(5,:) = [.000, 1.081, 2.172, 3.353, 4.323, 5.341, 6.381, 7.568];
cd_table(5,:) = [.257, .314, .487, .848, 1.363, 2.097, 2.951, 4.021];

% mach 0.75
cn_table(6,:) = [0.000, 1.118, 2.250, 3.503, 4.607, 5.853, 7.206, 8.818];
cm_table(6,:) = [0.000, -0.345, -0.528, -0.974, -0.628, 0.040, 0.636, 1.106];
ca_table(6,:) = [0.256, 0.236, 0.178, 0.129, 0.116, 0.141, 0.097, -0.009];
cp_table(6,:) = [-0.382, -0.308, -0.235, -0.278, -0.136, 0.007, 0.088, 0.125];
cl_table(6,:) = [.000, 1.099, 2.204, 3.399, 4.397, 5.452, 6.543, 7.790];
cd_table(6,:) = [.256, .314, .489, .855, 1.382, 2.135, 3.020, 4.132];

% mach 0.80
cn_table(7,:) = [0.000, 1.148, 2.324, 3.656, 4.850, 6.177, 7.617, 9.288];
cm_table(7,:) = [0.000, -0.366, -0.575, -1.115, -0.810, -0.218, 0.398, 0.701];
ca_table(7,:) = [0.271, 0.251, 0.189, 0.137, 0.123, 0.151, 0.107, -0.001];
cp_table(7,:) = [-0.392, -0.319, -0.247, -0.305, -0.167, -0.035, 0.052, 0.076];
cl_table(7,:) = [.000, 1.128, 2.275, 3.547, 4.628, 5.753, 6.915, 8.201];
cd_table(7,:) = [.271, .330, .510, .894, 1.455, 2.255, 3.196, 4.359];

% mach 0.93
cn_table(8,:) = [0.000, 1.254, 2.540, 4.024, 5.393, 6.903, 8.553, 10.586];
cm_table(8,:) = [0.000, -0.458, -0.795, -1.620,-1.509, -0.989, -0.510, -0.389];
ca_table(8,:) = [0.458, 0.455, 0.440, 0.421, 0.399, 0.370, 0.294, 0.147];
cp_table(8,:) = [-0.418, -0.365, -0.313, -0.403, -0.280, -0.143, -0.060, -0.037];
cl_table(8,:) = [.000, 1.219, 2.454, 3.848, 5.074, 6.360, 7.694, 9.278];
cd_table(8,:) = [.458, .541, .790, 1.248, 1.870, 2.709, 3.748, 5.100];


for i = 1:8
    figure(1);
    hold on;
    grid on;
    plot(alpha_table, cl_table(i,:))
    figure(2)
    hold on;
    grid on;
    plot(alpha_table, cd_table(i,:))
end

