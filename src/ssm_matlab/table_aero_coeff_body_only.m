clear; clc; close all;
mach_table = [0.2, 0.4, 0.5, 0.6, 0.7, 0.75, 0.8, 0.93];
alpha_table = [.00, 4.00, 8.00, 12.00, 16.00, 20.00, 24.00, 28.00];

% mach 0.2
cn_table(1,:) = [.000, 1.013, 2.044, 3.181, 4.150, 5.103, 5.967, 6.846];
cm_table(1,:) = [.000, -.041, .192, .339, 1.345, 2.745, 4.378, 5.630];
ca_table(1,:) = [.239, .221, .167, .122, .110, .141, .118, .056];
cl_table(1,:) = [.000, .995, 2.001, 3.086, 3.959, 4.747, 5.403, 6.019];
cd_table(1,:) = [.239, .292, .450, .780, 1.249, 1.878, 2.535, 3.264];

% mach 0.4
cn_table(2,:) = [.000, 1.027, 2.062, 3.192, 4.132, 5.086, 5.996, 6.957];
cm_table(2,:) = [.000, -.068, .153, .311, 1.412, 2.792, 4.345, 5.636];
ca_table(2,:) = [.236, .219, .166, .122, .110, .138, .112, .046];
cl_table(2,:) = [.000, 1.009, 2.019, 3.097, 3.941, 4.732, 5.432, 6.121];
cd_table(2,:) = [.236, .290, .451, .783, 1.244, 1.869, 2.541, 3.306];

% mach 0.5
cn_table(3,:) = [.000, 1.046, 2.096, 3.243, 4.188, 5.148, 6.084, 7.158];
cm_table(3,:) = [.000, -.094, .100, .197, 1.278, 2.623, 4.198, 5.696];
ca_table(3,:) = [.234, .216, .161, .115, .103, .134, .108, .036];
cl_table(3,:) = [.000, 1.028, 2.053, 3.149, 3.997, 4.792, 5.515, 6.304];
cd_table(3,:) = [.234, .289, .451, .787, 1.254, 1.886, 2.573, 3.392];

% mach 0.6
cn_table(4,:) = [.000, 1.069, 2.135, 3.296, 4.245, 5.229, 6.262, 7.452];
cm_table(4,:) = [.000, -.123, .047, .108, 1.172, 2.533, 4.201, 5.827];
ca_table(4,:) = [.232, .213, .156, .109, .098, .130, .101, .025];
cl_table(4,:) = [.000, 1.051, 2.093, 3.202, 4.053, 4.869, 5.680, 6.568];
cd_table(4,:) = [.232, .288, .452, .792, 1.264, 1.910, 2.639, 3.520];

% mach 0.7
cn_table(5,:) = [.000, 1.076, 2.145, 3.306, 4.278, 5.339, 6.450, 7.770];
cm_table(5,:) = [.000, -.137, .019, .065, 1.056, 2.521, 4.151, 5.839];
ca_table(5,:) = [.230, .211, .154, .107, .096, .127, .095, .014];
cl_table(5,:) = [.000, 1.059, 2.103, 3.212, 4.086, 4.974, 5.854, 6.854];
cd_table(5,:) = [.230, .286, .451, .792, 1.271, 1.945, 2.711, 3.660];

% mach 0.75
cn_table(6,:) = [.000, 1.094, 2.176, 3.352, 4.351, 5.448, 6.612, 7.995];
cm_table(6,:) = [.000, -.161, -.032, -.027, .919, 2.405, 4.021, 5.713];
ca_table(6,:) = [.230, .210, .152, .104, .093, .124, .092, .008];
cl_table(6,:) = [.000, 1.077, 2.134, 3.258, 4.156, 5.077, 6.002, 7.055];
cd_table(6,:) = [.230, .286, .453, .799, 1.289, 1.980, 2.774, 3.761];

% mach 0.80
cn_table(7,:) = [.000, 1.124, 2.250, 3.504, 4.588, 5.763, 7.008, 8.441];
cm_table(7,:) = [.000, -.183, -.077, -.163, .762, 2.194, 3.861, 5.428];
ca_table(7,:) = [.245, .225, .163, .112, .100, .134, .103, .017];
cl_table(7,:) = [.000, 1.106, 2.205, 3.404, 4.383, 5.369, 6.360, 7.445];
cd_table(7,:) = [.245, .302, .474, .838, 1.361, 2.097, 2.944, 3.978];

% mach 0.93
cn_table(8,:) = [.000, 1.227, 2.460, 3.862, 5.108, 6.451, 7.887, 9.638];
cm_table(8,:) = [.000, -.249, -.243, -.576, .241, 1.695, 3.348, 4.986];
ca_table(8,:) = [.433, .430, .416, .397, .377, .355, .292, .172];
cl_table(8,:) = [.000, 1.194, 2.378, 3.695, 4.806, 5.941, 7.086, 8.429];
cd_table(8,:) = [.433, .514, .754, 1.191, 1.771, 2.540, 3.475, 4.676];

legend_list = {'Mach 0.2', 'Mach 0.4', 'Mach 0.5', 'Mach 0.6', 'Mach 0.7', 'Mach 0.75', 'Mach 0.8', 'Mach 0.93'};

for i = 1:8
    figure(1);
    title("Lift Coefficient (CL) Diagram");
    hold on;
    grid on;
    plot(alpha_table, cl_table(i,:), 'DisplayName',legend_list{i});
    ylabel("CL");
    xlabel("\alpha - Angle of Attack (deg)");
    legend

    figure(2);
    title("Drag Coefficient (CD) Diagram");
    hold on;
    grid on;
    plot(alpha_table, cd_table(i,:), 'DisplayName',legend_list{i});
    ylabel("CD");
    xlabel("\alpha - Angle of Attack (deg)");
    legend

    figure(3);
    title("Pitching Moment Coefficient (CM) Diagram");
    hold on;
    grid on;
    plot(alpha_table, cm_table(i,:), 'DisplayName',legend_list{i});
    ylabel("CM");
    xlabel("\alpha - Angle of Attack (deg)");
    legend

    figure(4);
    title("Normal Coefficient (CN) Diagram");
    hold on;
    grid on;
    plot(alpha_table, cn_table(i,:), 'DisplayName',legend_list{i});
    ylabel("CN");
    xlabel("\alpha - Angle of Attack (deg)");
    legend

    figure(5);
    title("Axial Coefficient (CA) Diagram");
    hold on;
    grid on;
    plot(alpha_table, ca_table(i,:), 'DisplayName',legend_list{i});
    ylabel("CA");
    xlabel("\alpha - Angle of Attack (deg)");
    legend

end

