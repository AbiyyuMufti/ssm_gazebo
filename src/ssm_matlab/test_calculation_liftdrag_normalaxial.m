sweep = 2.02494e-21; % -- 1.1602e-19(DEG);
ctrlAng = 0;
ClDuectrl = 7.36158e-322;
velocity = 250;
alpha = 0.00216184;% -- 0.123864(Deg);
mach = 0.72875;
lift = 119.616;
lift_v = [-0.002041 0.427262 119.616];
drag = 837.093;
drag_v = [-837.093 -1.6e-05 -0.014281];
normal = 121.518;
normal_v = [-0.264776 0.434053 121.517];
axial = 828.639;
axial_v = [-828.637 -0.006415 -1.80551];
moment = 5.90391;
moment_v = [0 5.90388 -0.021088];


lift_calc = normal*cos(-alpha) - axial*sin(-alpha);
drag_calc = normal*sin(-alpha) + axial*cos(-alpha);
normal_calc = lift*cos(alpha) + drag*sin(alpha);
axial_calc = -1*lift*sin(alpha) + drag*cos(alpha);

lift_calc_v = normal_v*cos(-alpha) - axial_v*sin(-alpha);
drag_calc_v = normal_v*sin(-alpha) + axial_v*cos(-alpha);
normal_calc_v = lift_v*cos(alpha) + drag_v*sin(alpha);
axial_calc_v = -1*lift_v*sin(alpha) + drag_v*cos(alpha);