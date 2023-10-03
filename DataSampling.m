clc; clear;

%% Baseline, without constraints and control input
% Data generation
numSample = 1000;

length = 230/1000; %(m)

angle_lb = 28/180 * pi;
angle_ub = 32/180 * pi;
angleDist = makedist('Uniform','lower',angle_lb,'upper',angle_ub);
% angle = random(angleDist, 1, 1);
angle = random(angleDist, 1, numSample);

weight_mean = 25/1000;
weight_std = 0.1/1000;
weightDist = makedist('Normal','mu',weight_mean,'sigma',weight_std); %(kg)
weight = random(weightDist, 1, numSample); 

g_lb = 9.78;
g_ub = 9.83;
gDist = makedist('Uniform','lower',g_lb,'upper',g_ub); %m/s^2
g = random(gDist, 1, 1);
% g = random(gDist, 1, numSample);

friction_mean = 0.5;
friction_std = 0.1;
frictionNormalDist = makedist('Normal','mu',friction_mean,'sigma',friction_std);
friction = random(frictionNormalDist, 1, numSample);

%% Control and optimization
u = 0;


