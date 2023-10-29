clc; clear;

%% Baseline, without constraints and control input
% Data generation
numSample = 5000;
stopTime = 5;

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

% friction_mean = 0.5;
% friction_std = 0.1;
% frictionNormalDist = makedist('Normal','mu',friction_mean,'sigma',friction_std);
% friction = random(frictionNormalDist, 1, numSample);
friction_lb = 0.4;
friction_ub = 0.6;
% friction_lb = 0.49;
% friction_ub = 0.51;
frictionUniformDist = makedist('Uniform','lower',friction_lb,'upper',friction_ub);
friction = random(frictionUniformDist, 1, numSample);


baselineSet = [g * (sin(angle_ub) - friction_ub * cos(angle_ub)), ...
               g * (sin(angle_lb) - friction_ub * cos(angle_lb)), ...
               g * (sin(angle_ub) - friction_lb * cos(angle_ub)), ...
               g * (sin(angle_lb) - friction_lb * cos(angle_lb))];
baseline_lb = min(baselineSet);
baseline_ub = max(baselineSet);

% print the theoretical acceleration range
if baseline_lb<=0
    baseline_lb_fixed = 0;
else
    baseline_lb_fixed = baseline_lb;
end
disp(['Theoretically, the acceleration range of samples is [', num2str(baseline_lb), ', ', ...
        num2str(baseline_ub), '] -> [', num2str(baseline_lb_fixed), ', ', num2str(baseline_ub), '].']);
disp('----------------------------------------------------------------------');

%% Control and optimization
u = 0;

