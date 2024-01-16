clc; clear;

%% config the number of samples
% numSample in the range of [0,5000]
numSample = 1000;
stopTime = 5;

%% load fixed data
load('data.mat');

angle = angle(:,1:numSample);
friction = friction(:,1:numSample);
weight = weight(:,1:numSample);

angle_lb = min(angle);
angle_ub = max(angle);

friction_lb = min(friction);
friction_ub = max(friction);

baselineSet = [g * (sin(angle_ub) - friction_ub * cos(angle_ub)), ...
               g * (sin(angle_lb) - friction_ub * cos(angle_lb)), ...
               g * (sin(angle_ub) - friction_lb * cos(angle_ub)), ...
               g * (sin(angle_lb) - friction_lb * cos(angle_lb))];
baseline_lb = min(baselineSet);
baseline_ub = max(baselineSet);

if baseline_lb<=0
    baseline_lb_fixed = 0;
else
    baseline_lb_fixed = baseline_lb;
end
disp(['Theoretically, the acceleration range of samples is [', num2str(baseline_lb), ', ', ...
        num2str(baseline_ub), '] -> [', num2str(baseline_lb_fixed), ', ', num2str(baseline_ub), '].']);