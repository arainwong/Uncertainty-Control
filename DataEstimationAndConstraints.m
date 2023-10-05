close all;

%% Optimization Type

% 1 -> Case 1
% 2 -> Case 2
% 3 -> Case 3
optimizationType = [false, false, true];

disp('----------------------------------------------------------------------');
if find(optimizationType==true) == 1
    disp('Case 1');
elseif find(optimizationType==true) == 2
    disp('Case 2');
elseif find(optimizationType==true) == 3
    disp('Case 3');
else
    error('Wrong configuration in control type');
end
disp('----------------------------------------------------------------------');

%% Sensor Config

angleSensor = false;
frictionSensor = false;
weightSensor = true; 
% gSensor = true;   % it is easy and cheap to measure so that it would not be
                    % considered anymore

velocitySensor = false;

%% Constraints

% length = 0.23
time_lb = 3; % s
time_ub = 5; % s

% velocity and time constraints mode
vtMode = true;

velocity_lb = 1.5; % m/s
velocity_ub = 1.5; % m/s 

% acceleration constraints mode
accMode = false;

if vtMode == true && accMode == false
    con_lb = velocity_lb / time_ub;
    con_ub = velocity_ub / time_lb;
    
elseif vtMode == false && accMode == true
    con_lb = 2*length/(time_ub^2);
    con_ub = 2*length/(time_lb^2);

else
    error('Please choose a correct constaint mode')
end
disp(['The desired acceleration constraint is : [', num2str(con_lb), ', ', num2str(con_ub), '].']);

%% Estimation based on sampled data

overapproxFactor = 0.05;
overapproximation = 1 + overapproxFactor; % ?% redandency 

% estimated base acceleration
estAcc_ub = maxAcc;
estAcc_lb = min(ddot_x_out);
disp(['The estimated acceleration: [', num2str(estAcc_lb), ', ', num2str(estAcc_ub), '].']);
disp(['with ', num2str(overapproximation * 100), '% overapproximation the uncertain acceleration set is: [', ...
    num2str(estAcc_lb * overapproximation), ', ', ...
    num2str(estAcc_ub* overapproximation), '].']);

%% Parameters Estimation (from Expert opinion, Sensor, etc.)

% estimated angle (accurate: [28, 32] degree)
estAngle_lb = angle_lb * (1 - overapproxFactor);
estAngle_ub = angle_ub * overapproximation;
disp(['The estimated angle: [', num2str(estAngle_lb), ', ', num2str(estAngle_ub), '] degree.']);

% estimated friction (accurate: [0.4, 0.6])
estFriction_lb = (friction_mean - friction_std) * (1 - overapproxFactor);
estFriction_ub = (friction_mean - friction_std) * overapproximation;
disp(['The estimated friction: [', num2str(estFriction_lb), ', ', num2str(estFriction_ub), '].']);

% estimated weight (accurate: [0.0249, 0.0251] kg)
if weightSensor == true
    estWeight_lb = weight_mean - weight_std;
    estWeight_ub = weight_mean + weight_std;
    estWeight = weight;
    disp(['(SENSORED) The estimated weight: [', num2str(estWeight_lb), ', ', num2str(estWeight_ub), '] kg.']);
else
    estWeight_lb = (weight_mean - weight_std) * (1 - overapproxFactor);
    estWeight_ub = (weight_mean + weight_std) * overapproximation;
    disp(['The estimated weight: [', num2str(estWeight_lb), ', ', num2str(estWeight_ub), '] kg.']);
end


% estimated g (accurate: [9.78,9.83] m/s^2)
estG = g;
% estG_lb = g_lb;
% estG_up = g_ub;
% disp(['The estimated g: [', num2str(estG_lb), ', ', num2str(estG_up), '] m/s^2.']);
disp(['The estimated g: ', num2str(estG), ' m/s^2.']);


%% Control Input

if find(optimizationType==true) == 1
    k_lb = estFriction_lb / estWeight_ub;
    k_ub = estFriction_ub / estWeight_lb;
    
%     u_lbSet = [0, ...
%                (con_lb - estAcc_ub) /k_ub];
    u_ubSet = [(con_ub - estAcc_lb) /k_lb, ...
                estWeight_lb * estG * cos(estAngle_ub)];
%     u_lb = max(u_lbSet);
    u_lb = (con_lb - estAcc_ub) /k_ub;
    u_ub = min(u_ubSet);
    disp(['The control input set: [', num2str(u_lb), ', ', num2str(u_ub), ...
        '] m*kg/s^2.']);
    
    u = u_ub;
    
elseif find(optimizationType==true) == 2 
    if weightSensor ~= true
        error('Weight sensor is required in case 2');
    end
    u = zeros(1, numSample);
    validationSet = zeros(1, numSample);
    for i = 1:numSample
        k_lb = estFriction_lb / estWeight(i);
        k_ub = estFriction_ub / estWeight(i);

        validationSet(i) = estWeight(i) * estG * cos(estAngle_lb);

        u_lb = (con_lb - estAcc_ub) /k_ub;
        u_ub = (con_ub - estAcc_lb) /k_lb;
        if validationSet(i) <= u_ub
            u(i) = validationSet(i);
        else
            u(i) = u_ub;
        end
    end

elseif find(optimizationType==true) == 3
    % set the reference velocity
    t_des = 4; % s
    dot_x_des = 2*length/t_des;
    disp(['The reference velocity is ', num2str(dot_x_des), ' m/s.']);
    

    
end
disp('----------------------------------------------------------------------');