close all;

%% Optimization Type

% 1 -> Case 1, require no sensor
% 2 -> Case 2, require weight sensor, g sensor used as default
% 3 -> Case 3, require angle and weight sensors, g sensor used as default
% 4 -> Case 4, require angle, weight and friction sensors, g sensor used as default
% 5 -> Case 5, feedback control, have its own control goal based on reference parameter
% 6 -> Case 6, case 2 + case 3 == Feedforward + Feedback


optimizationType = [false, false, false, false, false, false];
optimizationType(6) = true;

disp('----------------------------------------------------------------------');
if find(optimizationType==true) == 1
    disp('Case 1');

elseif find(optimizationType==true) == 2
    disp('Case 2, require weight sensor, g sensor used as default.');

elseif find(optimizationType==true) == 3
    disp('Case 3, require angle and weight sensors, g sensor used as default.');

elseif find(optimizationType==true) == 4
    disp('Case 4, require angle, weight and friction sensors, g sensor used as default.');


elseif find(optimizationType==true) == 5
    disp('Case 5, purely feedback control.');

    % set the reference velocity, since feedback form control does not 
    % require any estimation and constraint
    t_des = 0.5; % s
    dot_x_des = length/t_des;
    PID = [10, 1, 1];
    disp(['The reference velocity is ', num2str(dot_x_des), ' m/s.']);
    disp('----------------------------------------------------------------------');
    return

elseif find(optimizationType==true) == 6
    disp('Case 6, feedforward and feedback control.');


else
    error('Wrong configuration in control type');
end
disp('----------------------------------------------------------------------');

%% Sensor Config

angleSensor = true;
weightSensor = true; 
% gSensor = true;   % it is easy and cheap to measure so that it would not be
                    % considered anymore

velocitySensor = false;

frictionSensor = true;

%% Constraints

disp('Constraints: ');

% length = 0.23
time_lb = 1; % s
time_ub = 1; % s

% velocity and time constraints mode
vtMode = false;

velocity_lb = 1; % m/s
velocity_ub = 1; % m/s 

% acceleration constraints mode
accMode = true;

if vtMode == true && accMode == false
    con_lb = velocity_lb / time_ub;
    con_ub = velocity_ub / time_lb;
    disp(('Now using the velocity and time constraints mode. '));
    disp(['The desired velocity constraint is : [', num2str(velocity_lb), ', ', num2str(velocity_ub), '] m/s.']);
    
elseif vtMode == false && accMode == true
    con_lb = 2*length/(time_ub^2);
    con_ub = 2*length/(time_lb^2);
    disp(('Now using the acceleration constraints mode. '));

else
    error('Please choose a correct constaint mode')
end
disp(['The time constraint is : [', num2str(time_lb), ', ', num2str(time_ub), '] s.']);
disp(['Meanwhile, the acceleration constraint is : [', num2str(con_lb), ', ', num2str(con_ub), '] m/s^2.']);
disp('----------------------------------------------------------------------');

%% Estimation based on sampled data

disp('Estimations: ');

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
if angleSensor == true
    estAngle_lb = angle_lb;
    estAngle_ub = angle_ub;
    estAngle = angle;
    disp(['(SENSORED) The estimated angle: [', num2str(estAngle_lb), ', ', num2str(estAngle_ub), '] degree.']);
else
    estAngle_lb = angle_lb * (1 - overapproxFactor);
    estAngle_ub = angle_ub * overapproximation;
    disp(['The estimated angle: [', num2str(estAngle_lb), ', ', num2str(estAngle_ub), '] degree.']);
end

% estimated friction (accurate: [0.4, 0.6])
if frictionSensor == true
    estFriction_lb = friction_mean - friction_std;
    estFriction_ub = friction_mean + friction_std;
    estFriction = friction;
    disp(['(SENSORED) The estimated friction: [', num2str(estFriction_lb), ', ', num2str(estFriction_ub), '].']);
else
    estFriction_lb = (friction_mean - friction_std) * (1 - overapproxFactor);
    estFriction_ub = (friction_mean + friction_std) * overapproximation;
    disp(['The estimated friction: [', num2str(estFriction_lb), ', ', num2str(estFriction_ub), '].']);

end

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
disp('----------------------------------------------------------------------');

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

        u(i) = min([validationSet(i), u_ub]);
        
    end
    disp('Case 2, the control sequence based on weight sensor has been derived.');

elseif find(optimizationType==true) == 3
    if angleSensor == false || weightSensor  == false
        error('Angle and weight sensors are required in case 5');
    end
    u = zeros(1, numSample);
    validationSet = zeros(1, numSample);
    for i = 1:numSample
        k_lb = estFriction_lb / estWeight(i);
        k_ub = estFriction_ub / estWeight(i);
        
        estAccSet = [estG * sin(estAngle(i)) - estFriction_lb * estG * cos(estAngle(i)), ...
                     estG * sin(estAngle(i)) - estFriction_ub * estG * cos(estAngle(i))];
        estAcc_lb = min(estAccSet);
        estAcc_ub = max(estAccSet);
        
        validationSet(i) = estWeight(i) * estG * cos(estAngle(i));

        u_lb = (con_lb - estAcc_ub) /k_ub;
        u_ub = (con_ub - estAcc_lb) /k_lb;

        u(i) = min([validationSet(i), u_ub]);

    end
    disp('Case 3, the control sequence based on angle and weight sensor has been derived.')
    
elseif find(optimizationType==true) == 4
    if angleSensor == false || weightSensor  == false || frictionSensor == false
        error('Angle, weight and friction sensors are required in case 6');
    end
    u = zeros(1, numSample);
    validationSet = zeros(1, numSample);
    for i = 1:numSample
        k = estFriction(i) / estWeight(i);

        estAcc = estG * sin(estAngle(i)) - estFriction(i) * estG * cos(estAngle(i));
        
        validationSet(i) = estWeight(i) * estG * cos(estAngle(i));

        u(i) = min([validationSet(i), (con_lb - estAcc) / k]);

    end
    disp('Case 4, the control sequence based on angle, weight and friction sensor has been derived.')
    

end
disp('----------------------------------------------------------------------');