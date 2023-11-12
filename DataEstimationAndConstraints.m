close all;

%% Optimization Type

% 1 -> Case 1, require no sensor
% 2 -> Case 2, require weight sensor, g sensor used as default
% 3 -> Case 3, require angle and weight sensors, g sensor used as default
% 4 -> Case 4, require angle, weight and friction sensors, g sensor used as default
% 5 -> Case 5, feedback control, have its own control goal based on reference parameter
% 6 -> Case 6, case 2 + case 3 == Feedforward + Feedback
% 7 -> Case 7, minimum energy consumption strategy

% Prepare for "Live Script" representation, here the redundency design is just
% used for easy modification
optimizationType = [false, false, false, false, false, false, false];
optimizationType(6) = true;

% additional config for case 7
if find(optimizationType==true) == 7
    type7Type = [false, false, false, false];
    type7Type(3) = true;
end

disp('----------------------------------------------------------------------');
% config the sensors
if find(optimizationType==true) == 1
    disp('Case 1');

    angleSensor = false;
    weightSensor = false; 
    % gSensor = true;   % it is easy and cheap to measure so that it would not be
                        % considered anymore
    
    frictionSensor = false;

elseif find(optimizationType==true) == 2
    disp('Case 2, require weight sensor, g sensor used as default.');

    angleSensor = false;
    weightSensor = true; 
    % gSensor = true;   % it is easy and cheap to measure so that it would not be
                        % considered anymore
    
    frictionSensor = false;
elseif find(optimizationType==true) == 3
    disp('Case 3, require angle and weight sensors, g sensor used as default.');
    
    angleSensor = true;
    weightSensor = true; 
    % gSensor = true;   % it is easy and cheap to measure so that it would not be
                        % considered anymore
    
    frictionSensor = false;

elseif find(optimizationType==true) == 4
    disp('Case 4, require angle, weight and friction sensors, g sensor used as default.');

    angleSensor = true;
    weightSensor = true; 
    % gSensor = true;   % it is easy and cheap to measure so that it would not be
                        % considered anymore
    
    frictionSensor = true;


elseif find(optimizationType==true) == 5
    disp('Case 5, purely feedback control.');

    % set the reference velocity, since feedback form control does not 
    % require any estimation and constraint
    t_des = 0.5; % s
    dot_x_des = length/t_des;
    PID = [3, 1, 0];
    disp(['The reference velocity is ', num2str(dot_x_des), ' m/s.']);
    disp('----------------------------------------------------------------------');
    return

elseif find(optimizationType==true) == 6
    disp('Case 6, feedforward and feedback control.');
    PID = [3, 1, 0];
    u_ref = u;
    dot_x_ref = dot_x(end, :);
    disp('The reference of control sequence and velocity have been set.');
    disp('----------------------------------------------------------------------');
    return

elseif find(optimizationType==true) == 7
    disp(['Case 7, the strategy is minimum energy consumption, which ', ...
        'require angle and weight sensors, g sensor used as default.']);
    
    if find(type7Type==true) == 1
        angleSensor = false;
        weightSensor = false; 
        frictionSensor = false;

    elseif find(type7Type==true) == 2
        angleSensor = false;
        weightSensor = true; 
        frictionSensor = false;

    elseif find(type7Type==true) == 3
        angleSensor = true;
        weightSensor = true; 
        frictionSensor = false;

    end
    
    Q = 1;
    R = 1; 

else
    error('Wrong configuration in control type.');
end
disp('----------------------------------------------------------------------');

%% Sensor Config

% angleSensor = true;
% weightSensor = true; 
% % gSensor = true;   % it is easy and cheap to measure so that it would not be
%                     % considered anymore
% 
% velocitySensor = false;
% 
% frictionSensor = true;

%% Constraints

disp('Constraints: ');

% length = 0.23
time_lb = 0.5; % s
time_ub = 0.5; % s

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

% use X sigma principle to estimate the range of a normal distribution
Xsigma = 6;

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
%     estFriction_lb = friction_mean - friction_std;
%     estFriction_ub = friction_mean + friction_std;
    estFriction_lb = friction_lb;
    estFriction_ub = friction_ub;
    estFriction = friction;
    disp(['(SENSORED) The estimated friction: [', num2str(estFriction_lb), ', ', num2str(estFriction_ub), '].']);
else
%     estFriction_lb = (friction_mean - friction_std) * (1 - overapproxFactor);
%     estFriction_ub = (friction_mean + friction_std) * overapproximation;
    estFriction_lb = friction_lb * (1 - overapproxFactor);
    estFriction_ub = friction_ub * overapproximation;
    disp(['The estimated friction: [', num2str(estFriction_lb), ', ', num2str(estFriction_ub), '].']);

end

% estimated weight (accurate: [0.0249, 0.0251] kg)
if weightSensor == true
    estWeight_lb = weight_mean - Xsigma * weight_std;
    estWeight_ub = weight_mean + Xsigma * weight_std;
    estWeight = weight;
    disp(['(SENSORED) The estimated weight: [', num2str(estWeight_lb), ', ', num2str(estWeight_ub), '] kg.']);
else
    estWeight_lb = (weight_mean - Xsigma * weight_std) * (1 - overapproxFactor);
    estWeight_ub = (weight_mean + Xsigma * weight_std) * overapproximation;
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
    
    validationSet = estWeight_lb * estG * cos(estAngle_ub);
%     validationSet = 999;
    
    u_lb = (con_lb - estAcc_ub) /k_ub;
    u_ub = min([validationSet, (con_ub - estAcc_lb) /k_lb]);

    u = u_ub;

    disp(['The control input set: [', num2str(u), ', ', num2str(u), ...
        '] m*kg/s^2.']);
    
    % validation: how much workpieces being blown away
    validateSample = g .* weight .* cos(angle) - u;
    failedSamples = sum(validateSample < 0);
    disp(['There are ', num2str(failedSamples), '/', num2str(numSample), ...
        ' samples failed in this case.']);
    
elseif find(optimizationType==true) == 2 
    if weightSensor ~= true
        error('Weight sensor is required in case 2');
    end
    u = zeros(1, numSample);
    validationSet = zeros(1, numSample);
    for i = 1:numSample
        k_lb = estFriction_lb / estWeight(i);
        k_ub = estFriction_ub / estWeight(i);
        
        % avoid the workpiece being blown away 
        validationSet(i) = estWeight(i) * estG * cos(estAngle_ub);

        u_lb = (con_lb - estAcc_ub) /k_ub;
        u_ub = (con_ub - estAcc_lb) /k_lb;

        u(i) = min([validationSet(i), u_ub]);
        
    end
    disp('Case 2, the control sequence based on weight sensor has been derived.');
    disp(['The control input set: [', num2str(min(u)), ', ', num2str(max(u)), ...
        '] m*kg/s^2.']);

elseif find(optimizationType==true) == 3
    if angleSensor == false || weightSensor  == false
        error('Angle and weight sensors are required in case 5');
    end
    u = zeros(1, numSample);
    validationSet = zeros(1, numSample);
    estAcc_lb = zeros(1, numSample);
    estAcc_ub = zeros(1, numSample);
    for i = 1:numSample
        k_lb = estFriction_lb / estWeight(i);
        k_ub = estFriction_ub / estWeight(i);
        
        estAccSet = [estG * sin(estAngle(i)) - estFriction_lb * estG * cos(estAngle(i)), ...
                     estG * sin(estAngle(i)) - estFriction_ub * estG * cos(estAngle(i))];
        estAcc_lb(i) = max([0, min(estAccSet)]);
        estAcc_ub(i) = max(estAccSet);
        
        validationSet(i) = estWeight(i) * estG * cos(estAngle(i));

        u_lb = (con_lb - estAcc_ub(i)) /k_ub;
        u_ub = (con_ub - estAcc_lb(i)) /k_lb;

        u(i) = min([validationSet(i), u_ub]);

    end
    disp('Case 3, the control sequence based on angle and weight sensor has been derived.')
    disp(['The control input set: [', num2str(min(u)), ', ', num2str(max(u)), ...
        '] m*kg/s^2.']);
    
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
    disp(['The control input set: [', num2str(min(u)), ', ', num2str(max(u)), ...
        '] m*kg/s^2.']);
    
elseif find(optimizationType==true) == 7

    u = zeros(1, numSample);
    validationSet = zeros(1, numSample);

    if find(type7Type==true) == 1 || find(type7Type==true) == 2
        estAccSet = [estG * (sin(estAngle_ub) - estFriction_ub * cos(estAngle_ub)), ...
                         estG * (sin(estAngle_lb) - estFriction_ub * cos(estAngle_lb)), ...
                         estG * (sin(estAngle_ub) - estFriction_lb * cos(estAngle_ub)), ...
                         estG * (sin(estAngle_lb) - estFriction_lb * cos(estAngle_lb))];
            estAcc_lb = min(estAccSet);
            estAcc_ub = max(estAccSet);
    end

    for i = 1:numSample
        k_lb = estFriction_lb / estWeight(i);
        k_ub = estFriction_ub / estWeight(i);
            
        if find(type7Type==true) == 3
            estAccSet = [estG * sin(estAngle(i)) - estFriction_lb * estG * cos(estAngle(i)), ...
                     estG * sin(estAngle(i)) - estFriction_ub * estG * cos(estAngle(i))];
            estAcc_lb = min(estAccSet);
            estAcc_ub = max(estAccSet);
            
        end
        
        
        validationSet(i) = estWeight(i) * estG * cos(estAngle(i));

%         u_lb = min(-[estAcc_lb /k_ub, estAcc_ub /k_lb]);
%         u_ub = max(-[estAcc_lb /k_ub, estAcc_ub /k_lb]);
        
        uSet = -[(k_ub*Q*estAcc_ub)/(k_ub^2*Q+R), ...
                 (k_lb*Q*estAcc_ub)/(k_lb^2*Q+R), ...
                 (k_ub*Q*estAcc_lb)/(k_ub^2*Q+R), ...
                 (k_lb*Q*estAcc_lb)/(k_lb^2*Q+R)];

        u_lb = min(uSet);    
        u_ub = max(uSet);

        u(i) = min([validationSet(i), u_ub]);

    end
    disp(['Case 7, with the minimum energy consumption strategy, ', ...
        'the control sequence based on angle and weight sensor has been derived.']);
    disp(['The control input set: [', num2str(min(u)), ', ', num2str(max(u)), ...
        '] m*kg/s^2.']);

end
disp('----------------------------------------------------------------------');