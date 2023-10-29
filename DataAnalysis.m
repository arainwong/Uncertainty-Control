% clc; 
close all;

%%% 
% Assume the data can be generated by "DataSampling.m".
% With the data of ddot_x, dot_x and x, the estimation of the parameter
% is available.
%
% Parameter: g, angle, friction, weight
% constraint: dot_x, t
%%%
tic;
%%
% length = 0.23;

% load the data from simulink model
x_out = out.x.Data;
dot_x_out = out.dot_x.Data;
ddot_x_out = out.ddot_x.Data;
t = out.tout;

[step, numSample] = size(x_out);

% copy for later which used for data fix
x = x_out;
dot_x = dot_x_out;
ddot_x = zeros(step, numSample);

%% Check how many sample failed
% definition of "fail": can not 
numFailed = sum(dot_x(end, :)<length);
disp('----------------------------------------------------------------------');
if numFailed > 0
    disp(['ATTENTION: ', num2str(numFailed), '/', num2str(numSample), ...
        ' samples are failed.']);
    disp('The possible reason could be excessive friction or a longer required time.');
    
else
    disp(['All ', num2str(numSample), ' samples succeed.']);
    
end
disp('----------------------------------------------------------------------');

%% Fix the relation between distance and velocity

indexForFinalVelocity = zeros(1, numSample);
% find the specific time when the workpiece reach the end
endTime = zeros(1, numSample); 

% fix the data -> consider the phsical limit
for i = 1:numSample
    % find the index when the workpiece reached the end
    temp = find(x_out(:, i)>=length, 1);
    ddot_x(:, i) = ddot_x_out(i);
    if temp ~= 0 
        indexForFinalVelocity(i) = temp;
        x(indexForFinalVelocity(i):end, i) = length;
        dot_x(indexForFinalVelocity(i):end, i) = dot_x_out(indexForFinalVelocity(i), i);
        endTime(i) = t(indexForFinalVelocity(i));
    else % workpiece does not move or has not reach the end yet
        indexForFinalVelocity(i) = -1;
        x(:, i) = 0;
        dot_x(:, i) = 0;
        ddot_x(:, i) = 0;
        endTime(i) = 0;
    end
end

%% Visualization
nbins = 50;
plotRow = 3;
plotCol = 2;
numPlot = 1;

subplot(plotRow,plotCol,numPlot)
plot(t(1:max(indexForFinalVelocity)), x(1:max(indexForFinalVelocity),:));
title('distance x');
xlabel('t'); ylabel('x');
axis([0 stopTime 0 length]);
numPlot = numPlot + 1;

subplot(plotRow,plotCol,numPlot)
scatter(endTime, dot_x(max(indexForFinalVelocity), :));
title('velocity v');
xlabel('t'); ylabel('v');
axis([0 stopTime -inf inf]);
numPlot = numPlot + 1;


subplot(plotRow,plotCol,numPlot)
plot(t(1:max(indexForFinalVelocity)), dot_x(1:max(indexForFinalVelocity),:));
title('velocity v');
xlabel('t'); ylabel('v');
axis([0 stopTime -inf inf]);
numPlot = numPlot + 1;

subplot(plotRow,plotCol,numPlot)
histogram(removeZero(dot_x(max(indexForFinalVelocity),:)), nbins);
title('velocity v');
xlabel('v'); ylabel('the number of samples');
numPlot = numPlot + 1;

subplot(plotRow,plotCol,numPlot)
% plot(t(1:max(indexForFinalVelocity)), ddot_x(1:max(indexForFinalVelocity),:));
% title('acceleration a');
% xlabel('t'); ylabel('a');
histogram(endTime, nbins);
title('the number of samples reached the end in certain time scale');
xlabel('t'); ylabel('the number of samples');
axis([0 stopTime -inf inf]);
numPlot = numPlot + 1;

subplot(plotRow,plotCol,numPlot)
histogram(removeZero(ddot_x(max(indexForFinalVelocity),:)), nbins);
title('acceleration a');
xlabel('a'); ylabel('the number of samples');
numPlot = numPlot + 1;

% velocity
maxVelocity = max(max(dot_x));
% minVelocityIndex = find(dot_x(end, :)>0);
% minVelocity = min(dot_x(end, minVelocityIndex));
minVelocity = min(dot_x(end, :));
% acceleration
maxAcc = max(ddot_x_out);
% minAccIndex = find(ddot_x(end, :)>0);
% minAcc = min(ddot_x(end, minVelocityIndex));
minAcc = min(ddot_x_out);

% min/max value analysis
disp(['In the ', num2str(numSample - numFailed), ' successful samples: '])

% velocity
disp(['The velocity range of the samples is : [', ...
    num2str(minVelocity), ', ', num2str(maxVelocity), '] m/s,']);
disp(['             the slowest takes ', ...
    num2str(t(max(indexForFinalVelocity))), 's to reach the end.']);

% acceleration
disp(['The acceleration range of the samples is : [', ...
    num2str(minAcc), ', ', num2str(maxAcc), '] m/s^2.']);
disp('----------------------------------------------------------------------');

 
toc;
