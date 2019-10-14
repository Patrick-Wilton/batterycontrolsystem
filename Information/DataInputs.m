close all;
%% Reading Data Inputs and Parameters
data = onedayexport;
dt = 5; % minutes

load = onedayexport(:,1);
load = table2array(load);
load = load.';
solar = onedayexport(:,2);
solar = table2array(solar);
solar = solar.';

%% Matrix Modifying

% Days Concatenating
Days = 7;
OGload = load;
OGsolar = solar;
for i = 1:Days - 1
    load = [load,OGload];
    solar = [solar,OGsolar];
end
sizeDiff = size(load,2)/size(onedayexport,1);
    
% Scale and Noise with Conditions
Limit = 0.05;
ScaleMaxLimit = 0.7;
ScaleMinLimit = 0.1;
ScaleDiff = ScaleMaxLimit - ScaleMinLimit;
smoothScale = 72;

% Direction Implementaion
dir = 0;
if dir == -1
    ScaleArray = (rand(1,size(load,2)/smoothScale))*-ScaleDiff + (1 -ScaleMinLimit);
elseif dir == 1
    ScaleArray = (rand(1,size(load,2)/smoothScale))*ScaleDiff + (1 + ScaleMinLimit);
else
    ScaleArray = (rand(1,size(load,2)/smoothScale) - 0.5)*ScaleMaxLimit*2 + 1;
end
ScaleArray = kron(ScaleArray,ones(1,smoothScale));


% Moving Average Filter
WS = 5;
b = (1/WS)*ones(1,WS);
a = 1;
ScaleArray = filter(b,a,ScaleArray);

% Scale and Noise Addition
load = load.*ScaleArray;
load = load + (rand(1,size(load,2)) - 0.5) * (2*Limit);
solar = solar.*ScaleArray;
solar = solar + (rand(1,size(load,2)) - 0.5) * (2*Limit);

% Logical Conditions
PeakLoad = 8; % kW
PeakSolar = -6; % kW
load(load<0) = 0;
load(load>PeakLoad) = PeakLoad;
solar(solar>0) = 0;
solar(solar<PeakSolar) = PeakSolar;
    

%% Writing Functionality

headers = {'aloadp','asolarp'};
totalMat = zeros(size(load,2), 2);
totalMat(:,1) = load(:);
totalMat(:,2) = solar(:);
csvwrite_with_headers('C:\Users\Owner\PycharmProjects\batterycontrolsystem\Code\data_input.csv',totalMat,headers);


%% Plotting Functionality

x = linspace(1,size(load,2),size(load,2));
x = x./(60/dt);
x = x./24;
figure;
subplot(2,2,1);
plot(x,load);
title('Load');
ylabel('kW');
xlabel('Days');
subplot(2,2,2);
plot(x,solar);
title('Solar');
ylabel('kW');
xlabel('Days');
subplot(2,2,3);
plot(x,ScaleArray);
title('Random Scales')

