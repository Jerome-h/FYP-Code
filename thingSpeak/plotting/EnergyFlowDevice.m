% Author: Jerome Hallett
% Plots Rectifier Voltage and Power Out for a given device
% Adapted from template MATLAB code for visualizing data using the YYAXIS and PLOT functions.

% Prior to running this MATLAB code template, assign the channel ID to read
% data from to the 'readChannelID' variable. Also, assign the field IDs
% within the channel that you want to read data from to 'fieldID1', and
% 'fieldID2'.

%User Inputs
deviceID = 1; %ID of device which this plots for
samples = 20; %Number of desired samples to plot

% Channel ID to read data from:
readChannelID = [793591];
% Field IDs to read data from:
fieldID1 = [1];
fieldID2 = [2];
fieldID3 = [3];
fieldID4 = [4];

% Channel Read API Key 
readAPIKey = 'SJVVGDMGUJWU7VJL';

%% Read Data %%

% Read timestamp variable
[timeData] = thingSpeakRead(readChannelID,'OutputFormat','table', 'Field', fieldID1, 'NumPoints', samples, 'ReadKey', readAPIKey);
t = timeData.Time; %Extracts 'Time' variable data from table
time = datetime(t,'InputFormat','dd-MM-yyyy HH:mm:ss'); %Converts to datetime array

% Read first data variable - Device ID
[IDData] = thingSpeakRead(readChannelID, 'Field', fieldID2, 'NumPoints', samples, 'ReadKey', readAPIKey);

% Read second data variable - Rectifier Voltage
[voltageData] = thingSpeakRead(readChannelID, 'Field', fieldID3, 'NumPoints', samples, 'ReadKey', readAPIKey);

% Read third data variable - Power Out
[powerData] = thingSpeakRead(readChannelID, 'Field', fieldID4, 'NumPoints', samples, 'ReadKey', readAPIKey);
%% Visualize Data %%
 
%Create array of samples to plot
%Initialise plot arrays and index
i = 1; 
timePlot = datetime.empty;
voltagePlot = double.empty;
powerPlot = double.empty;
for j = 1:samples
    if IDData(j) == deviceID 
        timePlot(i) = time(j);
        voltagePlot(i) = voltageData(j);
        powerPlot(i) = powerData(j);
        i = i + 1;
    end 
end

hold on;
title('Receiver Measurements for Device 1');
xlabel('Time');
yyaxis left;
ylabel('Voltage (V)');
ylim([0 20]);
plot(timePlot, voltagePlot, 'b');
yyaxis right;
ylabel ('Power (W)');
ylim([0 10]);
plot(timePlot, powerPlot, 'r');
%datetick('x', 'dd-mm-yyyy HH:MM:SS')
legend({'Rectifier Voltage','Power Out'}, 'Location' , 'northwest');

%EOF
