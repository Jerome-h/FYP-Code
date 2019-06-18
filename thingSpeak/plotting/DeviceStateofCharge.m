% Author: Jerome Hallett
% Adapted from template MATLAB code for visualizing data from a channel as a 2D line

% Prior to running this MATLAB code template, assign the channel ID to read
% data from to the 'readChannelID' variable. Also, assign the field ID
% within the channel that you want to read data from to plot. 

% Channel ID to read data from:
readChannelID = [793591];
% Field ID to read data from:
fieldID1 = [1];
fieldID2 = [2];
fieldID5 = [5];

% Channel Read API Key 
readAPIKey = 'SJVVGDMGUJWU7VJL';

%% Read Data %%
samples = 30;

% Read time variable
[timeData] = thingSpeakRead(readChannelID,'OutputFormat','table', 'Field', fieldID1, 'NumPoints', samples, 'ReadKey', readAPIKey);
t = timeData.Time; %Extracts 'Time' variable data from table
time = datetime(t,'InputFormat','dd-MM-yyyy HH:mm:ss'); %Converts to datetime array

% Read ID variable
[IDData] = thingSpeakRead(readChannelID, 'Field', fieldID2, 'NumPoints', samples, 'ReadKey', readAPIKey);

% Read State of Charge variable
[SoCData] = thingSpeakRead(readChannelID, 'Field', fieldID5, 'NumPoints', samples, 'ReadKey', readAPIKey);

%% Visualize Data %%

%Create array of samples to plot
%Initialise plot arrays and index
i = 1; 
timePlot = datetime.empty;
IDPlot = double.empty;
for j = 1:samples
    if SoCData(j) == 1
        timePlot(i) = time(j);
        IDPlot(i) = IDData(j);
        i = i + 1;
    end 
end

hold on;
title('Device State of Charge');
xlabel  ('Time');
ylabel  ('Device ID');
set(gca,'ytick',0:5)
ylim  ([0 5]);
stairs(timePlot, IDPlot , '*');
