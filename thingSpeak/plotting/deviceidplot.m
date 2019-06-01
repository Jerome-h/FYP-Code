% Author: Jerome Hallett
% Adapted from template MATLAB code for visualizing data from a channel as a 2D line

% Prior to running this MATLAB code template, assign the channel ID to read
% data from to the 'readChannelID' variable. Also, assign the field ID
% within the channel that you want to read data from to plot. 

% Channel ID to read data from:
readChannelID = [786238];
% Field ID to read data from:
fieldID1 = [1];
fieldID2 = [2];

% Channel Read API Key 
readAPIKey = 'P5M089D9VZBO8CI2';

%% Read Data %%
samples = 10;

% Read time variable
[timedata] = thingSpeakRead(readChannelID,'OutputFormat','table', 'Field', fieldID1, 'NumPoints', samples, 'ReadKey', readAPIKey);
t = timedata.Time; %Extracts 'Time' variable data from table
time = datetime(t,'InputFormat','dd-MMM-yyyy HH:mm:ss'); %Converts to datetime array

% Read ID variable
[data] = thingSpeakRead(readChannelID, 'Field', fieldID2, 'NumPoints', samples, 'ReadKey', readAPIKey);

%% Visualize Data %%
title('Device State of Charge');
xlabel  ('Time');
ylabel  ('Device ID');
stairs(time, data , '*');
