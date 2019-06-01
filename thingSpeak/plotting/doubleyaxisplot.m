% Author: Jerome Hallett
% Adapted from template MATLAB code for visualizing data using the YYAXIS and PLOT functions.

% Prior to running this MATLAB code template, assign the channel ID to read
% data from to the 'readChannelID' variable. Also, assign the field IDs
% within the channel that you want to read data from to 'fieldID1', and
% 'fieldID2'.

% Channel ID to read data from:
readChannelID = [785277];
% Field IDs to read data from:
fieldID1 = [1];
fieldID2 = [2];
fieldID3 = [3];
fieldID4 = [4];

% Channel Read API Key 
readAPIKey = '9NF50FOSF8G6KKKW';

%% Read Data %%
%Number of desired samples to plot
samples = 30;

% Read time variable
[timedata] = thingSpeakRead(readChannelID,'OutputFormat','table', 'Field', fieldID1, 'NumPoints', samples, 'ReadKey', readAPIKey);
t = timedata.Time; %Extracts 'Time' variable data from table
time = datetime(t,'InputFormat','dd-MMM-yyyy HH:mm:ss'); %Converts to datetime array

% Read first data variable
[data1] = thingSpeakRead(readChannelID, 'Field', fieldID2, 'NumPoints', samples, 'ReadKey', readAPIKey);

% Read second data variable
[data2] = thingSpeakRead(readChannelID, 'Field', fieldID3, 'NumPoints', samples, 'ReadKey', readAPIKey);

% Read third data variable
[data3] = thingSpeakRead(readChannelID, 'Field', fieldID4, 'NumPoints', samples, 'ReadKey', readAPIKey);
%% Visualize Data %%

hold on;
title('Receiver Measurements');
xlabel('Time');

yyaxis left;
ylabel  ('Voltage (V)');
ylim([0 30])

plot(time, data1, 'b');

yyaxis left;
plot(time, data2, 'black');

yyaxis right;
ylabel ('Current (mA)');
ylim([0 3000])
plot(time, data3, 'r');

%datetick('x', 'dd-mmm-yyyy HH:MM:SS')
legend({'VRect','VCap', 'Current'}, 'Location' , 'northwest');
