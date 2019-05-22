 %Checks period of time between now and last time of charge for each device. If this exceeds 
%a user specified threshold, then it will send a command to TalkBack app, using the ID as the command

%User Specified Values
devices = 1; % Number of devices in the network
threshold = hours(4); %User specified threshold before device requires recharging (hours)
samples = 10; %Number of samples code checks

% TalkBack App setup
TalkBack_ID = '33087'; 
TalkBack_apikey = '94TNPBP06I9IKFXX'; 

%Fields of Channel
fieldID1 = [1];
fieldID2 = [2];

% TODO - Replace the [] with channel ID to read data from:
readChannelID = [786238];
% TODO - Enter the Read API Key between the '' below:
readAPIKey = 'P5M089D9VZBO8CI2';

% TODO - Replace the [] with channel ID to write data to:
writeChannelID = [786238];
% TODO - Enter the Write API Key between the '' below:
writeAPIKey = 'WNDHKD0FYTLX4GCU';

% Read time data
[timedata] = thingSpeakRead(readChannelID,'OutputFormat','table', 'Field', fieldID1, 'NumPoints', samples, 'ReadKey', readAPIKey);
t = timedata.Time; %Extracts 'Time' variable data from table
time = datetime(t,'InputFormat','dd-MMM-yyyy HH:mm:ss'); %Converts to datetime array

% Read ID data
[IDdata] = thingSpeakRead(readChannelID, 'Field', fieldID2, 'NumPoints', samples, 'ReadKey', readAPIKey);

%% Analysis of Data %%

for i = 1:devices
    deviceflag = false; %Flags if Device ID is within sample set
    
    %Sets up TalkBack app command url
    url =  strcat('https://api.thingspeak.com/talkbacks/',TalkBack_ID,'/commands.json');
    options = weboptions('RequestMethod','post');
    
    for j = samples:-1:1 %Decrementing for loop to compare latest received samples
        disp(time(j));
        if IDdata(j) == i
            timenow = datetime('now');
            duration = timenow - time(j); %Calculates time difference between timestamp and now
            duration.Format = 'h'; %Converts the duration into number of hours
            disp(duration);
            deviceflag = true; % Device ID present in sample set
          if duration > threshold
            disp('Duration for device ' + string(i) + ' exceeds threshold');
            disp("Updating TalkBack App with command to charge ID ...") ;
            
            %Sends a command to the TalkBack app. Command is ID of device
            data = webread(url,'api_key',TalkBack_apikey,'command_string',i,options)
          end
          break;
        end 
    end
    
    %If device not in sample set, flag remains false. Needs to be scheduled for charging as last time of charging is unknown
    if deviceflag == false
        %Sends a command to the TalkBack app. Command is ID of device
        data = webread(url,'api_key',TalkBack_apikey,'command_string',i,options)
    end
end

