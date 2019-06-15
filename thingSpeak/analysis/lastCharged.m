%Author: Jerome Hallett
%Checks period of time between now and last time of charge for each device. If this exceeds 
%a user specified threshold, then it will send a command to TalkBack app, using the ID as the command

%User Specified Values
devices = 2; % Number of devices in the network
threshold = hours(4); %Threshold before device requires recharging (hours)
samples = 10; %Number of samples code checks

% TalkBack App setup
TalkBack_ID = '33087'; 
TalkBack_apikey = '94TNPBP06I9IKFXX'; 

%Channel ID to read data from:
readChannelID = [793591];
%Channel Read API Key:
readAPIKey = 'SJVVGDMGUJWU7VJL';

%Channel ID to write data to:
writeChannelID = [793591];
%Channel write API Key:
writeAPIKey = 'HGCRWQCN36SZJNGL';

%Fields of Channel
fieldID1 = [1];
fieldID2 = [2];
fieldID5 = [5];

% Read time data
[timeData] = thingSpeakRead(readChannelID,'OutputFormat','table', 'Field', fieldID1, 'NumPoints', samples, 'ReadKey', readAPIKey);
t = timeData.Time; %Extracts 'Time' variable data from table
time = datetime(t,'InputFormat','dd-MM-yyyy HH:mm:ss'); %Converts to datetime array

% Read ID data
[IDData] = thingSpeakRead(readChannelID, 'Field', fieldID2, 'NumPoints', samples, 'ReadKey', readAPIKey);

% Read State of Charge data
[SoCData] = thingSpeakRead(readChannelID, 'Field', fieldID5, 'NumPoints', samples, 'ReadKey', readAPIKey);

%% Analysis of Data %%

%Create array of entries with State of Charge == 1
%Initialise charged arrays and index
SoCIndex = 0; 
timeCharged = datetime.empty;
IDCharged = double.empty;
for j = 1:samples
    if SoCData(j) == 1
        SoCIndex = SoCIndex + 1;
        timeCharged(SoCIndex) = time(j);
        IDCharged(SoCIndex) = IDData(j);
    end 
end

%For loop to cycle through device IDs
for i = 1:devices
    deviceFlag = false; %Flags if Device ID is within sample set
    
    %Sets up TalkBack app command url
    url =  strcat('https://api.thingspeak.com/talkbacks/',TalkBack_ID,'/commands.json');
    options = weboptions('RequestMethod','post');
    
    %Decrementing for loop to cycle through latest received samples
    for j = SoCIndex:-1:1 
        disp(timeCharged(j));
        
        %Checks if sample's ID corresponds to ID of current iteration
        if IDCharged(j) == i
            timenow = datetime('now');
            duration = timenow - timeCharged(j); %Calculates time difference between sample's timestamp and now
            duration.Format = 'h'; %Converts the duration into number of hours
            disp(duration);
            deviceFlag = true; % Device ID present in sample set
            
          %Compares if duration has exceeded user specified threshold
          if duration > threshold
            disp('Duration for device ' + string(i) + ' exceeds threshold');
            disp("Updating TalkBack App with command to charge ID ...") ;
            %Sends a command to the TalkBack app. Command string is ID of device (Could be changed to GPS Coordinates)
            data = webread(url,'api_key',TalkBack_apikey,'command_string',i,options);
          end
          %Breaks when sample's ID corresponds to the ID of the current iteration
          break;
        end 
    end
    
    %If device not in sample set, flag remains false. Needs to be scheduled for charging as last time of charging is unknown
    if deviceFlag == false
        %Sends a command to the TalkBack app. Command is ID of device
        data = webread(url,'api_key',TalkBack_apikey,'command_string',i,options);
    end
end

