clear all;

threshold = hours(2); %User specified threshold in hours
disp(threshold);

timenow = datetime('now');
timestamp = datetime(2019,5,22,17,0,0); %Arbitrary timestamp

duration = timenow - timestamp; %Calculates time difference in times
duration.Format = 'h'; %Converts the time into number of hours
disp(duration);

if duration > threshold
    disp('Period exceeds threshold');
end