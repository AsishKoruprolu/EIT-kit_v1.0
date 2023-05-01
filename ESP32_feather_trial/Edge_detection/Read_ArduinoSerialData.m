%% Establish Connection

clear; clc;

%Desired COM port to connect to
COMPort = "COM3";

%Connect to COM port and set parameters
Handler = serialport(COMPort,115200,'Timeout', 2);
configureTerminator(Handler,"CR");
flush(Handler);
Time = 0;
StartTime = toc;
while Time<=10
    Time = toc;
    Time = Time-StartTime;
end

%% Start Measurement by sending command

%Length of time in (s) to run the measurement
StopTime = 3;
Num_measurements = 500;

%Start measurement
write(Handler,'Start',"uint8");

%Start timer and get current time
% tic;

Time = 0;
i = 1;
% while Time <= StopTime
% 
%     Temp = "";
%     while (Temp == "")
%         Temp = readline(Handler);
%         ReceivedData(i,1) = Temp;
%     end
%     i = i+1;
%     Time = toc;
% 
% end

while i <= Num_measurements

    Temp = "";
    while (Temp == "")
        Temp = readline(Handler);
        ReceivedData(i,1) = Temp;
    end
    i = i+1;
    % Time = toc;
    
end

%Stop measurement
write(Handler,'Stop',"uint8");

%Convert/process all the data
for i = 1:length(ReceivedData)
    Temp2 = split(ReceivedData(i,1));
    if length(Temp2) > 1
        Data(i,1) = str2double(Temp2(end));
    else
        Data(i,1) = str2double(Temp2);
    end
        
end

% Write Data to File
% name = datetime('now','Format','MMddyyyy-HHmmss');
% FileName = sprintf('Temp_Sensor_Data_%s.csv',name);
% writematrix(Data, FileName);

%clear the serial session
Handler = [];
%% Post Processing the obtained data
clc;

Vrefp = 2.043;
Vrefn = 1.012;
FSR = Vrefp-Vrefn;

lsb = FSR/1024;
fs = 50e3; %sampling frequency

voltages = Data*lsb+Vrefn;
x_axis = 1:1:length(voltages);

time = (x_axis-1)*1/fs;

%Plotting the ADC measured signal
figure;
plot(time,voltages)
grid on;
xlabel("Time (seconds)");
ylabel("Voltage (V)");
title("Measured Signal")

%Input signal parameters (Input from waveform generator)
offset = 1.5; %Volts
pk_pk = 0.9; %Volts

[pks,locs] = findpeaks(-voltages); %finding the troughs of the signal
voltages_pruned = voltages(locs(2):locs(end)); %pruning the signal to have points from the second trough to the last trough
x_axis_pruned = 1:1:length(voltages_pruned);
time_pruned = (x_axis_pruned-1)*1/fs;

%Plotting the pruned signal
figure;
plot(time_pruned,voltages_pruned)
grid on;
xlabel("Time (seconds)");
ylabel("Voltage (V)");
title("Measured Signal - Pruned")

% ADC output parameters
avg = mean(voltages_pruned)
meas_rms = sqrt(sum((voltages_pruned-avg).^2)/length(voltages_pruned))
meas_pk_pk = meas_rms*sqrt(2)*2

%Errors
midscale_error = offset-avg
pk_pk_error = pk_pk - meas_pk_pk
rms_ideal = pk_pk/(2*sqrt(2));
rms_error = rms_ideal-meas_rms
rms_error_percent = rms_error/rms_ideal*100

