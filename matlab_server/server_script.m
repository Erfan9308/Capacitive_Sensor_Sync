%% MATLAB Receiver Code for Coordinated Sensors (Cycle-Aligned Logging)

port = "COM3";
baudRate = 115200;
s = serialport(port, baudRate);

configureTerminator(s, "LF");
flush(s);

% Storage for each sensor
responses1 = [];
responses2 = [];
responses3 = [];
responses4 = [];

numSensors = 4;

while true
    
    %% ---- SEND READY COMMAND ----
    readyCmd = 'R';
    write(s, readyCmd, "uint8");

    %% ---- RECEIVE UP TO 4 FRAMES ----
    receivedCount = 0;
    maxWaitTime = 0.25;    % 250ms to get all 4 sensors
    tic;

    % Initialize placeholders for this cycle
    cycleValues = nan(1, numSensors);   % [NaN NaN NaN NaN]

    while receivedCount < numSensors && toc < maxWaitTime
        if s.NumBytesAvailable >= 2
            frame = read(s, 2, "uint8");
            receivedCount = receivedCount + 1;

            % Decode
            byte1 = frame(1);
            byte2 = frame(2);

            sensor_id = bitshift(byte1, -6);
            adc_msb = bitand(byte1, hex2dec('3F'));
            adc_value = bitshift(adc_msb, 8) + byte2;

            % Store into this cycle's data by sensor ID
            if sensor_id >= 0 && sensor_id <= 3
                cycleValues(sensor_id + 1) = adc_value;
            else
                fprintf("Invalid sensor ID: %d\n", sensor_id);
            end
        end
    end

    %% ---- Append cycle data to each sensor's vector ----
    responses1(end+1) = cycleValues(1);
    responses2(end+1) = cycleValues(2);
    responses3(end+1) = cycleValues(3);
    responses4(end+1) = cycleValues(4);

    %% ---- Print cycle summary ----
    fprintf("Cycle complete -> S1:%s  S2:%s  S3:%s  S4:%s\n", ...
        num2str(cycleValues(1)), ...
        num2str(cycleValues(2)), ...
        num2str(cycleValues(3)), ...
        num2str(cycleValues(4)));

    pause(0.5);
end
