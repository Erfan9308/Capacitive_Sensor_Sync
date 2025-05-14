%% MATLAB Receiver Code for Coordinated Sensors

% Configure serial port 
port = "COM3";              
baudRate = 115200;
s = serialport(port, baudRate);

% Optional: configure terminator and flush any data in the buffer
configureTerminator(s, "LF");
flush(s);

% Infinite loop: send global ready command and collect sensor responses
while true
    % --- Send Global "Ready" Command ---
    % The ready command is a 2-byte message:
    %   Byte 1: 0xA0 (indicates "ready")
    %   Byte 2: 0x00 (dummy byte)
    readyCmd = [hex2dec('A0'), hex2dec('00')];
    write(s, readyCmd, "uint8");
    
    % --- Collect Sensor Responses ---
    % Each sensor will respond with a 2-byte frame.
    % Set a timeout (in seconds) for receiving responses.
    responses = [];
    timeout = 0.5;  % seconds
    tic;
    while toc < timeout
        % Check if at least 2 bytes are available
        if s.NumBytesAvailable >= 2
            % Read 2 bytes as an unsigned 8-bit integer array
            data = read(s, 2, "uint8");
            responses = [responses; data];  % store the response
        end
    end
    
    % --- Process and Display Sensor Data ---
    if ~isempty(responses)
        % Loop through each 2-byte response
        for idx = 1:size(responses, 1)
            byte1 = responses(idx, 1);
            byte2 = responses(idx, 2);
            % Decode sensor ID from high nibble of the first byte
            sensor_id = bitshift(byte1, -4);
            % Extract ADC value:
            %   Lower 4 bits of first byte = ADC value bits [11:8]
            %   Second byte = ADC value bits [7:0]
            adc_msb = bitand(byte1, hex2dec('0F'));
            adc_value = bitshift(adc_msb, 8) + byte2;
            fprintf("Sensor %d: ADC Value = %d\n", sensor_id, adc_value);
        end
    else
        fprintf("No sensor response received.\n");
    end
    
    % Pause before sending the next ready command (adjust as needed)
    pause(1);  
end
