clear;
clc;

%% Serial configuration
portName = "COM8";
baudRate = 115200;

numberOfSamples = 100;    % Number of request rounds
roundTimeout = 0.20;      % Maximum waiting time per round, seconds
interRoundDelay = 0.010;  % Optional delay between rounds, seconds

requestByte = uint8('r');

%% Open serial port
disp("Available serial ports:");
disp(serialportlist("available"));

xbee = serialport(portName, baudRate);

flush(xbee, "input");
flush(xbee, "output");

%% Four transmitter arrays
%
% Every array has one entry per request round.
% A missing response is stored as NaN.
%
transmitter0Values = NaN(numberOfSamples, 1);
transmitter1Values = NaN(numberOfSamples, 1);
transmitter2Values = NaN(numberOfSamples, 1);
transmitter3Values = NaN(numberOfSamples, 1);

%% Arrival-time storage for debugging
transmitter0ArrivalMs = NaN(numberOfSamples, 1);
transmitter1ArrivalMs = NaN(numberOfSamples, 1);
transmitter2ArrivalMs = NaN(numberOfSamples, 1);
transmitter3ArrivalMs = NaN(numberOfSamples, 1);

%% Diagnostic counters
incompleteRoundCount = 0;
duplicatePacketCount = 0;
unexpectedPacketCount = 0;

%% Acquisition loop
for roundIndex = 1:numberOfSamples

    % Remove any delayed bytes from the previous round.
    flush(xbee, "input");

    %% Send one broadcast request
    write(xbee, requestByte, "uint8");

    roundTimer = tic;

    % Index mapping:
    % packetReceived(1) -> ID 0
    % packetReceived(2) -> ID 1
    % packetReceived(3) -> ID 2
    % packetReceived(4) -> ID 3
    packetReceived = false(1, 4);

    fprintf("\n----- Round %d -----\n", roundIndex);

    %% Receive packets until all four arrive or timeout occurs
    while ~all(packetReceived) && toc(roundTimer) < roundTimeout

        if xbee.NumBytesAvailable >= 2

            %% Read one two-byte packet
            receivedBytes = read(xbee, 2, "uint8");

            highByte = uint16(receivedBytes(1));
            lowByte  = uint16(receivedBytes(2));

            %% Reconstruct big-endian 16-bit packet
            packet = bitor( ...
                bitshift(highByte, 8), ...
                lowByte);

            %% Extract packet fields
            transmitterID = double( ...
                bitand(bitshift(packet, -14), uint16(3))); 

            sensorValue = double( ...
                bitand(packet, uint16(hex2dec("3FFF")))) / 10; % reconstructing the actual capacitance

            arrivalMs = 1000 * toc(roundTimer);

            % MATLAB index corresponding to IDs 0...3.
            transmitterIndex = transmitterID + 1;

            %% Ignore duplicate responses from the same transmitter
            if packetReceived(transmitterIndex)

                duplicatePacketCount = duplicatePacketCount + 1;

                fprintf( ...
                    "Duplicate ignored: ID=%d, value=%f\n", ...
                    transmitterID, ...
                    sensorValue);

                continue;
            end

            %% Mark this transmitter as received
            packetReceived(transmitterIndex) = true;

            %% Store according to transmitter ID
            switch transmitterID

                case 0
                    transmitter0Values(roundIndex) = sensorValue;
                    transmitter0ArrivalMs(roundIndex) = arrivalMs;

                case 1
                    transmitter1Values(roundIndex) = sensorValue;
                    transmitter1ArrivalMs(roundIndex) = arrivalMs;

                case 2
                    transmitter2Values(roundIndex) = sensorValue;
                    transmitter2ArrivalMs(roundIndex) = arrivalMs;

                case 3
                    transmitter3Values(roundIndex) = sensorValue;
                    transmitter3ArrivalMs(roundIndex) = arrivalMs;

                otherwise
                    % This should not occur because the ID field is two bits.
                    unexpectedPacketCount = unexpectedPacketCount + 1;
            end

            fprintf( ...
                "ID=%d, bytes=%02X %02X, packet=0x%04X, " + ...
                "value=%5.1f, arrival=%7.2f ms\n", ...
                transmitterID, ...
                receivedBytes(1), ...
                receivedBytes(2), ...
                packet, ...
                sensorValue, ...
                arrivalMs);

        else
            % Prevent MATLAB from continuously using all CPU time.
            pause(0.001);
        end
    end

    %% Report missing transmitters
    %
    % No data-storage decision is made here.
    % Values from received transmitters are already stored.
    % Missing values remain NaN.
    %
    if ~all(packetReceived)

        incompleteRoundCount = incompleteRoundCount + 1;

        missingIDs = find(~packetReceived) - 1;

        fprintf( ...
            "Round %d timed out. Missing IDs: %s\n", ...
            roundIndex, ...
            mat2str(missingIDs));
    else
        fprintf("Round %d complete.\n", roundIndex);
    end

    %% Print the stored round
    fprintf( ...
        "Stored: ID0=%s, ID1=%s, ID2=%s, ID3=%s\n", ...
        valueText(transmitter0Values(roundIndex)), ...
        valueText(transmitter1Values(roundIndex)), ...
        valueText(transmitter2Values(roundIndex)), ...
        valueText(transmitter3Values(roundIndex)));

    % Any incomplete bytes left after the timeout will be removed at
    % the start of the next round.
    pause(interRoundDelay);
end

%% Combined matrix for convenient processing
%
% Columns:
%   1 = ID 0
%   2 = ID 1
%   3 = ID 2
%   4 = ID 3
%
sensorValues = [ ...
    transmitter0Values, ...
    transmitter1Values, ...
    transmitter2Values, ...
    transmitter3Values];

arrivalTimeMs = [ ...
    transmitter0ArrivalMs, ...
    transmitter1ArrivalMs, ...
    transmitter2ArrivalMs, ...
    transmitter3ArrivalMs];

%% Reception masks
transmitter0Received = ~isnan(transmitter0Values);
transmitter1Received = ~isnan(transmitter1Values);
transmitter2Received = ~isnan(transmitter2Values);
transmitter3Received = ~isnan(transmitter3Values);

%% Display summary
fprintf("\n==============================\n");
fprintf("Acquisition complete.\n");
fprintf("Rounds recorded:      %d\n", numberOfSamples);
fprintf("Incomplete rounds:    %d\n", incompleteRoundCount);
fprintf("Duplicate packets:    %d\n", duplicatePacketCount);
fprintf("Unexpected packets:   %d\n", unexpectedPacketCount);

fprintf("ID 0 received:        %d/%d\n", ...
    nnz(transmitter0Received), numberOfSamples);

fprintf("ID 1 received:        %d/%d\n", ...
    nnz(transmitter1Received), numberOfSamples);

fprintf("ID 2 received:        %d/%d\n", ...
    nnz(transmitter2Received), numberOfSamples);

fprintf("ID 3 received:        %d/%d\n", ...
    nnz(transmitter3Received), numberOfSamples);

%% Save results
save( ...
    "four_transmitter_results.mat", ...
    "transmitter0Values", ...
    "transmitter1Values", ...
    "transmitter2Values", ...
    "transmitter3Values", ...
    "transmitter0Received", ...
    "transmitter1Received", ...
    "transmitter2Received", ...
    "transmitter3Received", ...
    "sensorValues", ...
    "arrivalTimeMs", ...
    "incompleteRoundCount", ...
    "duplicatePacketCount", ...
    "unexpectedPacketCount", ...
    "baudRate", ...
    "roundTimeout");

%% Close serial port
clear xbee;

%% Local helper function
function text = valueText(value)

    if isnan(value)
        text = "NaN";
    else
        text = sprintf("%d", value);
    end

end