%% Real-Time EMG - Raw + Envelope + Motor Control
clear; clc; close all;

% 1. Configuration
port = "COM10"; 
baudrate = 115200;      
fs = 1000;              
f_targets = [50, 76.5, 100, 150, 153, 200]; 
windowSize = 2000;      
envWinSize = 80;        
threshold = 10;        % <--- ADJUST THIS: Amplitude to trigger motor
motorState = 0;         % Current state (0 = OFF, 1 = ON)

% 2. Filter Design
numFilters = length(f_targets);
b_n = zeros(numFilters, 3); a_n = zeros(numFilters, 3);
zi_n = zeros(numFilters, 2); 
for i = 1:numFilters
    wo = f_targets(i) / (fs/2);
    [b_temp, a_temp] = iirnotch(wo, wo/35);
    b_n(i,:) = b_temp; a_n(i,:) = a_temp;
end
[b_b, a_b] = butter(4, [20 450]/(fs/2), 'bandpass');
zi_b = zeros(max(length(a_b), length(b_b)) - 1, 1);

% 3. Buffers
sqBuffer = zeros(1, envWinSize); 

% 4. Setup Serial
s = serialport(port, baudrate);
configureTerminator(s, "LF");
flush(s);

% 5. Setup Figure
fig = figure('Name', 'EMG Control System', 'Color', 'w');
ax1 = subplot(2,1,1); hRaw = animatedline('Color', 'b'); grid on;
title('Raw Sensor Data'); ylim([0, 1300]); xlim([0, windowSize]);

ax2 = subplot(2,1,2); hEnv = animatedline('Color', 'r', 'LineWidth', 2); grid on;
hold on; 
hThresh = yline(threshold, '--k', 'Threshold', 'LineWidth', 1.5); % Visual Threshold
title('RMS Envelope & Control Signal'); ylim([0, 100]); xlim([0, windowSize]);

% 6. Loop
count = 0;
while ishandle(fig)
    dataStr = readline(s);
    if ~isempty(dataStr)
        val = str2double(dataStr);
        if ~isnan(val)
            count = count + 1;
            
            % Filtering & Envelope
            currentVal = val;
            for i = 1:numFilters
                [currentVal, zi_n(i,:)] = filter(b_n(i,:), a_n(i,:), currentVal, zi_n(i,:));
            end
            [filtVal, zi_b] = filter(b_b, a_b, currentVal, zi_b);
            sqBuffer = [sqBuffer(2:end), filtVal^2];
            envVal = sqrt(mean(sqBuffer));

            % --- THRESHOLD LOGIC ---
            if envVal > threshold && motorState == 0
                write(s, '1', "char"); % Send ON command
                motorState = 1;
                hEnv.Color = [0 0.8 0]; % Change line color to green when active
            elseif envVal <= threshold && motorState == 1
                write(s, '0', "char"); % Send OFF command
                motorState = 0;
                hEnv.Color = 'r';      % Change back to red
            end

            % Update UI
            addpoints(hRaw, count, val);
            addpoints(hEnv, count, envVal);
            if count > windowSize
                xlim(ax1, [count - windowSize, count]);
                xlim(ax2, [count - windowSize, count]);
            end
            if mod(count, 25) == 0, drawnow limitrate; end
        end
    end
end
clear s;