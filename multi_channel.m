%% Real-Time EMG - N Sensor (Raw + Envelope) — Final Optimized
clear; clc; close all;

%% 1. Configuration
port        = "COM7";
baudrate    = 115200;
fs          = 1000;
NUM_CH      = 3;          % <-- CHANGE THIS for number of channels
f_targets   = [50, 76.5, 100, 150, 153, 200];
windowSize  = 2000;
envWinSize  = 80;
PLOT_EVERY  = 5;

% Plot colours per channel (add more if NUM_CH > 5)
rawColors = {'b', 'm', 'c', 'k', [1 0.5 0]};
envColors = {'r', 'g', [0.5 0 0.5], [0 0.5 0.5], [0.8 0.4 0]};

%% 2. Filter Design
numFilters = length(f_targets);
b_n = zeros(numFilters, 3);
a_n = zeros(numFilters, 3);

for i = 1:numFilters
    wo = f_targets(i) / (fs/2);
    [b_temp, a_temp] = iirnotch(wo, wo/35);
    b_n(i,:) = b_temp;
    a_n(i,:) = a_temp;
end

[b_b, a_b] = butter(4, [20 450]/(fs/2), 'bandpass');
bpStateLen = max(length(a_b), length(b_b)) - 1;

% Per-channel filter states
zi_n = zeros(NUM_CH, numFilters, 2);
zi_b = zeros(NUM_CH, bpStateLen);

%% 3. Circular Envelope Buffers
sqBuffer = zeros(NUM_CH, envWinSize);
envIdx   = 1;

%% 4. Serial Setup
s = serialport(port, baudrate);
configureTerminator(s, "LF");
s.Timeout = 1;
flush(s);
pause(0.5);
flush(s);

%% 5. Figure — 2 columns (Raw | Envelope), NUM_CH rows
fig = figure('Name', sprintf('EMG %d Sensor(s)', NUM_CH), 'Color', 'w');

axRaw = gobjects(NUM_CH, 1);
axEnv = gobjects(NUM_CH, 1);
hRaw  = gobjects(NUM_CH, 1);
hEnv  = gobjects(NUM_CH, 1);

for ch = 1:NUM_CH
    axRaw(ch) = subplot(NUM_CH, 2, (ch-1)*2 + 1);
    hRaw(ch)  = line(nan(1,windowSize), nan(1,windowSize), ...
                     'Color', rawColors{ch});
    grid on;
    title(sprintf('Raw A%d', ch-1));
    ylim([0 1300]);

    axEnv(ch) = subplot(NUM_CH, 2, (ch-1)*2 + 2);
    hEnv(ch)  = line(nan(1,windowSize), nan(1,windowSize), ...
                     'Color', envColors{ch});
    grid on;
    title(sprintf('Env A%d', ch-1));
    ylim([0 500]);
end

%% 6. Pre-allocate Circular Display Buffers
xBuf   = nan(1, windowSize);
rawBuf = nan(NUM_CH, windowSize);
envBuf = nan(NUM_CH, windowSize);
dispIdx = 1;

%% 7. Main Loop
count = 0;

while ishandle(fig)

    % --- Watchdog: flush if buffer is backlogging ---
    if s.NumBytesAvailable > 500
        flush(s);
        continue;
    end

    % --- Read line ---
    try
        dataStr = readline(s);
    catch
        flush(s);
        continue;
    end

    % --- Safe string conversion ---
    if isstring(dataStr)
        dataStr = char(dataStr);
    end
    dataStr = strtrim(dataStr);
    if isempty(dataStr), continue; end

    % --- Parse CSV ---
    data = split(dataStr, ',');
    if numel(data) ~= NUM_CH, continue; end

    vals = zeros(1, NUM_CH);
    for ch = 1:NUM_CH
        vals(ch) = str2double(data(ch));
    end
    if any(isnan(vals)), continue; end

    count = count + 1;

    %% --- Per-channel: Notch → Bandpass → Envelope ---
    envVals = zeros(1, NUM_CH);

    for ch = 1:NUM_CH
        x = vals(ch);

        % Notch filters
        for i = 1:numFilters
            zi_n_vec = squeeze(zi_n(ch, i, :));
            [x, zi_n_vec] = filter(b_n(i,:), a_n(i,:), x, zi_n_vec);
            zi_n(ch, i, :) = zi_n_vec;
        end

        % Bandpass filter
        zi_b_vec = zi_b(ch, :)';
        [fx, zi_b_vec] = filter(b_b, a_b, x, zi_b_vec);
        zi_b(ch, :) = zi_b_vec';

        % Envelope
        sqBuffer(ch, envIdx) = fx^2;
        envVals(ch) = sqrt(mean(sqBuffer(ch, :)));
    end

    % Advance envelope circular index
    envIdx = mod(envIdx, envWinSize) + 1;

    %% --- Write to circular display buffer ---
    xBuf(dispIdx)      = count;
    rawBuf(:, dispIdx) = vals';
    envBuf(:, dispIdx) = envVals';
    dispIdx = mod(dispIdx, windowSize) + 1;

    %% --- Plot every PLOT_EVERY samples ---
    if mod(count, PLOT_EVERY) == 0
        ord = [dispIdx:windowSize, 1:dispIdx-1];

        for ch = 1:NUM_CH
            set(hRaw(ch), 'XData', xBuf(ord), 'YData', rawBuf(ch, ord));
            set(hEnv(ch), 'XData', xBuf(ord), 'YData', envBuf(ch, ord));
        end

        if count > windowSize
            for ch = 1:NUM_CH
                xlim(axRaw(ch), [count-windowSize, count]);
                xlim(axEnv(ch), [count-windowSize, count]);
            end
        end

        drawnow limitrate;
    end
end

clear s;