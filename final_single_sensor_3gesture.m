%% Real-Time EMG - Auto-Calibration + Three-Strike Debounce Logic
clear; clc; close all;

% ==========================================
% 1. HARDWARE & FILTER CONFIGURATION
% ==========================================
port = "COM3";          % <--- CHANGE THIS IF NEEDED
baudrate = 115200;      
fs = 1000;              
f_targets = [50, 100, 150, 153, 200]; 
windowSize = 2000;      
envWinSize = 80;        
gain = 35;              

% ==========================================
% 2. AUTO-CALIBRATION & AREA THRESHOLDS
% ==========================================
calibrationSeconds = 30;           
calibSamples = calibrationSeconds * fs;
baselineOffset = 20;               

ampThreshold = 0;                  
fixedWindowSize = 600;             

% Area = Avg Amplitude * Window Size (600)
fistAreaThreshold = 120000; 
yooAreaThreshold = 25000;   

% State Variables
isCalibrating = true;
calibSum = 0;
calibCount = 0;

activeCounter = 0;         
relaxCounter = 0;
internalRelaxCounter = 0;  
currentGestureArea = 0;    

motorState = 0;            
previousProposedState = 0; 
strikeCount = 0;           % <--- NEW: Tracks consecutive matching windows

% ==========================================
% 3. FILTER DESIGN
% ==========================================
numFilters = length(f_targets);
b_n = zeros(numFilters, 3); 
a_n = zeros(numFilters, 3);
zi_n = zeros(numFilters, 2); 

for i = 1:numFilters
    f_center = f_targets(i);
    [b_temp, a_temp] = butter(1, [(f_center-1) (f_center+1)]/(fs/2), 'stop');
    b_n(i,:) = b_temp; 
    a_n(i,:) = a_temp;
end

[b_bs, a_bs] = butter(4, [70 80]/(fs/2), 'stop');
zi_bs = zeros(max(length(a_bs), length(b_bs)) - 1, 1);

[b_b, a_b] = butter(4, [20 450]/(fs/2), 'bandpass');
zi_b = zeros(max(length(a_b), length(b_b)) - 1, 1);
sqBuffer = zeros(1, envWinSize); 

% ==========================================
% 4. SETUP SERIAL & UI
% ==========================================
s = serialport(port, baudrate);
configureTerminator(s, "LF");
s.Timeout = 1; 
flush(s);

fig = figure('Name', 'EMG Gesture Classification', 'Color', 'w');

ax1 = subplot(2,1,1); 
hRaw = animatedline('Color', 'b'); 
grid on; title('Raw Sensor Data'); 
ylim([0, 1300]); xlim([0, windowSize]);

ax2 = subplot(2,1,2); 
hEnv = animatedline('Color', [0.5 0.5 0.5], 'LineWidth', 1.5); 
grid on; hold on; 

hThresh = []; 
title(ax2, ['CALIBRATING... Keep Arm Relaxed! (', num2str(calibrationSeconds), 's remaining)']); 
ylim([0, 600]); xlim([0, windowSize]);

drawnow; 

% ==========================================
% 5. MAIN LIVE LOOP
% ==========================================
count = 0;
disp('======================================');
disp('CALIBRATION STARTED: DO NOT MOVE ARM');
disp('======================================');

while ishandle(fig)
    dataStr = readline(s);
    
    if ~isvalid(hEnv) || ~ishandle(fig)
        break; 
    end
    
    if ~isempty(dataStr)
        val = str2double(dataStr);
        if ~isnan(val)
            count = count + 1;
            
            % -------- FILTERING --------
            currentVal = val;
            for i = 1:numFilters
                [currentVal, zi_n(i,:)] = filter(b_n(i,:), a_n(i,:), currentVal, zi_n(i,:));
            end
            [currentVal, zi_bs] = filter(b_bs, a_bs, currentVal, zi_bs);
            [filtVal, zi_b] = filter(b_b, a_b, currentVal, zi_b);
            
            sqBuffer = [sqBuffer(2:end), filtVal^2];
            envVal = sqrt(mean(sqBuffer)) * gain; 
            
            % ==========================================
            % PHASE 1: AUTO-CALIBRATION
            % ==========================================
            if isCalibrating
                calibSum = calibSum + envVal;
                calibCount = calibCount + 1;
                
                if mod(calibCount, fs) == 0 
                    secondsLeft = calibrationSeconds - (calibCount / fs);
                    title(ax2, ['CALIBRATING... Keep Arm Relaxed! (', num2str(secondsLeft), 's remaining)']);
                    fprintf('Calibrating... %ds remaining\n', secondsLeft);
                end
                
                if calibCount >= calibSamples
                    avgBaseline = calibSum / calibSamples;
                    ampThreshold = avgBaseline + baselineOffset; 
                    isCalibrating = false;
                    
                    disp('======================================');
                    disp(['CALIBRATION COMPLETE! Average: ', num2str(avgBaseline, '%.1f')]);
                    disp(['New Start Threshold set to: ', num2str(ampThreshold, '%.1f')]);
                    disp('System Ready. Waiting for gestures...');
                    disp('======================================');
                    
                    hThresh = yline(ax2, ampThreshold, '--k', 'Auto-Baseline', 'LineWidth', 1.5);
                    title(ax2, 'Classification: Green = FIST, Red = YOO');
                    
                    activeCounter = 0;
                    currentGestureArea = 0;
                    internalRelaxCounter = 0;
                end
                
            % ==========================================
            % PHASE 2: NORMAL GESTURE CLASSIFICATION
            % ==========================================
            else
                if (envVal > ampThreshold) || (activeCounter > 0)
                    
                    activeCounter = activeCounter + 1;
                    currentGestureArea = currentGestureArea + envVal;
                    relaxCounter = 0; 
                    
                    % Only turn yellow if we haven't confirmed a state yet
                    if motorState == 0
                        hEnv.Color = [1 0.8 0]; 
                        hEnv.LineWidth = 2.0;
                    end
                    
                    % --- EARLY ABORT CHECK (Fast Relax) ---
                    if envVal < ampThreshold
                        internalRelaxCounter = internalRelaxCounter + 1;
                    else
                        internalRelaxCounter = 0; 
                    end
                    
                    % If signal drops for 150ms, ABORT!
                    if internalRelaxCounter > 150
                        activeCounter = 0;
                        currentGestureArea = 0;
                        internalRelaxCounter = 0;
                        previousProposedState = 0; 
                        strikeCount = 0; % Reset strikes on fast relax
                        
                        if motorState ~= 0
                            write(s, '0', "char"); 
                            motorState = 0;
                            disp('--- Fast Relax -> INSTANT ZERO ---');
                        end
                        hEnv.Color = [0.5 0.5 0.5]; 
                        hEnv.LineWidth = 1.5;
                        
                    % --- NORMAL CLASSIFICATION (End of 600ms Window) ---
                    elseif activeCounter >= fixedWindowSize
                        
                        % 1. Figure out what the current window looks like
                        if currentGestureArea > fistAreaThreshold
                            currentProposedState = 1;
                            stateStr = 'FIST';
                        elseif currentGestureArea > yooAreaThreshold
                            currentProposedState = 2;
                            stateStr = 'YOO';
                        else
                            currentProposedState = 0;
                            stateStr = 'NOISE';
                        end
                        
                        fprintf('[Window] Area: %.0f | Proposed: %s ', currentGestureArea, stateStr);
                        
                        % 2. THE THREE-STRIKE LOGIC
                        if currentProposedState == previousProposedState
                            strikeCount = strikeCount + 1; % Increment if it matches
                        else
                            strikeCount = 1; % Reset to 1 if it's a new gesture
                            previousProposedState = currentProposedState;
                        end
                        
                        if strikeCount >= 3
                            % We have 3 consecutive matches!
                            if motorState ~= currentProposedState
                                motorState = currentProposedState;
                                write(s, num2str(motorState), "char"); 
                                disp(['-> STRIKE 3! Switching Motor to ', stateStr]);
                            else
                                disp(['-> (Maintained ', stateStr, ')']);
                            end
                            
                            % Update UI Colors to match CONFIRMED state
                            if motorState == 1
                                hEnv.Color = [0 0.8 0]; hEnv.LineWidth = 3.0;
                            elseif motorState == 2
                                hEnv.Color = 'r'; hEnv.LineWidth = 3.0;
                            else
                                hEnv.Color = [0.5 0.5 0.5]; hEnv.LineWidth = 1.5;
                            end
                        else
                            % Still building up to 3 strikes
                            disp(['-> STRIKE ', num2str(strikeCount), ' (Waiting for 3...)']);
                        end
                        
                        % Reset for the next contiguous window
                        activeCounter = 0;
                        currentGestureArea = 0;
                        internalRelaxCounter = 0;
                    end
                    
                else
                    % --- STATE: User is resting natively ---
                    relaxCounter = relaxCounter + 1;
                    
                    if relaxCounter > 150
                        if motorState ~= 0
                            write(s, '0', "char");
                            motorState = 0;
                            disp('--- Signal Dropped -> RELAX ---');
                        end
                        hEnv.Color = [0.5 0.5 0.5];
                        hEnv.LineWidth = 1.5;
                        relaxCounter = 0; 
                        previousProposedState = 0; 
                        strikeCount = 0; % Clear any pending strikes
                    end
                end
            end
            
            % -------- UI UPDATE --------
            addpoints(hRaw, count, val);
            addpoints(hEnv, count, envVal);
            
            if count > windowSize
                xlim(ax1, [count - windowSize, count]);
                xlim(ax2, [count - windowSize, count]);
            end
            
            if mod(count, 25) == 0
                drawnow limitrate;
            end
        end
    end
end

% --- CLEANUP ---
clear s; 
disp('Plot closed. Session ended.');