%% Safia Adair - May 2025

clear;
close all;
clc;
%% Data import

% Import the data
SensorData = Sensor_data('SensorDataLevel03.csv');
MocapData = Mocap_data('AdairFGC2 LevelIMU03.csv', [6, Inf]);

% Extract time and distance data 
Sensor_time = SensorData{:,1}/1000; % Convert ms to s
ToeSensor_distance = SensorData{:,"DistanceSensor2mm"} - 32; % 32 mm is measured dist from sensor to ground
HeelSensor_distance = SensorData{:,"DistanceSensor1mm"} - 40; % 40 mm is measured dist from sensor to ground
IMU_pitch = SensorData{:, "Pitchdeg"};
IMU_roll = SensorData{:, "Rolldeg"};

Mocap_time = MocapData{:,1}/100; % Convert to s
HeelMocap_distance = MocapData{:,"Z"};
ToeMocap_distance = MocapData{:,"Z3"};

% Remove invalid numerical values
for i = 1:length(ToeMocap_distance)
    if ToeMocap_distance(i) < -27 || HeelMocap_distance(i) < -25 % adjust toe and heel thresholds to fit data
        ToeMocap_distance(i) = NaN;
    end
end


% Remove NaN values with isfinite
valid_sensor = isfinite(Sensor_time) & isfinite(ToeSensor_distance) ...
    & isfinite(HeelSensor_distance) & isfinite(IMU_pitch) & isfinite(IMU_roll);
Sensor_time = Sensor_time(valid_sensor);
ToeSensor_distance = ToeSensor_distance(valid_sensor);
HeelSensor_distance = HeelSensor_distance(valid_sensor);
IMU_pitch = IMU_pitch(valid_sensor);
IMU_roll = IMU_roll(valid_sensor);

valid_mocap = isfinite(Mocap_time) & isfinite(ToeMocap_distance) & isfinite(HeelMocap_distance);
Mocap_time = Mocap_time(valid_mocap);
ToeMocap_distance = ToeMocap_distance(valid_mocap);
HeelMocap_distance = HeelMocap_distance(valid_mocap);

%% Plot uncorrected Sensor data

figure(1);
subplot(2,1,1);
plot(Sensor_time, ToeSensor_distance, 'b', 'LineWidth', 1.5, 'DisplayName', 'SensorToeDistance');
grid on;
title('Uncorrected Toe Sensor Data');
xlabel('Time (s)');
ylabel('Distance (mm)');
hold off;

subplot(2,1,2);
plot(Sensor_time, HeelSensor_distance, 'b', 'LineWidth', 1.5, 'DisplayName', 'SensorToeDistance');
grid on;
title('Uncorrected Heel Sensor Data');
xlabel('Time (s)');
ylabel('Distance (mm)');
hold off;

%% Correct IMU angles and drift

% Zero pitch and roll based on start position
IMU_roll = IMU_roll - 4.34;
IMU_pitch = IMU_pitch + 10.38;

% Find peaks and troughs in Toe and Heel sensor distance
[troughVals_toe, troughLocs_toe] = findpeaks(-ToeSensor_distance, Sensor_time, 'MinPeakProminence', 2);
[troughVals_heel, troughLocs_heel] = findpeaks(-HeelSensor_distance, Sensor_time, 'MinPeakProminence', 2);

% Combine and sort peak/trough times
refTimes = sort(troughLocs_heel);

% Find corresponding indices
refIndices = arrayfun(@(x) find(abs(Sensor_time - x) == min(abs(Sensor_time - x)), 1), refTimes);

% Extract the drifted values at reference times (when true value should be zero)
pitchOffsets = IMU_pitch(refIndices);
rollOffsets  = IMU_roll(refIndices);

% Interpolate drift offsets over entire time vector
pitchDriftEstimate = interp1(Sensor_time(refIndices), pitchOffsets, Sensor_time, 'linear', 'extrap');
rollDriftEstimate  = interp1(Sensor_time(refIndices),  rollOffsets, Sensor_time, 'linear', 'extrap');

% Subtract drift
pitchCorrected = IMU_pitch - pitchDriftEstimate;
rollCorrected  = IMU_roll - rollDriftEstimate;

% Convert IMU angles to radians
pitch_angle = deg2rad(pitchCorrected);
roll_angle = deg2rad(rollCorrected);

% Print corrected IMU angles
figure(2);
plot(Sensor_time, pitch_angle, 'b','LineWidth', 1.5, 'DisplayName', 'Corrected roll');
hold on;
plot(Sensor_time, roll_angle, 'r','LineWidth', 1.5, 'DisplayName', 'Corrected pitch');
legend('Location', 'best');


%% Apply foot angle correction

% Combined correction factor based on pitch and roll
correction_factor = cos(roll_angle) .* cos(pitch_angle);

% Define thresholds and exclusion window
threshold_heel = 60;     % mm, heel lifted
threshold_toe = 35;      % mm, toe near ground
threshold_angle = -0.5; % radians, foot pitched downward
exclusion_period = 0.4;  % seconds

% Estimate sample rate
sample_rate = 1 / mean(diff(Sensor_time));
exclusion_samples = round(exclusion_period * sample_rate);

% Initialize toe correction mask (default: apply correction)
toe_correction_mask = ones(size(Sensor_time));

% Detect events where toe is down, heel is up, and foot is pitched downward
% Note: this will return a logical vector
heel_up_toe_down = (HeelSensor_distance > threshold_heel) & ...
                   (ToeSensor_distance < threshold_toe) & ...
                   (roll_angle < threshold_angle);

% Get indices of those events
event_indices = find(heel_up_toe_down);

% For each event, turn off correction  during the event + exclusion window
for i = 1:length(event_indices)
    idx = event_indices(i);
    start_idx = max(1, idx);
    end_idx = min(length(Sensor_time), idx + exclusion_samples);
    
    % Disable correction in the mask during the exclusion window
    toe_correction_mask(start_idx:end_idx) = 0;
end

% Apply correction only where mask is enabled
toe_corrected_indices = toe_correction_mask == 1;
ToeSensor_distance(toe_corrected_indices) = ToeSensor_distance(toe_corrected_indices) .* correction_factor(toe_corrected_indices);

% Heel sensor always gets corrected
HeelSensor_distance = 0.65*HeelSensor_distance .* correction_factor;


% Plot results
figure(3);
subplot(2,1,1);
plot(Sensor_time, ToeSensor_distance, 'b', 'LineWidth', 1.5, 'DisplayName', 'Corrected Toe Sensor Data');
grid on;
title('Selectively Corrected Toe Sensor Data (No Correction in Yellow Regions)');
xlabel('Time (s)');
ylabel('Distance (mm)');
hold off;

subplot(2,1,2);
plot(Sensor_time, HeelSensor_distance, 'b', 'LineWidth', 1.5, 'DisplayName', 'Corrected Heel Sensor Data');
hold on;
plot(Sensor_time, IMU_pitch, 'LineWidth', 1.5);
hold on;
plot(Sensor_time, IMU_roll, 'LineWidth', 1.5);
grid on;
title('Corrected Heel Sensor Data');
xlabel('Time (s)');
ylabel('Distance (mm)');
hold off;

%% Plot unfiltered MOCAP data

figure(4);
subplot(2,1,1);
plot(Mocap_time, ToeMocap_distance, 'b', 'LineWidth', 1.5, 'DisplayName', 'Mocap Toe Distance');
grid on;
title('Unfiltered Toe Mocap Data');
xlabel('Time (s)');
ylabel("Distance (mm)");
legend('Location', 'best');
hold off;

subplot(2,1,2);
plot(Mocap_time, HeelMocap_distance, 'b', 'LineWidth', 1.5, 'DisplayName', 'Mocap Heel Distance');
grid on;
title('Unfiltered Heel Mocap Data');
xlabel('Time (s)');
ylabel('Distance (mm)')
legend('Location', 'best');
hold off;


%% Apply filter and remove irregular data points
% This section requires user adjustment
%   - deciding what filter to use 
%   - inputting best value for fc
%   - writing equation for fit_wave in manual filter (if needed)
% METHOD 1: High-pass butterworth filter 
% METHOD 2: Manual filter (fit a sin wave and subtract from signal)

% Save unfiltered data for comparison
ToeMocap_unfiltered = ToeMocap_distance;
HeelMocap_unfiltered = HeelMocap_distance;

butterworthFilter = false;  % Set true for METHOD 1, false for METHOD 2.
manualFilter = true;  % Set false for METHOD 1, true for METHOD 2.

fs = 1 / mean(diff(Mocap_time));  % Sampling frequency in Hz (adjust to your data)
fc = 1/2.5;  % Cutoff frequency in Hz (adjust based on sine wave frequency)

% Butterworth filter
if butterworthFilter == true
    [b, a] = butter(4, fc/(fs/2), 'high');

    % Apply the filter
    ToeMocap_distance = filtfilt(b, a, ToeMocap_distance);
    HeelMocap_distance = filtfilt(b, a, HeelMocap_distance);
end 

% Manual filter
if manualFilter == true
    y = 90*sin(Mocap_time*fc - 6.25) + 50;
    ToeMocap_distance = ToeMocap_unfiltered - y;
    HeelMocap_distance = HeelMocap_unfiltered - y;
end

% Remove invalid numerical values
for i = 1:length(ToeMocap_distance)
    if ToeMocap_distance(i) < -27 || HeelMocap_distance(i) < -25 % adjust toe and heel thresholds to fit data
        ToeMocap_distance(i) = NaN;
    end
end

valid_mocap = isfinite(Mocap_time) & isfinite(ToeMocap_distance) & isfinite(HeelMocap_distance);

Mocap_time = Mocap_time(valid_mocap);
ToeMocap_distance = ToeMocap_distance(valid_mocap);
HeelMocap_distance = HeelMocap_distance(valid_mocap);
ToeMocap_unfiltered = ToeMocap_unfiltered(valid_mocap);
HeelMocap_unfiltered = HeelMocap_unfiltered(valid_mocap);

%% Plot filtered data for visual validation

figure(5);
subplot(2,1,1);
plot(Mocap_time, ToeMocap_distance, 'b', 'LineWidth', 1.5, 'DisplayName', 'Mocap Toe Distance');
grid on;
title('Filtered Toe Mocap Data');
xlabel('Time (s)');
ylabel("Distance (mm)");
legend('Location', 'best');
hold off;

subplot(2,1,2);
plot(Mocap_time, HeelMocap_distance, 'b', 'LineWidth', 1.5, 'DisplayName', 'Mocap Heel Distance');
grid on;
title('Filtered Heel Mocap Data');
xlabel('Time (s)');
ylabel('Distance (mm)')
legend('Location', 'best');
hold off;

%% Overlay Mocap and Sensor Data

figure(6);
subplot(2,1,1);
plot(Sensor_time, ToeSensor_distance, 'b', 'LineWidth', 1.5, 'DisplayName', 'Sensor distance');
hold on;
plot(Mocap_time, ToeMocap_distance, 'r', 'LineWidth', 1.5, 'DisplayName', 'Mocap distance');
grid on;
title('Overlayed Sensor and Mocap Toe Distances');
xlabel('Time (s)');
ylabel('Distance (mm)');
legend('Location', 'best');
hold off;

subplot(2,1,2);
plot(Sensor_time, HeelSensor_distance, 'b', 'LineWidth', 1.5, 'DisplayName', 'Sensor Distance');
hold on;
plot(Mocap_time, HeelMocap_distance, 'r', 'LineWidth', 1.5, 'DisplayName', 'Mocap Distance');
grid on;
title('Overlayed Sensor and Mocap Heel Distances');
xlabel('Time (s)');
ylabel('Distance (mm)');
legend('Location', 'best');
hold off;


%% Align signals based on user input

% Plot both signals for visual selection
figure(7);
plot(Sensor_time, ToeSensor_distance, 'b'); 
hold on;
plot(Mocap_time, ToeMocap_distance, 'r');
legend('Sensor', 'MoCap');
title('Click one peak on each signal to align');

% Collect user input
[x_clicks, ~] = ginput(2);  % Click 1 point on each signal
sensor_peak_time = x_clicks(1);
mocap_peak_time = x_clicks(2);

% Create a common time grid with consistent sample rate
t_min = min([min(Sensor_time), min(Mocap_time)]);
t_max = max([max(Sensor_time), max(Mocap_time)]);
dt = 0.01; % 10ms time step (100Hz)
common_time = t_min:dt:t_max;

% Interpolate original data to common time grid
ToeSensor_interp = interp1(Sensor_time, ToeSensor_distance, common_time, 'linear', 'extrap');
HeelSensor_interp = interp1(Sensor_time, HeelSensor_distance, common_time, 'linear', 'extrap');
ToeMocap_interp  = interp1(Mocap_time, ToeMocap_distance, common_time, 'linear', 'extrap');
HeelMocap_interp = interp1(Mocap_time, HeelMocap_distance, common_time, 'linear', 'extrap');

% Calculate time shift
time_shift = sensor_peak_time - mocap_peak_time;

% Shift MoCap data using interpolation
ToeMocap_shifted  = interp1(common_time, ToeMocap_interp, common_time + time_shift, 'linear', 'extrap');
HeelMocap_shifted = interp1(common_time, HeelMocap_interp, common_time + time_shift, 'linear', 'extrap');

% Cut very beginning and end
t_start = 6.5;
t_end = 35;
idx = (common_time >= t_start) & (common_time <= t_end);

common_time1 = common_time(idx);
ToeSensor_interp1 = ToeSensor_interp(idx);
ToeMocap_shifted1 = ToeMocap_shifted(idx);
HeelSensor_interp1 = HeelSensor_interp(idx);
HeelMocap_shifted1 = HeelMocap_shifted(idx);

% Plot aligned signals
figure(8);
subplot(2,1,1);
plot(common_time1, ToeSensor_interp1, 'b', 'LineWidth', 1.5, 'DisplayName', 'Sensor distance'); 
hold on;
plot(common_time1, ToeMocap_shifted1, 'r', 'LineWidth', 1.5, 'DisplayName', 'MoCap distance');
grid on;
xlabel('Time (s)');
ylabel('Distance (mm)');
legend('Location', 'best');
title('Aligned Toe Sensor and MoCap Signals');

subplot(2,1,2);
plot(common_time1, HeelSensor_interp1, 'b', 'LineWidth', 1.5, 'DisplayName', 'Sensor distance'); 
hold on;
plot(common_time1, HeelMocap_shifted1, 'r', 'LineWidth', 1.5, 'DisplayName', 'MoCap distance');
grid on;
xlabel('Time (s)');
ylabel('Distance (mm)');
legend('Location', 'best');
title('Aligned Heel Sensor and MoCap Signals');

%% Calculate accuracy metrics
% Calculate Root Mean Square Error (RMSE)
toe_rmse = sqrt(mean((ToeSensor_interp - ToeMocap_shifted).^2));
heel_rmse = sqrt(mean((HeelSensor_interp - HeelMocap_shifted).^2));

% Calculate Mean Absolute Error (MAE)
toe_mae = mean(abs(ToeSensor_interp - ToeMocap_shifted));
heel_mae = mean(abs(HeelSensor_interp - HeelMocap_shifted));

% Calculate Pearson correlation coefficient for lineair correlation
toe_corr = corrcoef(ToeSensor_interp, ToeMocap_shifted);
toe_r = toe_corr(1,2);
heel_corr = corrcoef(HeelSensor_interp, HeelMocap_shifted);
heel_r = heel_corr(1,2);

% Calculate R-squared value
toe_rsquared = toe_r^2;
heel_rsquared = heel_r^2;

% % Calculate accuracy percentile based on 1 - normalized RMSE
% % Define the maximum acceptable error as a scalar
toe_max_error = max([ToeSensor_interp(:); ToeMocap_shifted(:)]) - ...
                min([ToeSensor_interp(:); ToeMocap_shifted(:)]);

heel_max_error = max([HeelSensor_interp(:); HeelMocap_shifted(:)]) - ...
                 min([HeelSensor_interp(:); HeelMocap_shifted(:)]);
fprintf('Max heel error: %.2f mm\n', heel_max_error);
fprintf('Max toe error: %.2f mm\n', toe_max_error);

% Now calculate the accuracy percentages
% This one is most interesting since it shows how closely the values
% match in actual distance
toe_accuracy_pct = max(0, min(100, 100 * (1 - toe_rmse / toe_max_error)));
heel_accuracy_pct = max(0, min(100, 100 * (1 - heel_rmse / heel_max_error)));

% Display accuracy metrics
fprintf('\n--- Accuracy Metrics ---\n');
fprintf('Toe Sensor vs. Mocap:\n');
fprintf('  RMSE: %.2f mm\n', toe_rmse);
fprintf('  MAE: %.2f mm\n', toe_mae);
fprintf('  Correlation (r): %.4f\n', toe_r);
fprintf('  R-squared: %.4f\n', toe_rsquared);
fprintf('  Accuracy: %.2f%%\n', toe_accuracy_pct);

fprintf('\nHeel Sensor vs. Mocap:\n');
fprintf('  RMSE: %.2f mm\n', heel_rmse);
fprintf('  MAE: %.2f mm\n', heel_mae);
fprintf('  Correlation (r): %.4f\n', heel_r);
fprintf('  R-squared: %.4f\n', heel_rsquared);
fprintf('  Accuracy: %.2f%%\n', heel_accuracy_pct);


%% Only plot from 25-35s

% Set time interval (in seconds)
t_start = 20;   % change as needed
t_end = 35;     % change as needed

% Create logical index for desired interval
interval_idx = (common_time >= t_start) & (common_time <= t_end);

% Chop data
common_time = common_time(interval_idx);
ToeSensor_cut = ToeSensor_interp(interval_idx);
ToeMocap_cut = ToeMocap_shifted(interval_idx);
HeelSensor_cut = HeelSensor_interp(interval_idx);
HeelMocap_cut = HeelMocap_shifted(interval_idx);

% Plot aligned signals
figure(9);
subplot(2,1,1);
plot(common_time, ToeSensor_cut, 'b', 'LineWidth', 1.5, 'DisplayName', 'Sensor distance'); 
hold on;
plot(common_time, ToeMocap_cut, 'r', 'LineWidth', 1.5, 'DisplayName', 'MoCap distance');
grid on;
xlabel('Time (s)');
ylabel('Distance (mm)');
legend('Location', 'best');
title('Aligned Toe Sensor and MoCap Signals');

subplot(2,1,2);
plot(common_time, HeelSensor_cut, 'b', 'LineWidth', 1.5, 'DisplayName', 'Sensor distance'); 
hold on;
plot(common_time, HeelMocap_cut, 'r', 'LineWidth', 1.5, 'DisplayName', 'MoCap distance');
grid on;
xlabel('Time (s)');
ylabel('Distance (mm)');
legend('Location', 'best');
title('Aligned Heel Sensor and MoCap Signals');


%% Calculate accuracy metrics
% Calculate Root Mean Square Error (RMSE)
toe_rmse = sqrt(mean((ToeSensor_cut - ToeMocap_cut).^2));
heel_rmse = sqrt(mean((HeelSensor_cut - HeelMocap_cut).^2));

% Calculate Mean Absolute Error (MAE)
toe_mae = mean(abs(ToeSensor_cut - ToeMocap_cut));
heel_mae = mean(abs(HeelSensor_cut - HeelMocap_cut));

% Calculate Pearson correlation coefficient for lineair correlation
toe_corr = corrcoef(ToeSensor_cut, ToeMocap_cut);
toe_r = toe_corr(1,2);
heel_corr = corrcoef(HeelSensor_cut, HeelMocap_cut);
heel_r = heel_corr(1,2);

% Calculate R-squared value
toe_rsquared = toe_r^2;
heel_rsquared = heel_r^2;

% % Calculate accuracy percentile based on 1 - normalized RMSE
% % Define the maximum acceptable error as a scalar
toe_max_error = max([ToeSensor_cut(:); ToeMocap_cut(:)]) - ...
                min([ToeSensor_cut(:); ToeMocap_cut(:)]);

heel_max_error = max([HeelSensor_cut(:); HeelMocap_cut(:)]) - ...
                 min([HeelSensor_cut(:); HeelMocap_cut(:)]);
fprintf('Max heel error: %.2f mm\n', heel_max_error);
fprintf('Max toe error: %.2f mm\n', toe_max_error);

% Now calculate the accuracy percentages
% This one is most interesting since it shows how closely the values
% match in actual distance
toe_accuracy_pct = max(0, min(100, 100 * (1 - toe_rmse / toe_max_error)));
heel_accuracy_pct = max(0, min(100, 100 * (1 - heel_rmse / heel_max_error)));

% Display accuracy metrics
fprintf('\n--- Accuracy Metrics ---\n');
fprintf('Toe Sensor vs. Mocap:\n');
fprintf('  RMSE: %.2f mm\n', toe_rmse);
fprintf('  MAE: %.2f mm\n', toe_mae);
fprintf('  Correlation (r): %.4f\n', toe_r);
fprintf('  R-squared: %.4f\n', toe_rsquared);
fprintf('  Accuracy: %.2f%%\n', toe_accuracy_pct);

fprintf('\nHeel Sensor vs. Mocap:\n');
fprintf('  RMSE: %.2f mm\n', heel_rmse);
fprintf('  MAE: %.2f mm\n', heel_mae);
fprintf('  Correlation (r): %.4f\n', heel_r);
fprintf('  R-squared: %.4f\n', heel_rsquared);
fprintf('  Accuracy: %.2f%%\n', heel_accuracy_pct);