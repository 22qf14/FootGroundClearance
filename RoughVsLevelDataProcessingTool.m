%% Safia Adair - May 2025

% This MATLAB Data analysis tool will take readings from multiple trials on
% different surfaces (stored in different .csv files), and apply a time
% shift to display and compare them on the same grid. 

clear;
close all;
clc;

%% Import Level Surface Data File
% Import the data for the level surface (replace file name in quotations
% with correct file for analysis
LevelData = Sensor_data('ReferenceData.csv', [2, Inf]); % must have Sensor_data funciton code open. 

% Extract time and distance data
Level_time = LevelData{:,1}/1000; % Convert ms to s
LevelToe_dist = LevelData{:,3};
LevelHeel_dist = LevelData{:,2};

% Remove NaN values (only keep table rows where all three data columns have
% valid readings).
valid_level = isfinite(Level_time) & isfinite(LevelToe_dist) & isfinite(LevelHeel_dist);

Level_time = Level_time(valid_level);
LevelToe_dist = LevelToe_dist(valid_level);
LevelHeel_dist = LevelHeel_dist(valid_level);

%% Plot level sensor distance over sensor time

figure(1);
plot(Level_time, LevelToe_dist,'b', 'LineWidth', 1.5, 'DisplayName', 'Toe Sensor Data');
hold on;
grid on;
plot(Level_time, LevelHeel_dist, 'g', 'LineWidth', 1.5, 'DisplayName', 'Heel Sensor Data');
hold on;
grid on;
title('Level Surface: Sensor Distance vs Time');
xlabel('Time (s)');
ylabel('Distance (mm)');
legend('Location', 'best');
hold off;


%% Allow user to click on graph to exclude unnecessary data

% Enter data trim mode
disp('====== DATA TRIM MODE ======');
disp('Click on either graph at the start and end of viable data section.');

% Trim the graph based on user clicks
[x_clicks, ~] = ginput(2);
x_clicks = sort(x_clicks);  % Make indices are in order

% Find indices of data within clicked range
indices = find(Level_time >= x_clicks(1) & Level_time <= x_clicks(2));
LevelTime_trimmed = Level_time(indices);
LevelToe_trimmed = LevelToe_dist(indices);
LevelHeel_trimmed = LevelHeel_dist(indices);

disp('=========== DONE ===========');
fprintf("\n");

%% Plot level surface toe and heel trimmed subplots

figure(2);
subplot(2,1,1); % subplot for toe sensor data over time
plot(LevelTime_trimmed, LevelToe_trimmed,'b', 'LineWidth', 1.5, 'DisplayName', 'Toe Sensor Data');
grid on;
title('Level Toe Sensor Data Over Time');
xlabel('Time (s)');
ylabel('Distance (mm)');
hold off;

subplot(2,1,2); % subplot for heel sensor data over time
plot(LevelTime_trimmed, LevelHeel_trimmed, 'b', 'LineWidth', 1.5, 'DisplayName', 'Heel Sensor Data');
grid on;
title('Level Heel Sensor Data Over Time');
xlabel('Time (s)');
ylabel('Distance (mm)');
hold off;


%% Import rough surface data file
% Import the data for the rough surface (replace file name in quotations
% with correct file for analysis
RoughData = Sensor_data('ExpFourData.csv', [2, Inf]); % must have Sensor_data funciton code open. 

% Extract time and distance data
Rough_time = RoughData{:,1}/1000; % Convert ms to s
RoughToe_dist = RoughData{:,3};
RoughHeel_dist = RoughData{:,2};

% Remove NaN values (only keep table rows where all three data columns have
% valid readings).
valid_rough = isfinite(Rough_time) & isfinite(RoughToe_dist) & isfinite(RoughHeel_dist);

Rough_time = Rough_time(valid_rough);
RoughToe_dist = RoughToe_dist(valid_rough);
RoughHeel_dist = RoughHeel_dist(valid_rough);

%% Plot rough sensor distance over sensor time

figure(3);
plot(Rough_time, RoughToe_dist,'b', 'LineWidth', 1.5, 'DisplayName', 'Toe Sensor Data');
hold on;
grid on;
plot(Rough_time, RoughHeel_dist, 'g', 'LineWidth', 1.5, 'DisplayName', 'Heel Sensor Data');
hold on;
grid on;
title('Rough Surface: Sensor Distance vs Time');
xlabel('Time (s)');
ylabel('Distance (mm)');
legend('Location', 'best');
hold off;


%% Allow user to click on graph to exclude unnecessary data

% Enter data trim mode
disp('====== DATA TRIM MODE ======');
disp('Click on either graph at the start and end of viable data section.');

% Trim the graph based on user clicks
[x_clicks, ~] = ginput(2);
x_clicks = sort(x_clicks);  % Make indices are in order

% Find indices of data within clicked range
indices = find(Rough_time >= x_clicks(1) & Rough_time <= x_clicks(2));
RoughTime_trimmed = Rough_time(indices);
RoughToe_trimmed = RoughToe_dist(indices);
RoughHeel_trimmed = RoughHeel_dist(indices);

disp('=========== DONE ===========');

%% Plot rough surface toe and heel trimmed subplots

figure(4);
subplot(2,1,1); % subplot for toe sensor data over time
plot(RoughTime_trimmed, RoughToe_trimmed,'b', 'LineWidth', 1.5, 'DisplayName', 'Toe Sensor Data');
grid on;
title('Rough Toe Sensor Data Over Time');
xlabel('Time (s)');
ylabel('Distance (mm)');
hold off;

subplot(2,1,2); % subplot for heel sensor data over time
plot(RoughTime_trimmed, RoughHeel_trimmed, 'b', 'LineWidth', 1.5, 'DisplayName', 'Heel Sensor Data');
grid on;
title('Rough Heel Sensor Data Over Time');
xlabel('Time (s)');
ylabel('Distance (mm)');
hold off;

%% Overlay the two graphs and apply a time shift

% Set both data sets to begin at t = 0
LevelTime_trimmed = LevelTime_trimmed - LevelTime_trimmed(1);
RoughTime_trimmed = RoughTime_trimmed - RoughTime_trimmed(1);

% Shift the rough surface graph to best match the level surface graph (best
% aligned peaks)
[cross_corr, lag] = xcorr(LevelToe_trimmed, RoughToe_trimmed);
[~, maxIndex] = max(cross_corr);
optimal_lag = lag(maxIndex);

dt = mean(diff(LevelTime_trimmed));
time_shift = optimal_lag * dt;

RoughToe_shifted = interp1(RoughTime_trimmed, RoughToe_trimmed, LevelTime_trimmed - time_shift, 'linear', 'extrap');
RoughHeel_shifted = interp1(RoughTime_trimmed, RoughHeel_trimmed, LevelTime_trimmed - time_shift, 'linear', 'extrap');

% Cut off the longer sample of data so that both data sets are the same
% length for comparison
minLength = min(length(LevelTime_trimmed), length(RoughTime_trimmed));

LevelTime_trimmed = LevelTime_trimmed(1:minLength);
RoughTime_trimmed = RoughTime_trimmed(1:minLength);
LevelToe_trimmed = LevelToe_trimmed(1:minLength);
RoughToe_shifted = RoughToe_shifted(1:minLength);
LevelHeel_trimmed = LevelHeel_trimmed(1:minLength);
RoughHeel_shifted = RoughHeel_shifted(1:minLength);

%% Print overlayed graph

figure(5);

% Toe
subplot(2,1,1);
plot(LevelTime_trimmed, LevelToe_trimmed, 'b', 'LineWidth', 1.5, 'DisplayName', 'Level Surface 1');
hold on;
grid on;
plot(RoughTime_trimmed, RoughToe_shifted, 'r', 'LineWidth', 1.5, 'DisplayName', 'Level Surface 2 with Time Shift');
title('Aligned Toe Sensor Data on Two Surfaces');
xlabel('Time (s)');
ylabel('Distance (mm)');
legend('Location', 'best');
hold off;

% Heel
subplot(2,1,2);
plot(LevelTime_trimmed, LevelHeel_trimmed, 'b', 'LineWidth', 1.5, 'DisplayName', 'Level Surface 1');
hold on;
grid on;
plot(RoughTime_trimmed, RoughHeel_shifted, 'r', 'LineWidth', 1.5, 'DisplayName', 'Level Surface 2 with Time Shift');
title('Aligned Heel Sensor Data on Two Surfaces');
xlabel('Time (s)');
ylabel('Distance (mm)');
legend('Location', 'best');
hold off;

%% Calculate Similarity Metrics over the interval

% Set time interval (in seconds)
t_start = 0;   % change as needed
t_end = 5;     % change as needed

% Create logical index for desired interval
interval_idx = (LevelTime_trimmed >= t_start) & (LevelTime_trimmed <= t_end);

% Extract relevant data within the interval
LevelToe_interval = LevelToe_trimmed(interval_idx);
RoughToe_interval = RoughToe_shifted(interval_idx);
LevelHeel_interval = LevelHeel_trimmed(interval_idx);
RoughHeel_interval = RoughHeel_shifted(interval_idx);
% Toe Sensor
errors_toe = LevelToe_interval - RoughToe_interval;
MAE_toe = mean(abs(errors_toe));
RMSE_toe = sqrt(mean(errors_toe.^2));
R2_toe = 1 - sum(errors_toe.^2) / sum((LevelToe_interval - mean(LevelToe_interval)).^2);

% Heel Sensor
errors_heel = LevelHeel_interval - RoughHeel_interval;
MAE_heel = mean(abs(errors_heel));
RMSE_heel = sqrt(mean(errors_heel.^2));
R2_heel = 1 - sum(errors_heel.^2) / sum((LevelHeel_interval - mean(LevelHeel_interval)).^2);

% Display Results
fprintf('\n=== Toe Metrics (%.1f to %.1f s) ===\n', t_start, t_end);
fprintf('MAE: %.4f, RMSE: %.4f, R^2: %.4f\n', MAE_toe, RMSE_toe, R2_toe);

fprintf('\n=== Heel Metrics (%.1f to %.1f s) ===\n', t_start, t_end);
fprintf('MAE: %.4f, RMSE: %.4f, R^2: %.4f\n\n', MAE_heel, RMSE_heel, R2_heel);


[h, p] = ttest(LevelToe_trimmed, RoughToe_shifted);

if h == 1
    fprintf('Difference is statistically significant (p = %.4f)\n', p);
else
    fprintf('No significant difference (p = %.4f)\n', p);
end