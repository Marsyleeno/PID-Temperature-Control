% Load CSV
data = readtable('Iteration6.csv');

% Time vector assuming 250 ms intervals (from Arduino delay)
t = (0:height(data)-1) * 0.25;

% Extract data
set_temp = data.SetTemp;
real_temp = data.RealTemp;

% Normalize real_temp for rise time calculation
setpoint = max(set_temp);  % assumes setpoint eventually steps to a constant
real_temp_norm = (real_temp - real_temp(1)) / (setpoint - real_temp(1));

% Compute Rise Time (10% to 90%)
rt_start_idx = find(real_temp_norm >= 0.1, 1);
rt_end_idx = find(real_temp_norm >= 0.9, 1);
if ~isempty(rt_start_idx) && ~isempty(rt_end_idx)
    rise_time = t(rt_end_idx) - t(rt_start_idx);
    rise_str = sprintf('Rise Time: %.2f s', rise_time);
else
    rise_str = 'Rise Time: Not Reached';
end

% Settling Time (±2% band of setpoint)
tolerance = 0.02 * setpoint;
within_band = abs(real_temp - setpoint) <= tolerance;
% look for last point after which it stays within the band
settling_time = NaN;
for i = 1:length(within_band)
    if all(within_band(i:end))
        settling_time = t(i);
        break;
    end
end
if isnan(settling_time)
    settling_time = t(end);
end
settle_str = sprintf('Settling Time: %.2f s', settling_time);

% Steady-State Error
ss_error = abs(real_temp(end) - setpoint);
ss_str = sprintf('Steady-State Error: %.2f °C', ss_error);

% Percent Overshoot
peak_val = max(real_temp);
overshoot = ((peak_val - setpoint) / setpoint) * 100;
overshoot_str = sprintf('Percent Overshoot: %.2f %%', overshoot);

% Plotting
figure;
plot(t, set_temp, '--', 'DisplayName', 'Set Temperature (°C)', 'LineWidth', 1.5);
hold on;
plot(t, real_temp, 'DisplayName', 'Actual Temperature (°C)', 'LineWidth', 1.5);
legend('show');
xlabel('Time (s)');
ylabel('Temperature (°C)');
title('PID System Response (Full Duration)');
grid on;

% Metrics annotation box in LOWER RIGHT
dim = [0.65, 0.15, 0.3, 0.2];  % [x y width height] - lower right
annotation('textbox', dim, 'String', ...
    {rise_str, settle_str, ss_str, overshoot_str}, ...
    'FitBoxToText', 'on', 'BackgroundColor', 'white');

