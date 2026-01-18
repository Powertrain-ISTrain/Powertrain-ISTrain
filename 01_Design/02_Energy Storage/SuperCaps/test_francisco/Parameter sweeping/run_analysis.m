clear; clc;

%% --- 1. DEFINE YOUR SWEEP RANGES ----------------------------
decel_list = 0.1 : 0.1 : 0.6;           % 6 values
target_speed_list = 1 : 2 : 15;         % 8 values
accel_list = 0.1 : 0.1 : 0.6;           % 6 values
pre_charge_list = 1 : 0.5 : 48;         % 95 values

% Total combinations = 6 * 8 * 6 * 95 = 27,360
% -------------------------------------------------------------


%% --- 2. AUTOMATIC SIMULATION RUNNER -------------------------
% Pre-allocate table for speed (otherwise 27k rows is slow)
num_runs = length(decel_list) * length(target_speed_list) * length(accel_list) * length(pre_charge_list);

% Initialize arrays to hold data (faster than growing a table)
data_decel = zeros(num_runs, 1);
data_speed = zeros(num_runs, 1);
data_accel = zeros(num_runs, 1);
data_precharge = zeros(num_runs, 1);
data_dist = zeros(num_runs, 1);

fprintf('Starting %d simulations. This may take a few minutes...\n', num_runs);

counter = 0;
tic; % Start timer

for d = decel_list
    for t_spd = target_speed_list
        for a = accel_list
            for p = pre_charge_list
                
                counter = counter + 1;
                
                % Run Engine
                [dist_empty, ~, ~] = simulate_regen(d, t_spd, a, p);
                
                % Store in arrays (faster)
                data_decel(counter) = d;
                data_speed(counter) = t_spd;
                data_accel(counter) = a;
                data_precharge(counter) = p;
                data_dist(counter) = dist_empty;
                
                % Progress Bar every 1000 runs
                if mod(counter, 1000) == 0
                    elapsed = toc;
                    percent = (counter/num_runs)*100;
                    fprintf('Progress: %.1f%% (%d/%d) - Time elapsed: %.0fs\n', ...
                        percent, counter, num_runs, elapsed);
                end
            end
        end
    end
end

% Create Table
results = table(data_decel, data_speed, data_accel, data_precharge, data_dist, ...
    'VariableNames', {'Decel', 'TgtSpeed', 'Accel', 'PreCharge', 'DistAtEmpty'});

%% --- 3. RESULTS ANALYSIS ------------------------------------

% Sort by best DISTANCE AT EMPTY
sorted_results = sortrows(results, 'DistAtEmpty', 'descend');

fprintf('\n--- TOP 10 CONFIGURATIONS (Max Distance) ---\n');
disp(sorted_results(1:10, :));

fprintf('\n--- WORST 5 CONFIGURATIONS ---\n');
disp(sorted_results(end-4:end, :));

% PLOTTING (New 4th plot added)
figure('Name', '4-Parameter Sweep Analysis');

subplot(2,2,1);
scatter(results.TgtSpeed, results.DistAtEmpty, 10, 'filled', 'MarkerFaceAlpha',0.1);
xlabel('Target Speed (km/h)'); ylabel('Distance (m)'); title('Impact of Speed'); grid on;

subplot(2,2,2);
scatter(results.Accel, results.DistAtEmpty, 10, 'filled', 'MarkerFaceAlpha',0.1);
xlabel('Accel (m/s^2)'); ylabel('Distance (m)'); title('Impact of Accel'); grid on;

subplot(2,2,3);
scatter(results.Decel, results.DistAtEmpty, 10, 'filled', 'MarkerFaceAlpha',0.1);
xlabel('Decel (m/s^2)'); ylabel('Distance (m)'); title('Impact of Braking'); grid on;

subplot(2,2,4);
scatter(results.PreCharge, results.DistAtEmpty, 10, 'filled', 'MarkerFaceAlpha',0.1);
xlabel('Pre-Charge (V)'); ylabel('Distance (m)'); title('Impact of Pre-Charge'); grid on;