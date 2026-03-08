clear; clc; close all;

active_controllers = [0, 1, 2]; % 0=PI, 1=LQR, 2=Decoupled
t_end  = 20;
h      = 0.01;
t_step = 1.0;
T_init   = 35;  
Q_init   = 0.6;
T_target = 40; 
Q_target = 0.6;

% Szenario: Einzelner Sprung bei t_step
scenario_fn = @(t) deal( ...
    T_init + (T_target - T_init) * (t >= t_step), ...
    Q_init + (Q_target - Q_init) * (t >= t_step));

[results, t_vec] = run_simulation(active_controllers, t_end, h, scenario_fn);

% Visu
figure('Name', 'Controller Step Comparisons', 'Position', [100, 100, 1000, 500]);
colors = {'r', 'g', 'b'};

% Sollwert-Referenz
ref_T = results(active_controllers(1)+1).tgt(1,:);
ref_Q = results(active_controllers(1)+1).tgt(2,:);

% Temperatur-Plot
subplot(1, 2, 1); hold on; grid on;
plot(t_vec, ref_T, 'k--', 'LineWidth', 1.2, 'DisplayName', 'Target');
for i = 1:numel(active_controllers)
    r = results(active_controllers(i)+1);
    plot(t_vec, r.x(4,:), 'Color', colors{i}, 'LineWidth', 1.5, 'DisplayName', r.name);
end
title('Temperature Control');
xlabel('Time [s]'); ylabel('Temp [°C]');
legend('Location', 'best');
xlim([0 t_end]); ylim([34, 42]);

% Durchfluss-Plot
subplot(1, 2, 2); hold on; grid on;
plot(t_vec, ref_Q, 'k--', 'LineWidth', 1.2, 'DisplayName', 'Target');
for i = 1:numel(active_controllers)
    r = results(active_controllers(i)+1);
    plot(t_vec, r.Q, 'Color', colors{i}, 'LineWidth', 1.5, 'DisplayName', r.name);
end
title('Flow Control');
xlabel('Time [s]'); ylabel('Flow [m^3/h]');
legend('Location', 'best');
xlim([0 8]); ylim([0.4, 0.8]);


