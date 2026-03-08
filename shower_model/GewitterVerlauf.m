clear; clc; close all; rng(100);

active_controllers = [0, 1, 2]; % 0=PI, 1=LQR, 2=Decoupled, 3=LQI

t_end = 60;
h     = 0.001;

% Szenario: Mehrere Sollwertsprünge (Temp & Flow)
scenario_fn = @scenario_gewitter;

[results, t_vec] = run_simulation(active_controllers, t_end, h, scenario_fn);

% Visualisierung
figure('Name', 'Shower Benchmark', 'Color', 'w', 'Position', [100, 100, 1000, 800]);

% Helper für Plots
plot_signal = @(sub, data_fn, lbl, tit) ...
    subplot(3,1,sub) && hold(subplot(3,1,sub), 'on') && grid(subplot(3,1,sub), 'on') && ...
    ylabel(lbl) && title(tit) && xlim([0 t_end]);

% Referenz (Sollwert)
base_idx = active_controllers(1) + 1;
tgt_T = results(base_idx).tgt(1,:);
tgt_Q = results(base_idx).tgt(2,:);

% 1. Temperatur
subplot(2,1,1); hold on; grid on;
plot(t_vec, tgt_T, 'k:', 'LineWidth', 2, 'DisplayName', 'Soll');
for c = active_controllers
    r = results(c+1);
    plot(t_vec, r.x(4,:), r.style, 'LineWidth', 1.5, 'DisplayName', r.name);
end
legend; title('Temperatur [°C]');

% 2. Durchfluss
subplot(2,1,2); hold on; grid on;
plot(t_vec, tgt_Q, 'k:', 'LineWidth', 2, 'DisplayName', 'Soll');
for c = active_controllers
    r = results(c+1);
    plot(t_vec, r.Q, r.style, 'LineWidth', 1.5, 'DisplayName', r.name);
end
title('Flow [m^3/h]');


fprintf('Fertig.\n');

% Verlauf
function [T_soll, Q_soll] = scenario_gewitter(t)
    T_soll = 38; Q_soll = 0.6;
    if t >=  5, T_soll = 32; Q_soll = 0.6; end
    if t >= 15, T_soll = 24; Q_soll = 1.0; end
    if t >= 35, T_soll = 45; Q_soll = 0.5; end
    if t >= 45, T_soll = 38; Q_soll = 0.7; end
end
