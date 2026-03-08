clear; clc; close all; rng(100);

% Konfig
active_controllers = [0, 1, 2, 3]; % 0=PI, 1=LQR, 2=Decoupled, 3=LQI
t_end = 50;
h     = 0.001;
T_init = 38;
Q_init = 0.6;

% Szenario: Mehrere Sollwertsprünge
scenario_fn = @(t) scenario(t, T_init, Q_init);

% Störung: Druckabfall Warmwasser ab 25s
disturbance_fn = @(t, p) set_pressure(t, p);

[results, t_vec] = run_simulation(active_controllers, t_end, h, scenario_fn, ...
    'disturbance_fn', disturbance_fn);

% Visualisierung
figure('Name', 'Controller Benchmark', 'Color', 'w', 'Position', [100, 100, 1000, 800]);

base_idx = active_controllers(1) + 1;
tgt_T = results(base_idx).tgt(1,:);
tgt_Q = results(base_idx).tgt(2,:);

% Temperatur-plot
subplot(3,1,1); hold on; grid on;
plot(t_vec, tgt_T, 'k:', 'LineWidth', 2, 'DisplayName', 'Sollwert');
for c = active_controllers
    r = results(c+1);
    plot(t_vec, r.x(4,:), r.style, 'LineWidth', 1.5, 'DisplayName', r.name);
end
legend; title('Temperatur (Achten Sie auf Sprünge bei Flow-Änderung bei t=15s!)');
ylabel('Temp [°C]');

% Durchfluss-plot
subplot(3,1,2); hold on; grid on;
plot(t_vec, tgt_Q, 'k:', 'LineWidth', 2, 'DisplayName', 'Sollwert');
for c = active_controllers
    r = results(c+1);
    plot(t_vec, r.Q, r.style, 'LineWidth', 1.5, 'DisplayName', r.name);
end
title('Durchfluss (Achten Sie auf Sprünge bei Temp-Änderung bei t=5s!)');
ylabel('Flow [m^3/h]');

% u_h-Plot
subplot(3,1,3); hold on; grid on;
for c = active_controllers
    r = results(c+1);
    plot(t_vec, r.u(1,:), r.style, 'LineWidth', 1, 'DisplayName', [r.name ' u_h']);
end
title('Stellgröße u_h (Reaktion auf Störung bei t=25s)');
ylabel('Ventil u_h [0-1]'); xlabel('Zeit [s]');
legend;

fprintf('Fertig.\n');

% Lokale Funktionen 

function [T_soll, Q_soll] = scenario(t, T_init, Q_init)
    T_soll = T_init;
    Q_soll = Q_init;
    if t >= 5,  T_soll = 45;  end
    if t >= 15, Q_soll = 0.8; end
end

function p = set_pressure(t, p)
    if t >= 25
        p.P_h = 2.0;
    else
        p.P_h = 3.0;
    end
end
