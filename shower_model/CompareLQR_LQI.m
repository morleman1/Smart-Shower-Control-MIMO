clear; clc; close all;

% Vergleich LQR vs LQI

t_end = 20;
h = 0.002;
N = round(t_end / h);
t = (0:N-1) * h;

T_init = 37;
T_ref = 40;
Q_ref = 0.7;
t_step = 2;
t_drop = 7;

% Arbeitspunkt
p = ShowerModel();
[u_init, x_init] = p.solve_op(T_init, Q_ref);
[u_ref, x_ref] = p.solve_op(T_ref, Q_ref);

x0 = x_init;

[K_lq, uop_lq, xop_lq] = Controller_LQR(p, T_ref, Q_ref);
[K_lqi, uop_lqi, xop_lqi] = Controller_LQI(p, T_ref, Q_ref);

T_lq = zeros(1, N);
T_lqi = zeros(1, N);
Q_lq = zeros(1, N);
Q_lqi = zeros(1, N);
T_setpoint = zeros(1, N);

% LQ simulation
x = x0;
p.P_h = 3.0;
uop_current = u_init;
xop_current = x_init;

for k = 1:N
    if t(k) >= t_step
        T_sp = T_ref;
        uop_current = u_ref;
        xop_current = x_ref;
    else
        T_sp = T_init;
        uop_current = u_init;
        xop_current = x_init;
    end
    
    % Druckabfall Warmwasser
    if t(k) >= t_drop
        p.P_h = 2.0;
    end

    T_setpoint(k) = T_sp;
    [Q_curr] = p.calculate_flows(x);
    u = uop_current - K_lq * (x - xop_current);
    u = max(0, min(1, u));

    x = p.rk4_step(x, u, h);
    T_lq(k) = x(4);
    Q_lq(k) = Q_curr;
end

% LQI simulation
x = x0;
err_int = [0; 0];
p.P_h = 3.0;
uop_current = u_init;
xop_current = x_init;

for k = 1:N
    if t(k) >= t_step
        T_sp = T_ref;
        uop_current = u_ref;
        xop_current = x_ref;
    else
        T_sp = T_init;
        uop_current = u_init;
        xop_current = x_init;
    end
    
    % Druckabfall Warmwasser
    if t(k) >= t_drop
        p.P_h = 2.0;
    end

    [Q_curr] = p.calculate_flows(x);
    err = [T_sp; Q_ref] - [x(4); Q_curr];
    err_int_next = err_int + err * h;

    dx = x - xop_current;
    u = uop_current - K_lqi * [dx; err_int_next];

    if any(u > 1 | u < 0)
        err_int_next = err_int;
        u = uop_current - K_lqi * [dx; err_int_next];
    end

    u = max(0, min(1, u));
    err_int = err_int_next;

    x = p.rk4_step(x, u, h);
    T_lqi(k) = x(4);
    Q_lqi(k) = Q_curr;
end

% Visualisierung
figure('Color', 'w', 'Name', 'LQ vs LQI: Step Response + Pressure Drop');

temp_xlim = [0 15];
temp_ylim = [36 40.6];
flow_xlim = [0 15];
flow_ylim = [0.6 0.75];

ax1 = subplot(2,1,1);
hold on; grid on;
plot(t, T_lq, 'g-', 'LineWidth', 1.6, 'DisplayName', 'LQ (LQR)');
plot(t, T_lqi, 'm-', 'LineWidth', 1.6, 'DisplayName', 'LQI');
hRefT = plot(t, T_setpoint, 'k--', 'LineWidth', 1.2, 'DisplayName', 'Temperature setpoint');
hStepT = plot([t_step t_step], temp_ylim, 'b:', 'LineWidth', 1.2, 'DisplayName', 'Step');
hDropT = plot([t_drop t_drop], temp_ylim, 'r--', 'LineWidth', 1.2, 'DisplayName', 'Pressure drop');
ax1.SortMethod = 'childorder';
uistack([hRefT, hStepT, hDropT], 'bottom');
ylabel('Temperature [°C]');
title('LQ vs LQI: Step + Pressure drop');
legend('Location', 'best');
xlim(temp_xlim);
ylim(temp_ylim);

ax2 = subplot(2,1,2);
hold on; grid on;
plot(t, Q_lq, 'g-', 'LineWidth', 1.6, 'DisplayName', 'LQ (LQR)');
plot(t, Q_lqi, 'm-', 'LineWidth', 1.6, 'DisplayName', 'LQI');
hRefQ = plot(flow_xlim, [Q_ref Q_ref], 'k--', 'LineWidth', 1.2, 'DisplayName', 'Flow setpoint');
hStepQ = plot([t_step t_step], flow_ylim, 'b:', 'LineWidth', 1.2, 'DisplayName', 'Step');
hDropQ = plot([t_drop t_drop], flow_ylim, 'r--', 'LineWidth', 1.2, 'DisplayName', 'Pressure drop');
ax2.SortMethod = 'childorder';
uistack([hRefQ, hStepQ, hDropQ], 'bottom');
xlabel('Time [s]');
ylabel('Flow [m^3/h]');
legend('Location', 'best');
xlim(flow_xlim);
ylim(flow_ylim);

linkaxes([ax1 ax2], 'x');
