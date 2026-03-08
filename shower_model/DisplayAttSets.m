clear; clc; close all;
% Setup
p = ShowerModel();
N = 50; 
u_vals = linspace(0, 1, N);
[U_h, U_c] = meshgrid(u_vals, u_vals);

% pre alloc Platz
Temp_steady = nan(size(U_h));
Flow_steady = nan(size(U_h));

for i = 1:numel(U_h)
    u_h = U_h(i);
    u_c = U_c(i);
    x_dummy = [0, u_h, u_c, 0];
    [Q_total, Q_h, Q_c, ~] = p.calculate_flows(x_dummy);
    
    if Q_total > 1e-6
        T_mix = (Q_h * p.T_h + Q_c * p.T_c) / Q_total;
    else
        T_mix = nan; 
    end
    
    Temp_steady(i) = T_mix;
    Flow_steady(i) = Q_total;
end

mask = ~isnan(Temp_steady); % Punkte ohne Wasserfluss entfernen
U_h_flat = U_h(mask);
U_c_flat = U_c(mask);
T_flat = Temp_steady(mask);
Q_flat = Flow_steady(mask);

% Visualisierung
figure('Color', 'white', 'Units', 'pixels', 'Position', [100 100 600 600]);

scatter3(U_h_flat, U_c_flat, Q_flat, 20, T_flat, 'filled');

xlabel('Valve Hot (u_h)');
ylabel('Valve Cold (u_c)');
zlabel('Flow Rate (Q)');
title('Shower Control Allocation: Flow vs. Valve Positions');

grid on;
view(-30, 30);
axis square;
pbaspect([1 1 1]);

% Colorbar setup
cb = colorbar;
cb.Label.String = 'Temperature [°C]';
colormap(parula);