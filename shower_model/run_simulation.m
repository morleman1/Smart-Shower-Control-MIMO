function [results, t_vec] = run_simulation(active_controllers, t_end, h, scenario_fn, varargin)
% Gemeinsame Simulationsfunktion für alle Reglervergleiche.
%
%   [results, t_vec] = run_simulation(active_controllers, t_end, h, scenario_fn)
%   [results, t_vec] = run_simulation(..., 'x_start', x0)
%   [results, t_vec] = run_simulation(..., 'disturbance_fn', @(t,p) ...)
%
% Eingänge:
%   active_controllers - Array der Controller-IDs [0=PI, 1=LQR, 2=Decoupled, 3=LQI]
%   t_end              - Simulationsdauer [s]
%   h                  - Zeitschritt [s]
%   scenario_fn        - Function Handle: [T_soll, Q_soll] = scenario_fn(t)
%
% Optionale Parameter (Name-Value):
%   'x_start'        - Anfangszustand [4x1]. Default: Arbeitspunkt von scenario_fn(0)
%   'disturbance_fn' - Function Handle: p = disturbance_fn(t, p) für Störungen
%
% Ausgänge:
%   results - Struct-Array mit Feldern: x, u, Q, tgt, name, style
%   t_vec   - Zeitvektor

    % Name-Value parsen
    p_parse = inputParser;
    addParameter(p_parse, 'x_start', []);
    addParameter(p_parse, 'disturbance_fn', []);
    parse(p_parse, varargin{:});
    x_start = p_parse.Results.x_start;
    disturbance_fn = p_parse.Results.disturbance_fn;

    % Konstanten
    ctrl_names  = {'PI', 'LQR', 'Decoupled', 'LQI'};
    plot_styles = {'r-', 'g-', 'b-', 'm-'};

    N = round(t_end / h);
    t_vec = (0:N-1) * h;

    % Anfangszustand bestimmen
    [T0, Q0] = scenario_fn(0);
    p_base = ShowerModel();
    [u_init, x_init_op] = p_base.solve_op(T0, Q0);

    if isempty(x_start)
        x_start = x_init_op;
    end

    results = struct();

    for ctrl_id = active_controllers
        name = ctrl_names{ctrl_id + 1};
        fprintf('Simuliere %s...\n', name);

        % Neue Modell-Instanz pro Regler (für unabhängige Parameteränderungen)
        p = ShowerModel();

        % Zustand & Integrator zurücksetzen
        x = x_start;
        err_sum = [0; 0];
        u_op = u_init;
        x_op = x_init_op;
        W = eye(2);
        targets = [T0; Q0];

        % Reglerparameter berechnen
        switch ctrl_id
            case 0, [K, u_op]      = Controller_PI(p, targets(1), targets(2));
            case 1, [K, u_op, x_op] = Controller_LQR(p, targets(1), targets(2));
            case 2, [K, u_op, W]    = Controller_Decoupled(p, targets(1), targets(2));
            case 3, [K, u_op, x_op] = Controller_LQI(p, targets(1), targets(2));
        end

        % Pre-Allocation
        x_hist = zeros(4, N);
        u_hist = zeros(2, N);
        Q_hist = zeros(1, N);
        targets_hist = zeros(2, N);

        % Simulationsschleife
        for k = 1:N
            t = (k-1) * h;

            % Sollwerte aus Szenario
            [T_soll, Q_soll] = scenario_fn(t);
            new_targets = [T_soll; Q_soll];

            % Störungen anwenden (z.B. Druckabfall)
            if ~isempty(disturbance_fn)
                p = disturbance_fn(t, p);
            end

            % Arbeitspunkt-Update bei Sollwertänderung 
            % (für faire Reglerbewertung sinnvoll, in Realität mit Gain Scheduling, zb. mit LUT umgesetzt)
            if any(new_targets ~= targets)
                [u_op, x_op] = p.solve_op(new_targets(1), new_targets(2));
                targets = new_targets;
            end

            % Messung & Fehler
            T_curr = x(4);
            [Q_curr] = p.calculate_flows(x);
            err = targets - [T_curr; Q_curr];

            % Regelgesetz
            switch ctrl_id
                case 1 % LQR
                    u_val = u_op - K * (x - x_op);

                case 3 % LQI
                    dx = x - x_op;
                    err_sum_next = err_sum + err * h;
                    u_val = u_op - K * [dx; err_sum_next];

                    % Anti-Windup
                    if any(u_val > 1 | u_val < 0)
                        err_sum_next = err_sum;
                    end
                    err_sum = err_sum_next;

                otherwise % PI (0) & Decoupled (2)
                    err_sum_next = err_sum + err * h;

                    pi_out = [K.Kp_T * err(1) + K.Ki_T * err_sum_next(1);
                              K.Kp_Q * err(2) + K.Ki_Q * err_sum_next(2)];

                    if ctrl_id == 2 % Decoupled
                        u_desired = u_op + W * pi_out; % Entkopplung
                        if any(u_desired > 1 | u_desired < 0)
                            err_sum_next = err_sum;
                        end
                    else % PI Standard
                        u_desired = u_op + pi_out;
                        % Anti-Windup
                        sat_T = (u_desired(1) > 1 && err(1) > 0) || (u_desired(1) < 0 && err(1) < 0);
                        sat_Q = (u_desired(2) > 1 && err(2) > 0) || (u_desired(2) < 0 && err(2) < 0);
                        if sat_T, err_sum_next(1) = err_sum(1); end
                        if sat_Q, err_sum_next(2) = err_sum(2); end
                    end

                    u_val = u_desired;
                    err_sum = err_sum_next;
            end

            % Sättigung & Simulation
            u_ctrl = max(0, min(1, u_val));
            x = p.rk4_step(x, u_ctrl, h);

            % Speichern
            x_hist(:,k) = x;
            u_hist(:,k) = u_ctrl;
            Q_hist(k) = Q_curr;
            targets_hist(:,k) = targets;
        end

        % Ergebnisse sichern
        res_idx = ctrl_id + 1;
        results(res_idx).x     = x_hist;
        results(res_idx).u     = u_hist;
        results(res_idx).Q     = Q_hist;
        results(res_idx).tgt   = targets_hist;
        results(res_idx).name  = name;
        results(res_idx).style = plot_styles{res_idx};
    end

    fprintf('Simulation abgeschlossen.\n');
end
