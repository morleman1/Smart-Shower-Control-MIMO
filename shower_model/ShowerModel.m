classdef ShowerModel

    properties
        V_mix = 0.001;   % [m^3] Mischvolumen (0.5l)
        rho_w = 997;      % [kg/m^3] Dichte
        Kv_h  = 0.8;      % [m^3/h] Durchflusskoeffizienten
        Kv_c  = 0.8;    
        Kv_nozzle = 0.8; 
        P_h   = 3.0;      % [bar] Versorgungsdrücke
        P_c   = 3.5; 
        T_h   = 60;       % [°C] Temperaturen
        T_c   = 15;  
        tau_valve = 0.4;  % [s] Zeitkonstanten
        tau_sensor = 0.2; 
    end

    methods
        function [Q_out, Qh, Qc, P_mix] = calculate_flows(obj, x)

            u_h = x(2);
            u_c = x(3);
            
            Gin = u_h * obj.Kv_h + u_c * obj.Kv_c;
            
            if Gin <= 1e-6
                Qh = 0; Qc = 0; Q_out = 0; P_mix = 0;
                return;
            end
            
            P_supply = (u_h * obj.Kv_h * obj.P_h + u_c * obj.Kv_c * obj.P_c) / Gin;
            P_mix = P_supply * (Gin^2) / (Gin^2 + obj.Kv_nozzle^2);
            
            Qh = (u_h * obj.Kv_h * sqrt(max(0, (obj.P_h - P_mix) * 1000 / obj.rho_w)));
            Qc = (u_c * obj.Kv_c * sqrt(max(0, (obj.P_c - P_mix) * 1000 / obj.rho_w)));
            Q_out = Qh + Qc;
        end
        
        function dx = f_shower(obj, x, u)
            % Zustände: x(1)=T_out (Mixer), x(2)=u_h_ist, x(3)=u_c_ist, x(4)=T_sens
            % Eingänge: u(1)=u_h_soll, u(2)=u_c_soll
            T_out = x(1);
            u_h_ist = x(2); 
            u_c_ist = x(3);
            T_sens = x(4);
            
            [Qout, Qh, Qc] = obj.calculate_flows(x);
        
            % Ableitungen berechnen
            dTdt = (1/obj.V_mix) * ((Qh * obj.T_h + Qc * obj.T_c - Qout * T_out)/3600);
            duhdt = (u(1) - u_h_ist) / obj.tau_valve; % Ventildynamik (PT1)
            ducdt = (u(2) - u_c_ist) / obj.tau_valve;
            dTsensdt = (T_out - T_sens) / obj.tau_sensor;
            
            dx = [dTdt; duhdt; ducdt; dTsensdt];
        end

        function x_next = rk4_step(obj, x, u, h)
            k1 = obj.f_shower(x, u);
            k2 = obj.f_shower(x + h/2*k1, u);
            k3 = obj.f_shower(x + h/2*k2, u);
            k4 = obj.f_shower(x + h*k3, u);
            x_next = x + h/6 * (k1 + 2*k2 + 2*k3 + k4);
        end

        % Berechnet Ventil-Stellung für AP-Werte
        function [u_op, x_op] = solve_op(obj, T_soll, Q_soll) 
            % Arbeitspunkt: Suche Ventilsteuerungen für gewünschtes T & Q
            cost_fn = @(u) obj.steady_state_error(u, T_soll, Q_soll); % SAgt, ob Ventilstellung zum gewünschten Ergebnis führt
            
            opts = optimoptions('fsolve', 'Display', 'none', 'FunctionTolerance', 1e-6);
            u_start = [0.5; 0.5]; 
            u_op = fsolve(cost_fn, u_start, opts); % In Realität würde man Kennfelder/LUT nutzen. 
            % fsolve verlässt sich auf perfektes Modell und ist sehr performance-teuer
            
            % Stationärer Zustand: Position = Soll-Position, Sensor = Mix
            x_op = [T_soll; u_op(1); u_op(2); T_soll];
        end

        function [A,B,C,D] = get_linearized_model(obj, x_op, u_op)
            % Numerische Linearisierung (Finite Differenzen)
            % System wird angestupst, um rauszufinden, wie es sich bei Änderungen verhält. 
            % Das Verhalten wird dann als Gerade angenommen, als ob das System linear wäre. 
            % So können die Regler anhand A B C D designed werden. Die Matrizen enthalten die
            % Gewichtungen, wie stark man einen Wert ändern muss, um das Ziel zu erreichen. 
            nx = 4;
            nu = 2;
            ny = 2;
            
            delta = 1e-5; % Perturbation size
            
            A = zeros(nx, nx);
            B = zeros(nx, nu);
            C = zeros(ny, nx);
            D = zeros(ny, nu);
            
            % Nominal point
            dx0 = obj.f_shower(x_op, u_op);
            [Q_out0] = obj.calculate_flows(x_op);
            y0 = [x_op(4); Q_out0];
            
            % A: Störung von x, Wirkung auf dx gemessen - Eigendynamik
            % C: Störung von x, Wirkung auf y gemessen - Beobachtbarkeit
            for i = 1:nx
                x_p = x_op;
                x_p(i) = x_p(i) + delta; % Stups
                
                dx_p = obj.f_shower(x_p, u_op);
                [Q_out_p] = obj.calculate_flows(x_p);
                y_p = [x_p(4); Q_out_p];
                
                A(:, i) = (dx_p - dx0) / delta; % Steigung am Punkt
                C(:, i) = (y_p - y0) / delta;
            end
            
            % B: Störung von u, Wirkung auf dx gemessen - Steuerbarkeit
            % D: Störung von u, Wirkung auf y gemessen - Durchgriff
            for j = 1:nu
                u_p = u_op;
                u_p(j) = u_p(j) + delta;
                
                dx_p = obj.f_shower(x_op, u_p);
                [Q_out_p] = obj.calculate_flows(x_op);
                
                y_p = [x_op(4); Q_out_p];
                
                B(:, j) = (dx_p - dx0) / delta;
                D(:, j) = (y_p - y0) / delta;
            end
        
            sys_ss = ss(A, B, C, D);
            sys_ss.StateName = {'T_{mix}', 'u_{h,ist}', 'u_{c,ist}', 'T_{sens}'};
            sys_ss.InputName = {'u_{h,soll}', 'u_{c,soll}'};
            sys_ss.OutputName = {'T_{sens}', 'Q_{out}'};
        end
    end

    methods (Access = private)
         function err = steady_state_error(obj, u_vals, T_t, Q_t)
            % Hilfsfunktion für fsolve
            % Stationärer Zustand: Eingestelltes Ventil = Wirkliches Ventil
            x_stat = [0; u_vals(1); u_vals(2); 0]; 
            [Q_out, Qh, Qc] = obj.calculate_flows(x_stat);
            
            if Q_out > 1e-6
                T_mix = (Qh * obj.T_h + Qc * obj.T_c) / Q_out;
            else
                T_mix = 0;
            end
            err = [T_mix - T_t; Q_out - Q_t];
        end
        
        function y = measurement_eq(obj, x)
            % y = [T_sensor, Q_out]
            [Q_out] = obj.calculate_flows(x);
            y = [x(4); Q_out];
        end
    end
end

