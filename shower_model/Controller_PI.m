% PI-Regler, der die System-Kopplung ingoriert und als zwei einzelne Strecken betrachtet

function [K_params, u_op, sys_ss] = Controller_PI(p, T_soll, Q_soll)
    
    [u_op, x_op] = p.solve_op(T_soll, Q_soll);
    [A, B, C, D] = p.get_linearized_model(x_op, u_op);
    
    % Gesamtsystem als State-Space Objekt (Zustandsraum)
    sys_ss = ss(A, B, C, D);

    % Entkoppelte Betrachtung: Nur Hauptdiagonalelemente nutzen
    % G_T: u_h -> T_sens
    % G_Q: u_c -> Q_out
    G_T = sys_ss(1,1);
    G_Q = sys_ss(2,2);

    % Automatische Auslegung
    C_T = pidtune(G_T, 'PI'); 
    C_Q = pidtune(G_Q, 'PI');

    K_params.Kp_T = C_T.Kp; K_params.Ki_T = C_T.Ki;
    K_params.Kp_Q = C_Q.Kp; K_params.Ki_Q = C_Q.Ki;
    
    disp(K_params);
end