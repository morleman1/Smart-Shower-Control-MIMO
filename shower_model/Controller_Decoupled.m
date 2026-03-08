function [K_params, u_op, W] = Controller_Decoupled(p, T_soll, Q_soll)
    
    [u_op, x_op] = p.solve_op(T_soll, Q_soll);
    [A, B, C, D] = p.get_linearized_model(x_op, u_op);
    sys_ss = ss(A, B, C, D);

    % Statische Entkopplung (Inverse der Gleichstromverstärkung)
    W = inv(dcgain(sys_ss));
    
    % Virtuelles, entkoppeltes System (weiterhin State-Space)
    sys_virt = sys_ss * W;

    C_T = pidtune(sys_virt(1,1), 'PI');
    C_Q = pidtune(sys_virt(2,2), 'PI');

    K_params.Kp_T = C_T.Kp; K_params.Ki_T = C_T.Ki;
    K_params.Kp_Q = C_Q.Kp; K_params.Ki_Q = C_Q.Ki;
    
    disp('Entkopplungs-Matrix W:');
    disp(W);
end