function [K_LQR, u_op, x_op] = Controller_LQR(p, T_soll, Q_soll)

    [u_op, x_op] = p.solve_op(T_soll, Q_soll);
    [A, B, C, D] = p.get_linearized_model(x_op, u_op);

    % Gewichtungsmatrizen
    % Q: Bestrafung der Regelabweichung der Ausgänge y = [T, Q]
    % R: Bestrafung der Stellgrößenänderung u = [u_h, u_c]
    Q_y = diag([1, 12000]); 
    R   = diag([5, 5]);     

    % Transformation auf Zustandsraum (Kostenfunktion J = x'Qx + u'Ru + 2x'Nu)
    % Da wir Ausgänge y gewichten wollen, gilt: y'Qy = (Cx+Du)'Q(Cx+Du)
    Q_lqr = C' * Q_y * C;
    R_lqr = R + D' * Q_y * D;
    N_lqr = C' * Q_y * D;

    K_LQR = lqr(A, B, Q_lqr, R_lqr, N_lqr);
    
    disp('LQR Design erfolgreich.');
    disp(['Eigenwerte Closed-Loop: ', mat2str(eig(A - B*K_LQR), 2)]);
end