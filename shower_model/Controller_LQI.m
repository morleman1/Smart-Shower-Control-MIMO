function [K_LQI, u_op, x_op] = Controller_LQI(p, T_soll, Q_soll)

    [u_op, x_op] = p.solve_op(T_soll, Q_soll);
    [A, B, C, ~] = p.get_linearized_model(x_op, u_op);

    nx = size(A, 1);
    nu = size(B, 2);
    ny = size(C, 1);

    % Zustandsraum-Erweiterung (für Integralanteil)
    % Neuer Zustand z = [x; xi] mit xi_dot = r - y = -C*x (im Abweichungsmodell)
    % Systemdynamik: [dx; dxi] = [A 0; -C 0] * [x; xi] + [B; 0] * u
    A_aug = [A, zeros(nx, ny); 
             -C, zeros(ny, ny)];
         
    B_aug = [B; zeros(ny, nu)];
    
    % Gewichtungsmatrizen
    Q_y_err = diag([1, 12000]); % Gewichtung Ausgangsfehler (P-Anteil)
    Q_i_err = Q_y_err * 0.5;    % Gewichtung Integralfehler (I-Anteil)
    R       = diag([5, 5]);     % Gewichtung Stellgrößen
    
    % Transformation auf Zustände x: x' (C'QC) x
    Q_x = C' * Q_y_err * C;
    
    % Gesamtkostenmatrix für z = [x; xi]
    Q_aug = blkdiag(Q_x, Q_i_err);
    
    K_LQI = lqr(A_aug, B_aug, Q_aug, R);

    disp('LQI Design erfolgreich.');
    disp(['Closed-Loop Eigenvalues: ', mat2str(eig(A_aug - B_aug*K_LQI), 2)]);
end
