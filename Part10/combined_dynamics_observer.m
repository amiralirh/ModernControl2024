function dX_total = combined_dynamics_observer(t, X_total, A, B, C_original, K_aug, L, r_val, m1, m2, l1, l2, g)
% COMBINED_DYNAMICS_OBSERVER computes the derivatives for the combined plant and observer system.
% X_total = [x_plant; x_i_plant; x_hat_plant]
% where x_plant is original state (4x1), x_i_plant is actual integral state (2x1)
% x_hat_plant is estimated original state (4x1)

    % Extract states from X_total
    x_plant = X_total(1:4); % Actual plant states (theta, d_theta)
    x_i_plant = X_total(5:6); % Actual integral states (integral of error)
    x_hat_plant = X_total(7:10); % Estimated original states from observer (theta_hat, d_theta_hat)

    % Reconstruct estimated augmented state for controller
    % Controller uses x_hat (from observer) and x_i_plant (actual integral state)
    x_hat_aug_for_controller = [x_hat_plant; x_i_plant];

    % 1. PLANT DYNAMICS (Nonlinear Plant with Integral State)
    
    % Control input from controller using estimated states and actual integral state
    v_synthetic_plant = -K_aug * x_hat_aug_for_controller; 
    
    % Actual torque calculation using feedback linearization (requires actual plant state for M, C_term, G_term)
    theta1_plant = x_plant(1); theta2_plant = x_plant(2);
    d_theta1_plant = x_plant(3); d_theta2_plant = x_plant(4);

    D1_plant = (m1 + m2)*l1^2 + m2*l2^2 + 2*m2*l1*l2*cos(theta2_plant);
    D2_plant = m2*l2^2 + m2*l1*l2*cos(theta2_plant);
    M_plant = [D1_plant, D2_plant; D2_plant, m2*l2^2]; 

    C_term_plant = [-m2*l1*l2*(2*d_theta1_plant*d_theta2_plant + d_theta2_plant^2)*sin(theta2_plant); 
                    -m2*l1*l2*d_theta1_plant*d_theta2_plant*sin(theta2_plant)];
    G_term_plant = [-(m1+m2)*g*l1*sin(theta1_plant) - m2*g*l2*sin(theta1_plant+theta2_plant); 
                    -m2*g*l2*sin(theta1_plant+theta2_plant)];
    
    tau_actual = M_plant * v_synthetic_plant + C_term_plant + G_term_plant;
    
    % Nonlinear dynamics of original plant states
    d_d_theta_plant = M_plant \ (tau_actual - C_term_plant - G_term_plant);
    dxdt_original_plant = [d_theta1_plant; d_theta2_plant; d_d_theta_plant(1); d_d_theta_plant(2)];

    % Dynamics of actual integral states (error integration)
    y_actual = C_original * x_plant; % Actual measurable output
    dxdt_i_plant = r_val - y_actual; % Actual error for integral state
    
    % 2. OBSERVER DYNAMICS (Luenberger Observer for Original 4 States)
    
    % Observer output feedback term: L * (y_actual - C_original * x_hat_plant)
    observer_feedback_term = L * (y_actual - C_original * x_hat_plant);
    
    % Observer dynamics: d(x_hat)/dt = A*x_hat + B*u_control_observer + L*(y_actual - C_original*x_hat)
    u_control_observer = v_synthetic_plant; % The synthetic input used by the controller, which affects the observer
    
    dxdt_hat_plant = A * x_hat_plant + B * u_control_observer + observer_feedback_term;
    
    % Combined derivative vector for ode45
    dX_total = [dxdt_original_plant; dxdt_i_plant; dxdt_hat_plant];
end