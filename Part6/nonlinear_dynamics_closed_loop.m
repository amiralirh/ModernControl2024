function dxdt_cl = nonlinear_dynamics_closed_loop(t, x_aug, K_aug, r, m1, m2, l1, l2, g)
% NONLINEAR_DYNAMICS_CLOSED_LOOP computes the derivative of the state vector for the closed-loop nonlinear system with integral control.
%   t: time
%   x_aug: Augmented state vector [x; xi] where x is [theta1; theta2; d_theta1; d_theta2] and xi is [integral_error_theta1; integral_error_theta2]
%   K_aug: Augmented state-feedback gain matrix [K Ki]
%   r: Reference input vector [r_theta1; r_theta2]
%   m1, m2, l1, l2, g: Physical parameters

    % Extract original states and integral states
    x = x_aug(1:4); % Original states
    xi = x_aug(5:6); % Integral states (integral of error)
    
    % State variables for physical dynamics
    theta1 = x(1);
    theta2 = x(2);
    d_theta1 = x(3);
    d_theta2 = x(4);
    
    % Physical parameters (local scope for this function)
    
    % M(theta) inertia matrix
    D1 = (m1 + m2)*l1^2 + m2*l2^2 + 2*m2*l1*l2*cos(theta2);
    D2 = m2*l2^2 + m2*l1*l2*cos(theta2);
    D3 = D2; 
    D4 = m2*l2^2;
    M = [D1, D2; D3, D4];
    
    % C(theta, d_theta) Coriolis and centrifugal forces
    C_term = [-m2*l1*l2*(2*d_theta1*d_theta2 + d_theta2^2)*sin(theta2); -m2*l1*l2*d_theta1*d_theta2*sin(theta2)];
    
    % G(theta) gravity torques
    G_term = [-(m1+m2)*g*l1*sin(theta1) - m2*g*l2*sin(theta1+theta2); -m2*g*l2*sin(theta1+theta2)];
    
    % Control law for the synthetic input v
    % v_synthetic = -K_aug * x_aug;
    v_synthetic = -K_aug * x_aug; 
    
    % Feedback linearization control law to get actual torque tau
    tau = M * v_synthetic + C_term + G_term;
    
    % Nonlinear dynamics of original states
    d_d_theta = M \ (tau - C_term - G_term);
    dxdt_original = [d_theta1; d_theta2; d_d_theta(1); d_d_theta(2)];

    % Output matrix for original states (to calculate error)
    C_original = [1, 0, 0, 0; 0, 1, 0, 0];
    
    % Integral of error dynamics
    % d(xi)/dt = error = r - y = r - C_original * x
    dxdt_integral_error = r - C_original * x;
    
    % Combined derivative for augmented state vector
    dxdt_cl = [dxdt_original; dxdt_integral_error];
end