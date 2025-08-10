function dxdt_cl = nonlinear_dynamics_static_precomp(t, x, K, N_v, r, m1, m2, l1, l2, g)
% NONLINEAR_DYNAMICS_STATIC_PRECOMP computes the derivative of the state vector for the closed-loop nonlinear system with static precompensator.
%   t: time
%   x: Original state vector [theta1; theta2; d_theta1; d_theta2] (4x1)
%   K: State-feedback gain matrix (2x4)
%   N_v: Static precompensator gain matrix (2x2)
%   r: Reference input vector [r_theta1; r_theta2] (2x1)
%   m1, m2, l1, l2, g: Physical parameters

    % State variables
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
    
    % Control law for the synthetic input v (with static precompensator)
    % v_synthetic = -K * x + N_v * r;
    v_synthetic = -K * x + N_v * r; 
    
    % Feedback linearization control law to get actual torque tau
    tau = M * v_synthetic + C_term + G_term;
    
    % Nonlinear dynamics equation
    d_d_theta = M \ (tau - C_term - G_term);
    dxdt_cl = [d_theta1; d_theta2; d_d_theta(1); d_d_theta(2)];
end