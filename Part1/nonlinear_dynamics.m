function dxdt = nonlinear_dynamics(t, x, tau)
% NONLINEAR_DYNAMICS defines the nonlinear differential equations for a two-link robot arm.
%   t: time
%   x: state vector [theta1; theta2; d_theta1; d_theta2]
%   tau: input torque vector [tau1; tau2]
    
    % Physical parameters (based on the provided document)
    m1 = 1; % Mass of the first link
    m2 = 1; % Mass of the second link
    l1 = 1; % Length of the first link
    l2 = 1; % Length of the second link
    g = 9.81; % Gravity
    
    % State variables
    theta1 = x(1);
    theta2 = x(2);
    d_theta1 = x(3);
    d_theta2 = x(4);
    
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
    
    % The dynamic model is M(theta)*d_d_theta + C(theta, d_theta) + G(theta) = tau
    % Therefore, d_d_theta = inv(M(theta)) * (tau - C(theta, d_theta) - G(theta))
    d_d_theta = M \ (tau - C_term - G_term);
    
    % State-space equations
    dxdt = [d_theta1; d_theta2; d_d_theta(1); d_d_theta(2)];
end


