function tau = feedback_linearization(x, v_synthetic)
% FEEDBACK_LINEARIZATION implements the feedback linearization control law.
%   x: state vector [theta1; theta2; d_theta1; d_theta2]
%   v_synthetic: synthetic control vector [v1; v2]

    % Physical parameters (ensure these match your main script)
    m1 = 1; 
    m2 = 1; 
    l1 = 1; 
    l2 = 1; 
    g = 9.81;
    
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
    
    % Feedback linearization control law: tau = M*v_synthetic + C_term + G_term
    tau = M * v_synthetic + C_term + G_term;
end