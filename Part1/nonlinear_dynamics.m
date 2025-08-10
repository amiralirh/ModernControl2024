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


% MAIN_SCRIPT simulates the open-loop behavior of the two-link robot arm.

% Clear previous data
%               copy this part into command window
% clc;
% clear;
% close all;
% 
% % Set simulation time span
% t_span = [0 10]; % e.g., from 0 to 10 seconds
% 
% % Define a zero input torque for open-loop simulation
% tau = [0; 0]; % Open-loop, no external torque applied
% 
% % Define initial conditions based on the problem statement
% % x = [theta1, theta2, d_theta1, d_theta2]^T
% x0_case1 = [0; 0; 0; 0]; % Case 1: Initial position at rest
% x0_case2 = [pi/2; -pi/2; 0; 0]; % Case 2: Initial angles as specified in Table 3 [cite: 201]
% 
% % --- Simulation for Case 1 ---
% disp('Simulating Case 1: Initial state [0; 0; 0; 0]');
% [t1, x1] = ode45(@(t, x) nonlinear_dynamics(t, x, tau), t_span, x0_case1);
% 
% figure;
% subplot(2,1,1);
% plot(t1, x1(:,1), 'b-', t1, x1(:,2), 'r--');
% title('Open-Loop Response - Initial Position at Origin');
% xlabel('Time (s)');
% ylabel('Angle (rad)');
% legend('theta1', 'theta2');
% grid on;
% 
% subplot(2,1,2);
% plot(t1, x1(:,3), 'b-', t1, x1(:,4), 'r--');
% xlabel('Time (s)');
% ylabel('Angular Velocity (rad/s)');
% legend('dtheta1', 'dtheta2');
% grid on;
% 
% % --- Simulation for Case 2 ---
% disp('Simulating Case 2: Initial state [pi/2; -pi/2; 0; 0]');
% [t2, x2] = ode45(@(t, x) nonlinear_dynamics(t, x, tau), t_span, x0_case2);
% 
% figure;
% subplot(2,1,1);
% plot(t2, x2(:,1), 'b-', t2, x2(:,2), 'r--');
% title('Open-Loop Response - Initial Angles [pi/2; -pi/2]');
% xlabel('Time (s)');
% ylabel('Angle (rad)');
% legend('theta1', 'theta2');
% grid on;
% 
% subplot(2,1,2);
% plot(t2, x2(:,3), 'b-', t2, x2(:,4), 'r--');
% xlabel('Time (s)');
% ylabel('Angular Velocity (rad/s)');
% legend('dtheta1', 'dtheta2');
% grid on;