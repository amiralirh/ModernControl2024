%==========================================================================
% Compare Open-Loop Response of Nonlinear and Linearized Systems
%==========================================================================

clc;
clear;
close all;

% Define simulation time span
t_span = [0 5];
% Create a time vector with a suitable step size (e.g., 0.01s)
t_linear = t_span(1):0.01:t_span(2);

% Define initial conditions based on the project document
% x = [theta1, theta2, d_theta1, d_theta2]^T
x0_nonlinear = [pi/2; -pi/2; 0; 0];
x0_linearized = [pi/2; -pi/2; 0; 0];

% Define A, B, and C matrices for the linearized system (from previous steps)
A = [0, 0, 1.0, 0;
     0, 0, 0, 1.0;
     -29.4300, 9.8100, 0, 0;
     19.6200, -9.8100, 0, 0];

B = [0, 0;
     0, 0;
     0.5, -1.0;
     -1.0, 3.0];
 
C = [1, 0, 0, 0;
     0, 1, 0, 0];

% Define the input for open-loop simulation (zero torque/control)
u_nonlinear = [0; 0]; % For nonlinear model

% Corrected: Create a zero input matrix with dimensions (number_of_time_points x number_of_inputs)
u_linearized = zeros(length(t_linear), 2);

%==========================================================================
% 1. Simulate the Nonlinear System
%==========================================================================
% Use the 'nonlinear_dynamics' function from the previous response.
% Make sure the function file is in the same directory.
disp('Simulating Nonlinear System...');
[t_nonlinear, x_nonlinear] = ode45(@(t, x) nonlinear_dynamics(t, x, u_nonlinear), t_span, x0_nonlinear);

%==========================================================================
% 2. Simulate the Linearized System
%==========================================================================
% Convert the state-space matrices to a state-space object
sys_lin = ss(A, B, C, zeros(2,2));

% Simulate the linear system response using lsim
disp('Simulating Linearized System...');
[y_linear, ~, x_linear] = lsim(sys_lin, u_linearized, t_linear, x0_linearized);

%==========================================================================
% 3. Plotting and Comparison
%==========================================================================
figure;

subplot(2,1,1);
plot(t_nonlinear, x_nonlinear(:,1), 'b-', 'LineWidth', 1.5);
hold on;
plot(t_nonlinear, x_nonlinear(:,2), 'b--', 'LineWidth', 1.5);
plot(t_linear, y_linear(:,1), 'r-', 'LineWidth', 1.5);
plot(t_linear, y_linear(:,2), 'r--', 'LineWidth', 1.5);
title('Comparison of Joint Angles (Open-Loop)');
xlabel('Time (s)');
ylabel('Angle (rad)');
legend('theta1 (Nonlinear)', 'theta2 (Nonlinear)', 'theta1 (Linearized)', 'theta2 (Linearized)');
grid on;
box on;

subplot(2,1,2);
plot(t_nonlinear, x_nonlinear(:,3), 'b-', 'LineWidth', 1.5);
hold on;
plot(t_nonlinear, x_nonlinear(:,4), 'b--', 'LineWidth', 1.5);
plot(t_linear, x_linear(:,3), 'r-', 'LineWidth', 1.5);
plot(t_linear, x_linear(:,4), 'r--', 'LineWidth', 1.5);
title('Comparison of Angular Velocities (Open-Loop)');
xlabel('Time (s)');
ylabel('Angular Velocity (rad/s)');
legend('dtheta1 (Nonlinear)', 'dtheta2 (Nonlinear)', 'dtheta1 (Linearized)', 'dtheta2 (Linearized)');
grid on;
box on;