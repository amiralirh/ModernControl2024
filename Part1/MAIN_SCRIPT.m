% MAIN_SCRIPT simulates the open-loop behavior of the two-link robot arm.

% Clear previous data
clc;
clear;
close all;

% Set simulation time span
t_span = [0 10]; % e.g., from 0 to 10 seconds

% Define a zero input torque for open-loop simulation
tau = [0; 0]; % Open-loop, no external torque applied

% Define initial conditions based on the problem statement
% x = [theta1, theta2, d_theta1, d_theta2]^T
x0_case1 = [0; 0; 0; 0]; % Case 1: Initial position at rest
x0_case2 = [pi/2; -pi/2; 0; 0]; % Case 2: Initial angles as specified in Table 3 [cite: 201]

% --- Simulation for Case 1 ---
disp('Simulating Case 1: Initial state [0; 0; 0; 0]');
[t1, x1] = ode45(@(t, x) nonlinear_dynamics(t, x, tau), t_span, x0_case1);

figure;
subplot(2,1,1);
plot(t1, x1(:,1), 'b-', t1, x1(:,2), 'r--');
title('Open-Loop Response - Initial Position at Origin');
xlabel('Time (s)');
ylabel('Angle (rad)');
legend('theta1', 'theta2');
grid on;

subplot(2,1,2);
plot(t1, x1(:,3), 'b-', t1, x1(:,4), 'r--');
xlabel('Time (s)');
ylabel('Angular Velocity (rad/s)');
legend('dtheta1', 'dtheta2');
grid on;

% --- Simulation for Case 2 ---
disp('Simulating Case 2: Initial state [pi/2; -pi/2; 0; 0]');
[t2, x2] = ode45(@(t, x) nonlinear_dynamics(t, x, tau), t_span, x0_case2);

figure;
subplot(2,1,1);
plot(t2, x2(:,1), 'b-', t2, x2(:,2), 'r--');
title('Open-Loop Response - Initial Angles [pi/2; -pi/2]');
xlabel('Time (s)');
ylabel('Angle (rad)');
legend('theta1', 'theta2');
grid on;

subplot(2,1,2);
plot(t2, x2(:,3), 'b-', t2, x2(:,4), 'r--');
xlabel('Time (s)');
ylabel('Angular Velocity (rad/s)');
legend('dtheta1', 'dtheta2');
grid on;