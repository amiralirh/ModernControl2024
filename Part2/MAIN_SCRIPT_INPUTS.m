% MAIN_SCRIPT_INPUTS simulates the open-loop response to various inputs.

clc;
clear;
close all;

% Set simulation time span and initial conditions
t_span = [0 10];
x0 = [0; 0; 0; 0]; % Starting from the origin for simplicity

%% 1. Response to Unit Step Input
disp('Simulating response to a unit step input...');

% Define step input function (1 Nm torque for both joints)
step_input = @(t) [1; 1];

% Simulate the system with step input
[t_step, x_step] = ode45(@(t, x) nonlinear_dynamics(t, x, step_input(t)), t_span, x0);

figure;
subplot(2,1,1);
plot(t_step, x_step(:,1), 'b-', t_step, x_step(:,2), 'r--');
title('Open-Loop Response to Unit Step Input');
xlabel('Time (s)');
ylabel('Angle (rad)');
legend('theta1', 'theta2');
grid on;

subplot(2,1,2);
plot(t_step, x_step(:,3), 'b-', t_step, x_step(:,4), 'r--');
xlabel('Time (s)');
ylabel('Angular Velocity (rad/s)');
legend('dtheta1', 'dtheta2');
grid on;

%% 2. Response to Unit Ramp Input
disp('Simulating response to a unit ramp input...');

% Define ramp input function (t Nm torque for both joints)
ramp_input = @(t) [t; t];

% Simulate the system with ramp input
[t_ramp, x_ramp] = ode45(@(t, x) nonlinear_dynamics(t, x, ramp_input(t)), t_span, x0);

figure;
subplot(2,1,1);
plot(t_ramp, x_ramp(:,1), 'b-', t_ramp, x_ramp(:,2), 'r--');
title('Open-Loop Response to Unit Ramp Input');
xlabel('Time (s)');
ylabel('Angle (rad)');
legend('theta1', 'theta2');
grid on;

subplot(2,1,2);
plot(t_ramp, x_ramp(:,3), 'b-', t_ramp, x_ramp(:,4), 'r--');
xlabel('Time (s)');
ylabel('Angular Velocity (rad/s)');
legend('dtheta1', 'dtheta2');
grid on;

%% 3. Response to Unit Impulse Input
disp('Simulating response to a unit impulse input...');

% Note: A true impulse is an idealization. In simulation, it's modeled as an instantaneous change in initial conditions.
% An impulse torque corresponds to a sudden change in angular momentum, which results in a step change in angular velocity.
% We can simulate a unit impulse by setting the initial angular velocity to a non-zero value.
% A unit impulse in torque (1 N.m.s) on a 1 kg, 1m arm would impart a delta_d_theta ~ 1/J. Let's assume a simple case.
x0_impulse = [0; 0; 1; 1]; % Initial velocity of 1 rad/s for both joints

% Simulate the system with zero torque after the initial impulse
[t_impulse, x_impulse] = ode45(@(t, x) nonlinear_dynamics(t, x, [0; 0]), t_span, x0_impulse);

figure;
subplot(2,1,1);
plot(t_impulse, x_impulse(:,1), 'b-', t_impulse, x_impulse(:,2), 'r--');
title('Open-Loop Response to Unit Impulse Input');
xlabel('Time (s)');
ylabel('Angle (rad)');
legend('theta1', 'theta2');
grid on;

subplot(2,1,2);
plot(t_impulse, x_impulse(:,3), 'b-', t_impulse, x_impulse(:,4), 'r--');
xlabel('Time (s)');
ylabel('Angular Velocity (rad/s)');
legend('dtheta1', 'dtheta2');
grid on;