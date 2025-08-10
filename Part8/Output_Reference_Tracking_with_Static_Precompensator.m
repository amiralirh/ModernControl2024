%==========================================================================
% Output Reference Tracking with Static Precompensator (Corrected)
%==========================================================================

clc;
clear;
close all;

% 1. Define the Linearized System Matrices (from Feedback Linearization)
A = [0, 0, 1, 0;
     0, 0, 0, 1;
     0, 0, 0, 0;
     0, 0, 0, 0];
B = [0, 0;
     0, 0;
     1, 0;
     0, 1];
C = [1, 0, 0, 0;
     0, 1, 0, 0]; % Output is theta1 and theta2

% Define physical parameters (for dynamics function)
m1 = 1; m2 = 1; l1 = 1; l2 = 1; g = 9.81;

% 2. Choose Desired Closed-Loop Pole Locations (for original 4-state system K)
desired_poles = [-3, -3.1, -4, -4.1]; % Only 4 poles for the 4-state system K

% 3. Calculate the State-Feedback Gain Matrix K (for original 4-state system)
K = place(A, B, desired_poles);
disp('State-Feedback Gain Matrix K:');
disp(K);

% 4. Calculate the Static Precompensator Gain N_v
A_cl = A - B*K; % Closed-loop system matrix
M_precompensator = -C * (A_cl \ B); 
N_v = inv(M_precompensator);
disp('Static Precompensator Gain N_v:');
disp(N_v);

% 5. Define Reference Input (non-zero desired angles)
reference_value = [pi/4; pi/4]; 
disp('Desired Reference Value (rad):');
disp(reference_value);

% 6. Simulation Setup
t_span_sim = [0 5]; % Simulation time
x0_sim = [0; 0; 0; 0]; % Start from origin (4 states only for this method)

% 7. Simulate the Closed-Loop Nonlinear System with Static Precompensator
% Now calling the new specific function 'nonlinear_dynamics_static_precomp'
ode_func_tracking = @(t, x) nonlinear_dynamics_static_precomp(t, x, K, N_v, reference_value, m1, m2, l1, l2, g);
[t_track, x_track] = ode45(ode_func_tracking, t_span_sim, x0_sim);

% 8. Plotting Results (remains largely same)
figure;
sgtitle('Output Reference Tracking with Static Precompensator');

% Plot Joint Angles and Reference
subplot(2,1,1);
plot(t_track, x_track(:,1), 'b-', 'LineWidth', 1.5, 'DisplayName', '\theta_1 (Actual)');
hold on;
plot(t_track, x_track(:,2), 'r--', 'LineWidth', 1.5, 'DisplayName', '\theta_2 (Actual)');
plot(t_track, ones(size(t_track)) * reference_value(1), 'k:', 'LineWidth', 1.5, 'DisplayName', 'Reference \theta_1');
plot(t_track, ones(size(t_track)) * reference_value(2), 'm:', 'LineWidth', 1.5, 'DisplayName', 'Reference \theta_2');
title('Joint Angles Tracking');
xlabel('Time (s)'); ylabel('Angle (rad)');
legend('show'); grid on; box on;

% Plot Angular Velocities
subplot(2,1,2);
plot(t_track, x_track(:,3), 'b-', 'LineWidth', 1.5, 'DisplayName', 'd\theta_1');
hold on;
plot(t_track, x_track(:,4), 'r--', 'LineWidth', 1.5, 'DisplayName', 'd\theta_2');
title('Angular Velocities');
xlabel('Time (s)'); ylabel('Velocity (rad/s)');
legend('show'); grid on; box on;

% Optionally plot control input tau (requires calculation in the loop)
tau_track = zeros(size(x_track, 1), 2);
for i = 1:size(x_track, 1)
    current_x = x_track(i,:)';
    % v_synthetic_val = -K * current_x + N_v * reference_value; % K is 2x4, x is 4x1, N_v is 2x2, ref is 2x1
    v_synthetic_val = -K * current_x + N_v * reference_value; 
    tau_track(i,:) = feedback_linearization(current_x, v_synthetic_val)';
end

figure;
sgtitle('Control Input Signals');
subplot(2,1,1);
plot(t_track, tau_track(:,1), 'b-', 'LineWidth', 1.5, 'DisplayName', '\tau_1');
title('Control Input \tau_1');
xlabel('Time (s)'); ylabel('Torque (Nm)');
legend('show'); grid on; box on;

subplot(2,1,2);
plot(t_track, tau_track(:,2), 'r--', 'LineWidth', 1.5, 'DisplayName', '\tau_2');
title('Control Input \tau_2');
xlabel('Time (s)'); ylabel('Torque (Nm)');
legend('show'); grid on; box on;