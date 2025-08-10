%==========================================================================
% Comparing Different Desired Pole Locations (Corrected for reference_value scope)
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
% Define C_original here, needed for nonlinear_dynamics_closed_loop (integral part)
C_original = [1, 0, 0, 0; 0, 1, 0, 0]; 

% Define physical parameters (needed for nonlinear dynamics and feedback linearization)
m1 = 1; m2 = 1; l1 = 1; l2 = 1; g = 9.81;

% Initial condition for simulation (a common starting point)
% IMPORTANT: x0_sim must now be 6x1 to match nonlinear_dynamics_closed_loop's expectation
x0_sim = [pi/2; -pi/2; 0; 0; 0; 0]; % Original states (4) + Integral states (2, initialized to 0)
t_span_sim = [0 5]; % Simulation time

% Define reference_value HERE, before it's used in ode_func_fast/slow
reference_value = [0;0]; % For stabilization, reference is zero (as per previous problem context)

% For pole placement, A_aug_controller and B_aug_controller are needed
A_aug_controller = [A, zeros(size(A,1), size(C_original,1)); 
                   -C_original, zeros(size(C_original,1), size(C_original,1))]; 
B_aug_controller = [B; zeros(size(C_original,1), size(B,2))];


%% Case 1: Fast & Damped Poles
disp('--- Simulating with FAST & DAMPED Poles ---');
desired_poles_fast = [-3, -3.1, -4, -4.1, -4.2, -4.3]; % Need 6 poles for 6 states
K_fast_aug = place(A_aug_controller, B_aug_controller, desired_poles_fast); % K_aug for fast poles
disp('K_aug matrix for FAST poles:');
disp(K_fast_aug);

% Anonymous function for ode45 (binds K_fast_aug and parameters)
ode_func_fast = @(t, x_aug) nonlinear_dynamics_closed_loop(t, x_aug, K_fast_aug, reference_value, m1, m2, l1, l2, g); % reference_value is now defined
[t_fast, x_fast_aug] = ode45(ode_func_fast, t_span_sim, x0_sim); % Pass 6-state x0_sim
x_fast = x_fast_aug(:, 1:4); % Extract original states for plotting


% Calculate control input (tau) for fast poles
tau_fast = zeros(size(x_fast_aug, 1), 2);
for i = 1:size(x_fast_aug, 1)
    current_x_aug = x_fast_aug(i,:)';
    v_synthetic_fast = -K_fast_aug * current_x_aug; % u = -Kx
    
    % Need actual original states to calculate M, C_term, G_term for feedback_linearization
    current_x_original_for_tau_calc = current_x_aug(1:4); 
    tau_fast(i,:) = feedback_linearization(current_x_original_for_tau_calc, v_synthetic_fast)';
end


%% Case 2: Slower & Damped Poles
disp('--- Simulating with SLOWER & DAMPED Poles ---');
desired_poles_slow = [-1, -1.1, -1.2, -1.3, -1.4, -1.5]; % Need 6 poles for 6 states
K_slow_aug = place(A_aug_controller, B_aug_controller, desired_poles_slow); % K_aug for slow poles
disp('K_aug matrix for SLOW poles:');
disp(K_slow_aug);

% Anonymous function for ode45 (binds K_slow_aug and parameters)
ode_func_slow = @(t, x_aug) nonlinear_dynamics_closed_loop(t, x_aug, K_slow_aug, reference_value, m1, m2, l1, l2, g); % reference_value is now defined
[t_slow, x_slow_aug] = ode45(ode_func_slow, t_span_sim, x0_sim); % Pass 6-state x0_sim
x_slow = x_slow_aug(:, 1:4); % Extract original states for plotting


% Calculate control input (tau) for slow poles
tau_slow = zeros(size(x_slow_aug, 1), 2);
for i = 1:size(x_slow_aug, 1)
    current_x_aug = x_slow_aug(i,:)';
    v_synthetic_slow = -K_slow_aug * current_x_aug; % u = -Kx

    % Need actual original states to calculate M, C_term, G_term for feedback_linearization
    current_x_original_for_tau_calc = current_x_aug(1:4); 
    tau_slow(i,:) = feedback_linearization(current_x_original_for_tau_calc, v_synthetic_slow)';
end


%% Plotting and Comparison

% reference_value is already defined as [0;0] for stabilization comparison

figure;
sgtitle('Comparison of System Performance with Different Pole Choices');

% Angles (theta1)
subplot(3,2,1);
plot(t_fast, x_fast(:,1), 'b-', 'LineWidth', 1.5);
hold on;
plot(t_slow, x_slow(:,1), 'r--', 'LineWidth', 1.5);
plot(t_fast, ones(size(t_fast)) * reference_value(1), 'k:', 'DisplayName', 'Reference \theta_1'); % Plot reference
title('Joint 1 Angle (\theta_1)');
xlabel('Time (s)'); ylabel('Angle (rad)');
legend('Fast Poles', 'Slow Poles', 'Location', 'best'); grid on;

% Angles (theta2)
subplot(3,2,2);
plot(t_fast, x_fast(:,2), 'b-', 'LineWidth', 1.5);
hold on;
plot(t_slow, x_slow(:,2), 'r--', 'LineWidth', 1.5);
plot(t_fast, ones(size(t_fast)) * reference_value(2), 'm:', 'DisplayName', 'Reference \theta_2'); % Plot reference
title('Joint 2 Angle (\theta_2)');
xlabel('Time (s)'); ylabel('Angle (rad)');
legend('Fast Poles', 'Slow Poles', 'Location', 'best'); grid on;

% Velocities (d_theta1)
subplot(3,2,3);
plot(t_fast, x_fast(:,3), 'b-', 'LineWidth', 1.5);
hold on;
plot(t_slow, x_slow(:,3), 'r--', 'LineWidth', 1.5);
title('Joint 1 Angular Velocity (d\theta_1)');
xlabel('Time (s)'); ylabel('Velocity (rad/s)');
legend('Fast Poles', 'Slow Poles', 'Location', 'best'); grid on;

% Velocities (d_theta2)
subplot(3,2,4);
plot(t_fast, x_fast(:,4), 'b-', 'LineWidth', 1.5);
hold on;
plot(t_slow, x_slow(:,4), 'r--', 'LineWidth', 1.5);
title('Joint 2 Angular Velocity (d\theta_2)');
xlabel('Time (s)'); ylabel('Velocity (rad/s)');
legend('Fast Poles', 'Slow Poles', 'Location', 'best'); grid on;

% Control Input (tau1)
subplot(3,2,5);
plot(t_fast, tau_fast(:,1), 'b-', 'LineWidth', 1.5);
hold on;
plot(t_slow, tau_slow(:,1), 'r--', 'LineWidth', 1.5);
title('Control Input \tau_1');
xlabel('Time (s)'); ylabel('Torque (Nm)');
legend('Fast Poles', 'Slow Poles', 'Location', 'best'); grid on;

% Control Input (tau2)
subplot(3,2,6);
plot(t_fast, tau_fast(:,2), 'b-', 'LineWidth', 1.5);
hold on;
plot(t_slow, tau_slow(:,2), 'r--', 'LineWidth', 1.5);
title('Control Input \tau_2');
xlabel('Time (s)'); ylabel('Torque (Nm)');
legend('Fast Poles', 'Slow Poles', 'Location', 'best'); grid on;