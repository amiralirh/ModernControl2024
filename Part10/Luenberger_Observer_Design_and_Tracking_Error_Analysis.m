%==========================================================================
% Luenberger Observer Design (for 4 states) and Tracking Error Analysis
%==========================================================================

clc;
clear;
close all;

% 1. Define the Linearized System Matrices (original A, B, C)
A = [0, 0, 1, 0;
     0, 0, 0, 1;
     0, 0, 0, 0;
     0, 0, 0, 0];
B = [0, 0;
     0, 0;
     1, 0;
     0, 1];
C_original = [1, 0, 0, 0;
              0, 1, 0, 0]; % Output is theta1 and theta2

% 2. Augment the System for Integral Control (for CONTROLLER design)
A_aug_controller = [A, zeros(size(A,1), size(C_original,1));
                   -C_original, zeros(size(C_original,1), size(C_original,1))]; 
B_aug_controller = [B;
                   zeros(size(C_original,1), size(B,2))];

% 3. Controller Design (K_aug for augmented system)
desired_poles_aug_controller = [-2, -2.1, -2.2, -2.3, -2.4, -2.5]; % 6 poles
K_aug = place(A_aug_controller, B_aug_controller, desired_poles_aug_controller);
disp('Augmented State-Feedback Gain Matrix K_aug (Controller):');
disp(K_aug);

% 4. Observer Design (L for ORIGINAL 4 states)
% Choose desired observer poles (faster than controller poles, e.g., -5 to -10)
% Need 4 observer poles for original states
desired_observer_poles = [-5, -5.1, -5.2, -5.3]; 

% Calculate observer gain L using duality: L' = place(A', C_original', desired_observer_poles)
L = place(A', C_original', desired_observer_poles)';
disp('Observer Gain Matrix L:');
disp(L);

% Define physical parameters
m1 = 1; m2 = 1; l1 = 1; l2 = 1; g = 9.81;

% 5. Define Reference Input
reference_value = [pi/4; pi/4]; 

% 6. Simulation Setup
t_span_sim = [0 5]; 

% Initial conditions for actual plant states (4 original + 2 integral)
x0_plant = [0; 0; 0; 0]; % Actual initial original states
x0_integral_error_plant = [0; 0]; % Actual initial integral error (usually 0)
x_actual_initial = [x0_plant; x0_integral_error_plant]; % 6 actual states

% Initial conditions for observer estimated original states (4 states)
% Observer usually starts with zero initial estimate (or some guess)
x0_hat_plant = zeros(4, 1); % Initial guess for estimated original states

% Combine initial conditions for ode45: [actual_original_states; actual_integral_states; estimated_original_states]
X_total_initial = [x_actual_initial; x0_hat_plant]; % Total 10 states for ode45

% 7. Simulate the Combined System
disp('Simulating Combined Plant and Observer System...');
ode_func_combined = @(t, X_total) combined_dynamics_observer(t, X_total, A, B, C_original, K_aug, L, reference_value, m1, m2, l1, l2, g);
[t_sim_obs, X_total_sim] = ode45(ode_func_combined, t_span_sim, X_total_initial);

% 8. Extract results
x_actual_sim = X_total_sim(:, 1:4); % Actual original states
x_i_actual_sim = X_total_sim(:, 5:6); % Actual integral states
x_hat_sim = X_total_sim(:, 7:10); % Estimated original states

% 9. Calculate Estimation Error (e = x_actual - x_hat)
error_estimation = x_actual_sim - x_hat_sim; 

% Calculate L2 Norm
% ||e||_2 = sqrt(sum(e_i^2))
% For the whole trajectory: L2 norm of the error vector over time
l2_norm_error = zeros(1, size(error_estimation, 2));
for i = 1:size(error_estimation, 2) % For each state component
    l2_norm_error(i) = sqrt(trapz(t_sim_obs, error_estimation(:,i).^2));
end
disp('L2 Norm of Estimation Error for each original state:');
disp(l2_norm_error);

% Calculate L-infinity Norm
% ||e||_inf = max(|e_i|)
l_inf_norm_error = max(abs(error_estimation)); % Max absolute value for each state component
disp('L-infinity Norm of Estimation Error for each original state:');
disp(l_inf_norm_error);

% 10. Plotting Results (remains largely same, adjust labels)

% Plot Actual vs. Estimated Angles
figure;
sgtitle('Actual vs. Estimated Angles with Luenberger Observer');
subplot(2,1,1);
plot(t_sim_obs, x_actual_sim(:,1), 'b-', 'LineWidth', 1.5, 'DisplayName', '\theta_1 (Actual)');
hold on;
plot(t_sim_obs, x_hat_sim(:,1), 'r--', 'LineWidth', 1.5, 'DisplayName', '\theta_1 (Estimated)');
plot(t_sim_obs, ones(size(t_sim_obs)) * reference_value(1), 'k:', 'DisplayName', 'Reference \theta_1');
title('Joint 1 Angle Tracking');
xlabel('Time (s)'); ylabel('Angle (rad)');
legend('show'); grid on; box on;

subplot(2,1,2);
plot(t_sim_obs, x_actual_sim(:,2), 'b-', 'LineWidth', 1.5, 'DisplayName', '\theta_2 (Actual)');
hold on;
plot(t_sim_obs, x_hat_sim(:,2), 'r--', 'LineWidth', 1.5, 'DisplayName', '\theta_2 (Estimated)');
plot(t_sim_obs, ones(size(t_sim_obs)) * reference_value(2), 'm:', 'DisplayName', 'Reference \theta_2');
title('Joint 2 Angle Tracking');
xlabel('Time (s)'); ylabel('Angle (rad)');
legend('show'); grid on; box on;


% Plot Estimation Error
figure;
sgtitle('Luenberger Observer Estimation Error');
subplot(2,1,1);
plot(t_sim_obs, error_estimation(:,1), 'b-', 'LineWidth', 1.5, 'DisplayName', 'Error \theta_1');
hold on;
plot(t_sim_obs, error_estimation(:,2), 'r--', 'LineWidth', 1.5, 'DisplayName', 'Error \theta_2');
title('Estimation Error (Angles)');
xlabel('Time (s)'); ylabel('Error (rad)');
legend('show'); grid on; box on;

subplot(2,1,2);
plot(t_sim_obs, error_estimation(:,3), 'b-', 'LineWidth', 1.5, 'DisplayName', 'Error d\theta_1');
hold on;
plot(t_sim_obs, error_estimation(:,4), 'r--', 'LineWidth', 1.5, 'DisplayName', 'Error d\theta_2');
title('Estimation Error (Velocities)');
xlabel('Time (s)'); ylabel('Error (rad/s)');

legend('show'); grid on; box on;
