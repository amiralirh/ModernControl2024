%==========================================================================
% Output Reference Tracking with Integral Control
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
C = [1, 0, 0, 0;
     0, 1, 0, 0]; % Output is theta1 and theta2

% 2. Augment the System for Integral Control
% A_aug = [A, 0; -C, 0]
A_aug = [A, zeros(size(A,1), size(C,1));
         -C, zeros(size(C,1), size(C,1))]; % size(C,1) is number of outputs (2)

% B_aug = [B; 0]
B_aug = [B;
         zeros(size(C,1), size(B,2))];

% Check controllability of augmented system (should be 6)
rank_aug_ctrb = rank(ctrb(A_aug, B_aug));
fprintf('Rank of Augmented Controllability Matrix: %d\n', rank_aug_ctrb);
if rank_aug_ctrb == size(A_aug, 1)
    disp('Augmented system is controllable.');
else
    disp('Augmented system is NOT controllable.');
end

% 3. Choose Desired Closed-Loop Pole Locations for Augmented System
% Need 6 poles (4 for original states + 2 for integral states)
% Choose stable and distinct poles
desired_poles_aug = [-2, -2.1, -2.2, -2.3, -2.4, -2.5]; 

% 4. Calculate the Augmented State-Feedback Gain Matrix K_aug
K_aug = place(A_aug, B_aug, desired_poles_aug);
disp('Augmented State-Feedback Gain Matrix K_aug:');
disp(K_aug);

% Define physical parameters (for nonlinear_dynamics_closed_loop)
m1 = 1; m2 = 1; l1 = 1; l2 = 1; g = 9.81;

% 5. Define Reference Input (non-zero desired angles)
reference_value = [pi/4; pi/4]; % Example: Make theta1 go to pi/4 and theta2 go to pi/4
disp('Desired Reference Value (rad):');
disp(reference_value);

% 6. Simulation Setup
t_span_sim = [0 5]; % Simulation time
% Initial state for augmented system: [x0_original; x0_integral_error]
x0_original = [0; 0; 0; 0]; % Start original states from origin
x0_integral_error = [0; 0]; % Start integral error at zero
x0_aug_sim = [x0_original; x0_integral_error]; 

% 7. Simulate the Closed-Loop Nonlinear System with Integral Control
disp('Simulating Closed-Loop Nonlinear System with Integral Control...');
ode_func_integral_tracking = @(t, x_aug) nonlinear_dynamics_closed_loop(t, x_aug, K_aug, reference_value, m1, m2, l1, l2, g);
[t_track_int, x_track_int] = ode45(ode_func_integral_tracking, t_span_sim, x0_aug_sim);

% 8. Extract original states and integral states from augmented results
x_original_sim = x_track_int(:, 1:4);
x_integral_error_sim = x_track_int(:, 5:6);

% 9. Plotting Results
figure;
sgtitle('Output Reference Tracking with Integral Control');

% Plot Joint Angles and Reference
subplot(2,1,1);
plot(t_track_int, x_original_sim(:,1), 'b-', 'LineWidth', 1.5, 'DisplayName', '\theta_1 (Actual)');
hold on;
plot(t_track_int, x_original_sim(:,2), 'r--', 'LineWidth', 1.5, 'DisplayName', '\theta_2 (Actual)');
plot(t_track_int, ones(size(t_track_int)) * reference_value(1), 'k:', 'LineWidth', 1.5, 'DisplayName', 'Reference \theta_1');
plot(t_track_int, ones(size(t_track_int)) * reference_value(2), 'm:', 'LineWidth', 1.5, 'DisplayName', 'Reference \theta_2');
title('Joint Angles Tracking');
xlabel('Time (s)'); ylabel('Angle (rad)');
legend('show'); grid on; box on;

% Plot Angular Velocities
subplot(2,1,2);
plot(t_track_int, x_original_sim(:,3), 'b-', 'LineWidth', 1.5, 'DisplayName', 'd\theta_1');
hold on;
plot(t_track_int, x_original_sim(:,4), 'r--', 'LineWidth', 1.5, 'DisplayName', 'd\theta_2');
title('Angular Velocities');
xlabel('Time (s)'); ylabel('Velocity (rad/s)');
legend('show'); grid on; box on;

% Optionally plot integral error states
figure;
subplot(2,1,1);
plot(t_track_int, x_integral_error_sim(:,1), 'b-', 'LineWidth', 1.5, 'DisplayName', 'Integral Error \theta_1');
hold on;
plot(t_track_int, x_integral_error_sim(:,2), 'r--', 'LineWidth', 1.5, 'DisplayName', 'Integral Error \theta_2');
title('Integral Error States');
xlabel('Time (s)'); ylabel('Integral of Error (rad.s)');
legend('show'); grid on; box on;

% Optionally plot control input tau (requires calculation in the loop)
tau_track_int = zeros(size(x_track_int, 1), 2);
for i = 1:size(x_track_int, 1)
    current_x_aug = x_track_int(i,:)';
    v_synthetic_val = -K_aug * current_x_aug;
    % Need original x for feedback_linearization to calculate M, C_term, G_term
    current_x_original = current_x_aug(1:4); 
    tau_track_int(i,:) = feedback_linearization(current_x_original, v_synthetic_val)';
end

subplot(2,1,2);
plot(t_track_int, tau_track_int(:,1), 'b-', 'LineWidth', 1.5, 'DisplayName', '\tau_1');
hold on;
plot(t_track_int, tau_track_int(:,2), 'r--', 'LineWidth', 1.5, 'DisplayName', '\tau_2');
title('Control Input \tau');
xlabel('Time (s)'); ylabel('Torque (Nm)');

legend('show'); grid on; box on;
