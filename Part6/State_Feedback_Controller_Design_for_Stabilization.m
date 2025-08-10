%==========================================================================
% State-Feedback Controller Design for Stabilization
%==========================================================================

clc;
clear;
close all;

% 1. Define the Linearized System Matrices
A = [0, 0, 1, 0;
     0, 0, 0, 1;
     0, 0, 0, 0;
     0, 0, 0, 0];
B = [0, 0;
     0, 0;
     1, 0;
     0, 1];

% Define C_original here, BEFORE it is used in A_aug_controller
C_original = [1, 0, 0, 0; 0, 1, 0, 0]; 

% 2. Augment the System for Integral Control (for controller design)
A_aug_controller = [A, zeros(size(A,1), size(C_original,1)); 
                   -C_original, zeros(size(C_original,1), size(C_original,1))]; 
B_aug_controller = [B; zeros(size(C_original,1), size(B,2))];

% Define physical parameters
m1 = 1; m2 = 1; l1 = 1; l2 = 1; g = 9.81;

% 3. Choose Desired Closed-Loop Pole Locations
desired_poles = [-3, -3.1, -4, -4.1, -4.2, -4.3]; % 6 poles for 6 states

% 4. Calculate the State-Feedback Gain Matrix K (for the augmented system)
K_aug = place(A_aug_controller, B_aug_controller, desired_poles);
disp('Augmented State-Feedback Gain Matrix K_aug:');
disp(K_aug);

% Define reference value for tracking (even if target is zero for stabilization initially)
reference_value = [0; 0]; % For stabilization, reference is zero

% 5. Simulate Closed-Loop Linearized System 
disp('Simulating Closed-Loop Linearized System...');
A_cl = A_aug_controller - B_aug_controller*K_aug; % Closed-loop system matrix for augmented system
sys_cl_lin = ss(A_cl, B_aug_controller, eye(size(A_cl,1)), 0); % Output all augmented states

t_span_sim = [0 3]; % Shorter time for stabilization demo
t_lin = t_span_sim(1):0.01:t_span_sim(2);
u_zero = zeros(length(t_lin), 2); % No external input for stabilization

% Initial Condition Set 1 for LINEARIZED system (augmented 6 states)
x0_cl_lin1_aug = [pi/2; -pi/2; 0; 0; 0; 0]; % Original states + 2 zeros for integral states
[~, ~, x_cl_lin1_aug] = lsim(sys_cl_lin, u_zero, t_lin, x0_cl_lin1_aug);

% Initial Condition Set 2 for LINEARIZED system (augmented 6 states)
x0_cl_lin2_aug = [0.1; 0.2; -0.05; 0.1; 0; 0]; % Original states + 2 zeros for integral states
[~, ~, x_cl_lin2_aug] = lsim(sys_cl_lin, u_zero, t_lin, x0_cl_lin2_aug);


% 6. Simulate Closed-Loop Nonlinear System 
disp('Simulating Closed-Loop Nonlinear System...');

% Initial Condition Set 1 for NONLINEAR system (augmented 6 states)
% Call nonlinear_dynamics_closed_loop with K_aug and reference_value=[0;0]
ode_func_nonlin_cl1 = @(t, x) nonlinear_dynamics_closed_loop(t, x, K_aug, reference_value, m1, m2, l1, l2, g);
[t_cl_nonlin1, x_cl_nonlin1] = ode45(ode_func_nonlin_cl1, t_span_sim, x0_cl_lin1_aug);

% Initial Condition Set 2 for NONLINEAR system (augmented 6 states)
ode_func_nonlin_cl2 = @(t, x) nonlinear_dynamics_closed_loop(t, x, K_aug, reference_value, m1, m2, l1, l2, g);
[t_cl_nonlin2, x_cl_nonlin2] = ode45(ode_func_nonlin_cl2, t_span_sim, x0_cl_lin2_aug);

% Plotting results (adjust indices to show only original 4 states for clarity)
figure;
subplot(2,2,1);
plot(t_cl_nonlin1, x_cl_nonlin1(:,1), 'b-', 'LineWidth', 1.5);
hold on;
plot(t_cl_nonlin1, x_cl_nonlin1(:,2), 'r--', 'LineWidth', 1.5);
title('Nonlinear System - IC1 (Angles)');
xlabel('Time (s)'); ylabel('Angle (rad)');
legend('\theta_1', '\theta_2'); grid on;

subplot(2,2,2);
plot(t_cl_nonlin1, x_cl_nonlin1(:,3), 'b-', 'LineWidth', 1.5);
hold on;
plot(t_cl_nonlin1, x_cl_nonlin1(:,4), 'r--', 'LineWidth', 1.5);
title('Nonlinear System - IC1 (Velocities)');
xlabel('Time (s)'); ylabel('Angular Velocity (rad/s)');
legend('d\theta_1', 'd\theta_2'); grid on;

subplot(2,2,3);
plot(t_cl_nonlin2, x_cl_nonlin2(:,1), 'b-', 'LineWidth', 1.5);
hold on;
plot(t_cl_nonlin2, x_cl_nonlin2(:,2), 'r--', 'LineWidth', 1.5);
title('Nonlinear System - IC2 (Angles)');
xlabel('Time (s)'); ylabel('Angle (rad)');
legend('\theta_1', '\theta_2'); grid on;

subplot(2,2,4);
plot(t_cl_nonlin2, x_cl_nonlin2(:,3), 'b-', 'LineWidth', 1.5);
hold on;
plot(t_cl_nonlin2, x_cl_nonlin2(:,4), 'r--', 'LineWidth', 1.5);
title('Nonlinear System - IC2 (Velocities)');
xlabel('Time (s)'); ylabel('Angular Velocity (rad/s)');
legend('d\theta_1', 'd\theta_2'); grid on;


figure; % Plot for linearized system response (to compare stabilization)
subplot(2,2,1);
plot(t_lin, x_cl_lin1_aug(:,1), 'b-', 'LineWidth', 1.5);
hold on;
plot(t_lin, x_cl_lin1_aug(:,2), 'r--', 'LineWidth', 1.5);
title('Linearized System - IC1 (Angles)');
xlabel('Time (s)'); ylabel('Angle (rad)');
legend('\theta_1', '\theta_2'); grid on;

subplot(2,2,2);
plot(t_lin, x_cl_lin1_aug(:,3), 'b-', 'LineWidth', 1.5);
hold on;
plot(t_lin, x_cl_lin1_aug(:,4), 'r--', 'LineWidth', 1.5);
title('Linearized System - IC1 (Velocities)');
xlabel('Time (s)'); ylabel('Angular Velocity (rad/s)');
legend('d\theta_1', 'd\theta_2'); grid on;

subplot(2,2,3);
plot(t_lin, x_cl_lin2_aug(:,1), 'b-', 'LineWidth', 1.5);
hold on;
plot(t_lin, x_cl_lin2_aug(:,2), 'r--', 'LineWidth', 1.5);
title('Linearized System - IC2 (Angles)');
xlabel('Time (s)'); ylabel('Angle (rad)');
legend('\theta_1', '\theta_2'); grid on;

subplot(2,2,4);
plot(t_lin, x_cl_lin2_aug(:,3), 'b-', 'LineWidth', 1.5);
hold on;
plot(t_lin, x_cl_lin2_aug(:,4), 'r--', 'LineWidth', 1.5);
title('Linearized System - IC2 (Velocities)');
xlabel('Time (s)'); ylabel('Angular Velocity (rad/s)');

legend('d\theta_1', 'd\theta_2'); grid on;

