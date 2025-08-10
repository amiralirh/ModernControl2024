% LINEARIZATION around the equilibrium point using Symbolic Math Toolbox

% Clear workspace
clc;
clear;
syms theta1 theta2 d_theta1 d_theta2 tau1 tau2 real;

% State and input vectors
x = [theta1; theta2; d_theta1; d_theta2];
u = [tau1; tau2];

% Physical parameters
m1 = 1; % Mass of the first link
m2 = 1; % Mass of the second link
l1 = 1; % Length of the first link
l2 = 1; % Length of the second link
g = 9.81; % Gravity

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

% Nonlinear state equations: x_dot = f(x,u)
f = [d_theta1;
     d_theta2;
     M \ (u - C_term - G_term)];

% Equilibrium point (robot hanging down, at rest)
x_eq = [0; 0; 0; 0];
u_eq = [0; 0];

% Calculate Jacobian matrices A and B
A_sym = jacobian(f, x);
B_sym = jacobian(f, u);

% Substitute the equilibrium point
A = double(subs(A_sym, [x; u], [x_eq; u_eq]));
B = double(subs(B_sym, [x; u], [x_eq; u_eq]));

% Define the C matrix for the output (angles theta1 and theta2)
C = [1, 0, 0, 0;
     0, 1, 0, 0];

% Display the linearized matrices
disp('Linearized A matrix:');
disp(A);
disp('Linearized B matrix:');
disp(B);
disp('Linearized C matrix:');
disp(C);