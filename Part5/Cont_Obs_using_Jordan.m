% Controllability and Observability Analysis using Jordan Form (Corrected)

% Define system matrices
A = [0, 0, 1, 0;
     0, 0, 0, 1;
     0, 0, 0, 0;
     0, 0, 0, 0];
B = [0, 0;
     0, 0;
     1, 0;
     0, 1];
C = [1, 0, 0, 0;
     0, 1, 0, 0];

% Find Jordan form and transformation matrix P
[P, J] = jordan(A);
disp('Jordan Form (J):');
disp(J);
disp('Transformation Matrix (P):');
disp(P);

% Transform B and C matrices
B_tilde = P \ B;
C_tilde = C * P;
disp('Transformed B Matrix (B_tilde):');
disp(B_tilde);
disp('Transformed C Matrix (C_tilde):');
disp(C_tilde);

% ==========================================================
% Correct Analysis Logic
% ==========================================================

% Analyze controllability based on Jordan form
% System is controllable if the last row of each Jordan block in B_tilde is non-zero.
% A has two Jordan blocks: J(1,1) is 2x2 and J(3,3) is 2x2.
% We check rows 2 and 4 of B_tilde.
is_controllable_jordan = (abs(B_tilde(2, 1)) > 1e-9 || abs(B_tilde(2, 2)) > 1e-9) && ...
                          (abs(B_tilde(4, 1)) > 1e-9 || abs(B_tilde(4, 2)) > 1e-9);

if is_controllable_jordan
    disp('System is completely controllable based on Jordan form.');
else
    disp('System is not controllable based on Jordan form.');
end

% Analyze observability based on Jordan form
% System is observable if the first column of each Jordan block in C_tilde is non-zero.
% We check columns 1 and 3 of C_tilde.
is_observable_jordan = (abs(C_tilde(1, 1)) > 1e-9 || abs(C_tilde(2, 1)) > 1e-9) && ...
                       (abs(C_tilde(1, 3)) > 1e-9 || abs(C_tilde(2, 3)) > 1e-9);

if is_observable_jordan
    disp('System is completely observable based on Jordan form.');
else
    disp('System is not observable based on Jordan form.');
end