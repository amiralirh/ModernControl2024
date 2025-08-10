% Controllability and Observability Analysis using CTRB and OBSV matrices

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

% Number of states
n = size(A, 1);

% Controllability Matrix
Cc = ctrb(A, B);
rank_Cc = rank(Cc);
disp('Controllability Matrix:');
disp(Cc);
fprintf('Rank of Controllability Matrix: %d\n', rank_Cc);

if rank_Cc == n
    disp('System is completely controllable.');
else
    disp('System is not completely controllable.');
end

% Observability Matrix
Co = obsv(A, C);
rank_Co = rank(Co);
disp('Observability Matrix:');
disp(Co);
fprintf('Rank of Observability Matrix: %d\n', rank_Co);

if rank_Co == n
    disp('System is completely observable.');
else
    disp('System is not completely observable.');
end