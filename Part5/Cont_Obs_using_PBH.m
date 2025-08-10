% Controllability and Observability Analysis using PBH Test

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

% Find eigenvalues of A
eigenvalues = eig(A);
disp('Eigenvalues of A:');
disp(eigenvalues);

% Check controllability for each eigenvalue
disp('--- PBH Controllability Test ---');
is_controllable_pbh = true;
n = size(A, 1);
for lambda = eigenvalues'
    M_controllability = [lambda*eye(n) - A, B];
    if rank(M_controllability) < n
        is_controllable_pbh = false;
        break;
    end
end
if is_controllable_pbh
    disp('System is completely controllable.');
else
    disp('System is not completely controllable.');
end

% Check observability for each eigenvalue
disp('--- PBH Observability Test ---');
is_observable_pbh = true;
for lambda = eigenvalues'
    M_observability = [lambda*eye(n) - A; C];
    if rank(M_observability) < n
        is_observable_pbh = false;
        break;
    end
end
if is_observable_pbh
    disp('System is completely observable.');
else
    disp('System is not completely observable.');
end