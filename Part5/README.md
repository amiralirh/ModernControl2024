# Part5

This part of the project analyzes the fundamental properties of the linearized system: controllability and observability. These scripts verify whether the system can be steered to any desired state using a control input and whether all system states can be inferred from the output.

The analysis is performed on the state-space matrices.

Three different methods are used to perform the analysis, providing a robust verification of the results.

1. Analysis using ctrb and obsv Matrices
File: Cont_Obs_using_matrices.m

This script computes the controllability matrix and the observability matrix using MATLAB's built-in functions. The system is deemed completely controllable if the rank of controllability matrix equals the number of states(n), and completely observable if the rank of 
observability matrix equals n.

2. Analysis using the PBH Test
File: Cont_Obs_using_PBH.m

This script implements the Popov-Belevitch-Hautus (PBH) test. It checks the rank of specific matrices for each eigenvalue of the system. The system is controllable if the matrix 
[λI−A B] has full rank for all eigenvalues λ. It is observable if the matrix [λI−A C]has full rank for all eigenvalues λ.

3. Analysis using Jordan Form
File: Cont_Obs_using_Jordan.m

This script transforms the system into its Jordan canonical form. It then analyzes the structure of the transformed B and C matrices(B_hat and C_hat) . The system is controllable if the last row of each Jordan block in B_hat is non-zero, and observable if the first column of each Jordan block in 
C_hat is non-zero.
