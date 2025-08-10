# Part4

This script, Compare_OpenLoop_Response.m, simulates both the nonlinear system and its linearized model in an open-loop configuration. The primary purpose is to compare the performance of these two models under the same initial conditions and demonstrate the validity of the linear approximation.

# Script Workflow

1. Defines the physical parameters of the nonlinear system.

2. Performs the linearization of the system dynamics to obtain the state-space matrices (A and B).

3. Sets a single set of initial conditions for both the nonlinear and linearized models.

4. Simulates the response of the nonlinear system using ode45.

5. Simulates the response of the linearized system using the lsim function for linear models.

6. Generates a single plot that overlays the responses of both systems, allowing for a direct comparison of their behavior.
