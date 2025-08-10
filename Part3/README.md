# Part3

This script, linearization.m, linearizes the nonlinear system dynamics around a specified operating point. Linearization is a fundamental technique used in control theory to approximate the behavior of a complex nonlinear system with a simpler linear model.

In this project, the linearization is performed around the equilibrium point (x=0).The script calculates the state matrix (A) and input matrix (B) of the linearized system, which are crucial for designing linear control systems.


# Script Workflow

1. Defines the nonlinear system's equations and parameters.

2. Specifies the operating point for linearization.

3. Calculates the Jacobian matrices of the system dynamics with respect to the state vector and the control input.

4. Derives the linear state-space matrices A and B.

5. Displays the resulting matrices, providing a linear representation of the system's behavior near the equilibrium point.
