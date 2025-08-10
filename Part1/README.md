#Part1

This function, named nonlinear_dynamics, defines the non-linear differential equations for the system. It is the core of the simulation, providing the time derivative of the state vector.

main script performs a complete open-loop simulation of the non-linear system. It is responsible for setting up the simulation, solving the differential equations, and visualizing the results.

Script Workflow

1.Parameter Definition: Defines the physical parameters of the system./n
2.Initial Conditions: Sets the initial state of the system .
3.Simulation: Uses MATLAB's ode45 solver to solve the differential equations defined in nonlinear_dynamics.m over a specified time span.
4.Data Visualization: Plots the simulation results to analyze the system's behavior under different initial conditions.
