# Part9

This script, Output_Reference_Tracking_with_Integral_Control.m, designs a controller using integral control to enable the system's output to accurately track a desired, non-zero reference value. This method ensures that the steady-state error is driven to zero, even in the presence of disturbances.

# Methodology
1. System Augmentation: The initial linearized system (with matrices A and B) is augmented by adding integral states, which represent the integral of the error between the system's output and the reference value. This creates an augmented system with 6 states.

2. State-Feedback Controller Design: The pole placement method is used to design the augmented state-feedback gain matrix, K_aug, for the new 6-state system. This matrix includes gains for both the original states and the integral states, ensuring stability and a fast response.

3. Closed-Loop Simulation: The script simulates the behavior of the nonlinear closed-loop system using MATLAB's ode45 solver. The nonlinear_dynamics_closed_loop.m function, which incorporates the state-feedback control law, is used for the simulation.

4. Results Analysis: The simulation results are plotted, showing the joint angles tracking the non-zero reference values, the system's velocities, and the control inputs (torques).
