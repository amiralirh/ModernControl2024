# Part6

File: nonlinear_dynamics_closed_loop.m

This function, nonlinear_dynamics_closed_loop, calculates the time derivative of the state vector for the closed-loop nonlinear system. It is designed to be used with MATLAB's ode45 solver to simulate the system's behavior when a state-feedback controller with integral action is applied.

File: State_Feedback_Controller_Design_for_Stabilization.m

This script designs a state-feedback controller for stabilizing a linearized system, specifically a two-link robotic arm. The controller is designed using the pole placement method to ensure the closed-loop system is stable. The code then simulates both the linearized and the original nonlinear system to evaluate the controller's performance under different initial conditions.

Methodology
1. System Augmentation: The system's state-space matrices (A and B) are augmented to include integral control. This is a crucial step for the controller to eliminate steady-state errors and achieve stabilization to a desired reference point.

2. Pole Placement: The script uses the place function to compute the state-feedback gain matrix K_aug. The desired closed-loop poles are strategically chosen in the left half of the s-plane to guarantee a stable and well-behaved system response.

3. Closed-Loop Simulation: The script simulates the behavior of the system with the designed controller. It performs two simulations:

4. Linearized System: The closed-loop response of the linear model is simulated using the lsim function.

5. Nonlinear System: The closed-loop response of the original nonlinear system is simulated using the ode45 solver, with the state-feedback control law applied.
