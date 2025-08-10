# Part10
Luenberger Observer Design and Tracking Error Analysis
This section of the project focuses on designing a Luenberger observer to estimate the system's internal states when not all states are directly measurable. The observer is designed for the linearized system, and the estimated states are then used by the state-feedback controller to generate the control signal for the nonlinear plant. This approach allows for closed-loop control without full state measurement.

The analysis also quantifies the observer's performance by calculating the L2-norm and L-infinity norm of the estimation error.


Files:
Luenberger_Observer_Design_and_Tracking_Error_Analysis.m: The main script that performs controller and observer design, sets up initial conditions, runs the combined simulation, and plots the results.

combined_dynamics_observer.m: A function that defines the combined dynamics of the nonlinear plant and the Luenberger observer, making it suitable for use with MATLAB's ode45 solver.
# Methodology

- Controller Design: The script first designs a state-feedback controller with integral action for the augmented system (6 states) to achieve output reference tracking. The gain matrix, K_aug, is calculated using pole placement, with poles chosen to ensure a stable and fast closed-loop response.

- Observer Design: A Luenberger observer is designed for the original 4-state system (A, C pair), not the augmented one. The observer gain matrix, L, is calculated using the place function on the dual system to place the observer's poles at desired locations. The observer poles are typically chosen to be faster than the controller poles to ensure the state estimation converges quickly.

- Combined Simulation: The combined_dynamics_observer.m function simulates the dynamics of the entire closed-loop system, which consists of the nonlinear plant and the Luenberger observer running in parallel. The control signal for the plant is generated using the estimated states from the observer and the actual integral states. This simulation provides the actual states, the estimated states, and the estimation error over time.

- Error Analysis: After the simulation, the script calculates the estimation error (e = x_actual - x_hat) and computes both the L2-norm and L-infinity norm of this error for each state component. These norms provide a quantitative measure of the observer's accuracy over the entire simulation period.
