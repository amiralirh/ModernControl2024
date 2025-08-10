# Part7

This script, Comparing_Different_Desired_Pole_Locations.m, is designed to demonstrate the critical relationship between the choice of desired closed-loop pole locations and the resulting system performance and control effort. It compares two state-feedback controllers, one designed for a fast response and one for a slower, more conservative response.

This function, feedback_linearization, implements the core feedback linearization control law for the nonlinear system. It translates a synthetic control input, v_synthetic, into the actual control torques, tau, needed to drive the physical system. This method effectively cancels the nonlinearities of the system (e.g., inertia, Coriolis, and gravity) to create a linear relationship between the input and the system dynamics.

 # Methodology

1. Controller Design: The script defines two sets of desired poles for the augmented system.

- Fast Poles: These poles are placed farther to the left in the s-plane (e.g., -3, -4, etc.). This design aims for a quick stabilization of the system.

- Slow Poles: These poles are placed closer to the imaginary axis (e.g., -1, -1.1, etc.). This design aims for a less aggressive, more gradual stabilization.

2. Simulation: The script uses the ode45 solver to simulate the closed-loop nonlinear system for both controllers, starting from the same initial conditions.

3. Performance Analysis: The script plots the system's states (joint angles and velocities) and the required control input (torques) over time for both cases.

Expected Results and Comparison:
- Response Time: The system controlled by the fast poles will exhibit a much quicker return to the equilibrium position, with a shorter settling time. The system with slow poles will take longer to stabilize.

- Control Effort: The controller with fast poles will require a significantly larger and more aggressive control signal (torque) to achieve the rapid response. The slow poles controller will demand a smaller, smoother control signal.

This analysis highlights a fundamental trade-off in control system design: faster response times often require a higher control effort.
