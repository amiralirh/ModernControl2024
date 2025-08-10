# Part8

This section of the project focuses on designing a control system that enables the system's output to track a desired, non-zero reference value. The chosen method utilizes a static precompensator to achieve zero steady-state error, ensuring the output accurately reaches the target.

Files:
Output_Reference_Tracking_with_Static_Precompensator.m: The main script that performs the controller design, precompensator calculation, and simulation.

nonlinear_dynamics_static_precomp.m: The function that models the closed-loop nonlinear system's dynamics, incorporating the state-feedback control and the static precompensator.
# Methodology

1. The process is executed by two key scripts: Output_Reference_Tracking_with_Static_Precompensator.m and nonlinear_dynamics_static_precomp.m.

2. State-Feedback Controller (K) Design: The process begins by designing a state-feedback gain matrix K for the linearized system using the pole placement method. This controller's primary role is to stabilize the system and place the closed-loop poles at desired locations for a fast, stable response.

3. Static Precompensator (N_v) Calculation: A static precompensator gain N_v is then calculated. This gain is applied to the reference input before it enters the control loop. Its purpose is to scale the reference value, allowing the system's output to precisely track the desired non-zero reference in steady-state.

4. Closed-Loop Simulation: The main script simulates the full nonlinear system using the ode45 solver. The dynamics are defined within the nonlinear_dynamics_static_precomp.m function, which incorporates both the state-feedback law and the static precompensator.
