# Advanced-Control-Systems

This repository documents my work on Optimal Control Theory and Model Predictive Control (MPC). It highlights the implementation of advanced control strategies, bridging the gap between theoretical formulation and numerical algorithm design.

# Repository Structure
* **/MPC/ :** Implementation of predictive control strategies for stabilization and trajectory tracking.
  * stabilization.m : MPC controller implementation featuring actuator saturation constraints.
  * tracking.m : MPC implementation using input increment ($\Delta u$) formulation for robust setpoint tracking.
* **/optimal_control_theory/ :** Foundational algorithms for optimal control.
  * Finite Horizon LQR.m : Numerical solver for the discrete-time Riccati equation using backward recursion for finite-horizon problems.
* **/estimation/ :**
  * Kalman Filtering: Implementation of state estimation for linear and non-linear systems (2D particle tracking with boundary conditions and harmonic oscillators).
* **/Segway/ :**
  * Modeling: Lagrangian derivation of inverted pendulum dynamics.
  * Control Strategy: Optimal stabilization using LQR (Linear Quadratic Regulator) and pole placement.
  * Observer Design: Luenberger observer implementation for state reconstruction (measuring position and angle).
  * Visualization: Custom MATLAB interface for real-time simulation visualization.
* **/docs/ :** Technical documentation, including lab reports and theoretical references.

# Key Technical Skills
* System Dynamics: Discretization, state-space representation, and modeling of non-linear constraints.
* Optimal Control: Implementation of backward Riccati recursion to solve finite-horizon optimal control problems.
* Model Predictive Control (MPC): Formulation of Quadratic Programming (QP) optimization under constraints, including actuator saturation and input rate limits.
* Control Theory: Competency in system discretization, state-space augmentation, and tracking problem formulation using input increments.
* Stochastic Estimation: Designing Kalman filters for state reconstruction in noisy environments.
* Numerical Validation: Performance analysis and verification of optimal gains convergence against theoretical benchmarks.

# Prerequisites
* MATLAB with Control System Toolbox and Optimization Toolbox.
* The scripts are designed to be run within a standard MATLAB environment.

# ReferencesThe implementations in this repository are based on the theoretical principles found in:
* **Kirk, D. E.** - *Optimal Control Theory: An Introduction*.
* Control Engineering Practicals - *Télécom Physique Strasbourg*.

