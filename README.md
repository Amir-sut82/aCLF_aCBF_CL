# Adaptive CLF-CBF Control with Concurrent Learning for Safety-Critical Nonlinear Systems

This repository implements an **adaptive Control Lyapunov Function (aCLF)** and **adaptive Control Barrier Function (aCBF)** framework combined with **Concurrent Learning (CL)** for safety-critical control of nonlinear uncertain systems.

The project focuses on adaptive control of a **6-state bi-copter attitude system** with uncertain parameters, combining:

- Adaptive Control Lyapunov Functions (aCLF) for stability
- Adaptive Control Barrier Functions (aCBF) for safety guarantees
- Concurrent Learning for online parameter estimation
- Quadratic Programming (QP)-based safety filtering

The implemented framework studies the trade-off between stability, safety, and uncertainty compensation in nonlinear autonomous systems.

---

# 1. System Overview

The nonlinear attitude dynamics are represented as:

$$
\dot{x}=f(x)+g(x)u
$$

where:

- $x$ is the system state
- $u$ is the control input
- $f(x)$ represents nonlinear dynamics
- $g(x)$ represents the control effectiveness

The attitude state is defined as:

$$
x=[\phi,\theta,\psi,p,q,r]^T
$$

where:

- $\phi,\theta,\psi$ are Euler angles
- $p,q,r$ are body angular velocities

Unknown aerodynamic and gyroscopic parameters are considered as model uncertainties.

---

# 2. Adaptive Control Lyapunov Function (aCLF)

A Control Lyapunov Function is used to guarantee convergence of the tracking error.

The CLF condition is:

$$
\dot V(x)+cV(x)\leq0
$$

For uncertain systems, the unknown parameters are estimated online:

$$
\dot{\hat{\theta}}=\Gamma \tau
$$

where $\Gamma$ is the adaptation gain matrix.

The adaptive controller uses the parameter estimates to compensate for uncertainty while maintaining stability.

---

# 3. Concurrent Learning Parameter Estimation

Concurrent Learning improves parameter convergence by using recorded historical data instead of relying only on current measurements.

For stored data points:

$$
Y_i=f(x_i)-\hat f(x_i)
$$

The parameter update law is designed as:

$$
\dot{\hat\theta}=\Gamma\sum_i L_i(Y_i-\hat Y_i)
$$

A history stack is maintained to guarantee sufficient excitation and improve convergence of the estimated parameters.

Advantages:

- Does not require persistent excitation of the trajectory
- Improves uncertainty estimation
- Provides faster parameter convergence

---

# 4. Adaptive Control Barrier Function (aCBF)

Safety constraints are formulated using Control Barrier Functions.

The safe set is defined as:

$$
C=\{x:h(x)\geq0\}
$$

The CBF condition is:

$$
\dot h(x)+\alpha(h(x))\geq0
$$

For relative-degree systems, a higher-order CBF formulation is used:

$$
\psi_0=h(x)
$$

$$
\psi_1=\dot\psi_0+\alpha_1(\psi_0)
$$

$$
\psi_2=\dot\psi_1+\alpha_2(\psi_1)
$$

The safety constraint becomes:

$$
\psi_2(x,u)\geq0
$$

---

# 5. Adaptive CBF Safety Filter

The nominal adaptive CLF controller generates the desired input:

$$
u_{nom}
$$

The safety filter solves a quadratic program:

$$
\min_u ||u-u_{nom}||^2
$$

subject to:

$$
\psi_2(x,u)\geq0
$$

and actuator constraints:

$$
u_{min}\leq u\leq u_{max}
$$

This guarantees safety while minimally modifying the stabilizing controller.

---

# 6. Robust Adaptive CBF Design

A robust adaptive CBF formulation is introduced to handle estimation errors.

The uncertainty bound is incorporated into the safety constraint to maintain safety despite imperfect parameter estimation.

The robust formulation improves:

- Safety margin
- Disturbance rejection
- Constraint satisfaction under uncertainty

---

# 7. Simulation Studies

The project evaluates several scenarios:

## Stability Performance

The adaptive CLF controller achieves attitude regulation while estimating uncertain parameters.

## Concurrent Learning Effect

Concurrent learning improves parameter estimation compared with stability-only adaptation.

## Safety-Critical Scenario

The adaptive CBF filter prevents violation of pitch constraints while maintaining convergence.

## Robustness Comparison

Robust adaptive CBF is compared with standard adaptive CBF under uncertainty.

---

# 8. Main Contributions

This project demonstrates:

- Adaptive nonlinear control
- Safety-critical autonomous systems
- Control Lyapunov Functions
- Control Barrier Functions
- Higher-order CBFs
- Concurrent Learning
- Optimization-based controllers

---

# 9. Repository Structure

```
aCLF_aCBF_CL/
│
├── MATLAB source files
├── Controller implementations
├── Simulation scripts
├── Results
└── README.md
```

---

# 10. Requirements

- MATLAB
- Optimization Toolbox
- Control System Toolbox

---

# References

- Ames, A. D. et al. Control Barrier Function Based Quadratic Programs for Safety Critical Systems.
- Khalil, H. K. Nonlinear Systems.
- Concurrent Learning based Adaptive Control literature.
