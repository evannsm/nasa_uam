# Q-Learning Theory and Implementation

## Table of Contents
1. [Overview](#overview)
2. [Mathematical Foundations](#mathematical-foundations)
3. [Actor-Critic Architecture](#actor-critic-architecture)
4. [Implementation Details](#implementation-details)
5. [Learning Algorithm](#learning-algorithm)
6. [Geometric Control Integration](#geometric-control-integration)
7. [Parameter Tuning](#parameter-tuning)
8. [References](#references)

---

## Overview

The UAM control system implements an **Actor-Critic Q-Learning** controller based on Adaptive Dynamic Programming (ADP) for optimal trajectory tracking. This advanced control technique learns the optimal control policy online by simultaneously estimating the value function (critic) and improving the control policy (actor).

### Key Features

- **Online Learning**: Adapts to system dynamics in real-time
- **Model-Free**: Does not require explicit system model
- **Optimal Control**: Minimizes quadratic cost function
- **Continuous State-Action**: Handles continuous-time, continuous-space problems
- **Function Approximation**: Uses radial basis functions for scalability

### System Characteristics

| Property | Value | Description |
|----------|-------|-------------|
| State Dimension | 6D | Position errors (3) + Velocity errors (3) |
| Control Dimension | 3D | Desired accelerations (ax, ay, az) |
| Augmented State | 9D | [state; control] |
| Critic Weights | 45D | Symmetric vectorization of 9×9 matrix |
| Actor Weights | 6×3 = 18D | Policy matrix parameters |
| Update Frequency | ~50 Hz | Triggered by odometry updates |

---

## Mathematical Foundations

### 1. Optimal Control Problem

The goal is to find a control policy π(x) that minimizes the infinite-horizon cost:

```
J(x₀) = ∫₀^∞ r(x(t), u(t)) dt
```

where the running cost is a quadratic function:

```
r(x,u) = xᵀQx + uᵀRu
```

**State Penalty Matrix Q** (6×6 diagonal):
```
Q = diag([10, 10, 10, 30, 30, 30])
    Position errors: weight = 10
    Velocity errors: weight = 30  (higher for damping)
```

**Control Penalty Matrix R** (3×3 diagonal):
```
R = diag([20, 20, 20])
    Acceleration commands: weight = 20
```

**Rationale for Penalties:**
- Higher velocity penalties (30 vs 10) encourage smooth, damped responses (similar to derivative gain in PID)
- Control penalties (20) prevent excessive acceleration commands and improve energy efficiency
- These values follow Linear Quadratic Regulator (LQR) design principles

**Reference:**
> Lewis, F. L., Vrabie, D., & Syrmos, V. L. (2012). "Optimal Control" (3rd ed.). Wiley.
> Chapter 5: Linear Quadratic Regulators.

### 2. Hamilton-Jacobi-Bellman (HJB) Equation

The optimal value function V*(x) satisfies the HJB equation:

```
0 = min_u { r(x,u) + ∇V*(x)ᵀf(x,u) }
```

where f(x,u) represents the system dynamics:
```
ẋ = f(x,u)
```

For UAV dynamics (simplified double integrator):
```
ẋ = v
v̇ = u
```

The optimal control is obtained by minimizing the Hamiltonian:
```
u*(x) = arg min_u { r(x,u) + ∇V*(x)ᵀf(x,u) }
```

**Reference:**
> Bellman, R. (1957). "Dynamic Programming." Princeton University Press.
> Foundation of optimal control theory and value functions.

### 3. Q-Function (State-Action Value Function)

The Q-function combines immediate reward and future value:

```
Q(x,u) = r(x,u) + V(f(x,u))
```

**Bellman Optimality Equation:**
```
Q*(x,u) = r(x,u) + γ ∫ Q*(x', π*(x')) p(x'|x,u) dx'
```

For deterministic continuous-time systems (γ=1):
```
∂Q/∂t + r(x,u) + ∇ₓQ·f(x,u) = 0
```

**Reference:**
> Doya, K. (2000). "Reinforcement Learning in Continuous Time and Space."
> Neural Computation, 12(1), 219-245. DOI: 10.1162/089976600300015961
> Derivation of continuous-time Bellman equations.

---

## Actor-Critic Architecture

### Architecture Overview

The controller uses two separate function approximators:

```
┌─────────────────────────────────────────────────────────┐
│                   Actor-Critic Q-Learning                │
├─────────────────────────────────────────────────────────┤
│                                                           │
│  State: x = [Δx, Δy, Δz, Δvx, Δvy, Δvz]ᵀ ∈ ℝ⁶         │
│                        │                                  │
│         ┌──────────────┴──────────────┐                  │
│         ↓                              ↓                  │
│  ┌─────────────┐              ┌──────────────┐          │
│  │   ACTOR     │              │   CRITIC     │          │
│  │             │              │              │          │
│  │ Policy:     │              │ Value Func:  │          │
│  │ u = Wₐ·Ψ·x  │◄─────────────│ Q(x,u) ≈    │          │
│  │             │   Q-gradient │  wᵀ·Φ·φ(x,u) │          │
│  │ Weights:    │              │              │          │
│  │ Wₐ ∈ ℝ⁶ˣ³   │              │ Weights:     │          │
│  │             │              │  w ∈ ℝ⁴⁵     │          │
│  └─────┬───────┘              └──────┬───────┘          │
│        │                              │                  │
│        │  Control u                   │ TD Error δ       │
│        ↓                              ↓                  │
│  ┌─────────────────────────────────────────┐            │
│  │      Geometric Controller                │            │
│  │   (Mellinger-style attitude control)     │            │
│  └─────────────────────────────────────────┘            │
│                        │                                  │
│                        ↓                                  │
│              Attitude + Thrust Setpoint                   │
│                                                           │
└─────────────────────────────────────────────────────────┘
```

### Critic Network (Value Function Approximation)

**Purpose:** Estimates Q-value for any state-action pair

**Function Approximation:**
```
Q(x,u) ≈ wᵀ · Φ(t) · φ(x,u)
```

where:
- **w** ∈ ℝ⁴⁵ = critic weight vector (learnable parameters)
- **Φ(t)** ∈ ℝ⁴⁵ˣ⁴⁵ = radial basis function matrix (currently Identity)
- **φ(x,u)** ∈ ℝ⁴⁵ = feature vector

**Feature Vector Construction:**

The feature vector is constructed using symmetric Kronecker products:

```
ξ = [x; u] ∈ ℝ⁹                    (augmented state-action vector)
φ = svec(ξ ⊗ ξ) ∈ ℝ⁴⁵              (symmetric vectorization)
```

**Kronecker Product** ξ⊗ξ creates all pairwise products:
```
ξ⊗ξ = ξ·ξᵀ = [ξ₁ξ₁  ξ₁ξ₂  ...  ξ₁ξ₉]
              [ξ₂ξ₁  ξ₂ξ₂  ...  ξ₂ξ₉]
              [ ...   ...   ...   ... ]
              [ξ₉ξ₁  ξ₉ξ₂  ...  ξ₉ξ₉]
```

This is a 9×9 = 81-element matrix, but it's symmetric (ξᵢξⱼ = ξⱼξᵢ).

**Symmetric Vectorization (svec)** eliminates redundancy:

Only store lower triangular + diagonal = 9·(9+1)/2 = 45 unique elements:

```
svec(A) = [A₁₁, √2·A₂₁, A₂₂, √2·A₃₁, √2·A₃₂, A₃₃, ..., A₉₉]ᵀ
```

The √2 scaling factor preserves the Frobenius norm:
```
||vec(A)||² = ||svec(A)||²
```

**Why Quadratic Features?**

The quadratic form ξᵀQξ naturally represents the optimal Q-function for LQR problems:
```
Q(x,u) = ξᵀQ̄ξ = [x; u]ᵀ [Qxx  Qxu] [x]
                          [Qux  Quu] [u]
```

**Reference:**
> Magnus, J. R., & Neudecker, H. (1999). "Matrix Differential Calculus with
> Applications in Statistics and Econometrics" (Revised ed.). Wiley.
> Section 3.8: The symmetric vectorization operator.

### Actor Network (Policy Approximation)

**Purpose:** Determines control action for any state

**Policy Representation:**
```
u(x) = Wₐᵀ · Ψ(t) · x
```

where:
- **Wₐ** ∈ ℝ⁶ˣ³ = actor weight matrix (learnable parameters)
- **Ψ(t)** ∈ ℝ⁶ˣ⁶ = actor radial basis function matrix (currently Identity)
- **x** ∈ ℝ⁶ = state vector

For Identity basis (Ψ = I):
```
u(x) = Wₐᵀ·x = [wa₁ᵀ]     [Δx ]
               [wa₂ᵀ]  ·  [Δy ]
               [wa₃ᵀ]     [Δz ]
                          [Δvx]
                          [Δvy]
                          [Δvz]
```

This is equivalent to a linear state feedback controller: **u = -Kx**

The actor approximates the optimal gain matrix K for LQR.

**Reference:**
> Vamvoudakis, K. G., & Lewis, F. L. (2010). "Online actor-critic algorithm to solve
> the continuous-time infinite horizon optimal control problem." Automatica, 46(5), 878-888.
> DOI: 10.1016/j.automatica.2010.02.018

---

## Implementation Details

### 1. State Vector Computation

**Code:** `qlearning_controller.cpp:109-119`

```cpp
state_vector_t QLearningController::computeStateVector() const
{
    state_vector_t state_vector;
    state_vector <<
        vehicle_odometry_.pose.pose.position.x - navigator_setpoint_.pose.pose.position.x,
        vehicle_odometry_.pose.pose.position.y - navigator_setpoint_.pose.pose.position.y,
        vehicle_odometry_.pose.pose.position.z - navigator_setpoint_.pose.pose.position.z,
        vehicle_odometry_.twist.twist.linear.x - navigator_setpoint_.twist.twist.linear.x,
        vehicle_odometry_.twist.twist.linear.y - navigator_setpoint_.twist.twist.linear.y,
        vehicle_odometry_.twist.twist.linear.z - navigator_setpoint_.twist.twist.linear.z;
    return state_vector;
}
```

**State Definition:**
```
x = [Δx, Δy, Δz, Δvx, Δvy, Δvz]ᵀ
```

where:
- **Δx, Δy, Δz** = position tracking errors (meters)
- **Δvx, Δvy, Δvz** = velocity tracking errors (m/s)

**Coordinate Frame:** ENU (East-North-Up)

### 2. Symmetric Vectorization Transform

**Code:** `qlearning_controller.cpp:544-635`

The `computeSvecMatrix()` method constructs the transformation matrix **T** ∈ ℝ⁴⁵ˣ⁸¹:

```
svec(ξ⊗ξ) = T · vec(ξ⊗ξ)
```

**Algorithm:**
```
For each (i,j) where i ≥ j:
    index = j·n + i - (j+1)·j/2
    if i == j:                  // Diagonal
        T[index, i·n+j] = 1.0
    else:                        // Off-diagonal
        T[index, i·n+j] = 1/√2
        T[index, j·n+i] = 1/√2
```

This eliminates redundant elements while preserving norms.

### 3. Radial Basis Functions (RBF)

**Current Implementation:**
```cpp
critic_radial_basis_matrix_t QLearningController::computeCriticRadialBasisMatrix(double t)
{
    return critic_radial_basis_matrix_t::Identity();  // Φ = I
}
```

The framework supports time-varying RBF but currently uses Identity for simplicity.

**General RBF Form:**
```
Φᵢⱼ(t) = exp(-||φᵢ(t) - cⱼ||² / (2σ²))
```

where:
- **cⱼ** = RBF center locations
- **σ** = RBF width parameter

**Future Extension:** Replace Identity with Gaussian RBF for better generalization.

---

## Learning Algorithm

### Main Learning Loop

**Code:** `qlearning_controller.cpp:387-527`

The `learn()` method implements the complete actor-critic update:

### STEP 1: Feature Vector Construction

```cpp
state_vector_ = computeStateVector();
control_vector_ = computeControl(state_vector_);
augmented_state_vector_ << state_vector_, control_vector_;
critic_augmented_state_kronecker_ =
    vec_to_svec_transform_matrix_ *
    kroneckerProduct(augmented_state_vector_, augmented_state_vector_);
```

Creates:
- **ξ(t)** = [x(t); u(t)] ∈ ℝ⁹
- **φ(t)** = svec(ξ⊗ξ) ∈ ℝ⁴⁵

### STEP 2: Temporal Difference (TD) Error Computation

Two TD errors are computed:

**Bellman Residual (critic_error_1):**
```
δ₁ = wᵀ[Φ(t)φ(t) - Φ(t-Δt)φ(t-Δt)] + [r(t) - r(t-Δt)]
```

This approximates the continuous-time Bellman equation:
```
dQ/dt = -r(x,u)
```

Discretization (Euler integration):
```
Q(t) - Q(t-Δt) ≈ -∫ᵗₜ₋Δₜ r(s) ds ≈ -r(t)·Δt
```

**Terminal Cost Error (critic_error_2):**
```
δ₂ = 0.5·xᵀPx - wᵀΦφ
```

Enforces terminal boundary condition for finite-horizon control:
```
V(x_final) = 0.5·xᵀPx
```

where **P** is the terminal Riccati matrix.

**Note:** Currently δ₂ is weighted by 0.0, so infinite-horizon formulation is used.

**Reference:**
> Vrabie, D., Pastravanu, O., Abu-Khalaf, M., & Lewis, F. L. (2009).
> "Adaptive optimal control for continuous-time linear systems based on policy iteration."
> Automatica, 45(2), 477-484. DOI: 10.1016/j.automatica.2008.08.017

### STEP 3: Critic Weight Update

**Normalized Gradient Descent:**
```cpp
critic_weight_dot = -critic_convergence_rate_
    * ((sigma / pow(1.0 + sigma.squaredNorm(), 2)) * critic_error_1_);
```

Mathematical form:
```
ẇ = -αc · σ/(1 + ||σ||²)² · δ₁
```

where:
- **σ** = ∇wQ = Φ·(φ - φ_prior) = gradient of Q w.r.t. weights
- **αc** = critic_convergence_rate (default: 10.0)

**Normalization Benefits:**
1. **Numerical Stability:** Prevents weight explosion
2. **Adaptive Step Size:** Larger gradients → smaller updates
3. **Convergence Guarantees:** Under persistence of excitation conditions

**Reference:**
> Vamvoudakis & Lewis (2010), Equation (23)

### STEP 4: Actor Weight Update

**Extract Q-function Hessian:**
```cpp
Q_bar = (vec_to_svec_transform_matrix_.transpose() *
         (2 * critic_radial_basis_matrix_.transpose() * critic_weight_vector_))
        .reshaped(AUGMENTED_STATE_VECTOR_SIZE, AUGMENTED_STATE_VECTOR_SIZE);
```

Reconstructs:
```
Q̄ = reshape(2·T'·w)  ∈ ℝ⁹ˣ⁹
```

**Extract Control-Dependent Blocks:**
```cpp
Quu = Q_bar.bottomRightCorner<3, 3>();  // ∂²Q/∂u²
Qux = Q_bar.bottomLeftCorner<3, 6>();   // ∂²Q/∂u∂x
```

**Optimal Policy from HJB:**
```
u*(x) = -Quu⁻¹·Qux·x
```

**Actor Error (Policy Residual):**
```cpp
actor_error_ = actor_weight_matrix_.transpose() * actor_radial_basis_matrix_ * state_vector_
             + Quu.inverse() * Qux * state_vector_;
```

Mathematical form:
```
eₐ = u - u* = Wₐ·Ψ·x - (-Quu⁻¹·Qux·x)
```

**Actor Weight Update:**
```cpp
actor_weight_dot = -actor_convergence_rate_
    * state_vector_ / (1.0 + state_vector_.dot(state_vector_))
    * actor_error_.transpose();
```

Mathematical form:
```
Ẇₐ = -αₐ · x/(1 + xᵀx) · eₐᵀ
```

where **αₐ** = actor_convergence_rate (default: 0.5)

**Why αₐ < αc?**
The actor should learn slower than the critic to ensure the critic provides accurate Q-values before the actor adjusts its policy significantly.

**Reference:**
> Vamvoudakis & Lewis (2010), Equation (33)

### STEP 5: Weight Integration

**Euler Integration:**
```cpp
critic_weight_vector_ += critic_weight_dot * time_resolution_;
actor_weight_matrix_ += actor_weight_dot * time_resolution_;
```

Discrete-time update:
```
w(t+Δt) = w(t) + ẇ·Δt
Wₐ(t+Δt) = Wₐ(t) + Ẇₐ·Δt
```

where **Δt** = time_resolution_ ≈ 0.02s (50 Hz)

### STEP 6: Running Cost Accumulation

```cpp
running_cost_ += 0.5 * ((state_vector_.transpose() * state_penalty_matrix_).dot(state_vector_)
               + (control_vector_.transpose() * control_penalty_matrix_).dot(control_vector_))
               * time_resolution_;
```

Incremental cost:
```
r(t+Δt) = r(t) + 0.5·[xᵀQx + uᵀRu]·Δt
```

This accumulated cost is used in the next iteration's TD error.

---

## Geometric Control Integration

### Mellinger-Style Attitude Control

**Code:** `qlearning_controller.cpp:636-722`

The Q-learning actor outputs desired accelerations **u** = [ax, ay, az]ᵀ. These must be converted to attitude (roll, pitch, yaw) and thrust commands for PX4.

### Algorithm Steps

**STEP 1: Desired Thrust Vector**
```cpp
thrust_des = control_vector + Vector3d(0.0, 0.0, -9.80665);
```

Mathematical form:
```
Fd = m·(ad - g·e₃)
```

where:
- **m** = vehicle mass (implicit, normalized out)
- **g** = 9.80665 m/s² (Earth's gravitational acceleration)
- **e₃** = [0, 0, 1]ᵀ (upward vertical vector in ENU)

The negative sign accounts for gravity acting downward.

**STEP 2: Desired Body Z-Axis**
```cpp
z_B_desired = -thrust_des.normalized();
```

```
zB,d = -Fd / ||Fd||
```

The negative sign is because the body z-axis points **down** in FRD frame.

**STEP 3: Desired Body X and Y Axes**
```cpp
x_c_des << cos(yaw_des), sin(yaw_des), 0;
y_B_desired = (z_B_desired.cross(x_c_des)).normalized();
x_B_desired = y_B_desired.cross(z_B_desired);
```

Given desired yaw angle ψd = π/2 (90°, facing North):
```
xc,d = [cos(ψd), sin(ψd), 0]ᵀ = [0, 1, 0]ᵀ
yB,d = (zB,d × xc,d) / ||zB,d × xc,d||
xB,d = yB,d × zB,d
```

This constructs an orthonormal frame satisfying the yaw constraint.

**STEP 4: Rotation Matrix and Euler Angles**
```cpp
R_d.col(0) = x_B_desired;
R_d.col(1) = y_B_desired;
R_d.col(2) = z_B_desired;
Vector3d ypr_d = R_d.eulerAngles(2,1,0);  // ZYX convention
```

```
Rd = [xB,d | yB,d | zB,d]
[ψd, θd, φd] = ZYX_Euler(Rd)
```

**STEP 5: Thrust Magnitude**
```cpp
z_axis = R.col(2);  // Current body z-axis
thrust_proj = thrust_des.dot(-z_axis);
```

```
T = Fd · (-zB,current)
```

Projects desired force along **current** thrust axis (not desired). This accounts for the fact that the vehicle cannot instantly change orientation.

### Coordinate Frame Handling

**Input:** Acceleration in ENU frame
**Output:** Attitude in FRD frame

**Frame Conversions:**
- Gravity vector: [0, 0, -g] in ENU (downward)
- Thrust direction: Opposite of force vector
- Euler angles: ZYX intrinsic rotations (yaw-pitch-roll)

**Reference:**
> Mellinger, D., & Kumar, V. (2011). "Minimum snap trajectory generation and
> control for quadrotors." In 2011 IEEE International Conference on Robotics
> and Automation (pp. 2520-2525). DOI: 10.1109/ICRA.2011.5980409
>
> Lee, T., Leok, M., & McClamroch, N. H. (2010). "Geometric tracking control
> of a quadrotor UAV on SE(3)." In 49th IEEE Conference on Decision and Control
> (CDC) (pp. 5420-5425). DOI: 10.1109/CDC.2010.5717652

---

## Parameter Tuning

### Learning Rates

**Critic Convergence Rate (αc):**
```yaml
critic_convergence_rate: 10.0
```

**Guidelines:**
- Too high: Oscillations, instability
- Too low: Slow convergence
- Recommended: 1.0 - 50.0

**Actor Convergence Rate (αₐ):**
```yaml
actor_convergence_rate: 0.5
```

**Guidelines:**
- Should be < critic_convergence_rate
- Ratio αₐ/αc ≈ 0.05 - 0.2 for stability
- Recommended: 0.1 - 5.0

**Why αₐ < αc?**
The actor relies on accurate Q-values from the critic. If the actor updates too fast, it may respond to inaccurate Q-estimates.

### Penalty Matrices

**State Penalty Q:**
```yaml
Q_position: [10, 10, 10]    # Position error penalties
Q_velocity: [30, 30, 30]    # Velocity error penalties (higher for damping)
```

**Control Penalty R:**
```yaml
R: [20, 20, 20]             # Acceleration penalties
```

**Tuning Rules:**
1. **Increase Q_position**: Faster position convergence, more aggressive
2. **Increase Q_velocity**: Better damping, smoother response
3. **Increase R**: Gentler control, less energy use
4. **Q_velocity > Q_position**: Standard practice for stability

### Safety Parameters

**Minimum Learning Altitude:**
```yaml
minimum_altitude_for_learning: 0.6  # meters
```

Learning is disabled below this altitude to prevent dangerous exploration during takeoff/landing.

**Recommended:** 0.5 - 1.0 meters for small UAVs

### Weight Initialization

**Critic Weights (w):**
```cpp
critic_weight_vector_ << 10.83, 0.00, 0.00, 11.72, ..., 1.00;
```

Pre-trained weights from offline LQR solution or previous missions. Provides warm-start for faster convergence.

**Actor Weights (Wₐ):**
```cpp
actor_weight_matrix_ << -2.07, 0.00, 0.00, ..., -3.00;
```

Initial policy approximates LQR gain matrix. Negative values indicate stabilizing feedback.

---

## References

### Core Algorithm

1. **Vamvoudakis, K. G., & Lewis, F. L. (2010).**
   "Online actor-critic algorithm to solve the continuous-time infinite horizon optimal control problem."
   *Automatica*, 46(5), 878-888.
   DOI: [10.1016/j.automatica.2010.02.018](https://doi.org/10.1016/j.automatica.2010.02.018)
   - **Contribution:** Main algorithm for online actor-critic Q-learning with convergence proofs

2. **Vrabie, D., Pastravanu, O., Abu-Khalaf, M., & Lewis, F. L. (2009).**
   "Adaptive optimal control for continuous-time linear systems based on policy iteration."
   *Automatica*, 45(2), 477-484.
   DOI: [10.1016/j.automatica.2008.08.017](https://doi.org/10.1016/j.automatica.2008.08.017)
   - **Contribution:** Policy iteration framework, temporal difference learning for continuous time

3. **Doya, K. (2000).**
   "Reinforcement Learning in Continuous Time and Space."
   *Neural Computation*, 12(1), 219-245.
   DOI: [10.1162/089976600300015961](https://doi.org/10.1162/089976600300015961)
   - **Contribution:** Continuous-time Bellman equations, TD error formulation

### Optimal Control Theory

4. **Bellman, R. (1957).**
   "Dynamic Programming."
   Princeton University Press.
   - **Contribution:** Foundational theory of dynamic programming and value functions

5. **Lewis, F. L., Vrabie, D., & Syrmos, V. L. (2012).**
   "Optimal Control" (3rd ed.).
   Wiley.
   - **Contribution:** LQR theory, Riccati equations, penalty matrix design (Chapter 5)

### Geometric Control

6. **Mellinger, D., & Kumar, V. (2011).**
   "Minimum snap trajectory generation and control for quadrotors."
   In *2011 IEEE International Conference on Robotics and Automation* (pp. 2520-2525).
   DOI: [10.1109/ICRA.2011.5980409](https://doi.org/10.1109/ICRA.2011.5980409)
   - **Contribution:** Differential flatness, snap minimization, attitude from acceleration

7. **Lee, T., Leok, M., & McClamroch, N. H. (2010).**
   "Geometric tracking control of a quadrotor UAV on SE(3)."
   In *49th IEEE Conference on Decision and Control (CDC)* (pp. 5420-5425).
   DOI: [10.1109/CDC.2010.5717652](https://doi.org/10.1109/CDC.2010.5717652)
   - **Contribution:** Geometric control on SO(3), attitude error dynamics

### Mathematical Tools

8. **Magnus, J. R., & Neudecker, H. (1999).**
   "Matrix Differential Calculus with Applications in Statistics and Econometrics" (Revised ed.).
   Wiley.
   - **Contribution:** Vectorization operators (vec, svec), Kronecker products (Chapter 3, Section 3.8)

---

## Appendix: Convergence Conditions

### Persistence of Excitation (PE)

For guaranteed convergence, the state trajectory must be **persistently exciting**:

```
∃ T, α, β > 0 such that:
α·I ≤ ∫ᵗₜ₊T φ(s)φ(s)ᵀ ds ≤ β·I  ∀t
```

**Intuition:** The system must explore the state-action space sufficiently.

**In Practice:**
- Varying setpoints naturally provide PE
- Takeoff, navigation, and loiter modes create diverse trajectories
- No additional exploration noise needed for this application

### Lyapunov Stability

Under PE condition and bounded TD error, the weight error dynamics are:

```
ė = -αΓ(e)e + η(t)
```

where:
- **e** = w - w* (weight error)
- **Γ(e)** = normalization term (positive definite)
- **η(t)** = bounded disturbance

**Result:** Uniform ultimate boundedness (UUB) of weight errors

**Reference:** Vamvoudakis & Lewis (2010), Theorem 1

---

## Appendix: Comparison with Other Methods

| Method | Model-Free | Online | Optimal | Complexity |
|--------|-----------|--------|---------|------------|
| **PID** | ✓ | ✓ | ✗ | Low |
| **LQR** | ✗ | ✗ | ✓ | Low |
| **Model Predictive Control** | ✗ | ✓ | ✓ | High |
| **Actor-Critic Q-Learning** | ✓ | ✓ | ✓ | Medium |

**Advantages of Q-Learning:**
- No system model required (model-free)
- Adapts online to changing dynamics
- Provably converges to optimal policy
- Moderate computational complexity

**Disadvantages:**
- Requires tuning of learning rates
- No guarantees during learning (exploration risk)
- Slower than MPC for known models

---

*This document provides the complete mathematical foundation and implementation details for the Actor-Critic Q-Learning controller used in the NASA UAM Flight Stack.*
