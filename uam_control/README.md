# uam_control - Q-Learning Adaptive Controller

## Overview

The `uam_control` package implements an **adaptive reinforcement learning controller** using Q-learning for UAV trajectory tracking. It provides advanced control beyond basic PID by learning optimal control policies through online interaction with the vehicle dynamics.

**Note**: This is a **research-grade** component for experimental adaptive control. For production flights, standard PID or model-based control is recommended.

## Package Purpose

### What Does It Do?

The Q-learning controller:
1. **Receives** position setpoints from the navigator
2. **Computes** optimal attitude and thrust commands using learned policy
3. **Publishes** attitude setpoints to the vehicle interface
4. **Learns** continuously to improve performance over time

### Why Use It?

- **Adaptive**: Learns vehicle-specific dynamics online
- **Robust**: Adapts to changing conditions (wind, mass changes, battery drain)
- **Optimal**: Minimizes tracking error and control effort
- **Model-Free**: Doesn't require explicit system identification

### When NOT to Use It

- **First flights**: Requires stable baseline control for initial learning
- **Safety-critical**: Not certified for critical operations
- **Simple tasks**: Overkill for basic waypoint following
- **Unstable environments**: Strong disturbances may disrupt learning

---

## Architecture

### Package Structure

```
uam_control/
├── uam_control/                     # Main controller node
│   ├── src/qlearning_controller.cpp
│   └── include/uam_control/
│       └── qlearning_controller.hpp
└── uam_control_msgs/                # Message definitions
    └── msg/
        ├── AttitudeSetpoint.msg
        └── QLearningStatus.msg
```

### Data Flow

```
Navigator                QLearningController         VehicleInterface
─────────                ───────────────────         ────────────────
Position Setpoint ──────────►│                   │
                             │ State Extraction  │
Vehicle Odometry ◄──────────►│ (x, y, z, vx,    │
                             │  vy, vz)          │
                             │                   │
                             │ Q-Learning Update │
                             │ (Critic + Actor)  │
                             │                   │
                             │ Attitude Compute  │
                             │                   ├────► Attitude Setpoint
                             │                   │      (roll, pitch, yaw, thrust)
```

---

## Q-Learning Background

### Reinforcement Learning Basics

**Goal**: Learn control policy that minimizes cost over time

**Key Concepts**:
- **State (s)**: [x, y, z, vx, vy, vz] - vehicle pose and velocity
- **Action (u)**: [roll, pitch, thrust] - control commands
- **Reward (r)**: -(tracking_error² + control_effort²) - negative cost
- **Policy (π)**: Mapping from states to actions
- **Value (Q)**: Expected cumulative reward for state-action pair

### Q-Learning Algorithm

**Update Rule**:
```
Q(s, u) ← Q(s, u) + α[r + γQ(s', π(s')) - Q(s, u)]
```

Where:
- α = learning rate (critic convergence rate)
- γ = discount factor
- s' = next state
- π(s') = actor policy at next state

### Actor-Critic Architecture

**Critic**: Estimates Q-value (how good is state-action pair?)
- Neural network approximator
- Weights updated via temporal difference learning

**Actor**: Determines action (what should I do?)
- Policy network
- Weights updated to maximize Q-value

**Location**: Implementation in `qlearning_controller.cpp:160-240`

---

## Communication Interfaces

### Subscriptions

| Topic | Type | Source | Purpose | Rate |
|-------|------|--------|---------|------|
| `/uam_vehicle_interface/odometry` | `nav_msgs/Odometry` | Vehicle Interface | Current state | 50 Hz |
| `/uam_navigator/position_setpoint` | `nav_msgs/Odometry` | Navigator | Desired state | 20-100 Hz |

### Publications

| Topic | Type | Subscriber | Purpose | Rate |
|-------|------|------------|---------|------|
| `/uam_control/attitude_setpoint` | `uam_control_msgs/AttitudeSetpoint` | Vehicle Interface | Control commands | 100 Hz |
| `/uam_control/qlearning_status` | `uam_control_msgs/QLearningStatus` | Monitoring | Learning status | 10 Hz |

### Message Definitions

**AttitudeSetpoint.msg**:
```
float64 roll_body            # Desired roll (radians)
float64 pitch_body           # Desired pitch (radians)
float64 yaw_body             # Desired yaw (radians)
float64[4] q_d               # Desired quaternion
float64 thrust_body_normalized  # Thrust (Newtons)
```

**QLearningStatus.msg**:
```
std_msgs/Header header
bool learning_enabled        # Whether learning is active
float64 critic_error         # Temporal difference error
float64 tracking_error       # Position tracking error
```

---

## Configuration

### Parameters

From `params/sitl_params.yaml`:

```yaml
uam_control:
  ros__parameters:
    rrtx_static:
      critic_convergence_rate: 10.0         # Critic learning rate (α_c)
      actor_convergence_rate: 0.5           # Actor learning rate (α_a)
      minimum_altitude_for_learning: 0.6    # Don't learn below this altitude (safety)
      learning_update_frequency: 100.0      # Hz (control loop rate)
```

### Parameter Details

**`critic_convergence_rate`**: How fast the critic adapts
- Higher = faster learning but less stable
- Lower = slower but more stable
- Typical: 5.0 - 20.0
- Default: 10.0

**`actor_convergence_rate`**: How fast the policy adapts
- Should be < critic rate (actor follows critic)
- Typical: 0.1 - 1.0
- Default: 0.5

**`minimum_altitude_for_learning`**: Safety threshold
- Learning disabled below this altitude
- Prevents dangerous learning during takeoff/landing
- Default: 0.6 meters

**`learning_update_frequency`**: Control loop rate
- Should match or exceed navigator setpoint rate
- Higher = smoother control but more CPU
- Default: 100 Hz

---

## State and Control Representation

### State Vector (6 dimensions)

```cpp
mavState = [x, y, z, vx, vy, vz]ᵀ
```

- **Position** (x, y, z): meters in map frame
- **Velocity** (vx, vy, vz): m/s in map frame

### Control Vector (3 dimensions)

```cpp
control = [roll, pitch, thrust]ᵀ
```

- **Roll**: radians (affects lateral motion)
- **Pitch**: radians (affects longitudinal motion)
- **Thrust**: Newtons (affects vertical motion)

**Note**: Yaw is currently held constant (not learned)

### Augmented State Vector

```cpp
augmented_state = [x, y, z, vx, vy, vz, roll, pitch, thrust]ᵀ
```

Combined state + control for Q-function approximation.

**Location**: Definitions in `qlearning_controller.hpp:22-96`

---

## How It Works

### 1. Initialization

- Load learning rate parameters
- Initialize critic weights (random or zeros)
- Initialize actor weights (random or zeros)
- Create radial basis functions for function approximation

### 2. Control Loop (100 Hz)

```cpp
while (rclcpp::ok()) {
    // 1. Get current state from odometry
    state = extractState(vehicle_odom);

    // 2. Get desired state from navigator
    state_desired = extractState(position_setpoint);

    // 3. Compute tracking error
    error = state_desired - state;

    // 4. Actor computes control action
    control = actor.computeAction(error);

    // 5. Apply control, observe next state
    publishAttitudeSetpoint(control);

    // 6. Critic evaluates action
    Q_value = critic.evaluate(state, control);
    reward = -||error||² - λ||control||²

    // 7. Temporal difference learning
    td_error = reward + γ*Q(next_state, next_control) - Q(state, control);

    // 8. Update critic weights (only if altitude > threshold)
    if (altitude > minimum_altitude_for_learning) {
        critic_weights += α_c * td_error * ∇Q;
    }

    // 9. Update actor weights
    if (altitude > minimum_altitude_for_learning) {
        actor_weights += α_a * ∇Q;
    }
}
```

### 3. Attitude Computation

From control vector to attitude setpoint:

```cpp
// Normalize thrust
thrust_normalized = clamp(thrust, 0.0, max_thrust);

// Compute quaternion from euler angles
q = eulerToQuaternion(roll, pitch, yaw);

// Publish setpoint
setpoint.roll_body = roll;
setpoint.pitch_body = pitch;
setpoint.yaw_body = yaw;
setpoint.q_d = q;
setpoint.thrust_body_normalized = thrust_normalized;
```

**Location**: `qlearning_controller.cpp:250-280`

---

## Usage

### Basic Usage

The controller runs automatically when launched:

```bash
# Included in sitl_launch.py
ros2 launch uam_vehicle_interface sitl_launch.py
```

Or standalone:

```bash
# Load parameters
ros2 run uam_control qlearning_controller --ros-args --params-file params/sitl_params.yaml
```

### Monitoring Learning

```bash
# Watch learning status
ros2 topic echo /uam_control/qlearning_status

# Monitor tracking error
ros2 topic echo /uam_control/qlearning_status | grep tracking_error

# Check if learning is enabled
ros2 topic echo /uam_control/qlearning_status | grep learning_enabled
```

### Tuning Learning Rates

**If tracking error is high**:
```yaml
# Increase actor learning rate
actor_convergence_rate: 1.0  # from 0.5
```

**If control is oscillating**:
```yaml
# Decrease critic learning rate
critic_convergence_rate: 5.0  # from 10.0
```

**If learning is too slow**:
```yaml
# Increase both rates (carefully!)
critic_convergence_rate: 15.0
actor_convergence_rate: 0.8
```

---

## Safety Features

### Altitude Threshold

Learning is **disabled** when `z < minimum_altitude_for_learning`:

```cpp
if (current_altitude >= minimum_altitude_for_learning) {
    // Safe to learn
    updateCritic();
    updateActor();
} else {
    // Use last known policy, don't update
    control = actor.computeAction(error);  // No learning
}
```

**Rationale**: Prevents dangerous exploration near ground.

### Control Saturation

All control commands are clamped to safe ranges:

```cpp
roll = clamp(roll, -max_roll, max_roll);      // Typically ±30°
pitch = clamp(pitch, -max_pitch, max_pitch);
thrust = clamp(thrust, min_thrust, max_thrust);
```

---

## Troubleshooting

### Issue 1: Vehicle Unstable After Enabling Controller

**Symptom**: Oscillations or divergence

**Causes**:
1. Learning rates too high
2. Initial weights far from optimal
3. Interference with PX4 inner loop

**Solutions**:
```yaml
# Reduce learning rates
critic_convergence_rate: 2.0   # Very conservative
actor_convergence_rate: 0.1

# Or disable learning initially (use as baseline controller)
actor_convergence_rate: 0.0
```

### Issue 2: Learning Not Improving Performance

**Symptom**: Tracking error stays constant over time

**Causes**:
1. Learning rates too low
2. Altitude threshold preventing learning
3. Insufficient exploration

**Solutions**:
```yaml
# Increase learning rates
critic_convergence_rate: 20.0
actor_convergence_rate: 1.0

# Lower altitude threshold (CAREFULLY!)
minimum_altitude_for_learning: 0.4
```

### Issue 3: Control Commands Saturating

**Symptom**: Roll/pitch at limits, jerky motion

**Cause**: Actor outputting extreme values

**Solution**:
```cpp
// In code, increase control cost weight
reward = -tracking_error.squaredNorm() - 10.0 * control.squaredNorm();
// Higher weight penalizes large control more
```

---

## Advanced Topics

### Function Approximation

The Q-function is approximated using **radial basis functions (RBF)**:

```cpp
ϕ(s, u) = exp(-||[s; u] - c_i||² / σ²)
Q(s, u) ≈ Σ w_i * ϕ_i(s, u)
```

Where:
- c_i = RBF centers
- σ = RBF width
- w_i = critic weights (learned)

**Location**: Radial basis matrix setup in `qlearning_controller.cpp`

### Exploration vs. Exploitation

Currently: **Pure exploitation** (no exploration noise)

To add exploration:
```cpp
// Add Gaussian noise to actor output
control = actor.computeAction(state) + exploration_noise;
exploration_noise = N(0, σ_explore²)
```

Gradually decay σ_explore over time (annealing).

---

## Performance Characteristics

### Computational Cost

- **CPU**: 10-20% (single core, 100 Hz)
- **Memory**: ~50 MB (weight matrices)

### Learning Time

- **Initial improvement**: 10-30 seconds of flight
- **Convergence**: 2-5 minutes (depends on task complexity)
- **Refinement**: Continues indefinitely (minor improvements)

### Tracking Performance

**After Learning**:
- Position error: < 0.05 meters (steady state)
- Velocity error: < 0.1 m/s
- Settling time: ~1-2 seconds for step changes

**Note**: Performance depends heavily on tuning.

---

## Limitations

1. **No obstacle awareness**: Purely tracking-focused
2. **Fixed yaw**: Yaw not learned/controlled
3. **Stationary targets only**: Assumes slow-moving setpoints
4. **Single vehicle**: Not designed for formation control
5. **Research-grade**: Lacks extensive flight testing

---

## Future Enhancements

1. **Deep Q-Learning**: Replace RBF with neural networks
2. **Experience Replay**: Store and replay past transitions
3. **Multi-Agent**: Coordinate multiple vehicles
4. **Safe Exploration**: Constrained learning with safety guarantees
5. **Transfer Learning**: Pre-train on simulation, fine-tune on hardware

---

## References

- [Q-Learning Paper (Watkins, 1992)](http://www.cs.rhul.ac.uk/~chrisw/new_thesis.pdf)
- [Actor-Critic Methods](https://papers.nips.cc/paper/1786-actor-critic-algorithms.pdf)
- Adaptive Dynamic Programming for UAV Control

---

## License

Apache License 2.0

## Maintainers

See main repository README for maintainer information.
