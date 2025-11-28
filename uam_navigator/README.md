# uam_navigator - Navigation State Machine

## Overview

The `uam_navigator` package implements a high-level navigation state machine for autonomous flight missions. It manages different flight modes (takeoff, land, loiter, navigate-to-pose), coordinates with the path planner, and publishes position setpoints to the control stack.

Think of it as the **mission manager** that orchestrates autonomous flights by switching between different behavioral modes based on mission requirements and completion criteria.

## Package Architecture

### Sub-Packages

```
uam_navigator/
├── uam_navigator/                # Core navigator node
│   ├── src/
│   │   ├── navigator.cpp        # Main state machine
│   │   └── navigator_modes/     # Flight mode plugins
│   │       ├── takeoff.cpp
│   │       ├── land.cpp
│   │       ├── loiter.cpp
│   │       └── navigate_to_pose.cpp
│   └── include/uam_navigator/
│       ├── navigator.hpp
│       └── navigator_modes/     # Mode headers
└── uam_navigator_msgs/          # Message/action definitions
    ├── action/NavigatorCommand.action
    └── msg/NavigatorStatus.msg
```

---

## Core Concept: State Machine Navigation

### Flight Modes

The navigator implements a finite state machine with the following modes:

1. **Idle**: Initial state, no active mission
2. **Takeoff**: Ascend to configured altitude
3. **Loiter**: Hover at current position
4. **NavigateToPose**: Fly to goal using path planner
5. **Land**: Descend and land (future)

### State Transition Diagram

```
┌──────┐
│ Idle │
└──┬───┘
   │ takeoff command
   v
┌──────────┐
│ Takeoff  │ ────► (reaches altitude) ────┐
└──────────┘                               │
                                           v
┌────────────────┐                    ┌────────┐
│ NavigateToPose │◄───────────────────┤ Loiter │
└────────┬───────┘  navigate command  └────┬───┘
         │                                  │
         └────► (reaches goal) ─────────────┘
                                            │
                                            v
                                        ┌──────┐
                                        │ Land │
                                        └──────┘
```

### Transition Rules

Valid state transitions are defined in `navigator_transitions_` map:

| From State | To States |
|------------|-----------|
| Idle | Takeoff |
| Takeoff | Loiter, Land |
| Loiter | Any navigation mode, Land |
| NavigateToPose | Loiter, Land |

**Invalid transitions** are automatically rejected with an error response.

**Location**: `navigator.cpp:35-42`

---

## Communication Interfaces

### Actions

| Action Name | Type | Purpose | Server/Client |
|-------------|------|---------|---------------|
| `send_navigator_command` | `uam_navigator_msgs/NavigatorCommand` | Request mode change | Server |
| `compute_path_to_pose` | `uam_interfaces/ComputePathToPose` | Request path from planner | Client |

### Topics (Subscriptions)

| Topic | Type | Source | Purpose | Rate |
|-------|------|--------|---------|------|
| `/uam_vehicle_interface/odometry` | `nav_msgs/Odometry` | Vehicle Interface | Current vehicle state | 50 Hz |

### Topics (Publications)

| Topic | Type | Subscriber | Purpose | Rate |
|-------|------|------------|---------|------|
| `/uam_navigator/position_setpoint` | `nav_msgs/Odometry` | Control Node | Desired position/velocity | 20-100 Hz |
| `/uam_navigator/navigator_status` | `uam_navigator_msgs/NavigatorStatus` | GUI/Monitoring | Current navigation mode | 100 Hz |

---

## Navigator Modes (Plugins)

### Mode Interface

All modes implement the `NavigatorMode` interface:

```cpp
class NavigatorMode {
public:
    virtual bool configure(...) = 0;      // Initialize parameters
    virtual bool activate(goal) = 0;       // Start mode execution
    virtual bool deactivate() = 0;         // Stop mode
    virtual bool cleanup() = 0;            // Release resources
    virtual void publishNavigatorSetpoint() = 0;  // Publish setpoint
};
```

### 1. Takeoff Mode

**Purpose**: Ascend to a configured altitude directly above the current position.

**Parameters** (`params/sitl_params.yaml`):
```yaml
navigator:
  ros__parameters:
    takeoff:
      altitude: 0.7                    # meters (NED, negative is up)
      position_tolerance: 0.1          # meters
      velocity_tolerance: 0.3          # m/s
      update_frequency: 30.0           # Hz
```

**Activation**:
```cpp
navigator->takeoff();  // Internal API
// OR via action:
ros2 action send_goal send_navigator_command \
  uam_navigator_msgs/action/NavigatorCommand "{command: 'Takeoff'}"
```

**Behavior**:
1. Records current XY position
2. Sets target Z = -altitude (NED convention, negative is up)
3. Publishes constant position setpoint at configured frequency
4. Monitors position and velocity
5. Transitions to **Loiter** when goal reached (within tolerances)

**Completion Criteria**:
```cpp
distance_to_goal < position_tolerance &&
velocity_magnitude < velocity_tolerance
```

**Location**: `navigator_modes/takeoff.cpp`

---

### 2. Loiter Mode

**Purpose**: Hold current position indefinitely.

**Parameters**: None (uses current position)

**Activation**:
```cpp
navigator->loiter();  // Internal API
// OR via action
```

**Behavior**:
1. Captures current position on activation
2. Publishes this fixed setpoint continuously
3. Remains active until another mode is commanded
4. Acts as "safe state" after completing other missions

**Location**: `navigator_modes/loiter.cpp:58`

---

### 3. NavigateToPose Mode

**Purpose**: Autonomous navigation to a goal pose using obstacle-aware path planning.

**Parameters**:
```yaml
navigator:
  ros__parameters:
    navigate_to_pose:
      planner: "rrtx_static"           # Planner plugin to use
      waypoint_position_tolerance: 0.1  # meters (waypoint reached threshold)
      goal_position_tolerance: 0.1      # meters (final goal threshold)
      goal_velocity_tolerance: 0.1      # m/s (must be slow at goal)
      update_frequency: 100.0           # Hz (setpoint publish rate)
```

**Activation**:
```cpp
// Via action with goal pose
NavigatorCommand::Goal goal;
goal.command = "navigate_to_pose_rrtx_static";
goal.goal.pose.position.x = 2.0;  // ENU coordinates
goal.goal.pose.position.y = 2.0;
goal.goal.pose.position.z = 1.0;
```

**Behavior Flow**:

```
1. Activate called
    │
    v
2. Request path from planner
    │  (async action call to /compute_path_to_pose)
    v
3. Wait for path
    │  (blocks until planner responds)
    v
4. Path received? ──No──> Return failure
    │
   Yes
    v
5. Start waypoint following loop
    │  (timer at update_frequency)
    v
6. For each loop iteration:
    a. Check distance to current waypoint
    b. If reached → advance to next waypoint
    c. Publish current waypoint as setpoint
    d. Check if final goal reached
    e. If goal reached → transition to Loiter
```

**Waypoint Following**:
- Path is a sequence of waypoints from planner
- Current waypoint published as position setpoint
- When within `waypoint_position_tolerance`, advances to next
- Uses 3D Euclidean distance for checks

**Coordinate Frame Handling**:
- Receives odometry in `map_ned` frame from vehicle interface
- Converts to `map` (ENU) for internal processing
- Converts back to `map_ned` when publishing setpoints

**Frame Transform** (NED ↔ ENU):
```cpp
// NED to ENU
odom_enu.x = odom_ned.y;
odom_enu.y = odom_ned.x;
odom_enu.z = -odom_ned.z;
```

**Location**: `navigator_modes/navigate_to_pose.cpp`

**Key Methods**:
- `activate()`: Line 43 - Requests path from planner
- `onLoopCallback()`: Line 103 - Updates waypoint tracking
- `updateWaypoint()`: Line 195 - Advances along path
- `missionComplete()`: Line 206 - Checks goal criteria
- `publishNavigatorSetpoint()`: Line 181 - Publishes setpoint

---

### 4. Land Mode

**Purpose**: Controlled descent and landing.

**Status**: Partially implemented (placeholder)

**Location**: `navigator_modes/land.cpp`

---

## Navigator State Machine Implementation

### Main Loop

The navigator runs a 100 Hz loop (`onLoop()`) that:

1. Publishes current mode status
2. Calls active mode's `publishNavigatorSetpoint()` method
3. Continues until mode changes or deactivates

**Location**: `navigator.cpp:136`

```cpp
void Navigator::onLoop() {
    // Publish status
    nav_status_msg.nav_mode = current_nav_mode_;
    navigator_status_publisher_->publish(nav_status_msg);

    // Skip if idle
    if (current_nav_mode_ == "Idle") return;

    // Active mode publishes its setpoint
    navigators_[current_nav_mode_]->publishNavigatorSetpoint();
}
```

### Mode Switching

**Command Callback** (`commandCallback()`, line 180):

1. **Validate Goal**: Check if requested mode exists
2. **Check Transition**: Verify transition is valid from current state
3. **Activate New Mode**: Call mode's `activate(goal)` method
4. **Deactivate Old Mode**: Call previous mode's `deactivate()` method
5. **Update State**: Set `current_nav_mode_`

**Special Case - Direct Navigation Transitions**:
If switching between two navigation modes (e.g., navigate_to_pose_A → navigate_to_pose_B):
- First transitions to Loiter
- Then transitions to new navigation mode
- Prevents abrupt course changes

**Location**: `navigator.cpp:210-220`

---

## Configuration

### Parameters File Example

From `sitl_params.yaml`:

```yaml
navigator:
  ros__parameters:
    takeoff:
      altitude: 0.7
      position_tolerance: 0.1
      velocity_tolerance: 0.3
      update_frequency: 30.0

    navigate_to_pose:
      planner: "rrtx_static"
      waypoint_position_tolerance: 0.1
      goal_position_tolerance: 0.1
      goal_velocity_tolerance: 0.1
      update_frequency: 100.0
```

### Plugin Configuration

Navigator modes can be configured via parameter:

```yaml
navigator:
  ros__parameters:
    navigator_plugins: ["navigate_to_pose_rrtx_static"]  # List of navigation plugins
```

**Default**: `["navigate_to_pose_rrtx_static"]`

---

## Usage Examples

### Example 1: Basic Takeoff and Navigation

```bash
# Terminal 1: Launch full stack
ros2 launch uam_vehicle_interface sitl_launch.py

# Terminal 2: Command takeoff
ros2 action send_goal /send_navigator_command \
  uam_navigator_msgs/action/NavigatorCommand \
  "{command: 'Takeoff'}"

# Wait for takeoff to complete (auto-transitions to Loiter)

# Terminal 3: Navigate to goal
ros2 action send_goal /send_navigator_command \
  uam_navigator_msgs/action/NavigatorCommand \
  "{command: 'navigate_to_pose_rrtx_static',
    goal: {header: {frame_id: 'map'},
           pose: {position: {x: 2.0, y: 2.0, z: 1.0}}}}"
```

### Example 2: Monitor Navigator Status

```bash
# Watch current mode
ros2 topic echo /uam_navigator/navigator_status

# Watch position setpoints being published
ros2 topic echo /uam_navigator/position_setpoint

# Check action server status
ros2 action list
ros2 action info /send_navigator_command
```

### Example 3: Programmatic Control (C++)

```cpp
#include "rclcpp_action/rclcpp_action.hpp"
#include "uam_navigator_msgs/action/navigator_command.hpp"

class MissionController : public rclcpp::Node {
public:
    using NavigatorCommand = uam_navigator_msgs::action::NavigatorCommand;

    MissionController() : Node("mission_controller") {
        client_ = rclcpp_action::create_client<NavigatorCommand>(
            this, "send_navigator_command");
    }

    void sendTakeoffCommand() {
        auto goal = NavigatorCommand::Goal();
        goal.command = "Takeoff";

        client_->async_send_goal(goal);
    }

    void navigateToPosition(double x, double y, double z) {
        auto goal = NavigatorCommand::Goal();
        goal.command = "navigate_to_pose_rrtx_static";
        goal.goal.header.frame_id = "map";
        goal.goal.pose.position.x = x;
        goal.goal.pose.position.y = y;
        goal.goal.pose.position.z = z;

        auto send_goal_options = rclcpp_action::Client<NavigatorCommand>::SendGoalOptions();
        send_goal_options.result_callback =
            std::bind(&MissionController::resultCallback, this, std::placeholders::_1);

        client_->async_send_goal(goal, send_goal_options);
    }

    void resultCallback(const rclcpp_action::ClientGoalHandle<NavigatorCommand>::WrappedResult & result) {
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
            RCLCPP_INFO(get_logger(), "Navigation succeeded!");
        } else {
            RCLCPP_ERROR(get_logger(), "Navigation failed!");
        }
    }

private:
    rclcpp_action::Client<NavigatorCommand>::SharedPtr client_;
};
```

---

## Lifecycle Management

The Navigator is a **ROS2 Lifecycle Node**, providing controlled state transitions.

### Lifecycle States

```
┌─────────────┐  configure()   ┌──────────┐  activate()   ┌────────┐
│ Unconfigured├───────────────►│ Inactive ├──────────────►│ Active │
└─────────────┘                └──────────┘               └────┬───┘
                                     ▲                          │
                                     │                          │
                                     └──────── deactivate() ────┘
```

### Lifecycle Callbacks

**`on_configure()`** (line 18):
- Create mode instances (Takeoff, Loiter, NavigateToPose, Land)
- Configure each mode with parameters
- Create publishers, subscribers, action server
- Define state transition rules
- Start main loop timer (100 Hz)

**`on_activate()`** (line 73):
- Activate publishers
- Activate action server to accept goals

**`on_deactivate()`** (line 86):
- Deactivate publishers
- Deactivate action server
- Deactivate all navigation modes

**`on_cleanup()`** (line 103):
- Reset all subscribers, publishers
- Clear all modes
- Stop timers

---

## Error Handling

### Exception Types

Defined in `navigator_exceptions.hpp`:

1. **InvalidNavigator**: Requested mode doesn't exist
   ```cpp
   throw InvalidNavigator("Navigator plugin X is invalid");
   ```

2. **InvalidTransition**: Transition not allowed from current state
   ```cpp
   throw InvalidTransition("Cannot go from Takeoff to NavigateToPose");
   ```

3. **NavigatorModeActivationFailed**: Mode failed to activate
   ```cpp
   throw NavigatorModeActivationFailed("Planner server unavailable");
   ```

### Error Codes

Action results include error codes:

```cpp
// From NavigatorCommand.action
uint8 NAV_RESPONSE_SUCCESS = 0
uint8 NAV_RESPONSE_INVALID_NAVIGATOR = 1
uint8 NAV_RESPONSE_INVALID_TRANSITION = 2
uint8 NAV_RESPONSE_NAVIGATOR_MODE_ACTIVATION_FAILED = 3
```

**Location**: Error handling in `navigator.cpp:224-233`

---

## Troubleshooting

### Issue 1: "Navigator Mode Not Activating"

**Symptoms**: Mode command sent but navigator stays in current mode

**Causes**:
1. Invalid transition (e.g., can't go directly from Takeoff to NavigateToPose)
2. Planner server not available (for NavigateToPose mode)
3. Mode activation returned false

**Solutions**:
```bash
# Check current mode
ros2 topic echo /uam_navigator/navigator_status

# Check valid transitions
# Must go: Idle → Takeoff → Loiter → NavigateToPose

# For NavigateToPose, verify planner is running
ros2 node list | grep planner
ros2 action info /compute_path_to_pose

# Check logs for activation errors
ros2 run uam_navigator navigator_main --ros-args --log-level debug
```

### Issue 2: "Takeoff Never Completes"

**Symptoms**: Vehicle reaches altitude but doesn't transition to Loiter

**Cause**: Velocity not settling below tolerance

**Solution**:
```yaml
# Increase velocity tolerance in params
navigator:
  ros__parameters:
    takeoff:
      velocity_tolerance: 0.5  # Increase from 0.3
```

### Issue 3: "NavigateToPose Fails Immediately"

**Symptoms**: Navigation action returns failure without moving

**Causes**:
1. Planner returns no path (goal unreachable/obstructed)
2. Planner server timeout
3. Invalid goal coordinates

**Solutions**:
```bash
# Test planner directly
ros2 action send_goal /compute_path_to_pose \
  uam_interfaces/action/ComputePathToPose \
  "{planner_id: 'rrtx_static',
    start: {pose: {position: {x: 0, y: 0, z: 1}}},
    goal: {pose: {position: {x: 2, y: 2, z: 1}}}}"

# Check for obstacles blocking path
ros2 topic echo /obstacles

# Increase planner solve time in params
planner_server:
  ros__parameters:
    rrtx_static:
      solve_time: 2.0  # Increase from 0.5
```

### Issue 4: "Waypoint Following Oscillates"

**Symptoms**: Vehicle overshoots waypoints, circles back

**Cause**: Waypoint tolerance too tight for control performance

**Solution**:
```yaml
navigator:
  ros__parameters:
    navigate_to_pose:
      waypoint_position_tolerance: 0.2  # Increase from 0.1
```

---

## Performance Characteristics

### Update Rates

- **Status Publishing**: 100 Hz (main loop)
- **Setpoint Publishing (Takeoff)**: 30 Hz (configurable)
- **Setpoint Publishing (NavigateToPose)**: 100 Hz (configurable)
- **Setpoint Publishing (Loiter)**: 100 Hz (via main loop)

### Latency

- **Mode Switch**: ~10-50ms (depending on mode activation)
- **Path Planning Request**: 0.5-5 seconds (depends on planner solve time)
- **Setpoint Update**: < 10ms (from odometry receipt to setpoint publish)

### Resource Usage

- **CPU**: ~2-5% (single core, Intel i5)
- **Memory**: ~30 MB RSS

---

## Advanced Topics

### Adding a Custom Navigation Mode

1. **Create Mode Class**:
```cpp
// include/navigator_modes/my_mode.hpp
class MyMode : public NavigatorMode {
public:
    bool configure(...) override;
    bool activate(goal) override;
    bool deactivate() override;
    void publishNavigatorSetpoint() override;
};
```

2. **Implement Methods**:
```cpp
// src/navigator_modes/my_mode.cpp
bool MyMode::activate(goal) {
    // Initialize mode-specific state
    // Set up timers, load parameters
    return true;
}

void MyMode::publishNavigatorSetpoint() {
    // Compute and publish setpoint
    navigator_->publishOdometrySetpoint(setpoint);
}
```

3. **Register in Navigator**:
```cpp
// In navigator.cpp on_configure()
navigators_.insert({"my_mode", std::make_shared<MyMode>()});

// Add transition rules
navigator_transitions_.insert({"my_mode", {"Loiter", "Land"}});
```

4. **Add Parameters**:
```yaml
# In params file
navigator:
  ros__parameters:
    my_mode:
      param1: value1
```

### Multi-Waypoint Missions

To fly through multiple waypoints:

**Option 1**: Sequential navigation actions
```cpp
for (auto & waypoint : waypoints) {
    navigateToPosition(waypoint.x, waypoint.y, waypoint.z);
    wait_for_completion();
}
```

**Option 2**: Create custom mode with waypoint queue
- Implement in new `FollowWaypoints` mode
- Accept array of poses in goal
- Sequentially navigate through each

---

## Future Enhancements

1. **Dynamic Replanning**: Replan if obstacles change during flight
2. **Velocity Setpoints**: Smoother trajectories with velocity profiles
3. **Formation Flight**: Coordinate multiple vehicles
4. **Failsafe Recovery**: Auto-land on critical errors
5. **Mission Scripting**: YAML-based mission definitions
6. **Geofencing**: Boundary enforcement

---

## References

- [ROS2 Lifecycle Nodes](https://design.ros2.org/articles/node_lifecycle.html)
- [ROS2 Actions](https://docs.ros.org/en/galactic/Tutorials/Understanding-ROS2-Actions.html)
- Main Architecture: `docs/ARCHITECTURE.md`
- Planner Documentation: `uam_planner/README.md`

---

## License

Apache License 2.0

## Maintainers

See main repository README for maintainer information.
