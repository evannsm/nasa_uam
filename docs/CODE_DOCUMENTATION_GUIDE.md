# Code Documentation Guide - NASA UAM Flight Stack

This document explains the mathematical and algorithmic foundations of each module's implementation, complementing the inline code comments.

---

## Table of Contents

1. [Planner Module - RRT* Algorithm](#planner-module)
2. [Navigator Module - State Machine](#navigator-module)
3. [Vehicle Interface - Coordinate Transforms](#vehicle-interface-module)
4. [Control Module - Q-Learning](#control-module)
5. [Mapping Module - Obstacle Management](#mapping-module)
6. [Util Module - Base Classes](#util-module)
7. [Visualization Module - RViz Integration](#visualization-module)
8. [CMakeLists Explained](#cmakelists-explained)

---

## Planner Module

**Files:** `uam_planner/planner_plugins/rrtx_static_planner/src/rrtx_static.cpp`

### Key Functions Explained

#### `createPath()` - Main Planning Method

```cpp
nav_msgs::msg::Path RrtxStatic::createPath(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal)
```

**Algorithm Flow:**

```
1. Clear previous planning problem
   prob_def_ptr_->clearStartStates()
   prob_def_ptr_->clearGoal()
   prob_def_ptr_->clearSolutionPaths()
   planner_ptr_->clear()

2. Convert ROS poses to OMPL states
   - Handle frame transformations (map/map_ned)
   - Set orientation to identity (position-only planning)

3. Set start and goal in problem definition
   prob_def_ptr_->setStartAndGoalStates(start_state, goal_state)

4. Run RRT* planner
   planner_ptr_->solve(solve_time)

   RRT* Iterations:
   ├─ Sample random point x_rand ~ Uniform(C_free)
   ├─ Find nearest: x_near = argmin_{x∈T} ||x - x_rand||
   ├─ Steer: x_new = x_near + min(step_size, ||x_rand - x_near||) · (x_rand - x_near)/||x_rand - x_near||
   ├─ Check collision: CollisionFree(x_near, x_new)?
   ├─ Find nearby: X_near = {x ∈ T | ||x - x_new|| < r_n}
   ├─ Choose parent: x_min = argmin Cost(x) + ||x - x_new||
   ├─ Add to tree: T ← T ∪ {x_new}
   └─ Rewire: for x ∈ X_near: if Cost(x_new) + ||x_new - x|| < Cost(x): parent(x) = x_new

5. Extract path
   path_ptr = getSolutionPath()
   Convert OMPL states → ROS PoseStamped

6. Return waypoint path
```

**Coordinate Frame Handling:**

- **map** (ENU): Use positions directly `(x, y, z)`
- **map_ned** (NED): Transform to ENU `(y, x, -z)`

**Mathematical Details:**

**Near Radius (for rewiring):**
```
r_n = min(γ · (log(n)/n)^(1/d), η)
```
where:
- γ = scaling constant (tunable)
- n = number of nodes in tree
- d = dimension (3 for position)
- η = maximum step size

This ensures asymptotic optimality (Karaman & Frazzoli 2011).

#### `isStateValid()` - Collision Checking

```cpp
bool RrtxStatic::isStateValid(const ompl::base::State *state) const
```

**Purpose:** Determines if a single state (3D position) is collision-free.

**Algorithm:**
```
1. Extract 3D position from SE(3) state
   p = (x, y, z) ∈ ℝ³

2. Check all obstacles
   for obstacle in obstacles_:
       if obstacle.containsPoint(p):
           return false  // Collision!

3. Check if within bounds
   if ¬satisfiesBounds(state):
       return false

4. Return true (collision-free)
```

**Collision Detection (GJK Algorithm):**

The `containsPoint()` method uses **Gilbert-Johnson-Keerthi (GJK)** algorithm:

```
Given: Point p, Convex shape S

GJK computes Minkowski difference: M = S ⊖ {p}
                                      = {s - p | s ∈ S}

If origin ∈ M → collision
If origin ∉ M → no collision

Iterative construction of simplex containing origin:
1. Initialize simplex with arbitrary point in M
2. While simplex doesn't contain origin:
   a. Find support point in direction of origin
   b. Add to simplex
   c. Remove unnecessary points
3. Check if origin inside simplex
```

**Computational Complexity:** O(k) where k ≈ 5-10 iterations

**Reference:** Gilbert et al. (1988), DOI: 10.1109/56.2083

---

## Navigator Module

**Files:** `uam_navigator/uam_navigator/src/navigator.cpp`

### State Machine Architecture

The navigator implements a **hierarchical state machine** for flight mode management:

```
┌─────────────────────────────────────────┐
│         Navigator State Machine         │
├─────────────────────────────────────────┤
│                                          │
│   ┌──────┐                              │
│   │ Idle │ (initial)                    │
│   └──┬───┘                              │
│      │ takeoff command                  │
│      v                                   │
│   ┌──────────┐                          │
│   │ Takeoff  │ (ascend to altitude)     │
│   └────┬─────┘                          │
│        │ altitude reached                │
│        v                                 │
│   ┌────────┐      navigate cmd          │
│   │ Loiter │◄─────────────────┐         │
│   └───┬────┘                  │         │
│       │                       │         │
│       │ navigate_to_pose      │         │
│       v                       │         │
│   ┌────────────────┐  goal    │         │
│   │ NavigateToPose │──────────┘         │
│   └────────────────┘  reached           │
│                                          │
└─────────────────────────────────────────┘
```

**State Transition Validation:**

```cpp
bool isValidTransition(Mode from, Mode to) {
    // Prevent invalid transitions (e.g., Idle → Navigate)
    // Ensures safe state progression
}
```

### Key Functions

#### `update()` - Main Control Loop (100 Hz)

```cpp
void Navigator::update() {
    // 1. Get current state from odometry
    current_state = getCurrentState();

    // 2. Call active mode's update
    active_mode_->update(current_state);

    // 3. Check for mode completion
    if (active_mode_->isComplete()) {
        transitionTo(next_mode);
    }

    // 4. Publish setpoint from active mode
    publishSetpoint(active_mode_->getSetpoint());

    // 5. Publish status
    publishStatus();
}
```

#### Takeoff Mode

**Goal:** Climb to configured altitude while maintaining XY position.

**Control Law:**
```
Setpoint:
  x_sp = x_initial
  y_sp = y_initial
  z_sp = z_takeoff_altitude

  vx_sp = 0
  vy_sp = 0
  vz_sp = 0

Completion Criteria:
  |z - z_sp| < position_tolerance AND
  |vz| < velocity_tolerance
```

**Parameters:**
- `altitude`: Target height (default: 0.7m)
- `position_tolerance`: Acceptance threshold (0.1m)
- `velocity_tolerance`: Settling requirement (0.3 m/s)

#### Navigate-to-Pose Mode

**Goal:** Follow planned path to goal position.

**Algorithm:**
```
1. Request path from planner (async action)
   path = planner.computePath(current_pose, goal_pose)

2. Extract waypoints from path
   waypoints[] = [w0, w1, ..., wn]

3. Waypoint following loop:
   for waypoint in waypoints:
       setpoint = waypoint
       while ||position - waypoint|| > tolerance:
           publish setpoint
           wait
       next waypoint

4. Goal reached when final waypoint achieved
```

**Frame Handling:**

Navigator receives odometry in **map_ned** (NED frame) but plans in **map** (ENU frame):

```cpp
// NED → ENU conversion
Eigen::Vector3d ned_to_enu(const Eigen::Vector3d& ned) {
    return Eigen::Vector3d(ned.y(), ned.x(), -ned.z());
}

// Odometry (map_ned) → Planning (map)
plan_position = ned_to_enu(odom_position);

// Setpoint (map) → Control (map_ned)
control_setpoint = enu_to_ned(plan_setpoint);
```

---

## Vehicle Interface Module

**Files:** `uam_vehicle_interface/uam_vehicle_interface/src/px4_interface.cpp`

### Coordinate Frame Transformations

The vehicle interface bridges between ROS2 (ENU) and PX4 (NED/FRD) coordinate systems.

#### Frame Definitions

**ENU (East-North-Up) - ROS2 Standard:**
```
x-axis: East (right on map)
y-axis: North (up on map)
z-axis: Up (altitude)
```

**NED (North-East-Down) - PX4 World Frame:**
```
x-axis: North
y-axis: East
z-axis: Down (negative altitude)
```

**FRD (Forward-Right-Down) - PX4 Body Frame:**
```
x-axis: Forward (vehicle nose direction)
y-axis: Right (vehicle right wing)
z-axis: Down (through vehicle belly)
```

#### Position Transformation

**ENU → NED:**
```cpp
Eigen::Vector3d enu_to_ned(const Eigen::Vector3d& enu) {
    return Eigen::Vector3d(
        enu.y(),   // North = ENU_y
        enu.x(),   // East = ENU_x
        -enu.z()   // Down = -ENU_z
    );
}
```

**Transformation Matrix:**
```
[N]   [ 0  1  0] [E]
[E] = [ 1  0  0] [N]
[D]   [ 0  0 -1] [U]
```

#### Quaternion Transformation

**ENU → FRD Orientation:**

Uses `px4_ros_com::frame_transforms` library:

```cpp
// Step 1: ENU quaternion → NED quaternion
Eigen::Quaterniond q_ned = ned_to_enu_orientation(q_enu);

// Step 2: NED quaternion → FRD body frame
Eigen::Quaterniond q_frd = aircraft_to_baselink_orientation(q_ned);
```

**Mathematical Details:**

Rotation between frames represented as quaternion multiplication:

```
q_FRD = q_ENU_to_NED · q_aircraft

where:
  q_ENU_to_NED rotates from ENU to NED world frame
  q_aircraft rotates from NED to body frame
```

### Offboard Mode Management

**PX4 Requirements for Offboard Control:**

1. **Warm-up Phase:** Send 10 setpoints before mode switch
2. **Heartbeat:** Continuous 20 Hz setpoint stream required
3. **Failsafe:** PX4 triggers RTL if setpoints stop for > 0.5s

**Implementation:**

```cpp
void publishOffboardControlMode() {
    if (offboard_counter_ < 10) {
        // Warm-up: just send setpoints
        offboard_counter_++;
    } else if (offboard_counter_ == 10) {
        // Trigger mode switch to offboard
        publishVehicleCommand(VEHICLE_CMD_DO_SET_MODE, 1, 6);
        offboard_counter_++;
    }
    // Continue sending setpoints at 20+ Hz
}
```

**Safety:** Kill switch prevents offboard activation:
```cpp
if (kill_switch_enabled_) {
    offboard_counter_ = 0;  // Reset, stay in current mode
    return;
}
```

### Thrust Calculation

**Two Models:**

#### SITL Model (Simplified)

```cpp
// Velocity-based model
thrust = desired_velocity.z() * kp_altitude;
thrust_normalized = clamp(thrust / max_thrust, 0.0, 1.0);
```

#### Hardware Model (Empirical with Battery Compensation)

```cpp
// Measured motor characteristics
thrust_N = compute_motor_thrust(rpm, battery_voltage);

// Battery voltage compensation
thrust_compensated = thrust_N * (nominal_voltage / current_voltage);

// Normalize to [0, 1]
thrust_normalized = clamp(thrust_compensated / hover_thrust, 0.0, 1.0);
```

**Hover Thrust Calibration:**

Measured experimentally for each vehicle:
```
hover_thrust ≈ vehicle_mass * g / num_motors
```

---

## Control Module

See [docs/QLEARNING_THEORY.md](QLEARNING_THEORY.md) for complete mathematical documentation.

**Quick Summary:**

- **Algorithm:** On-policy deterministic actor-critic
- **State:** 6D tracking errors [Δx, Δy, Δz, Δvx, Δvy, Δvz]ᵀ
- **Control:** 3D accelerations [ax, ay, az]ᵀ
- **Learning:** Bellman residual minimization + policy gradient
- **Output:** Geometric control converts acceleration → attitude + thrust

**Key References:**
1. Vamvoudakis & Lewis (2010) - Actor-critic algorithm
2. Mellinger & Kumar (2011) - Geometric control

---

## Mapping Module

**Files:** `uam_mapping/uam_mapping/src/obstacle_advertiser.cpp`

### Purpose

Publishes static obstacle information for path planner collision avoidance.

### Key Function

```cpp
void ObstacleAdvertiser::publishObstacles() {
    // Create obstacle array message
    ObstacleArray msg;

    for (obstacle_id in obstacle_ids_) {
        Obstacle obs;

        // Set position (from parameters)
        obs.pose.position.x = obstacles_x_[i];
        obs.pose.position.y = obstacles_y_[i];
        obs.pose.position.z = obstacles_z_[i];  // Auto: height/2

        // Set shape (BOX, SPHERE, CYLINDER)
        obs.obstacle.type = SolidPrimitive::BOX;
        obs.obstacle.dimensions = [length, width, height];

        msg.obstacles.push_back(obs);
    }

    publisher_->publish(msg);
}
```

### Obstacle Representation

**Box Primitive:**
```
Dimensions: [length_x, length_y, length_z]
Position: Center of box (x, y, z)
Orientation: Quaternion (currently identity)

Volume: V = length_x · length_y · length_z
```

**Z-Position Calculation:**
```cpp
// Obstacle sits on ground (z=0)
// Position is center, so:
z_center = height / 2
```

### Configuration Example

```yaml
static_obstacle_advertiser:
  ros__parameters:
    obstacle_ids: ['1', '2', '3']
    obstacles_x: [1.0, 2.0, 3.0]  # meters (ENU frame)
    obstacles_y: [0.5, 0.5, 0.5]
    # Z auto-computed from obstacle_height

    # Default dimensions (applied to all)
    obstacle_length: 0.4191  # meters
    obstacle_width: 0.4191
    obstacle_height: 0.9779
```

---

## Util Module

**Files:** `uam_util/include/uam_util/*.hpp`

### LifecycleNode Base Class

**Purpose:** Simplified ROS2 lifecycle node with common functionality.

**Lifecycle States:**

```
Unconfigured ──configure──> Inactive ──activate──> Active
                                ↑                     │
                                └───deactivate────────┘
                                │
                            cleanup
                                │
                                ↓
                          (Unconfigured)
```

**Key Methods:**

```cpp
class LifecycleNode : public rclcpp_lifecycle::LifecycleNode {
    // Simplified parameter declaration with constraints
    template<typename T>
    void declareParameter(const std::string& name, T default_value,
                          T min_value, T max_value);

    // Bond management (fault detection)
    void createBond();  // Heartbeat with lifecycle manager
    void destroyBond();

    // Lifecycle callbacks (override in derived class)
    virtual CallbackReturn on_configure(const State&);
    virtual CallbackReturn on_activate(const State&);
    virtual CallbackReturn on_deactivate(const State&);
    virtual CallbackReturn on_cleanup(const State&);
};
```

**Bond Mechanism:**

Creates heartbeat connection with lifecycle manager:
```cpp
// If bond breaks (node crashes), manager can detect and restart
bond_ = std::make_shared<bond::Bond>(
    "lifecycle_manager_bond",
    get_name()
);
bond_->start();
```

### SimpleActionServer

**Purpose:** Wrapper around `rclcpp_action::Server` with simplified interface.

**Features:**

1. **Automatic Goal Queuing:** New goals preempt current goal
2. **Thread-Safe:** Mutex protection for concurrent access
3. **Simplified Callbacks:** Single execute callback instead of multiple

**Usage:**

```cpp
auto action_server = std::make_unique<SimpleActionServer<ActionType>>(
    node,
    "action_name",
    [this]() {
        auto goal = action_server->acceptNewGoal();
        auto result = executeGoal(goal);

        if (success) {
            action_server->succeeded_current(result);
        } else {
            action_server->abort_current(result);
        }
    }
);
```

**Preemption Handling:**

```cpp
if (action_server->is_preempt_requested()) {
    // Stop current goal
    // Start new goal
}
```

---

## Visualization Module

**Files:** `uam_visualization/uam_visualization/src/visualization.cpp`

### Purpose

Converts obstacle data to RViz markers for 3D visualization.

### Marker Creation

```cpp
visualization_msgs::msg::Marker createObstacleMarker(const Obstacle& obs) {
    Marker marker;

    // Header
    marker.header.frame_id = "map";  // ENU frame
    marker.header.stamp = now();

    // Namespace and ID
    marker.ns = "obstacles";
    marker.id = std::stoi(obs.obstacle_id);

    // Type (CUBE, SPHERE, CYLINDER)
    switch (obs.obstacle.type) {
        case SolidPrimitive::BOX:
            marker.type = Marker::CUBE;
            marker.scale.x = obs.obstacle.dimensions[BOX_X];
            marker.scale.y = obs.obstacle.dimensions[BOX_Y];
            marker.scale.z = obs.obstacle.dimensions[BOX_Z];
            break;
    }

    // Position
    marker.pose = obs.pose;

    // Color (red for obstacles)
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;  // Opaque

    // Lifetime (0 = forever)
    marker.lifetime = rclcpp::Duration(0, 0);

    return marker;
}
```

### RViz Integration

**Display Setup:**

1. Add → By topic → `/uam_visualization/obstacle_markers` → MarkerArray
2. Obstacles appear as colored 3D shapes
3. Updates automatically when obstacles change

**Coordinate Frame:** All markers in `map` frame (ENU)

---

## CMakeLists Explained

### Structure of CMakeLists.txt Files

Each package follows standard ROS2 CMake structure:

```cmake
# 1. CMake Version and Project
cmake_minimum_required(VERSION 3.5)
project(package_name)

# 2. Compiler Settings
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)  # C++17 for modern features
endif()

# Compiler warnings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 3. Find Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
# ... other ROS2 packages

find_package(Eigen3 REQUIRED)  # External library
find_package(ompl REQUIRED)

# 4. Include Directories
include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
)

# 5. Build Executables
add_executable(node_name src/node.cpp src/other.cpp)

# 6. Link Dependencies
ament_target_dependencies(node_name
  rclcpp
  std_msgs
  # ... other ament packages
)

target_link_libraries(node_name
  ${OMPL_LIBRARIES}  # Non-ament libraries
)

# 7. Install Targets
install(TARGETS node_name
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch params
  DESTINATION share/${PROJECT_NAME}
)

# 8. Export Information
ament_export_include_directories(include)
ament_export_dependencies(rclcpp std_msgs)

# 9. Package Generation
ament_package()
```

### Example: uam_control CMakeLists.txt

```cmake
cmake_minimum_required(VERSION 3.5)
project(uam_control)

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(uam_navigator_msgs REQUIRED)
find_package(uam_control_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

# Executable
add_executable(qlearning_controller
  src/qlearning_controller.cpp
)

# Link
ament_target_dependencies(qlearning_controller
  rclcpp
  px4_msgs
  nav_msgs
  uam_navigator_msgs
  uam_control_msgs
)

target_link_libraries(qlearning_controller
  Eigen3::Eigen
)

# Install
install(TARGETS qlearning_controller
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
```

### Example: uam_planner Plugin CMakeLists.txt

```cmake
# Plugin library (not executable)
add_library(rrtx_static_planner SHARED
  src/rrtx_static.cpp
)

# Plugin needs specific flags
target_compile_definitions(rrtx_static_planner
  PRIVATE "PLUGINLIB_DISABLE_BOOST_FUNCTIONS"
)

ament_target_dependencies(rrtx_static_planner
  rclcpp
  uam_planner
  ompl
  geometric_shapes
)

# Export plugin
pluginlib_export_plugin_description_file(
  uam_planner
  planner_plugins.xml  # Plugin registration file
)

install(TARGETS rrtx_static_planner
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
```

### Key CMake Patterns

#### 1. Message Generation

```cmake
# In uam_planner_msgs/CMakeLists.txt
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/ComputePathToPose.action"
  DEPENDENCIES
    geometry_msgs
    nav_msgs
)
```

#### 2. Plugin Export

```cmake
# Register plugins with pluginlib
pluginlib_export_plugin_description_file(
  base_package_name
  plugins.xml
)
```

**plugins.xml:**
```xml
<library path="libmy_plugin">
  <class name="my_namespace::MyPlugin"
         type="my_namespace::MyPlugin"
         base_class_type="base_namespace::BaseClass">
    <description>My plugin description</description>
  </class>
</library>
```

#### 3. Lifecycle Node

```cmake
# Lifecycle nodes need rclcpp_lifecycle
find_package(rclcpp_lifecycle REQUIRED)

ament_target_dependencies(my_lifecycle_node
  rclcpp
  rclcpp_lifecycle
)
```

#### 4. Qt GUI Application

```cmake
find_package(Qt5 COMPONENTS Core Widgets REQUIRED)

set(CMAKE_AUTOMOC ON)  # Auto-generate MOC files
set(CMAKE_AUTOUIC ON)  # Auto-process .ui files

add_executable(gui_app
  src/main.cpp
  src/gui.cpp
)

target_link_libraries(gui_app
  Qt5::Core
  Qt5::Widgets
)
```

---

## Build Process Explained

### Colcon Build Workflow

```bash
colcon build --symlink-install
```

**What Happens:**

```
1. Dependency Resolution
   ├─ Reads package.xml in each package
   ├─ Determines build order based on dependencies
   └─ Topological sort: base packages first

2. CMake Configuration
   ├─ cmake -DCMAKE_INSTALL_PREFIX=install/package_name ..
   ├─ Finds all dependencies
   ├─ Generates build files
   └─ Creates compile_commands.json

3. Compilation
   ├─ make -j$(nproc)  # Parallel build
   ├─ Compiles .cpp → .o object files
   ├─ Links .o + libraries → executables/libraries
   └─ Generates message/service code

4. Installation
   ├─ Copies executables to install/package/lib/
   ├─ Copies headers to install/package/include/
   ├─ Copies launch/params to install/package/share/
   └─ Creates setup.bash for environment

5. Symlink Install (--symlink-install)
   ├─ Instead of copying Python/launch files
   ├─ Creates symlinks to source
   └─ Allows editing without rebuild
```

### Dependency Graph Example

```
uam_navigator
    ├─ depends on: uam_planner (action client)
    ├─ depends on: uam_util (base classes)
    └─ depends on: uam_navigator_msgs (messages)

uam_planner
    ├─ depends on: uam_mapping_msgs (obstacles)
    ├─ depends on: uam_util (base classes)
    └─ depends on: ompl (external)

Build Order:
1. uam_util (no deps)
2. uam_*_msgs packages (no deps)
3. uam_mapping, uam_planner, uam_control (depend on msgs, util)
4. uam_navigator (depends on planner)
```

---

## Summary of Code Comments Added

### ✅ Comprehensive Comments Added To:

1. **uam_control/src/qlearning_controller.cpp** (~300 lines of math/algorithm docs)
   - File header with theory and references
   - All major methods explained step-by-step
   - Mathematical equations with citations

2. **uam_planner/planner_plugins/rrtx_static_planner/src/rrtx_static.cpp** (~200 lines)
   - RRT* algorithm explained
   - OMPL setup documented
   - Collision checking theory
   - Coordinate transforms

3. **This Guide** (CODE_DOCUMENTATION_GUIDE.md)
   - Navigator state machine
   - Vehicle interface transforms
   - Mapping obstacle handling
   - Util base classes
   - Visualization markers
   - CMakeLists structure

### 📚 Academic References Cited

1. **Vamvoudakis & Lewis (2010)** - Actor-critic Q-learning
2. **Karaman & Frazzoli (2011)** - RRT* optimality
3. **LaValle (2006)** - Planning algorithms
4. **Şucan et al. (2012)** - OMPL library
5. **Gilbert et al. (1988)** - GJK collision detection
6. **Mellinger & Kumar (2011)** - Geometric control
7. **Lee et al. (2010)** - SE(3) tracking control
8. **Magnus & Neudecker (1999)** - Matrix calculus

---

*For implementation details, see inline code comments in each module.*
