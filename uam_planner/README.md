# uam_planner - Path Planning Server

## Overview

The `uam_planner` package provides collision-free path planning for autonomous navigation in 3D environments. It uses the **OMPL (Open Motion Planning Library)** with the **RRT*-static** algorithm to compute optimal paths that avoid obstacles while considering vehicle constraints.

The planner acts as a service that the navigator calls whenever it needs a path from current position to a goal.

## Package Architecture

### Sub-Packages

```
uam_planner/
├── uam_planner/                    # Planner server
│   ├── src/planner_server.cpp     # Main server node
│   └── include/uam_planner/
│       ├── planner_server.hpp
│       └── planner_base.hpp       # Plugin interface
├── planner_plugins/                # Planner algorithm plugins
│   └── rrtx_static_planner/
│       ├── src/rrtx_static.cpp    # RRT*-static implementation
│       └── include/rrtx_static_planner/
│           └── rrtx_static.hpp
└── uam_planner_msgs/              # Message definitions
    └── action/ComputePathToPose.action
```

### Design Pattern: Plugin Architecture

The planner uses a plugin-based design allowing multiple planning algorithms:

```
PlannerServer (manages plugins)
    │
    ├─> PlannerBase (interface)
    │       │
    │       └─> RrtxStatic (concrete implementation)
    │       └─> [Future: A*, RRT-Connect, PRM, etc.]
```

**Benefits**:
- Easy to add new planning algorithms
- Switch algorithms at runtime via configuration
- Test and compare different approaches

---

## Core Components

### 1. Planner Server

**Purpose**: ROS2 lifecycle node that manages planner plugins and provides path planning as an action.

**Location**: `uam_planner/src/planner_server.cpp`

**Responsibilities**:
- Load and manage planner plugins
- Provide `compute_path_to_pose` action server
- Handle planning requests from navigator
- Publish computed paths for visualization
- Manage lifecycle (configure, activate, deactivate)

### 2. RRT*-static Planner Plugin

**Purpose**: Implements the RRT*-static (Rapidly-exploring Random Tree Star) algorithm for optimal path planning.

**Location**: `planner_plugins/rrtx_static_planner/src/rrtx_static.cpp`

**Algorithm**: RRT*-static
- Asymptotically optimal sampling-based planner
- Incrementally improves solution quality over time
- Efficient for high-dimensional spaces
- Well-suited for UAV 3D planning

---

## Communication Interfaces

### Actions

| Action Name | Type | Purpose | Rate |
|-------------|------|---------|------|
| `compute_path_to_pose` | `uam_planner_msgs/ComputePathToPose` | Plan path from start to goal | On-demand |

**Action Definition**:
```
# Goal
geometry_msgs/PoseStamped start    # Starting pose
geometry_msgs/PoseStamped goal     # Goal pose
string planner_id                  # Which planner plugin to use

---
# Result
nav_msgs/Path path                 # Computed waypoint path
duration planning_time             # Time taken to compute
uint8 error_code                   # Error code (if failed)

# Error codes
uint8 NONE=0
uint8 INVALID_PLANNER=1
uint8 START_OCCUPIED=2
uint8 GOAL_OCCUPIED=3
uint8 NO_VALID_PATH=4
uint8 TIMEOUT=5
uint8 START_OUTSIDE_MAP=6
uint8 GOAL_OUTSIDE_MAP=7
uint8 UNKNOWN=255

---
# Feedback
(none currently)
```

### Subscriptions

| Topic | Type | Source | Purpose |
|-------|------|--------|---------|
| `uam_mapping/obstacles` | `uam_mapping_msgs/ObstacleArray` | Mapping Node | Obstacle positions for collision checking |

### Publications

| Topic | Type | Subscriber | Purpose | Rate |
|-------|------|------------|---------|------|
| `planner_server/path` | `nav_msgs/Path` | Visualization | Computed path for RViz | On-demand |

---

## RRT*-static Algorithm

### What is RRT*-static?

**RRT* (Rapidly-exploring Random Tree Star)** is a sampling-based path planning algorithm that:

1. **Builds a tree** of random samples in the configuration space
2. **Connects samples** with feasible edges (collision-free)
3. **Rewires the tree** to improve path cost (optimality)
4. **Returns best path** from start to goal

**Static variant**: Optimized for static environments (obstacles don't move).

### Algorithm Flow

```
Initialize:
  - Start state
  - Goal state
  - Empty tree

Loop (until timeout or solution found):
  1. Sample random point in space
  2. Find nearest node in tree
  3. Extend tree toward sample
  4. Check for collisions
  5. If collision-free:
      a. Add to tree
      b. Rewire nearby nodes if improves cost
  6. Check if goal reachable from new node

Return: Best path found
```

### Implementation Details

**State Space**: SE(3) - Special Euclidean group in 3D
- **Position**: (x, y, z) ∈ ℝ³
- **Orientation**: Currently ignored (identity rotation)

**Collision Checking**:
- Uses `geometric_shapes` library with body collision primitives
- Checks if position intersects with any obstacle
- Discretizes edges for continuous collision checking

**Bounds**: Configurable 3D bounding box
```yaml
bounds_low:  [-0.5, -0.5, 0.6]   # [x_min, y_min, z_min]
bounds_high: [4.2672, 4.8768, 1.2]  # [x_max, y_max, z_max]
```

**Location**:
- Configuration: `rrtx_static.cpp:33-48`
- Path creation: `rrtx_static.cpp:143-209`
- Collision checking: `rrtx_static.cpp:211-223`

---

## Configuration

### Parameters

From `params/sitl_params.yaml`:

```yaml
planner_server:
  ros__parameters:
    planner_plugins: ["rrtx_static"]  # List of available planners
    global_frame: "map"                # Planning frame
    robot_base_frame: "base_link"     # Robot frame

    rrtx_static:
      plugin: "rrtx_static_planner::RrtxStatic"  # Plugin class
      obstacle_scaling: 2.0            # Safety margin multiplier
      bounds_low:  [-0.5, -0.5, 0.6]  # Planning bounds (meters)
      bounds_high: [4.2672, 4.8768, 1.2]
      solve_time: 0.5                  # Max planning time (seconds)
```

### Parameter Details

**`obstacle_scaling`**: Safety margin around obstacles
- Value > 1.0 inflates obstacles
- Prevents getting too close
- Default: 2.0 (doubles obstacle size)
- **Location**: `rrtx_static.cpp:78-79`

**`bounds_low/bounds_high`**: Planning space limits
- Defines allowed (x, y, z) region
- Samples only within bounds
- Should match physical environment
- **Location**: `rrtx_static.cpp:96-103`

**`solve_time`**: Maximum planning duration
- RRT* is anytime algorithm (improves over time)
- Longer time = better path quality
- Trade-off: latency vs. optimality
- Typical values: 0.5-5.0 seconds
- **Location**: `rrtx_static.cpp:184`

---

## Obstacle Handling

### Obstacle Subscription

The planner subscribes to obstacle updates from the mapping node:

**Topic**: `uam_mapping/obstacles`

**Message**: `ObstacleArray`
```
string[] obstacle_ids
Obstacle[] obstacles

Obstacle:
  string obstacle_id
  geometry_msgs/Pose pose
  shape_msgs/SolidPrimitive obstacle  # Shape (BOX, SPHERE, etc.)
```

### Obstacle Processing

**Dynamic Updates** (`rrtx_static.cpp:50-92`):
1. Receive obstacle array message
2. Remove obstacles no longer in list
3. Update existing obstacles with new poses
4. Add new obstacles to collision checker

**Collision Bodies**:
- Uses `bodies::Box` from `geometric_shapes`
- Scaled by `obstacle_scaling` factor
- Supports only BOX primitives currently
- Future: Add SPHERE, CYLINDER support

### Collision Checking

**State Validity** (`isStateValid()`, line 211):
```cpp
bool RrtxStatic::isStateValid(const ompl::base::State *state) {
    // Extract 3D position
    Eigen::Vector3d position(x, y, z);

    // Check all obstacles
    for (auto& obstacle : obstacles_) {
        if (obstacle.containsPoint(position)) {
            return false;  // Collision detected
        }
    }

    // Check within bounds
    return space_info_->satisfiesBounds(state);
}
```

**Edge Validation**:
- OMPL uses `DiscreteMotionValidator`
- Discretizes edge into small segments
- Checks each segment for collision
- Resolution: 0.01 (configurable via `setStateValidityCheckingResolution`)
- **Location**: `rrtx_static.cpp:110-111`

---

## Coordinate Frame Handling

### Frame Transformations

The planner handles both `map` (ENU) and `map_ned` (NED) frames:

**ENU → Internal** (`createPath()`, line 152-162):
```cpp
if (start.header.frame_id == "map") {
    // ENU: use as-is
    start_state->setXYZ(x, y, z);
} else if (start.header.frame_id == "map_ned") {
    // NED → ENU conversion
    start_state->setXYZ(y, x, -z);
}
```

**Output**: Always returns path in `global_frame` (typically "map")

---

## Usage Examples

### Example 1: Request Path via Action

```bash
# Terminal 1: Ensure planner server is running
ros2 lifecycle set /planner_server configure
ros2 lifecycle set /planner_server activate

# Terminal 2: Request path
ros2 action send_goal /compute_path_to_pose \
  uam_planner_msgs/action/ComputePathToPose \
  "{start: {header: {frame_id: 'map'},
            pose: {position: {x: 0.0, y: 0.0, z: 1.0}}},
    goal: {header: {frame_id: 'map'},
           pose: {position: {x: 2.0, y: 2.0, z: 1.0}}},
    planner_id: 'rrtx_static'}"

# Observe result:
# - SUCCESS: Result contains nav_msgs/Path
# - FAILURE: Error code indicates reason
```

### Example 2: Programmatic Usage (C++)

```cpp
#include "rclcpp_action/rclcpp_action.hpp"
#include "uam_planner_msgs/action/compute_path_to_pose.hpp"

class PathRequester : public rclcpp::Node {
public:
    using ComputePath = uam_planner_msgs::action::ComputePathToPose;

    PathRequester() : Node("path_requester") {
        client_ = rclcpp_action::create_client<ComputePath>(
            this, "compute_path_to_pose");
    }

    void requestPath() {
        auto goal = ComputePath::Goal();
        goal.planner_id = "rrtx_static";

        // Set start pose
        goal.start.header.frame_id = "map";
        goal.start.pose.position.x = 0.0;
        goal.start.pose.position.y = 0.0;
        goal.start.pose.position.z = 1.0;

        // Set goal pose
        goal.goal.header.frame_id = "map";
        goal.goal.pose.position.x = 3.0;
        goal.goal.pose.position.y = 3.0;
        goal.goal.pose.position.z = 1.0;

        // Send goal
        auto send_options = rclcpp_action::Client<ComputePath>::SendGoalOptions();
        send_options.result_callback =
            std::bind(&PathRequester::resultCallback, this, std::placeholders::_1);

        client_->async_send_goal(goal, send_options);
    }

    void resultCallback(const rclcpp_action::ClientGoalHandle<ComputePath>::WrappedResult & result) {
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
            RCLCPP_INFO(get_logger(), "Path found with %zu waypoints",
                result.result->path.poses.size());

            // Process path
            for (const auto & pose : result.result->path.poses) {
                // Use waypoints...
            }
        } else {
            RCLCPP_ERROR(get_logger(), "Planning failed with error code: %d",
                result.result->error_code);
        }
    }

private:
    rclcpp_action::Client<ComputePath>::SharedPtr client_;
};
```

### Example 3: Monitor Planning

```bash
# Watch planned paths
ros2 topic echo /planner_server/path

# Monitor action server
ros2 action info /compute_path_to_pose

# Check planner server lifecycle state
ros2 lifecycle get /planner_server

# View in RViz
rviz2
# Add -> By Topic -> /planner_server/path -> Path
```

---

## Lifecycle Management

The planner server is a **ROS2 Lifecycle Node**:

### Lifecycle Callbacks

**`on_configure()`** (line 47):
- Load planner plugin(s) via pluginlib
- Configure each plugin with parameters
- Create action server
- Create path publisher

**`on_activate()`** (line 108):
- Activate action server (start accepting goals)
- Activate path publisher
- Activate all planner plugins

**`on_deactivate()`** (line 127):
- Deactivate action server (reject new goals)
- Deactivate publisher
- Deactivate plugins

**`on_cleanup()`** (line 143):
- Reset action server and publisher
- Cleanup all plugins
- Clear plugin map

---

## Error Handling

### Exception Types

Defined in `planner_exceptions.hpp`:

1. **InvalidPlanner**: Requested planner doesn't exist
2. **StartOccupied**: Start position in collision
3. **GoalOccupied**: Goal position in collision
4. **NoValidPathCouldBeFound**: Algorithm couldn't find solution
5. **PlannerTimedOut**: Exceeded solve_time
6. **StartOutsideMapBounds**: Start outside planning bounds
7. **GoalOutsideMapBounds**: Goal outside planning bounds

### Error Handling Flow

**In `computePath()`** (line 200-252):
```cpp
try {
    result->path = get_path(start, goal, planner_id);
    action_server_pose_->succeeded_current(result);
} catch (InvalidPlanner & ex) {
    result->error_code = INVALID_PLANNER;
    action_server_pose_->terminate_current(result);
} catch (NoValidPathCouldBeFound & ex) {
    result->error_code = NO_VALID_PATH;
    action_server_pose_->terminate_current(result);
}
// ... handle other exceptions
```

---

## Troubleshooting

### Issue 1: "NO_VALID_PATH Error"

**Symptoms**: Planner returns error, no path found

**Causes**:
1. Obstacles blocking all paths to goal
2. solve_time too short for complex environment
3. Goal in collision with obstacle
4. Start or goal outside bounds

**Solutions**:
```yaml
# Increase solve time
rrtx_static:
  solve_time: 2.0  # Increase from 0.5

# Reduce obstacle scaling
  obstacle_scaling: 1.5  # Reduce from 2.0

# Expand planning bounds
  bounds_high: [5.0, 5.0, 2.0]  # Larger region
```

```bash
# Check if goal is valid
ros2 topic echo /uam_mapping/obstacles
# Verify goal not inside any obstacle

# Test with simpler goal
ros2 action send_goal /compute_path_to_pose ... # closer goal
```

### Issue 2: "Planning Takes Too Long"

**Symptoms**: Planner succeeds but takes several seconds

**Cause**: solve_time too high or complex environment

**Solutions**:
```yaml
# Reduce solve time (trade quality for speed)
rrtx_static:
  solve_time: 0.3  # Faster but may be suboptimal

# Reduce valid segment count (less collision checks)
# In rrtx_static.cpp:104
state_space_ptr_->setValidSegmentCountFactor(50);  # Reduce from 100
```

### Issue 3: "Path Too Close to Obstacles"

**Symptoms**: Path navigable but uncomfortably near obstacles

**Solution**:
```yaml
# Increase obstacle scaling
rrtx_static:
  obstacle_scaling: 3.0  # Increase from 2.0
```

### Issue 4: "GOAL_OCCUPIED but Goal Looks Clear"

**Cause**: Obstacle scaling inflates obstacles into goal

**Solution**:
```yaml
# Reduce obstacle scaling
rrtx_static:
  obstacle_scaling: 1.2  # Minimal inflation
```

### Issue 5: "Planner Server Not Responding"

**Symptoms**: Action calls timeout

**Causes**:
1. Server not in active state
2. Server crashed

**Solutions**:
```bash
# Check server state
ros2 lifecycle get /planner_server

# If not active, activate
ros2 lifecycle set /planner_server configure
ros2 lifecycle set /planner_server activate

# Check if running
ros2 node list | grep planner

# Restart if necessary
ros2 run uam_planner planner_server
```

---

## Performance Characteristics

### Planning Time

Depends on:
- **Environment complexity**: More obstacles → longer time
- **Distance**: Longer paths → longer time
- **solve_time parameter**: Linear relationship
- **Obstacle count**: ~O(n) per collision check

**Typical Values**:
- Simple (1-3 obstacles): 0.1-0.5 seconds
- Medium (5-10 obstacles): 0.3-1.0 seconds
- Complex (20+ obstacles): 0.5-2.0 seconds

### Path Quality

- **Path length**: Decreases with longer solve_time (RRT* optimizes)
- **Smoothness**: Raw RRT* paths are not smooth (many small segments)
- **Clearance**: Controlled by obstacle_scaling

### Resource Usage

- **CPU**: 20-80% single core during planning
- **Memory**: ~50-100 MB (depends on tree size)
- **Idle**: ~1% CPU when not planning

---

## Advanced Topics

### Adding a New Planner Plugin

1. **Create Plugin Class**:
```cpp
// my_planner.hpp
#include "uam_planner/planner_base.hpp"

class MyPlanner : public uam_planner::PlannerBase {
public:
    void configure(...) override;
    void activate() override;
    void deactivate() override;
    void cleanup() override;
    nav_msgs::msg::Path createPath(start, goal) override;
};
```

2. **Implement createPath()**:
```cpp
// my_planner.cpp
nav_msgs::msg::Path MyPlanner::createPath(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal)
{
    // Your planning algorithm here
    // Return path as nav_msgs::msg::Path
}
```

3. **Export as Plugin**:
```cpp
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(my_planner::MyPlanner, uam_planner::PlannerBase)
```

4. **Register in XML**:
```xml
<!-- my_planner_plugins.xml -->
<library path="libmy_planner">
  <class name="my_planner::MyPlanner" type="my_planner::MyPlanner" base_class_type="uam_planner::PlannerBase">
    <description>My custom planner</description>
  </class>
</library>
```

5. **Configure**:
```yaml
planner_server:
  ros__parameters:
    planner_plugins: ["my_planner"]
    my_planner:
      plugin: "my_planner::MyPlanner"
      # ... custom parameters
```

### Path Smoothing

RRT* paths are often jagged. To smooth:

**Option 1**: Post-process path with splines
```cpp
#include "nav_msgs/msg/path.hpp"

nav_msgs::msg::Path smoothPath(const nav_msgs::msg::Path & raw_path) {
    // Fit cubic spline through waypoints
    // Resample at regular intervals
    // Return smoothed path
}
```

**Option 2**: Use OMPL path simplification
```cpp
// In createPath() after solving
ompl::geometric::PathSimplifier simplifier(space_info_ptr_);
simplifier.smoothBSpline(*path_ptr);
```

---

## Future Enhancements

1. **Dynamic Obstacles**: Replanning when obstacles move
2. **Velocity Constraints**: Respect max velocity in path
3. **Multi-Query Planning**: PRM* for repeated queries
4. **Orientation Planning**: Plan full SE(3) poses, not just positions
5. **Multi-Goal**: Find path visiting multiple waypoints
6. **Informed Sampling**: Use heuristics to guide sampling

---

## References

- [OMPL Documentation](https://ompl.kavrakilab.org/)
- [RRT* Paper](https://arxiv.org/abs/1105.1186)
- [Geometric Shapes Library](http://wiki.ros.org/geometric_shapes)
- Architecture: `docs/ARCHITECTURE.md`

---

## License

Apache License 2.0 (Portions adapted from Navigation2)

## Maintainers

See main repository README for maintainer information.
