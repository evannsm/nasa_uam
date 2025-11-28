# uam_mapping - Obstacle Management

## Overview

The `uam_mapping` package provides obstacle information for path planning and collision avoidance. It publishes static obstacle positions and shapes that the planner uses to compute collision-free paths.

**Current Implementation**: Static obstacles only (positions loaded from configuration).
**Future**: Dynamic obstacle detection from sensors (LiDAR, cameras, etc.).

## Package Architecture

```
uam_mapping/
├── uam_mapping/                      # Obstacle advertiser node
│   ├── src/obstacle_advertiser.cpp  #  Main implementation
│   └── include/uam_mapping/
│       └── obstacle_advertiser.hpp
└── uam_mapping_msgs/                 # Message definitions
    └── msg/
        ├── Obstacle.msg
        └── ObstacleArray.msg
```

---

## Core Component: Obstacle Advertiser

### Purpose

Publishes obstacle information at regular intervals for path planning.

**Location**: `src/obstacle_advertiser.cpp`

### Responsibilities

1. **Load** obstacle positions from parameters
2. **Create** obstacle messages with shapes and poses
3. **Publish** obstacle array at 10 Hz
4. **Maintain** obstacle map for updates

---

## Communication Interfaces

### Publications

| Topic | Type | Subscriber | Purpose | Rate |
|-------|------|------------|---------|------|
| `uam_mapping/obstacles` | `uam_mapping_msgs/ObstacleArray` | Planner, Visualization | Obstacle positions and shapes | 10 Hz |

### Message Definitions

**Obstacle.msg**:
```
uint32 obstacle_id                     # Unique obstacle identifier
geometry_msgs/Pose pose                # Position and orientation
shape_msgs/SolidPrimitive obstacle     # Shape (BOX, SPHERE, CYLINDER)
```

**ObstacleArray.msg**:
```
std_msgs/Header header
Obstacle[] obstacles                   # Array of obstacles
string[] obstacle_ids                  # List of IDs for tracking
```

---

## Configuration

### Parameters

Obstacles are defined in `params/sitl_params.yaml`:

```yaml
static_obstacle_advertiser:
  ros__parameters:
    obstacle_ids: ['1', '2', '3', '4', '5']  # Obstacle IDs
    obstacles_x: [1.524, 2.134, 2.743, 3.353, 3.962]  # X positions (meters)
    obstacles_y: [0.305, 0.305, 0.305, 0.305, 0.914]  # Y positions (meters)
```

### Parameter Format

**`obstacle_ids`**: List of string identifiers
- Must match length of X and Y arrays
- Used for tracking obstacle updates

**`obstacles_x`**: X coordinates in map frame (ENU)
- Units: meters
- Must match length of IDs and Y arrays

**`obstacles_y`**: Y coordinates in map frame (ENU)
- Units: meters
- Z coordinate is auto-calculated from obstacle height

### Default Obstacle Properties

**Shape**: BOX (hardcoded in `obstacle_advertiser.cpp:47`)

**Dimensions** (meters):
```cpp
dimensions = {0.4191, 0.4191, 0.9779}  // [width, depth, height]
```

**Z Position**:
```cpp
z = height / 2.0  // Center at half-height above ground
```

**Location**: `obstacle_advertiser.cpp:49-52`

---

## How It Works

### Initialization

1. **Load Parameters**:
```cpp
get_parameter("obstacle_ids", obstacle_ids_);
get_parameter("obstacles_x", obstacles_x_);
get_parameter("obstacles_y", obstacles_y_);
```

2. **Validate**: Check all arrays have same length

3. **Build Map**:
```cpp
for (i = 0; i < obstacle_ids.size(); i++) {
    obstacles_map_[id] = {x[i], y[i]};
}
```

4. **Start Timer**: Publish at 10 Hz

**Location**: Constructor `obstacle_advertiser.cpp:6-36`

### Publishing Loop

Every 100ms:

1. **Create message**:
```cpp
ObstacleArray msg;
msg.header.frame_id = "map";
msg.header.stamp = now();
```

2. **For each obstacle in map**:
```cpp
Obstacle obs;
obs.obstacle_id = id;
obs.obstacle.type = SolidPrimitive::BOX;
obs.obstacle.dimensions = {0.4191, 0.4191, 0.9779};
obs.pose.position.x = x;
obs.pose.position.y = y;
obs.pose.position.z = height / 2;
msg.obstacles.push_back(obs);
```

3. **Publish**:
```cpp
obstacle_array_pub_->publish(msg);
```

**Location**: `publishObstacles()` at line 38

---

## Usage

### Basic Usage

```bash
# Included in sitl_launch.py
ros2 launch uam_vehicle_interface sitl_launch.py
```

Or standalone:

```bash
ros2 run uam_mapping obstacle_advertiser --ros-args \
  --params-file params/sitl_params.yaml
```

### Monitor Obstacles

```bash
# View obstacle messages
ros2 topic echo /uam_mapping/obstacles

# Check publication rate
ros2 topic hz /uam_mapping/obstacles
# Should show ~10 Hz
```

### Visualize in RViz

```bash
# Launch RViz
rviz2

# Add -> By Topic -> /uam_visualization/obstacle_markers -> MarkerArray
```

---

## Customizing Obstacles

### Adding Obstacles

**In `params/sitl_params.yaml`**:

```yaml
static_obstacle_advertiser:
  ros__parameters:
    obstacle_ids: ['1', '2', '3', '4', '5', '6']  # Add '6'
    obstacles_x: [1.524, 2.134, 2.743, 3.353, 3.962, 1.0]  # Add X
    obstacles_y: [0.305, 0.305, 0.305, 0.305, 0.914, 2.0]  # Add Y
```

**Important**: All three arrays must have the same length!

### Changing Obstacle Size

Currently hardcoded. To change:

**Edit `obstacle_advertiser.cpp:49`**:
```cpp
obstacle_msg.obstacle.dimensions = {0.6, 0.6, 1.5};  // Larger obstacles
```

Then rebuild:
```bash
colcon build --packages-select uam_mapping
```

### Changing Obstacle Shape

Currently only BOX supported. To add SPHERE:

**Edit `obstacle_advertiser.cpp:47-52`**:
```cpp
// Instead of:
obstacle_msg.obstacle.type = shape_msgs::msg::SolidPrimitive::BOX;
obstacle_msg.obstacle.dimensions = {0.4191, 0.4191, 0.9779};

// Use:
obstacle_msg.obstacle.type = shape_msgs::msg::SolidPrimitive::SPHERE;
obstacle_msg.obstacle.dimensions = {0.3};  // Radius only
```

**Note**: Planner currently only handles BOX collision checking.

---

## Coordinate Frame

### Frame Convention

**Frame**: `map` (ENU - East-North-Up)
- X: East (positive to the east)
- Y: North (positive to the north)
- Z: Up (positive upward)

### Obstacle Positioning

- **(X, Y)**: Loaded from parameters
- **Z**: Auto-calculated as `height / 2` (center of obstacle)
- **Orientation**: Identity (no rotation)

---

## Integration with Other Packages

### Planner Integration

```
ObstacleAdvertiser ────[ObstacleArray]────> PlannerServer
                                                   │
                                                   └─> RRT* Collision Checking
```

Planner subscribes to obstacles and uses them for:
- State validity checking (is position collision-free?)
- Edge validation (is path segment collision-free?)

**See**: `uam_planner/README.md` for collision checking details

### Visualization Integration

```
ObstacleAdvertiser ────[ObstacleArray]────> Visualization Node
                                                   │
                                                   └─> RViz Markers
```

Visualization converts obstacles to colored 3D markers.

**See**: `uam_visualization/README.md`

---

## Limitations

### Current Limitations

1. **Static Only**: No support for moving obstacles
2. **BOX Only**: Other shapes not fully implemented
3. **2D Position**: Z position auto-calculated (can't set explicitly)
4. **Hardcoded Size**: All obstacles same dimensions
5. **No Removal**: Can't remove obstacles at runtime

### Future Enhancements

1. **Dynamic Obstacles**: Support for moving objects
2. **Sensor Integration**: LiDAR/camera-based detection
3. **Flexible Shapes**: Configurable shape per obstacle
4. **Runtime Updates**: Add/remove/modify via service
5. **Occupancy Grid**: 3D voxel grid representation

---

## Troubleshooting

### Issue 1: Obstacles Not Appearing in Planner

**Symptom**: Planner ignores obstacles

**Causes**:
1. Topic mismatch
2. Obstacle advertiser not running

**Solutions**:
```bash
# Check if publishing
ros2 topic list | grep obstacles
# Should see: /uam_mapping/obstacles

# Check message content
ros2 topic echo /uam_mapping/obstacles

# Verify planner is subscribing
ros2 node info /planner_server
# Should list /uam_mapping/obstacles under subscriptions
```

### Issue 2: Array Length Mismatch Error

**Symptom**: "Invalid size of obstacle parameter vectors" error

**Cause**: `obstacle_ids`, `obstacles_x`, `obstacles_y` have different lengths

**Solution**:
```yaml
# Ensure all arrays have exactly N elements
obstacle_ids: ['1', '2', '3']    # 3 elements
obstacles_x: [1.0, 2.0, 3.0]     # 3 elements
obstacles_y: [0.5, 0.5, 0.5]     # 3 elements
```

### Issue 3: Obstacles in Wrong Location

**Symptom**: Obstacles appear displaced in RViz

**Cause**: Coordinate frame mismatch or incorrect parameters

**Solution**:
```bash
# Check TF frames
ros2 run tf2_tools view_frames

# Verify obstacle positions
ros2 topic echo /uam_mapping/obstacles | grep position
```

---

## Performance

### Resource Usage

- **CPU**: <1% (trivial computation)
- **Memory**: ~5 MB (small obstacle map)
- **Network**: ~1 KB/s at 10 Hz

### Scalability

Can handle hundreds of obstacles efficiently:
- **Current**: ~20 obstacles
- **Tested**: Up to 100 obstacles
- **Limit**: Thousands (limited by planner, not advertiser)

---

## Advanced Usage

### Programmatic Obstacle Updates (Future)

**Planned Service Interface**:
```cpp
// Add obstacle
AddObstacle::Request req;
req.obstacle_id = "dynamic_1";
req.pose.position.x = 2.0;
req.pose.position.y = 3.0;
req.shape.type = SolidPrimitive::SPHERE;
req.shape.dimensions = {0.5};  // radius

add_obstacle_client_->call(req);
```

**Note**: Not currently implemented.

---

## References

- ROS2 Message Types: [shape_msgs](http://docs.ros.org/en/api/shape_msgs/html/index-msg.html)
- Coordinate Frames: `docs/ARCHITECTURE.md`

---

## License

Apache License 2.0

## Maintainers

See main repository README for maintainer information.
