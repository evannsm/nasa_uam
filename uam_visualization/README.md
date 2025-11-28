# uam_visualization - RViz Visualization

## Overview

The `uam_visualization` package provides real-time 3D visualization of the flight stack in RViz. It converts obstacle information and path data into RViz markers for intuitive monitoring and debugging.

## Package Architecture

```
uam_visualization/
├── uam_visualization/              # Visualization node
│   ├── src/visualization.cpp      # Main implementation
│   └── include/uam_visualization/
│       └── visualization.hpp
└── uam_visualization_msgs/         # (Currently none)
```

---

## Core Component: Visualization Node

### Purpose

Convert flight stack data into RViz-compatible marker messages.

**Location**: `src/visualization.cpp`

### Responsibilities

1. **Subscribe** to obstacle arrays
2. **Convert** shapes to RViz markers
3. **Publish** marker arrays for display
4. **Maintain** marker IDs for updates

---

## Communication Interfaces

### Subscriptions

| Topic | Type | Source | Purpose | Rate |
|-------|------|--------|---------|------|
| `uam_mapping/obstacles` | `uam_mapping_msgs/ObstacleArray` | Mapping Node | Obstacle data to visualize | 10 Hz |

### Publications

| Topic | Type | Subscriber | Purpose | Rate |
|-------|------|------------|---------|------|
| `uam_visualization/obstacle_markers` | `visualization_msgs/MarkerArray` | RViz | 3D obstacle visualization | 10 Hz |
| `uam_visualization/explored_path` | `nav_msgs/Path` | RViz | Historical path (future) | N/A |
| `uam_visualization/frontier_path` | `nav_msgs/Path` | RViz | Planned path (future) | N/A |

---

## Supported Visualizations

### 1. Obstacle Markers

**Supported Shapes**:
- **BOX**: Rendered as CUBE marker
- **SPHERE**: Rendered as SPHERE marker
- **CYLINDER**: Rendered as CYLINDER marker

**Marker Properties**:
- **Color**: Red (R=1.0, G=0.0, B=0.0, A=1.0)
- **Namespace**: "obstacles"
- **ID**: Matches obstacle ID from mapping node
- **Frame**: "map" (ENU)

**Location**: `publishObstacleMarkers()` at line 22

### 2. Path Markers (Planned)

**Topics created but not yet implemented**:
- `explored_path`: Historical trajectory
- `frontier_path`: Planned future path

---

## How It Works

### Obstacle Marker Conversion

For each obstacle message:

1. **Create Marker**:
```cpp
visualization_msgs::msg::Marker marker;
marker.header.frame_id = "map";
marker.header.stamp = now();
marker.id = obstacle.obstacle_id;
marker.ns = "obstacles";
marker.action = Marker::ADD;
```

2. **Set Color** (Red):
```cpp
marker.color.r = 1.0;
marker.color.g = 0.0;
marker.color.b = 0.0;
marker.color.a = 1.0;
```

3. **Convert Shape**:
```cpp
switch (obstacle.type) {
    case BOX:
        marker.type = Marker::CUBE;
        marker.scale.x = dimensions[BOX_X];
        marker.scale.y = dimensions[BOX_Y];
        marker.scale.z = dimensions[BOX_Z];
        break;

    case SPHERE:
        marker.type = Marker::SPHERE;
        marker.scale.x = 2 * radius;
        marker.scale.y = 2 * radius;
        marker.scale.z = 2 * radius;
        break;

    case CYLINDER:
        marker.type = Marker::CYLINDER;
        marker.scale.x = 2 * radius;
        marker.scale.y = 2 * radius;
        marker.scale.z = height;
        break;
}
```

4. **Set Pose**:
```cpp
marker.pose.position = obstacle.pose.position;
```

5. **Add to Array**:
```cpp
marker_array.markers.push_back(marker);
```

6. **Publish**:
```cpp
obstacle_marker_pub_->publish(marker_array);
```

**Location**: Lines 22-62

---

## Usage

### Launch with Flight Stack

```bash
# Included in sitl_launch.py
ros2 launch uam_vehicle_interface sitl_launch.py
```

Or standalone:

```bash
ros2 run uam_visualization visualization
```

### View in RViz

1. **Launch RViz**:
```bash
rviz2
```

2. **Set Fixed Frame**:
   - Global Options → Fixed Frame → `map`

3. **Add Obstacle Markers**:
   - Add → By Topic → `/uam_visualization/obstacle_markers` → MarkerArray

4. **Configure Display** (optional):
   - Adjust transparency: Alpha → 0.5 (semi-transparent)
   - Change color: Override Color → Custom

### Pre-configured RViz

```bash
# Use included config
rviz2 -d uam_vehicle_interface/rviz/config_file.rviz
```

Includes:
- Obstacle markers
- TF frames
- Grid
- Axes

---

## Customization

### Change Obstacle Color

**In `visualization.cpp:32-35`**:

```cpp
// Green obstacles
obstacle_marker.color.r = 0.0;
obstacle_marker.color.g = 1.0;
obstacle_marker.color.b = 0.0;
obstacle_marker.color.a = 1.0;
```

### Add Semi-Transparency

```cpp
// 50% transparent
obstacle_marker.color.a = 0.5;
```

### Change Marker Lifetime

**Add to marker creation**:

```cpp
marker.lifetime = rclcpp::Duration::from_seconds(1.0);  // Fade after 1s
```

**Default**: `lifetime = 0` (permanent until removed)

---

## Marker Management

### Marker IDs

- **ID**: Matches `obstacle_id` from mapping node
- **Namespace**: "obstacles"
- **Unique Key**: (namespace, id) pair

### Marker Updates

When obstacle moves or changes:
1. Receive updated `ObstacleArray`
2. Publish marker with **same ID**
3. RViz updates existing marker (doesn't create duplicate)

### Marker Removal

To remove an obstacle:

```cpp
marker.action = visualization_msgs::msg::Marker::DELETE;
marker.id = obstacle_id_to_remove;
obstacle_marker_pub_->publish(marker);
```

**Note**: Currently not implemented (all obstacles permanent).

---

## Future Enhancements

### Planned Visualizations

1. **Path Trails**:
```cpp
// Publish to explored_path
nav_msgs::msg::Path historical_path;
// Add poses from vehicle odometry history
```

2. **Planned Trajectory**:
```cpp
// Publish to frontier_path
nav_msgs::msg::Path planned_path;
// Add waypoints from planner
```

3. **Vehicle Model**: 3D mesh of quadcopter

4. **Velocity Vectors**: Arrows showing motion

5. **Sensor FOV**: Camera frustums, LiDAR range

### Customization Options

**Via Parameters**:
```yaml
uam_visualization:
  ros__parameters:
    obstacle_color: [1.0, 0.0, 0.0]  # [R, G, B]
    obstacle_alpha: 0.8
    show_path_history: true
    path_history_length: 100  # waypoints
```

---

## Integration with RViz

### RViz Marker Types

**Supported in this package**:
- `CUBE`: Box obstacles
- `SPHERE`: Spherical obstacles
- `CYLINDER`: Cylindrical obstacles

**Available in RViz** (future use):
- `ARROW`: Velocity vectors
- `LINE_STRIP`: Paths
- `MESH_RESOURCE`: Custom 3D models
- `TEXT_VIEW_FACING`: Labels

### Display Configuration

**RViz Display Settings**:
- **Topic**: `/uam_visualization/obstacle_markers`
- **Queue Size**: 10
- **Color**: Can override per-marker color
- **Transparency**: Global alpha multiplier

---

## Troubleshooting

### Issue 1: Markers Not Showing in RViz

**Symptom**: No obstacles visible

**Causes**:
1. Wrong fixed frame
2. Topic not subscribed
3. Markers outside view

**Solutions**:
```bash
# Check fixed frame
# In RViz: Global Options → Fixed Frame = "map"

# Verify topic publishing
ros2 topic echo /uam_visualization/obstacle_markers

# Check TF tree
ros2 run tf2_tools view_frames

# Reset view
# In RViz: Views → Reset
```

### Issue 2: Markers Flickering

**Symptom**: Obstacles appear and disappear

**Cause**: Marker lifetime expiring

**Solution**:
- Ensure `marker.lifetime = rclcpp::Duration(0)` (permanent)
- Or increase publish rate to refresh faster

### Issue 3: Wrong Obstacle Positions

**Symptom**: Markers displaced from expected locations

**Cause**: Frame mismatch or coordinate convention

**Solution**:
```bash
# Verify frame IDs match
ros2 topic echo /uam_mapping/obstacles | grep frame_id
ros2 topic echo /uam_visualization/obstacle_markers | grep frame_id
# Both should be "map"

# Check TF transforms
ros2 run tf2_ros tf2_echo map base_link
```

---

## Performance

### Resource Usage

- **CPU**: <1% (simple message conversion)
- **Memory**: ~10 MB
- **Network**: ~2-5 KB/s (depends on obstacle count)

### RViz Performance

- **100 obstacles**: No noticeable lag
- **1000+ obstacles**: May slow RViz rendering
- **Recommendation**: Keep obstacles < 500 for smooth visualization

---

## Advanced Usage

### Custom Marker Colors Per Obstacle

**Modify `publishObstacleMarkers()`**:

```cpp
// Color based on obstacle ID
if (obstacle.obstacle_id % 2 == 0) {
    // Even IDs: Red
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
} else {
    // Odd IDs: Blue
    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0;
}
```

### Add Text Labels

```cpp
// Create text marker for each obstacle
visualization_msgs::msg::Marker text_marker;
text_marker.type = Marker::TEXT_VIEW_FACING;
text_marker.text = "Obstacle " + std::to_string(obstacle.obstacle_id);
text_marker.pose = obstacle.pose;
text_marker.pose.position.z += 0.5;  // Above obstacle
text_marker.scale.z = 0.2;  // Text height
text_marker.color.r = 1.0;
text_marker.color.g = 1.0;
text_marker.color.b = 1.0;
marker_array.markers.push_back(text_marker);
```

### Visualize Paths

**Add path visualization**:

```cpp
// In header: create subscriber
path_sub_ = create_subscription<nav_msgs::msg::Path>(
    "/planner_server/path", 10,
    std::bind(&Visualization::pathCallback, this, std::placeholders::_1));

// Callback: republish directly (already in correct format)
void Visualization::pathCallback(const nav_msgs::msg::Path::SharedPtr msg) {
    frontier_path_pub_->publish(*msg);
}
```

**In RViz**:
- Add → By Topic → `/uam_visualization/frontier_path` → Path
- Set color to green for planned path

---

## References

- [RViz Markers](http://wiki.ros.org/rviz/DisplayTypes/Marker)
- [visualization_msgs](http://docs.ros.org/en/api/visualization_msgs/html/index-msg.html)

---

## License

Apache License 2.0

## Maintainers

See main repository README for maintainer information.
