# NASA UAM Flight Stack - Usage Examples

## Overview

This document provides practical examples for common use cases of the NASA UAM flight stack. Each example includes step-by-step instructions and expected behavior.

## Table of Contents

1. [Quick Start - First Flight](#quick-start---first-flight)
2. [Basic Navigation Mission](#basic-navigation-mission)
3. [Multi-Waypoint Flight](#multi-waypoint-flight)
4. [Testing the Planner](#testing-the-planner)
5. [Monitoring System Status](#monitoring-system-status)
6. [Debugging Common Issues](#debugging-common-issues)
7. [Advanced Scenarios](#advanced-scenarios)

---

## Quick Start - First Flight

### Prerequisites

- PX4 SITL and Gazebo installed
- ROS2 Galactic workspace built
- MicroXRCE Agent available

### Step 1: Start PX4 SITL

```bash
# Terminal 1: Navigate to PX4-Autopilot directory
cd ~/PX4-Autopilot

# Start SITL with Gazebo
make px4_sitl gazebo
```

**Expected Output**:
```
INFO  [simulator] Waiting for simulator to accept connection on TCP port 4560
INFO  [init] Mixer: etc/mixers/quad_w.main.mix on /dev/pwm_output0
INFO  [mavlink] mode: Normal, data rate: 4000000 B/s on udp port 18570...
pxh>
```

### Step 2: Start MicroXRCE DDS Agent

```bash
# Terminal 2: Start DDS bridge
MicroXRCEAgent udp4 -p 8888
```

**Expected Output**:
```
Enter 'q' to exit
[1619458987.123456] info     | UDPv4AgentLinux.cpp | init | running... | port: 8888
[1619458987.456789] info     | Root.cpp           | create_client | create | client_key: 0x00000001
```

### Step 3: Launch ROS2 Flight Stack

```bash
# Terminal 3: Launch full stack with SITL parameters
cd ~/nasa_uam
source install/setup.bash
ros2 launch uam_vehicle_interface sitl_launch.py
```

**Expected Output**:
- Navigator and Planner transition to Active state
- Vehicle interface starts publishing odometry
- RViz opens showing map and TF frames

### Step 4: Perform Takeoff

```bash
# Terminal 4: Send takeoff command
ros2 action send_goal /send_navigator_command \
  uam_navigator_msgs/action/NavigatorCommand \
  "{command: 'Takeoff'}"
```

**Expected Behavior**:
1. Kill switch automatically disables
2. Vehicle arms
3. Enters offboard mode
4. Ascends to configured altitude (0.7m default)
5. Automatically transitions to Loiter mode
6. Hovers in place

**Verification**:
```bash
# Check current mode
ros2 topic echo /uam_navigator/navigator_status --once

# Should show: nav_mode: "Loiter"
```

---

## Basic Navigation Mission

### Scenario

Fly from current position to a goal waypoint at (2.0, 2.0, 1.0) meters.

### Prerequisites

- Vehicle already in Loiter mode (after takeoff)
- Obstacles configured in environment

### Step 1: Send Navigation Goal

```bash
ros2 action send_goal /send_navigator_command \
  uam_navigator_msgs/action/NavigatorCommand \
  "{command: 'navigate_to_pose_rrtx_static',
    goal: {
      header: {frame_id: 'map'},
      pose: {
        position: {x: 2.0, y: 2.0, z: 1.0},
        orientation: {w: 1.0, x: 0.0, y: 0.0, z: 0.0}
      }
    }
  }"
```

### Step 2: Monitor Progress

**Watch Navigator Status**:
```bash
# Terminal 5
ros2 topic echo /uam_navigator/navigator_status
```

**Expected States**:
1. `nav_mode: "Loiter"` → `nav_mode: "navigate_to_pose_rrtx_static"`
2. Mode stays as navigate_to_pose during flight
3. Upon reaching goal: `nav_mode: "Loiter"`

**Watch Planned Path**:
```bash
ros2 topic echo /planner_server/path
```

Shows waypoints the vehicle will follow.

**Watch Vehicle Position**:
```bash
ros2 topic echo /uam_vehicle_interface/odometry | grep "position:"
```

### Step 3: Visualize in RViz

In RViz window:
- **Red boxes**: Obstacles
- **Green line**: Planned path (from planner)
- **Blue arrow**: Vehicle position and orientation
- **Yellow spheres**: Waypoints being followed

### Expected Timeline

- **0-2 seconds**: Path planning
- **2-15 seconds**: Flight to goal
- **15-17 seconds**: Settling at goal
- **17+ seconds**: Loitering at goal

---

## Multi-Waypoint Flight

### Scenario

Visit multiple waypoints in sequence: (1,1,1) → (2,2,1) → (3,1,1)

### Approach

Send navigation commands sequentially after each goal is reached.

### Script Example (Python)

```python
#!/usr/bin/env python3
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from uam_navigator_msgs.action import NavigatorCommand
from geometry_msgs.msg import PoseStamped
import time

class WaypointMission(Node):
    def __init__(self):
        super().__init__('waypoint_mission')
        self.client = ActionClient(self, NavigatorCommand, 'send_navigator_command')

        # Define waypoints
        self.waypoints = [
            (1.0, 1.0, 1.0),
            (2.0, 2.0, 1.0),
            (3.0, 1.0, 1.0),
        ]
        self.current_waypoint = 0

    def send_goal(self, x, y, z):
        goal = NavigatorCommand.Goal()
        goal.command = 'navigate_to_pose_rrtx_static'
        goal.goal.header.frame_id = 'map'
        goal.goal.pose.position.x = x
        goal.goal.pose.position.y = y
        goal.goal.pose.position.z = z
        goal.goal.pose.orientation.w = 1.0

        self.client.wait_for_server()
        future = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)

        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected')
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        return result_future.result().result.error_code == 0

    def run_mission(self):
        for x, y, z in self.waypoints:
            self.get_logger().info(f'Flying to ({x}, {y}, {z})')
            success = self.send_goal(x, y, z)
            if success:
                self.get_logger().info('Waypoint reached!')
            else:
                self.get_logger().error('Failed to reach waypoint')
                break

def main():
    rclpy.init()
    mission = WaypointMission()
    mission.run_mission()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Usage

```bash
# Make executable
chmod +x waypoint_mission.py

# Run
python3 waypoint_mission.py
```

---

## Testing the Planner

### Test 1: Simple Path Request

```bash
ros2 action send_goal /compute_path_to_pose \
  uam_planner_msgs/action/ComputePathToPose \
  "{
    planner_id: 'rrtx_static',
    start: {
      header: {frame_id: 'map'},
      pose: {position: {x: 0.0, y: 0.0, z: 1.0}}
    },
    goal: {
      header: {frame_id: 'map'},
      pose: {position: {x: 3.0, y: 3.0, z: 1.0}}
    }
  }"
```

**Expected Result**:
```
Result:
  path:
    header:
      frame_id: map
    poses: [... 15-30 waypoints ...]
  planning_time:
    sec: 0
    nanosec: 450000000  # ~0.45 seconds
  error_code: 0  # SUCCESS
```

### Test 2: Path with Obstacles

Configure obstacles blocking direct path, verify planner finds alternate route.

**View planned path**:
```bash
ros2 topic echo /planner_server/path
```

**Check in RViz**: Green line should curve around red obstacle boxes.

### Test 3: Impossible Goal

Request path to position inside obstacle or outside bounds.

```bash
ros2 action send_goal /compute_path_to_pose \
  uam_planner_msgs/action/ComputePathToPose \
  "{
    planner_id: 'rrtx_static',
    start: {pose: {position: {x: 0.0, y: 0.0, z: 1.0}}},
    goal: {pose: {position: {x: 10.0, y: 10.0, z: 1.0}}}  # Outside bounds
  }"
```

**Expected Result**:
```
error_code: 7  # GOAL_OUTSIDE_MAP
```

---

## Monitoring System Status

### Check All Nodes

```bash
ros2 node list
```

**Expected Nodes**:
```
/navigator
/planner_server
/qlearning_controller
/static_obstacle_advertiser
/uam_vehicle_interface
/uam_visualization
/rviz2
```

### Check Lifecycle States

```bash
# Navigator
ros2 lifecycle get /navigator
# Should show: active [id=3]

# Planner
ros2 lifecycle get /planner_server
# Should show: active [id=3]
```

### Monitor Topic Rates

```bash
# Odometry (should be ~50 Hz)
ros2 topic hz /uam_vehicle_interface/odometry

# Position setpoints (should be ~20-100 Hz when navigating)
ros2 topic hz /uam_navigator/position_setpoint

# Attitude setpoints (should be ~20 Hz)
ros2 topic hz /uam_control/attitude_setpoint
```

### View TF Tree

```bash
# Generate PDF
ros2 run tf2_tools view_frames

# View
evince frames.pdf
```

**Expected Frames**:
```
map
 └─> map_ned
      └─> base_link
           └─> base_link_frd
```

---

## Debugging Common Issues

### Issue: Vehicle Won't Arm

**Symptoms**: Stays disarmed after takeoff command

**Debug Steps**:

1. Check kill switch:
```bash
ros2 topic echo /uam_vehicle_interface/vehicle_interface_status
# vehicle_kill_switch_enabled should be FALSE
```

2. Disable kill switch if needed:
```bash
ros2 topic pub --once /uam_vehicle_interface/vehicle_interface_commands \
  uam_vehicle_interface_msgs/msg/VehicleInterfaceCommand "{command: 1}"
```

3. Check PX4 pre-flight checks:
```bash
# In PX4 SITL terminal
pxh> commander status
```

### Issue: Path Planning Fails

**Symptoms**: `NO_VALID_PATH` error

**Debug Steps**:

1. Check obstacle configuration:
```bash
ros2 topic echo /uam_mapping/obstacles
```

2. Verify goal is reachable:
```bash
# Check bounds
ros2 param get /planner_server rrtx_static.bounds_low
ros2 param get /planner_server rrtx_static.bounds_high
```

3. Increase solve time:
```bash
ros2 param set /planner_server rrtx_static.solve_time 2.0
```

4. Visualize in RViz to see obstacles and goal position

### Issue: Jerky Flight

**Symptoms**: Vehicle oscillates around waypoints

**Debug Steps**:

1. Check waypoint tolerance:
```bash
ros2 param get /navigator navigate_to_pose.waypoint_position_tolerance
# Try increasing to 0.2
```

2. Reduce control gains (if using Q-learning):
```bash
ros2 param get /uam_control rrtx_static.actor_convergence_rate
# Try reducing to 0.1
```

---

## Advanced Scenarios

### Scenario 1: Custom Obstacle Configuration

**Create custom environment** in `params/my_params.yaml`:

```yaml
static_obstacle_advertiser:
  ros__parameters:
    obstacle_ids: ['wall_1', 'wall_2', 'tower_1']
    obstacles_x: [2.0, 2.0, 3.5]
    obstacles_y: [1.0, 3.0, 2.0]
```

**Launch with custom params**:
```bash
ros2 launch uam_vehicle_interface sitl_launch.py params_file:=my_params.yaml
```

### Scenario 2: Programmatic Mission Control (C++)

```cpp
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include "uam_navigator_msgs/action/navigator_command.hpp"

class MissionController : public rclcpp::Node {
public:
    using NavigatorCommand = uam_navigator_msgs::action::NavigatorCommand;
    using GoalHandle = rclcpp_action::ClientGoalHandle<NavigatorCommand>;

    MissionController() : Node("mission_controller") {
        client_ = rclcpp_action::create_client<NavigatorCommand>(
            this, "send_navigator_command");
    }

    void run_mission() {
        // Takeoff
        send_command("Takeoff");
        wait_for_completion();

        // Navigate to waypoint
        auto goal = NavigatorCommand::Goal();
        goal.command = "navigate_to_pose_rrtx_static";
        goal.goal.header.frame_id = "map";
        goal.goal.pose.position.x = 2.5;
        goal.goal.pose.position.y = 2.5;
        goal.goal.pose.position.z = 1.0;

        send_goal(goal);
        wait_for_completion();

        // Land
        send_command("Land");
    }

private:
    rclcpp_action::Client<NavigatorCommand>::SharedPtr client_;

    void send_command(const std::string& cmd) {
        auto goal = NavigatorCommand::Goal();
        goal.command = cmd;
        client_->async_send_goal(goal);
    }

    void send_goal(const NavigatorCommand::Goal& goal) {
        auto options = rclcpp_action::Client<NavigatorCommand>::SendGoalOptions();
        options.result_callback = [this](const GoalHandle::WrappedResult& result) {
            RCLCPP_INFO(get_logger(), "Mission step completed");
        };
        client_->async_send_goal(goal, options);
    }

    void wait_for_completion() {
        rclcpp::sleep_for(std::chrono::seconds(5));  // Simplified
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto controller = std::make_shared<MissionController>();
    controller->run_mission();
    rclcpp::spin(controller);
    rclcpp::shutdown();
    return 0;
}
```

### Scenario 3: Record and Replay Flight Data

**Record bag file**:
```bash
ros2 bag record -o my_flight \
  /uam_vehicle_interface/odometry \
  /uam_navigator/position_setpoint \
  /uam_navigator/navigator_status \
  /planner_server/path
```

**Replay**:
```bash
ros2 bag play my_flight
```

**Analyze**:
```bash
# Check recording
ros2 bag info my_flight

# Plot odometry
python3 plot_odometry.py my_flight
```

---

## Safety Reminders

1. **Always test in SITL first** before hardware deployment
2. **Keep kill switch accessible** via GUI or command
3. **Monitor battery voltage** in hardware flights
4. **Have manual RC override** ready for hardware
5. **Start with conservative parameters** (low speeds, high tolerances)
6. **Geofence your flight area** in PX4 parameters

---

## Next Steps

- **Learn More**: Read individual package READMEs for detailed APIs
- **Customize**: Modify parameters for your specific vehicle/environment
- **Extend**: Add new navigator modes or planner algorithms
- **Contribute**: Report issues and improvements on GitHub

---

## Quick Reference Commands

```bash
# Launch full stack
ros2 launch uam_vehicle_interface sitl_launch.py

# Takeoff
ros2 action send_goal /send_navigator_command \
  uam_navigator_msgs/action/NavigatorCommand "{command: 'Takeoff'}"

# Navigate to (2, 2, 1)
ros2 action send_goal /send_navigator_command \
  uam_navigator_msgs/action/NavigatorCommand \
  "{command: 'navigate_to_pose_rrtx_static', \
    goal: {header: {frame_id: 'map'}, \
           pose: {position: {x: 2.0, y: 2.0, z: 1.0}}}}"

# Loiter (hover in place)
ros2 action send_goal /send_navigator_command \
  uam_navigator_msgs/action/NavigatorCommand "{command: 'Loiter'}"

# Check status
ros2 topic echo /uam_navigator/navigator_status --once

# Disable kill switch
ros2 topic pub --once /uam_vehicle_interface/vehicle_interface_commands \
  uam_vehicle_interface_msgs/msg/VehicleInterfaceCommand "{command: 1}"
```

---

For more information, see:
- Architecture: `docs/ARCHITECTURE.md`
- Package Details: `<package_name>/README.md`
- Installation: `docs/installation.md`
