# uam_vehicle_interface - PX4 Vehicle Interface

## Overview

The `uam_vehicle_interface` package provides the low-level interface between the ROS2 flight stack and the PX4 autopilot firmware. It translates high-level control commands (attitude, position, velocity) into PX4-specific messages and publishes vehicle state information (odometry, status) back to ROS2.

This package acts as the **critical bridge** between your autonomous flight stack and the actual flight controller, supporting both Software-in-the-Loop (SITL) simulation and real hardware deployment.

## Package Architecture

### Sub-Packages

The `uam_vehicle_interface` meta-package contains three sub-packages:

1. **uam_vehicle_interface** - Core PX4 interface node
2. **uam_vehicle_interface_msgs** - Custom ROS2 message definitions
3. **uam_vehicle_interface_gui** - Qt-based ground control station GUI

```
uam_vehicle_interface/
├── uam_vehicle_interface/        # Core interface
│   ├── src/px4_interface.cpp
│   ├── include/uam_vehicle_interface/px4_interface.hpp
│   ├── launch/                   # Launch files
│   └── params/                   # Parameter files
├── uam_vehicle_interface_msgs/   # Message definitions
│   └── msg/
│       ├── VehicleInterfaceCommand.msg
│       └── VehicleInterfaceStatus.msg
└── uam_vehicle_interface_gui/    # Ground control GUI
    └── src/vehicle_interface_gui.cpp
```

---

## Core Component: Px4Interface Node

### Purpose

The `Px4Interface` class (`px4_interface.hpp` / `px4_interface.cpp`) is the main interface node that:

1. **Receives** attitude/position/velocity commands from the control stack
2. **Translates** commands between ROS2 (ENU) and PX4 (FRD) coordinate frames
3. **Manages** PX4 offboard mode activation and safety features
4. **Publishes** vehicle state (odometry, battery, status) to ROS2
5. **Handles** coordinate frame transformations (TF2)
6. **Provides** kill switch and arming/disarming functionality

### Data Flow

```
ROS2 Flight Stack                PX4Interface                 PX4 Autopilot
─────────────────                ────────────                 ─────────────

AttitudeSetpoint  ──────────────►│                          │
   (from Control)                │  Frame Transform         │  VehicleAttitudeSetpoint
                                 │  (ENU → FRD)             ├──────────────►
                                 │                          │
                                 │  Offboard Mode Mgmt      │  OffboardControlMode
                                 │  Thrust Scaling          ├──────────────►
                                 │                          │
                                 │                          │  VehicleCommand
                                 │  Arm/Disarm Commands     ├──────────────►
                                 │                          │
                                 │                          │
                                 │  Frame Transform         │  VehicleOdometry
nav_msgs/Odometry ◄──────────────┤  (FRD → ENU)             │◄──────────────
                                 │                          │
VehicleInterfaceStatus ◄─────────┤  Status Monitoring       │  VehicleStatus
                                 │                          │◄──────────────
                                 │                          │
                                 │                          │  BatteryStatus
                                 │  Battery Compensation    │◄──────────────
```

---

## Communication Interfaces

### Subscriptions (Inputs to Interface)

| Topic | Type | Source | Purpose | Rate |
|-------|------|--------|---------|------|
| `/uam_control/attitude_setpoint` | `uam_control_msgs/AttitudeSetpoint` | Control Node | Desired attitude and thrust | 20 Hz |
| `/uam_vehicle_interface/vehicle_interface_commands` | `uam_vehicle_interface_msgs/VehicleInterfaceCommand` | GUI/User | Kill switch enable/disable | Event |
| `/fmu/out/vehicle_odometry` | `px4_msgs/VehicleOdometry` | PX4 | Vehicle pose and velocity | 50+ Hz |
| `/fmu/out/battery_status` | `px4_msgs/BatteryStatus` | PX4 | Battery voltage and current | 1 Hz |
| `/fmu/out/vehicle_status` | `px4_msgs/VehicleStatus` | PX4 | Arming state, nav mode | 2 Hz |
| `/fmu/out/rc_channels` | `px4_msgs/RcChannels` | PX4 | RC transmitter inputs | 50 Hz |

### Publications (Outputs from Interface)

| Topic | Type | Subscriber | Purpose | Rate |
|-------|------|------------|---------|------|
| `/fmu/in/vehicle_attitude_setpoint` | `px4_msgs/VehicleAttitudeSetpoint` | PX4 | Desired attitude (quaternion + thrust) | 20 Hz |
| `/fmu/in/offboard_control_mode` | `px4_msgs/OffboardControlMode` | PX4 | Which control modes are active | 20 Hz |
| `/fmu/in/vehicle_command` | `px4_msgs/VehicleCommand` | PX4 | Arm, disarm, mode change commands | Event |
| `/uam_vehicle_interface/odometry` | `nav_msgs/Odometry` | Navigator, Control | Vehicle state in ROS2 format (ENU) | 50+ Hz |
| `/uam_vehicle_interface/vehicle_interface_status` | `uam_vehicle_interface_msgs/VehicleInterfaceStatus` | GUI | Kill switch status | 10 Hz |

### TF2 Frames Published

| Frame | Parent Frame | Type | Purpose |
|-------|--------------|------|---------|
| `map_ned` | `map` | Static | Rotation from ENU (map) to NED |
| `base_link_frd` | `base_link` | Static | Rotation from ENU body to FRD body |
| `base_link` | `map` | Dynamic | Vehicle pose in ENU coordinates |

---

## Coordinate Frame Transformations

### Frame Conventions

The interface handles critical coordinate frame transformations between ROS2 and PX4:

**ROS2 Convention**: ENU (East-North-Up)
- X: East
- Y: North
- Z: Up

**PX4 Convention**: NED (North-East-Down) / FRD (Forward-Right-Down)
- X: North/Forward
- Y: East/Right
- Z: Down

### Transform Chain

```
map (ENU) ──[static]──> map_ned (NED)
    │
    └──[dynamic]──> base_link (ENU) ──[static]──> base_link_frd (FRD)
```

### Implementation Details

Transformations use the `px4_ros_com` library:

```cpp
// Position: NED → ENU
Eigen::Vector3d pose_enu = px4_ros_com::frame_transforms::ned_to_enu_local_frame(pose_ned);

// Orientation: NED → ENU → Baselink
Eigen::Quaterniond q_enu = px4_ros_com::frame_transforms::ned_to_enu_orientation(q_ned);
Eigen::Quaterniond q_baselink = px4_ros_com::frame_transforms::aircraft_to_baselink_orientation(q_enu);
```

**Location**: See `vehicleOdometryCallback()` in `px4_interface.cpp:238`

---

## Offboard Mode Management

### What is Offboard Mode?

PX4's **offboard mode** allows external computers (like this ROS2 stack) to control the vehicle. The interface automatically manages transitions to/from offboard mode.

### Activation Sequence

1. **Warm-up Period**: Send at least 10 attitude setpoints before switching modes
   - Counter increments with each setpoint (`offboard_counter_`)
   - Prevents PX4 from rejecting offboard mode due to missing commands

2. **Mode Switch**: After 10 setpoints, send mode change command
   ```cpp
   publishVehicleCommand(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
   // Param1=1: Custom mode
   // Param2=6: Offboard mode
   ```

3. **Arming**: Automatically arm if not already armed
   ```cpp
   arm(); // Sends VEHICLE_CMD_COMPONENT_ARM_DISARM
   ```

4. **Continuous Control**: Keep sending setpoints at ≥20Hz or PX4 will failsafe

**Location**: See `publishAttitudeSetpoint()` in `px4_interface.cpp:208`

### Kill Switch Safety

The kill switch provides emergency stop functionality:

- **Enabled** (default): Offboard mode activation blocked, setpoints ignored
- **Disabled**: Normal offboard operation allowed

Control via command message:
```cpp
VehicleInterfaceCommand cmd;
cmd.command = VehicleInterfaceCommand::VEHICLE_KILL_SWITCH_DISABLE;
// Publish to /uam_vehicle_interface/vehicle_interface_commands
```

**Location**: Command handling in `px4_interface.cpp:71-85`

---

## Thrust Calculation and Motor Modeling

### Purpose

PX4 expects normalized thrust values ([0, 1]), but the control stack works with force in Newtons. The interface converts between these using motor models.

### SITL Mode (Simulation)

Uses motor velocity model:

```cpp
motor_speed = sqrt(collective_thrust / (4.0 * motor_constant))
thrust_command = (motor_speed - motor_velocity_armed) / motor_input_scaling
```

**Parameters**:
- `motor_constant`: Thrust coefficient (default: 5.84e-6)
- `motor_velocity_armed`: Idle motor speed (default: 100 rad/s)
- `motor_input_scaling`: Conversion factor (default: 1000.0)

### Vehicle Mode (Real Hardware)

Uses empirical motor characteristic with battery compensation:

```cpp
rel_thrust = ((thrust_per_motor - motor_thrust_armed) / (motor_thrust_max - motor_thrust_armed))
normalized = 0.54358075 * rel_thrust
           + 0.25020242 * sqrt(3.6484 * rel_thrust + 0.00772641)
           - 0.021992793

// Battery voltage compensation (if voltage > 14V)
normalized *= (1.0 - 0.0779 * (battery_voltage - 16.0))
```

**Parameters**:
- `motor_thrust_max`: Maximum thrust per motor (default: 12.5 N)
- `motor_thrust_armed`: Idle thrust (default: 0.027 N)
- `vehicle_mass`: Total vehicle mass for gravity compensation

**Location**: `computeRelativeThrust()` in `px4_interface.cpp:323`

---

## Configuration

### Parameters

Defined in `params/sitl_params.yaml` or `params/vehicle_params.yaml`:

```yaml
uam_vehicle_interface:
  ros__parameters:
    environment: "SITL"              # "SITL" or "Vehicle"
    vehicle_mass: 1.56               # kg
    motor_thrust_max: 12.5           # N (per motor)
    motor_thrust_armed: 0.026975     # N (idle thrust)
    motor_constant: 0.00000584       # Thrust coefficient
    motor_input_scaling: 1000.0      # Motor command scaling
```

### Environment-Specific Configurations

**SITL (Software-in-the-Loop)**:
- Connects to PX4 SITL via DDS (MicroXRCE)
- Uses simplified motor model
- Typically lighter vehicle mass

**Vehicle (Real Hardware)**:
- Connects to real PX4 flight controller
- Uses empirical motor model with battery compensation
- Actual measured vehicle mass

---

## Launch Files

### 1. SITL Launch (`sitl_launch.py`)

Launches complete flight stack with simulation:

```bash
ros2 launch uam_vehicle_interface sitl_launch.py
```

**Components Launched**:
- PX4 Interface
- Navigator (lifecycle node)
- Planner Server (lifecycle node)
- Q-learning Controller
- Obstacle Mapping
- Visualization
- RViz

**Use Case**: Development, testing, algorithm validation

### 2. Vehicle Launch (`vehicle_launch.py`)

For deployment on onboard computer with real PX4:

```bash
ros2 launch uam_vehicle_interface vehicle_launch.py
```

**Components**: Same as SITL but uses `vehicle_params.yaml`

**Use Case**: Real flight operations

### 3. GCS Launch (`gcs_launch.py`)

Ground control station only (GUI + visualization):

```bash
ros2 launch uam_vehicle_interface gcs_launch.py
```

**Components**:
- Vehicle Interface GUI
- RViz

**Use Case**: Remote monitoring and control

---

## Usage Examples

### Example 1: Basic Startup with SITL

```bash
# Terminal 1: Start PX4 SITL (in PX4-Autopilot directory)
make px4_sitl gazebo

# Terminal 2: Start MicroXRCE Agent (DDS bridge)
MicroXRCEAgent udp4 -p 8888

# Terminal 3: Launch ROS2 stack
ros2 launch uam_vehicle_interface sitl_launch.py

# Terminal 4: Send navigation goal
ros2 action send_goal /navigator/navigate_to_pose uam_interfaces/action/NavigateToPose \
  "{pose: {position: {x: 2.0, y: 2.0, z: 1.0}}}"
```

### Example 2: Disable Kill Switch

```bash
# Publish command to disable kill switch
ros2 topic pub --once /uam_vehicle_interface/vehicle_interface_commands \
  uam_vehicle_interface_msgs/msg/VehicleInterfaceCommand \
  "{command: 1}"  # 1 = DISABLE
```

### Example 3: Monitor Vehicle State

```bash
# Watch odometry
ros2 topic echo /uam_vehicle_interface/odometry

# Check interface status
ros2 topic echo /uam_vehicle_interface/vehicle_interface_status

# View TF tree
ros2 run tf2_tools view_frames
```

### Example 4: Send Direct Attitude Command

```cpp
#include "uam_control_msgs/msg/attitude_setpoint.hpp"

auto pub = node->create_publisher<uam_control_msgs::msg::AttitudeSetpoint>(
    "/uam_control/attitude_setpoint", 10);

uam_control_msgs::msg::AttitudeSetpoint msg;
msg.thrust_body_normalized = 15.0;  // N (for vehicle mass ~1.5kg)
msg.roll_body = 0.0;                // radians
msg.pitch_body = 0.0;
msg.yaw_body = 0.0;
// Quaternion calculated from Euler angles
msg.q_d[0] = 1.0; // w
msg.q_d[1] = 0.0; // x
msg.q_d[2] = 0.0; // y
msg.q_d[3] = 0.0; // z

pub->publish(msg);
```

---

## Vehicle Interface GUI

### Features

The Qt-based GUI (`uam_vehicle_interface_gui`) provides:

1. **Kill Switch Control**: Enable/disable offboard mode
2. **Mode Selection**: Takeoff, land, loiter, navigate_to_pose
3. **Goal Specification**: Set navigation waypoints
4. **Status Display**: Vehicle state, battery, arming status
5. **Emergency Controls**: Quick access to safety features

### Usage

```bash
ros2 run uam_vehicle_interface_gui vehicle_interface_gui
```

**Location**: `uam_vehicle_interface_gui/src/vehicle_interface_gui.cpp`

---

## Safety Features

### 1. Kill Switch

- **Purpose**: Emergency stop for offboard control
- **Default**: Enabled (safe state)
- **Behavior**: When enabled, all offboard commands ignored, offboard counter reset

### 2. Automatic Arming

- **Trigger**: Offboard mode activation with kill switch disabled
- **Safety**: Only arms if PX4 pre-flight checks pass
- **Location**: `px4_interface.cpp:214-216`

### 3. Offboard Heartbeat

- **Requirement**: Setpoints must arrive at ≥20Hz
- **Failure Mode**: PX4 enters failsafe (typically Return-to-Land) if commands stop
- **Implementation**: Control stack sends setpoints at 20-30Hz

### 4. Command Validation

- **Thrust Limits**: Automatically clamped to valid range [0, 1] after scaling
- **Frame Checks**: Ensures proper coordinate transformations

---

## Troubleshooting

### Issue 1: "Offboard Mode Rejected by PX4"

**Symptoms**: Vehicle doesn't enter offboard mode, stays in Manual/Stabilized

**Causes**:
1. Not enough setpoints sent before mode switch (< 10)
2. Setpoint rate too slow (< 2Hz required by PX4)
3. Kill switch enabled

**Solutions**:
```bash
# Check kill switch status
ros2 topic echo /uam_vehicle_interface/vehicle_interface_status

# Disable kill switch
ros2 topic pub --once /uam_vehicle_interface/vehicle_interface_commands \
  uam_vehicle_interface_msgs/msg/VehicleInterfaceCommand "{command: 1}"

# Verify setpoint publication rate
ros2 topic hz /fmu/in/vehicle_attitude_setpoint
```

### Issue 2: "Vehicle Immediately Failsafes After Takeoff"

**Symptoms**: Offboard mode activated but quickly exits, vehicle lands

**Cause**: Setpoint stream interrupted

**Solution**:
- Ensure control node is running and publishing continuously
- Check for node crashes: `ros2 node list`
- Monitor setpoint rate: should be 20+ Hz

### Issue 3: "Thrust Too High/Low in Simulation"

**Symptoms**: Vehicle climbs/falls unexpectedly

**Cause**: Incorrect motor parameters for SITL

**Solution**:
```yaml
# In params/sitl_params.yaml
uam_vehicle_interface:
  ros__parameters:
    environment: "SITL"  # Ensure this matches mode
    vehicle_mass: 1.56   # Adjust to match simulated vehicle
    motor_constant: 0.00000584  # Match PX4 SITL params
```

### Issue 4: "Odometry Not Publishing"

**Symptoms**: `/uam_vehicle_interface/odometry` has no data

**Causes**:
1. MicroXRCE Agent not running
2. PX4 not sending vehicle_odometry
3. Frame transformation failure

**Solutions**:
```bash
# Check PX4 messages arriving
ros2 topic hz /fmu/out/vehicle_odometry

# Restart MicroXRCE Agent
MicroXRCEAgent udp4 -p 8888

# Check logs for transform errors
ros2 run uam_vehicle_interface px4_interface --ros-args --log-level debug
```

### Issue 5: "Battery Voltage Compensation Incorrect"

**Symptoms**: Thrust changes unexpectedly as battery drains

**Cause**: Battery voltage threshold (14V) not appropriate for your battery

**Solution**:
Modify battery compensation logic in `computeRelativeThrust()` (line 335-339):
```cpp
if (battery_status_.voltage_filtered_v > YOUR_THRESHOLD_VOLTAGE) {
    // Adjust compensation formula
}
```

---

## Performance Characteristics

### Latency

- **Attitude Command**: ~2-5ms (ROS2 → PX4)
- **Odometry Feedback**: ~10-20ms (PX4 → ROS2)
- **Total Control Loop**: ~15-30ms

### CPU Usage

- **Idle**: ~1-2% (on Intel i5, waiting for messages)
- **Active**: ~5-10% (processing 50Hz odometry + 20Hz commands)

### Memory

- **Resident Set Size**: ~15-20 MB

---

## Dependencies

### ROS2 Packages

- `rclcpp`: ROS2 C++ client library
- `tf2_ros`: Transform library
- `tf2_eigen`: Eigen ↔ TF2 conversions
- `nav_msgs`: Standard odometry messages
- `geometry_msgs`: Pose/transform messages

### PX4 Packages

- `px4_msgs`: PX4 message definitions (git submodule)
- `px4_ros_com`: PX4 ↔ ROS2 communication utilities (git submodule)

### External Libraries

- **Eigen3**: Linear algebra (frame transformations)
- **Qt5**: GUI development (for vehicle_interface_gui)

### Build Dependencies

Defined in `package.xml` for each sub-package.

---

## Advanced Topics

### Custom Control Modes

Currently, only attitude control is implemented. To add position/velocity control:

1. Modify `publishOffboardControlMode()` to enable desired mode:
   ```cpp
   msg.position = true;  // Enable position control
   msg.velocity = false; // Disable velocity
   msg.attitude = false; // Disable attitude
   ```

2. Create subscriber for position setpoints

3. Publish to `/fmu/in/vehicle_local_position_setpoint`

**Reference**: Commented code at lines 293-321 (`publish_local_position_setpoint`)

### Rate Control

Rate control (angular velocity setpoints) can be enabled similarly:

**Reference**: Commented code at lines 185-206 (`publish_rate_control`)

### Multi-Vehicle Support

To control multiple vehicles:

1. Launch separate interface nodes with different namespaces
2. Configure PX4 system IDs differently (`target_system` parameter)
3. Use separate MicroXRCE Agent instances on different ports

---

## Testing

### Unit Tests

Currently not implemented. Recommended tests:

1. Thrust calculation accuracy (SITL vs Vehicle modes)
2. Frame transformation correctness
3. Offboard mode activation sequence
4. Kill switch state machine

### Integration Tests

1. **SITL Full Stack**: Run `sitl_launch.py` and verify all nodes communicate
2. **Odometry Loop**: Confirm odometry arrives at ≥50Hz
3. **Command Loop**: Publish attitude commands, verify PX4 receives them
4. **Failsafe**: Stop commands, confirm PX4 enters failsafe after timeout

### Hardware-in-the-Loop (HIL)

Test with real PX4 before flight:

1. Connect PX4 via USB/UART
2. Use `vehicle_launch.py` with real hardware
3. Verify thrust calculations with actual motors (propellers OFF)
4. Test kill switch and arming/disarming

---

## Future Enhancements

1. **Position/Velocity Control**: Full implementation (currently commented)
2. **Multi-Vehicle**: Support for vehicle swarms
3. **Advanced Failsafes**: Configurable failsafe behaviors
4. **Diagnostics**: Built-in health monitoring and reporting
5. **Parameter Server**: Dynamic reconfiguration of motor parameters
6. **Log Replay**: Record and replay flight data for debugging

---

## References

- [PX4 Offboard Control](https://docs.px4.io/main/en/flight_modes/offboard.html)
- [PX4 ROS2 Interface](https://docs.px4.io/main/en/ros/ros2_comm.html)
- [MicroXRCE-DDS](https://micro-xrce-dds.docs.eprosima.com/)
- [PX4 Message Reference](https://github.com/PX4/px4_msgs)
- [TF2 Documentation](http://wiki.ros.org/tf2)

---

## License

Apache License 2.0

## Maintainers

See main repository README for maintainer information.
