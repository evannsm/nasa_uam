# NASA UAM Flight Stack - Architecture Documentation

## Overview

The NASA Urban Air Mobility (UAM) Flight Stack is a complete autonomous flight system for unmanned aerial vehicles (UAVs) operating in urban environments. Built on ROS2 (Robot Operating System 2), it provides end-to-end functionality from low-level vehicle control to high-level mission planning.

## System Architecture

### High-Level Component Diagram

```
┌─────────────────────────────────────────────────────────────────┐
│                        Ground Control Station                    │
│                     (uam_vehicle_interface_gui)                  │
└────────────────────────────┬────────────────────────────────────┘
                             │ ROS2 Topics/Services
┌────────────────────────────┴────────────────────────────────────┐
│                      ROS2 Flight Stack                           │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐          │
│  │  Navigator   │─>│   Planner    │─>│ Visualization│          │
│  │  (Mission    │  │  (Path       │  │   (RViz)     │          │
│  │   Manager)   │  │  Planning)   │  │              │          │
│  └──────┬───────┘  └──────┬───────┘  └──────────────┘          │
│         │                  │                                     │
│         │                  │         ┌──────────────┐           │
│         │                  └────────>│   Mapping    │           │
│         │                            │  (Obstacles) │           │
│         │                            └──────────────┘           │
│         v                                                        │
│  ┌──────────────┐         ┌──────────────┐                     │
│  │   Control    │<───────>│   Vehicle    │                     │
│  │ (Q-learning) │         │  Interface   │                     │
│  └──────────────┘         └──────┬───────┘                     │
└────────────────────────────────────┼───────────────────────────┘
                                     │ Offboard Commands
                             ┌───────┴────────┐
                             │  PX4 Autopilot │
                             │   (Hardware/   │
                             │     SITL)      │
                             └────────────────┘
```

### Component Responsibilities

#### 1. **uam_vehicle_interface** (Low-Level Vehicle Control)
- **Purpose**: Bridges ROS2 and PX4 autopilot firmware
- **Inputs**: Position/velocity setpoints, mode commands
- **Outputs**: Vehicle odometry, status information
- **Key Features**:
  - Manages offboard control mode with PX4
  - Translates ROS2 geometry messages to PX4 commands
  - Provides kill switch functionality
  - Publishes vehicle state (position, velocity, attitude)
  - GUI for ground control station operations

#### 2. **uam_navigator** (Mission Management)
- **Purpose**: High-level navigation state machine
- **Inputs**: Navigation goals, planner feedback
- **Outputs**: Position setpoints to vehicle interface
- **Key Features**:
  - Plugin-based mode architecture (takeoff, land, loiter, navigate_to_pose)
  - Manages transitions between flight modes
  - Coordinates with planner for goal-directed navigation
  - Implements ROS2 action servers for mission control
  - Lifecycle node for controlled startup/shutdown

#### 3. **uam_planner** (Path Planning)
- **Purpose**: Computes collision-free paths
- **Inputs**: Start pose, goal pose, obstacle map
- **Outputs**: Waypoint paths
- **Key Features**:
  - Plugin-based planner architecture
  - RRT*-static algorithm using OMPL library
  - 3D path planning with configurable bounds
  - Collision checking with geometric shapes
  - Lifecycle node with configurable parameters

#### 4. **uam_control** (Advanced Control)
- **Purpose**: Reinforcement learning-based control
- **Inputs**: Desired trajectories, current state
- **Outputs**: Control commands (acceleration/attitude)
- **Key Features**:
  - Q-learning adaptive controller
  - State discretization for learning
  - Configurable reward functions
  - Training and deployment modes
  - Optional direct control mode

#### 5. **uam_mapping** (Environment Representation)
- **Purpose**: Manages obstacle information
- **Inputs**: Obstacle definitions (from config)
- **Outputs**: Obstacle positions for planner
- **Key Features**:
  - Static obstacle publication
  - Configurable obstacle primitives (spheres, boxes)
  - Integration with planner for collision checking

#### 6. **uam_visualization** (Data Visualization)
- **Purpose**: RViz visualization of system state
- **Inputs**: Vehicle state, obstacles, paths
- **Outputs**: RViz marker messages
- **Key Features**:
  - Obstacle visualization (spheres, boxes)
  - Path visualization
  - Real-time updates

#### 7. **uam_util** (Common Utilities)
- **Purpose**: Shared base classes and utilities
- **Key Components**:
  - `LifecycleNode`: Base class for managed lifecycle nodes
  - `SimpleActionServer`: Simplified ROS2 action server wrapper
  - `ServiceClient`: Managed service client with timeout handling
  - `NodeUtils`: Common ROS2 node utilities (QoS profiles, parameter handling)

## Software Design Patterns

### 1. Plugin Architecture
Both the navigator and planner use plugin-based designs:
- **Navigator Modes**: Takeoff, Land, Loiter, NavigateToPose are separate plugins
- **Planner Algorithms**: RRT*-static is a plugin; new algorithms can be added
- **Benefits**: Extensibility, modularity, easy testing of individual components

### 2. Lifecycle Management
All core nodes use ROS2 lifecycle pattern:
- **States**: Unconfigured → Inactive → Active → (Cleanup)
- **Benefits**: Controlled initialization, graceful shutdown, fault recovery
- **Implementation**: `uam_util::LifecycleNode` base class

### 3. Action-Based Communication
Long-running tasks use ROS2 actions:
- **Navigator**: `NavigateToPose` action for goal-directed flight
- **Planner**: `ComputePath` action for path computation
- **Benefits**: Feedback during execution, cancellation support, goal queuing

### 4. Topic-Based Real-Time Data
High-frequency data uses ROS2 topics:
- Vehicle odometry (50+ Hz)
- Position setpoints (20+ Hz)
- Sensor data forwarding

## Data Flow

### Typical Navigation Mission Flow

1. **Mission Initiation**:
   - User sends `NavigateToPose` goal to Navigator
   - Navigator transitions to `navigate_to_pose` mode

2. **Path Planning**:
   - Navigator requests path from Planner via `ComputePath` action
   - Planner queries Mapping for obstacle information
   - Planner computes RRT*-static path avoiding obstacles
   - Planner returns waypoint sequence

3. **Path Execution**:
   - Navigator sends waypoints as position setpoints
   - Control (optional) refines commands using Q-learning
   - Vehicle Interface translates to PX4 offboard commands
   - PX4 executes low-level attitude/thrust control

4. **Feedback & Monitoring**:
   - Vehicle Interface publishes odometry from PX4
   - Navigator monitors progress toward goal
   - Visualization publishes markers for RViz
   - GUI displays vehicle status

5. **Mission Completion**:
   - Navigator detects goal reached
   - Returns success to action client
   - Transitions to loiter mode (if configured)

## Communication Interfaces

### Topics

| Topic | Type | Publisher | Subscriber | Purpose |
|-------|------|-----------|------------|---------|
| `/odom` | `nav_msgs/Odometry` | vehicle_interface | navigator, control | Vehicle state |
| `/position_setpoint` | `geometry_msgs/PoseStamped` | navigator | vehicle_interface | Desired position |
| `/obstacles` | `uam_interfaces/ObstacleArray` | mapping | planner, visualization | Obstacle positions |
| `/path` | `nav_msgs/Path` | planner | visualization | Planned path |

### Actions

| Action | Type | Server | Client | Purpose |
|--------|------|--------|--------|---------|
| `navigate_to_pose` | `uam_interfaces/NavigateToPose` | navigator | GUI/user | Goal-directed navigation |
| `compute_path` | `uam_interfaces/ComputePath` | planner | navigator | Path planning |

### Services

| Service | Type | Server | Client | Purpose |
|---------|------|--------|--------|---------|
| `/change_mode` | `uam_interfaces/ChangeMode` | navigator | GUI | Change flight mode |
| `/kill_switch` | `std_srvs/Trigger` | vehicle_interface | GUI | Emergency stop |

## Configuration Management

### Parameter Hierarchy

Each package uses YAML configuration files:

```
config/
├── vehicle_params.yaml          # Vehicle-specific (mass, limits)
├── navigator_params.yaml        # Navigator settings
├── planner_params.yaml          # Planner algorithm config
├── controller_params.yaml       # Control gains and Q-learning
├── mapping_params.yaml          # Obstacle definitions
└── visualization_params.yaml    # Visualization settings
```

### Launch File Architecture

Three deployment configurations:

1. **SITL (Software-in-the-Loop)**:
   - `sitl_launch.py`: Full stack + PX4 SITL + Gazebo
   - Use for development and testing

2. **Vehicle (Onboard)**:
   - `vehicle_launch.py`: Flight stack only
   - Deploy on companion computer with real PX4

3. **GCS (Ground Control Station)**:
   - `gcs_launch.py`: GUI and visualization only
   - Run on ground station laptop

## Technology Stack

### Core Dependencies

- **ROS2 Galactic**: Middleware and framework
- **PX4 Autopilot**: Low-level flight control
- **Eigen3**: Linear algebra operations
- **OMPL**: Motion planning algorithms
- **Qt5**: GUI development
- **geometric_shapes**: Collision checking
- **Boost**: Utility libraries

### Build System

- **colcon**: ROS2 build tool
- **CMake**: Build configuration
- **ament_cmake**: ROS2 CMake macros

## Coordinate Frames

### Frame Conventions

- **map**: Fixed world frame (origin at takeoff location or global reference)
- **odom**: Vehicle odometry frame (ENU - East-North-Up)
- **base_link**: Vehicle body frame (FRD - Forward-Right-Down for PX4)
- **base_link_enu**: Vehicle body frame (ENU for ROS2 compatibility)

### Transform Tree

```
map
 └─> odom
      └─> base_link_enu
           └─> base_link (FRD)
```

Vehicle Interface handles frame transformations between ROS2 (ENU) and PX4 (FRD).

## Deployment Scenarios

### 1. SITL Simulation
- **Purpose**: Development, testing, algorithm validation
- **Components**: All nodes + PX4 SITL + Gazebo
- **Hardware**: Single development machine
- **Benefits**: Fast iteration, safe testing, no hardware required

### 2. Hardware-in-the-Loop (HIL)
- **Purpose**: Hardware validation with simulated sensors
- **Components**: All nodes + Real PX4 + Simulated environment
- **Hardware**: Development machine + PX4 flight controller
- **Benefits**: Test real hardware without flight risk

### 3. Real Flight
- **Purpose**: Actual UAV operations
- **Components**:
  - Onboard: All nodes on companion computer
  - Ground: GUI and monitoring on GCS laptop
- **Hardware**:
  - UAV: PX4 flight controller + Companion computer + Sensors
  - Ground: Laptop with ROS2
- **Benefits**: Full system validation

## Safety Features

### Multi-Layer Safety

1. **Application Layer**:
   - Goal validation in Navigator
   - Collision checking in Planner
   - Kill switch service

2. **Interface Layer**:
   - Offboard mode timeout detection
   - Command rate monitoring
   - State validation before PX4 commands

3. **PX4 Layer**:
   - Geofencing
   - Failsafe modes
   - Manual override

### Emergency Procedures

1. **Kill Switch**: Immediate disarm via service call
2. **RC Override**: Manual control always available
3. **Offboard Timeout**: PX4 enters failsafe if no commands
4. **Geofence**: PX4 RTL (return-to-land) if boundary exceeded

## Performance Characteristics

### Typical Update Rates

- **Vehicle Odometry**: 50 Hz (from PX4)
- **Position Setpoints**: 20 Hz (to PX4)
- **Control Loop**: 20 Hz (Q-learning)
- **Planner**: On-demand (typically 1-5 seconds per query)
- **Visualization**: 10 Hz

### Resource Requirements

- **CPU**: Multi-core recommended (4+ cores)
- **RAM**: 2-4 GB for full stack
- **Storage**: ~500 MB for workspace build
- **Network**: Low latency (<10ms) for distributed deployment

## Extensibility Points

### Adding New Components

1. **New Navigator Mode**:
   - Inherit from `NavigatorMode` base class
   - Implement `onEnter()`, `onExit()`, `update()` methods
   - Register as plugin in `navigator_plugins.xml`

2. **New Planner Algorithm**:
   - Inherit from `PlannerPlugin` base class
   - Implement `computePath()` method
   - Register as plugin in `planner_plugins.xml`

3. **New Controller**:
   - Inherit from `uam_util::LifecycleNode`
   - Subscribe to odometry, publish to vehicle interface
   - Add to launch file

## Development Workflow

### Standard Development Cycle

1. **Make Changes**: Edit source code
2. **Build**: `colcon build --symlink-install`
3. **Test SITL**: `ros2 launch uam_vehicle_interface sitl_launch.py`
4. **Validate**: Monitor in RViz, test scenarios
5. **Debug**: Use ROS2 tools (`ros2 topic`, `ros2 node`, `rqt`)

### Testing Strategy

- **Unit Tests**: Test individual classes (future work)
- **Integration Tests**: SITL with automated scenarios
- **Flight Tests**: Staged outdoor testing

## Known Limitations & Future Work

### Current Limitations

1. Static obstacles only (no dynamic obstacle avoidance)
2. Single vehicle (no multi-agent coordination)
3. Limited sensor integration (mostly GPS/IMU)
4. Q-learning control is research-grade (not production-ready)

### Planned Enhancements

1. Dynamic obstacle detection and avoidance
2. Vision-based navigation
3. Multi-vehicle coordination
4. Advanced learning-based control
5. Comprehensive test suite
6. Improved fault detection and recovery

## References

- [ROS2 Documentation](https://docs.ros.org/en/galactic/)
- [PX4 Autopilot](https://docs.px4.io/)
- [OMPL](https://ompl.kavrakilab.org/)
- [RRT* Algorithm Paper](https://arxiv.org/abs/1105.1186)

## Getting Help

- **Installation Issues**: See `docs/installation.md`
- **Usage Examples**: See `docs/EXAMPLES.md` (when created)
- **API Reference**: See `docs/API.md` (when created)
- **Package-Specific**: See README in each package directory
