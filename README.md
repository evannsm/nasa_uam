# NASA Urban Air Mobility (UAM) Flight Stack

A complete ROS2-based autonomous flight system for unmanned aerial vehicles operating in urban environments, featuring advanced path planning, Q-learning adaptive control, and PX4 integration.

[![ROS2](https://img.shields.io/badge/ROS2-Galactic-blue)](https://docs.ros.org/en/galactic/)
[![PX4](https://img.shields.io/badge/PX4-Autopilot-orange)](https://px4.io/)
[![License](https://img.shields.io/badge/License-Apache%202.0-green.svg)](LICENSE)

---

## Table of Contents

- [Overview](#overview)
- [Key Features](#key-features)
- [System Architecture](#system-architecture)
- [Quick Start](#quick-start)
- [Installation](#installation)
- [Usage](#usage)
- [Documentation](#documentation)
- [Package Descriptions](#package-descriptions)
- [Development](#development)
- [Troubleshooting](#troubleshooting)
- [Contributing](#contributing)
- [License](#license)

---

## Overview

The NASA UAM Flight Stack provides a complete autonomous navigation solution for UAVs, from high-level mission planning to low-level motor control. Built on ROS2 and integrated with PX4 autopilot, it enables safe, intelligent flight in complex urban environments.

### What Can It Do?

- ✈️ **Autonomous Navigation**: Fly from point A to B avoiding obstacles
- 🧠 **Adaptive Learning**: Q-learning controller that improves over time
- 🗺️ **Path Planning**: RRT*-based optimal path computation in 3D
- 🚁 **Multiple Flight Modes**: Takeoff, loiter, navigate, land
- 🛡️ **Safety Features**: Kill switch, altitude limits, collision avoidance
- 📊 **Visualization**: Real-time RViz display of vehicle state and plans

### Use Cases

- **Research**: Algorithm development for autonomous UAVs
- **Education**: Learning about robotics, control, and path planning
- **Testing**: SITL simulation before hardware deployment
- **Demonstrations**: Showcasing UAM capabilities

---

## Key Features

### Advanced Control

- **Q-Learning Adaptive Controller**: Online reinforcement learning-based trajectory tracking
  - Actor-critic architecture
  - Learns optimal policy from experience
  - Model-free control (no system identification needed)
  - See [docs/QLEARNING_THEORY.md](docs/QLEARNING_THEORY.md) for mathematical details

### Intelligent Planning

- **RRT*-static Path Planner**: Asymptotically optimal sampling-based motion planning
  - 3D collision-free paths
  - Configurable safety margins
  - Anytime algorithm (improves path quality over time)

### Robust Architecture

- **Plugin-Based Design**: Easily swap planners and controllers
- **Lifecycle Management**: Controlled startup/shutdown of components
- **Action-Based Communication**: Feedback and cancellation support
- **Multi-Rate Control**: Independent update rates for planning, control, and interface

---

## System Architecture

```
┌────────────────────────────────────────────────────────────────┐
│                        UAM Flight Stack                         │
├────────────────────────────────────────────────────────────────┤
│                                                                  │
│  ┌──────────┐    ┌────────────┐    ┌──────────────┐           │
│  │Navigator │───>│  Planner   │───>│Visualization │           │
│  │(Mission) │    │  (RRT*)    │    │   (RViz)     │           │
│  └────┬─────┘    └──────┬─────┘    └──────────────┘           │
│       │                  │                                      │
│       │                  │ obstacles  ┌──────────┐             │
│       │                  └───────────>│ Mapping  │             │
│       │ setpoints                     └──────────┘             │
│       v                                                         │
│  ┌──────────────┐      ┌────────────────────┐                 │
│  │  Q-Learning  │<────>│ Vehicle Interface  │                 │
│  │  Controller  │ odom │  (PX4 Bridge)      │                 │
│  └──────────────┘      └─────────┬──────────┘                 │
└─────────────────────────────────┼────────────────────────────┘
                                   │
                          ┌────────┴─────────┐
                          │  PX4 Autopilot   │
                          │ (SITL/Hardware)  │
                          └──────────────────┘
```

### Data Flow

1. **Mission Planning**: User/GCS sends goal to Navigator
2. **Path Computation**: Navigator requests collision-free path from Planner
3. **Waypoint Following**: Navigator publishes position setpoints
4. **Adaptive Control**: Q-learning controller computes optimal attitude/thrust
5. **Low-Level Control**: Vehicle Interface sends commands to PX4
6. **State Feedback**: PX4 odometry flows back up the stack

See [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md) for detailed architecture documentation.

---

## Quick Start

### Prerequisites

- Ubuntu 22.04
- ROS2 Galactic
- PX4 Autopilot (v1.12+)
- Gazebo Classic
- Micro XRCE DDS Agent

### SITL Simulation (5-Minute Setup)

**Terminal 1: Start PX4 SITL**
```bash
cd ~/PX4-Autopilot
make px4_sitl gazebo-classic_iris_vision
```

**Terminal 2: Start DDS Bridge**
```bash
MicroXRCEAgent udp4 -p 8888
```

**Terminal 3: Launch UAM Stack**
```bash
cd ~/nasa_uam
source install/setup.bash
ros2 launch uam_vehicle_interface sitl_launch.py
```

**Result:** RViz opens with the UAM control panel on the right.

### Basic Flight Sequence

1. **Disable Kill Switch**: Click "Disable Kill Switch" in RViz panel
2. **Takeoff**: Click "Takeoff" button
3. **Wait**: Vehicle climbs to 0.7m and enters loiter mode
4. **Navigate**: Use "2D Goal Pose" tool (top toolbar) to set destination
5. **Watch**: Vehicle plans path (red line) and flies autonomously

---

## Installation

### Full Installation Guide

See [docs/installation.md](docs/installation.md) for comprehensive installation instructions.

### Quick Installation

```bash
# 1. Install ROS2 Galactic
# (Follow official ROS2 installation guide)

# 2. Install dependencies
sudo apt install ros-galactic-geographic-msgs \
                 ros-galactic-geometric-shapes \
                 ros-galactic-ompl \
                 ros-galactic-tf2-eigen \
                 libeigen3-dev

# 3. Clone repository
mkdir -p ~/nasa_uam_ws/src
cd ~/nasa_uam_ws/src
git clone <repository-url> nasa_uam
cd nasa_uam

# 4. Initialize submodules (PX4 messages)
git submodule update --init --recursive

# 5. Build
cd ~/nasa_uam_ws
colcon build --symlink-install

# 6. Source workspace
source install/setup.bash
```

### PX4 Setup

```bash
# Clone PX4
cd ~
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
cd PX4-Autopilot

# Build for SITL
make px4_sitl

# Install Micro XRCE Agent
# (Follow PX4 documentation)
```

---

## Usage

### SITL Simulation

**Complete SITL Workflow:**

```bash
# Terminal 1: PX4 SITL + Gazebo
cd ~/PX4-Autopilot
make px4_sitl gazebo-classic_iris_vision

# Terminal 2: DDS Bridge
MicroXRCEAgent udp4 -p 8888

# Terminal 3: UAM Flight Stack
cd ~/nasa_uam
source install/setup.bash
ros2 launch uam_vehicle_interface sitl_launch.py
```

### Hardware Deployment

**Onboard Computer Launch:**

```bash
# On companion computer (connected to PX4 via UART/USB)
ros2 launch uam_vehicle_interface vehicle_launch.py
```

**Ground Control Station:**

```bash
# On laptop (connected via WiFi)
ros2 launch uam_vehicle_interface gcs_launch.py
```

### Command-Line Control

**Takeoff:**
```bash
ros2 action send_goal /send_navigator_command \
  uam_navigator_msgs/action/NavigatorCommand \
  "{command: 'Takeoff'}"
```

**Navigate to Position:**
```bash
ros2 action send_goal /send_navigator_command \
  uam_navigator_msgs/action/NavigatorCommand \
  "{command: 'navigate_to_pose_rrtx_static',
    goal: {header: {frame_id: 'map'},
           pose: {position: {x: 2.0, y: 2.0, z: 1.0}}}}"
```

**Disable Kill Switch:**
```bash
ros2 topic pub --once /uam_vehicle_interface/vehicle_interface_commands \
  uam_vehicle_interface_msgs/msg/VehicleInterfaceCommand "{command: 1}"
```

### Monitoring

```bash
# Vehicle status
ros2 topic echo /uam_navigator/navigator_status

# Q-learning performance
ros2 topic echo /uam_control/qlearning_status

# Planned path
ros2 topic echo /planner_server/path

# Node graph
rqt_graph
```

---

## Documentation

### Core Documentation

| Document | Description |
|----------|-------------|
| [ARCHITECTURE.md](docs/ARCHITECTURE.md) | System architecture and component descriptions |
| [QLEARNING_THEORY.md](docs/QLEARNING_THEORY.md) | Mathematical theory of Q-learning controller |
| [installation.md](docs/installation.md) | Complete installation instructions |
| [EXAMPLES.md](docs/EXAMPLES.md) | Usage examples and tutorials |

### Package Documentation

| Package | README | Description |
|---------|--------|-------------|
| uam_control | [README](uam_control/README.md) | Q-learning adaptive controller |
| uam_planner | [README](uam_planner/README.md) | RRT* path planning server |
| uam_navigator | [README](uam_navigator/README.md) | Mission state machine |
| uam_vehicle_interface | [README](uam_vehicle_interface/README.md) | PX4 interface and GUI |
| uam_mapping | [README](uam_mapping/README.md) | Obstacle management |
| uam_util | [README](uam_util/README.md) | Shared utilities |
| uam_visualization | [README](uam_visualization/README.md) | RViz integration |

### Code Comments

The Q-learning controller (`uam_control/src/qlearning_controller.cpp`) contains **extensive inline comments** (~200 lines) explaining:
- Mathematical foundations
- Algorithm step-by-step
- References to academic papers
- Implementation details

---

## Package Descriptions

### uam_vehicle_interface

**Purpose:** Low-level bridge between ROS2 and PX4 autopilot

**Key Features:**
- Offboard mode management
- Coordinate frame transformations (ENU ↔ NED)
- Kill switch functionality
- Qt-based ground control GUI

**Topics:**
- Publishes: `/uam_vehicle_interface/odometry`
- Subscribes: `/uam_control/attitude_setpoint`

### uam_navigator

**Purpose:** High-level mission state machine

**Flight Modes:**
- **Takeoff**: Climb to configured altitude
- **Loiter**: Hover in place
- **NavigateToPose**: Autonomous flight to goal
- **Land**: Controlled descent (partial implementation)

**Actions:**
- Server: `send_navigator_command`
- Client: `compute_path_to_pose` (to planner)

### uam_planner

**Purpose:** 3D collision-free path planning

**Algorithm:** RRT*-static (OMPL library)

**Features:**
- Asymptotically optimal paths
- Configurable obstacle inflation
- Plugin architecture for multiple algorithms

**Actions:**
- Server: `compute_path_to_pose`

### uam_control

**Purpose:** Advanced reinforcement learning-based control

**Algorithm:** Actor-Critic Q-Learning

**Features:**
- Online learning during flight
- Model-free adaptive control
- Geometric control integration

**Topics:**
- Publishes: `/uam_control/attitude_setpoint`, `/uam_control/qlearning_status`
- Subscribes: `/uam_navigator/position_setpoint`, `/uam_vehicle_interface/odometry`

### uam_mapping

**Purpose:** Obstacle information management

**Features:**
- Static obstacle publishing
- Configurable shapes (box, sphere, cylinder)
- Integration with planner

**Topics:**
- Publishes: `/uam_mapping/obstacles`

### uam_visualization

**Purpose:** RViz visualization of obstacles and paths

**Features:**
- Obstacle markers (colored shapes)
- Planned path display
- Real-time updates

**Topics:**
- Publishes: `/uam_visualization/obstacle_markers`

### uam_util

**Purpose:** Shared utility classes

**Components:**
- `LifecycleNode`: Base class for managed nodes
- `SimpleActionServer`: Simplified action server wrapper
- `ServiceClient`: Timeout-aware service client
- `NodeUtils`: Common ROS2 utilities

---

## Development

### Building

```bash
# Full rebuild
cd ~/nasa_uam
colcon build --symlink-install

# Single package
colcon build --packages-select uam_control

# With debug symbols
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Debug

# Clean build
rm -rf build install log
colcon build
```

### Running Tests

```bash
# Run all tests
colcon test

# Test specific package
colcon test --packages-select uam_planner

# View test results
colcon test-result --verbose
```

### Code Style

Follow ROS2 style guide:
- C++: Google C++ Style Guide
- Python: PEP 8
- Naming: snake_case for variables, CamelCase for classes

### Adding a New Feature

1. **Create branch**: `git checkout -b feature/my-feature`
2. **Develop**: Make changes, add tests
3. **Document**: Update README, add comments
4. **Test**: Verify in SITL
5. **Commit**: `git commit -m "Add feature: description"`
6. **Push**: `git push origin feature/my-feature`
7. **Pull Request**: Create PR for review

---

## Troubleshooting

### Common Issues

#### Issue 1: "Failed to create planner plugin"

**Symptom:** Planner server fails to load

**Solution:**
```bash
# Source workspace
source ~/nasa_uam/install/setup.bash

# Verify plugin registration
ros2 pkg list | grep uam_planner

# Check parameters
ros2 param describe /planner_server planner_plugins
```

#### Issue 2: "No path found" errors

**Symptom:** Planner returns `NO_VALID_PATH`

**Causes:**
- Obstacles blocking path
- Planning time too short
- Goal inside obstacle

**Solutions:**
```yaml
# In params file, increase solve time
rrtx_static:
  solve_time: 2.0  # from 0.5

# Reduce obstacle scaling
  obstacle_scaling: 1.5  # from 2.0
```

#### Issue 3: Vehicle won't arm/takeoff

**Symptoms:** Vehicle stays on ground

**Checks:**
```bash
# 1. Check PX4 status
ros2 topic echo /fmu/out/vehicle_status

# 2. Verify kill switch disabled
ros2 topic echo /uam_vehicle_interface/vehicle_interface_commands

# 3. Check navigator mode
ros2 topic echo /uam_navigator/navigator_status

# 4. PX4 pre-flight checks
# In PX4 console (Gazebo terminal):
# commander status
```

#### Issue 4: Q-learning controller unstable

**Symptom:** Oscillations or divergence

**Solution:**
```yaml
# Reduce learning rates
uam_control:
  ros__parameters:
    rrtx_static:
      critic_convergence_rate: 5.0   # from 10.0
      actor_convergence_rate: 0.2    # from 0.5
```

### Debug Tools

```bash
# View all topics
ros2 topic list

# Monitor topic
ros2 topic echo /topic_name

# Check node status
ros2 node info /node_name

# View TF tree
ros2 run tf2_tools view_frames

# Record data
ros2 bag record -a  # all topics
ros2 bag record /topic1 /topic2  # specific topics

# Visualize computation graph
rqt_graph
```

### Getting Help

- **Documentation**: See [docs/](docs/) folder
- **Issues**: Check existing [GitHub issues](<repository-url>/issues)
- **Slack/Discord**: [Community channel if available]

---

## Performance Characteristics

### Update Rates

| Component | Rate | Purpose |
|-----------|------|---------|
| Vehicle Odometry | 50+ Hz | State feedback |
| Q-Learning Control | ~50 Hz | Attitude commands |
| Navigator Setpoints | 20-100 Hz | Mode-dependent |
| Path Planning | On-demand | 0.5-2s per query |
| Obstacle Updates | 10 Hz | Mapping |
| Visualization | 10 Hz | RViz markers |

### Resource Usage

| Component | CPU | Memory |
|-----------|-----|--------|
| Full Stack (idle) | ~5% | 160-200 MB |
| Full Stack (active) | 40-50% | 200-300 MB |
| During Planning | +20-80% | +50-100 MB |

### Timing

| Operation | Duration |
|-----------|----------|
| Takeoff | 3-5 seconds |
| Path Planning (simple) | 0.1-0.5s |
| Path Planning (complex) | 0.5-2.0s |
| Q-learning Convergence | 2-5 minutes |
| Waypoint Transition | 1-3 seconds |

---

## Safety Considerations

### Safety Features

1. **Kill Switch**: Emergency motor cutoff (default: ENABLED)
2. **Altitude Limits**: Learning disabled below 0.6m
3. **Offboard Timeout**: PX4 failsafe if no commands (2s)
4. **Collision Avoidance**: Path planning with safety margins
5. **Manual Override**: RC control always available

### Before Flying Hardware

1. ✅ Test extensively in SITL
2. ✅ Verify all sensors functional
3. ✅ Check battery voltage
4. ✅ Confirm RC link active
5. ✅ Set geofence boundaries in PX4
6. ✅ Have manual override ready
7. ✅ Fly in open area away from people

### Known Limitations

- ⚠️ Q-learning control is **research-grade** (not production-ready)
- ⚠️ Static obstacles only (no dynamic obstacle detection)
- ⚠️ Limited sensor integration (GPS/IMU primarily)
- ⚠️ No multi-vehicle coordination
- ⚠️ Orientation planning not implemented (position only)

---

## Contributing

We welcome contributions! Please:

1. Fork the repository
2. Create a feature branch
3. Make your changes with tests
4. Add documentation
5. Submit a pull request

### Contribution Guidelines

- Follow ROS2 style guide
- Add unit tests for new features
- Update relevant README files
- Document mathematical algorithms with references
- Test in SITL before submitting

---

## References

### Academic Papers

The implementation is based on these key papers:

**Q-Learning Control:**
- Vamvoudakis & Lewis (2010). "Online actor-critic algorithm to solve the continuous-time infinite horizon optimal control problem." *Automatica*, 46(5), 878-888.

**Path Planning:**
- Karaman & Frazzoli (2011). "Sampling-based algorithms for optimal motion planning." *IJRR*, 30(7), 846-894.

**Geometric Control:**
- Mellinger & Kumar (2011). "Minimum snap trajectory generation and control for quadrotors." *IEEE ICRA*.
- Lee et al. (2010). "Geometric tracking control of a quadrotor UAV on SE(3)." *IEEE CDC*.

See [docs/QLEARNING_THEORY.md](docs/QLEARNING_THEORY.md) for complete references.

### Software Libraries

- [ROS2 Galactic](https://docs.ros.org/en/galactic/)
- [PX4 Autopilot](https://docs.px4.io/)
- [OMPL](https://ompl.kavrakilab.org/)
- [Eigen3](https://eigen.tuxfamily.org/)

---

## License

This project is licensed under the Apache License 2.0 - see the [LICENSE](LICENSE) file for details.

Portions of the code are adapted from:
- Navigation2 (Apache 2.0)
- PX4-ROS2 Bridge (BSD 3-Clause)

---

## Acknowledgments

- NASA for funding and support
- PX4 Development Team
- ROS2 Community
- OMPL Developers

---

## Citation

If you use this code in your research, please cite:

```bibtex
@software{nasa_uam_2024,
  title = {NASA Urban Air Mobility Flight Stack},
  author = {[Authors]},
  year = {2024},
  url = {<repository-url>}
}
```

---

**Happy Flying! 🚁**

For questions, issues, or contributions, please visit the [GitHub repository](<repository-url>).
