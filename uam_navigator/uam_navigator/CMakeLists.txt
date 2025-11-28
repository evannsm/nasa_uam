# ==============================================================================
# UAM Navigator - CMake Build Configuration
# ==============================================================================
#
# This CMakeLists.txt demonstrates a more complex ROS2 build pattern:
# 1. Multiple shared libraries (navigator_modes, navigator)
# 2. Executable (navigator_main)
# 3. Component-based architecture (rclcpp_components)
#
# Build Outputs:
# --------------
# - libnavigator_modes.so  → Shared library with navigation mode implementations
# - libnavigator.so        → Shared library with main navigator state machine
# - navigator_main         → Executable that loads navigator component
#
# Key Concepts:
# -------------
#
# 1. COMPONENT-BASED ARCHITECTURE
# --------------------------------
# ROS2 supports two ways to run nodes:
#
# a) Standalone Executable (traditional):
#    - Each node is separate process
#    - Communication via DDS (inter-process)
#    - Higher overhead (separate memory spaces)
#
# b) Component-Based (modern ROS2):
#    - Multiple nodes in ONE process
#    - Communication via intra-process (zero-copy shared pointers)
#    - Lower latency, less memory overhead
#    - Can still be launched as standalone if needed
#
# This package supports BOTH:
# - Component: Register Navigator as composable (rclcpp_components)
# - Executable: Provide standalone binary (navigator_main)
#
# 2. LIBRARY LAYERING
# -------------------
# We build THREE separate targets:
#
# Layer 1: navigator_modes (low-level)
#   - Takeoff, Loiter, NavigateToPose implementations
#   - Reusable navigation mode implementations
#
# Layer 2: navigator (mid-level)
#   - State machine logic
#   - Depends on navigator_modes
#   - Composable ROS2 component
#
# Layer 3: navigator_main (high-level)
#   - Thin executable wrapper
#   - Instantiates navigator component
#   - Handles ROS2 node lifecycle
#
# Dependency Chain:
#   navigator_main → navigator → navigator_modes
#
# 3. WHY SEPARATE LIBRARIES?
# ---------------------------
# Benefits of splitting into libraries:
#
# - Modularity: Each library has clear responsibility
# - Reusability: Other packages can link against libraries
# - Faster recompilation: Change mode → only rebuild navigator_modes
# - Testing: Can unit test libraries without executable
# - Composition: Different executables can reuse libraries
#
# 4. INCLUDE DIRECTORIES - GENERATOR EXPRESSIONS
# -----------------------------------------------
# We use CMake generator expressions for include paths:
#
# $<BUILD_INTERFACE:...>  → Used during build in source tree
# $<INSTALL_INTERFACE:...> → Used after installation by other packages
#
# Why?
# - Build: Headers in source tree (include/uam_navigator/...)
# - Install: Headers in install tree (install/include/uam_navigator/...)
# - Other packages need install paths, not source paths
#
# ==============================================================================

# Minimum CMake version
cmake_minimum_required(VERSION 3.8)

# Project name
project(uam_navigator)

# ==============================================================================
# SECTION 1: FIND DEPENDENCIES
# ==============================================================================

# --- Core ROS2 ---
find_package(ament_cmake REQUIRED)              # Build system
find_package(rclcpp REQUIRED)                   # ROS2 C++ library
find_package(rclcpp_action REQUIRED)            # Action server/client
find_package(rclcpp_lifecycle REQUIRED)         # Lifecycle nodes
find_package(rclcpp_components REQUIRED)        # Component-based nodes
find_package(pluginlib REQUIRED)                # Plugin system

# --- Math & Geometry ---
find_package(eigen3_cmake_module REQUIRED)      # Eigen3 helper
find_package(Eigen3 REQUIRED)                   # Linear algebra

# --- ROS2 Standard Messages ---
find_package(std_msgs REQUIRED)                 # Standard message types
find_package(nav_msgs REQUIRED)                 # Navigation messages (Path, Odometry)
find_package(builtin_interfaces REQUIRED)       # Time, Duration

# --- Custom Messages ---
find_package(uam_navigator_msgs REQUIRED)       # Navigator commands/status
find_package(uam_planner_msgs REQUIRED)         # Planner interface messages

# --- UAM Packages ---
find_package(uam_util REQUIRED)                 # Utility functions (action servers, QoS)
find_package(uam_vehicle_interface REQUIRED)    # Vehicle interface (currently unused in CMake)

# ==============================================================================
# SECTION 2: INCLUDE DIRECTORIES
# ==============================================================================

# Add our headers to include path
# Used by all targets in this CMakeLists.txt
include_directories(include)

# ==============================================================================
# SECTION 3: BUILD NAVIGATOR MODES LIBRARY (Layer 1)
# ==============================================================================

# Create shared library with navigation mode implementations
# Modes: Takeoff, Loiter, NavigateToPose, (Land - commented out)
add_library(navigator_modes SHARED
  src/navigator_modes/navigate_to_pose.cpp   # Path following mode
  src/navigator_modes/loiter.cpp             # Station-keeping mode
  src/navigator_modes/takeoff.cpp            # Vertical ascent mode
)

# Link against dependencies
# These modes need: ROS2 API, actions, Eigen, messages
ament_target_dependencies(navigator_modes
  rclcpp
  rclcpp_action
  Eigen3
  nav_msgs
  uam_util
  uam_planner_msgs
  uam_navigator_msgs
)

# Set include directories using generator expressions
# BUILD_INTERFACE: Used during compilation in source tree
# INSTALL_INTERFACE: Used by other packages linking against installed library
target_include_directories(navigator_modes PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

# ==============================================================================
# SECTION 4: BUILD NAVIGATOR LIBRARY (Layer 2)
# ==============================================================================

# Define dependencies for navigator library
# Shared between library and executable
set(navigator_dependencies
  rclcpp                 # ROS2 C++ API
  rclcpp_action          # Action servers/clients
  rclcpp_lifecycle       # Lifecycle state management
  pluginlib              # Plugin loading (for mode plugins)
  builtin_interfaces     # Time/Duration messages
  uam_navigator_msgs     # Navigator commands/status
  nav_msgs               # Odometry, Path messages
  uam_util               # Utility classes
  rclcpp_components      # Component registration
)

# Create navigator library (state machine)
add_library(navigator SHARED
  src/navigator.cpp     # Main state machine implementation
)

# Link against navigator_modes library
# Navigator depends on modes (Layer 2 → Layer 1)
target_link_libraries(navigator navigator_modes)

# Link against ROS2 dependencies
ament_target_dependencies(navigator ${navigator_dependencies})

# Set include directories for navigator library
target_include_directories(navigator PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

# ==============================================================================
# SECTION 5: REGISTER AS COMPOSABLE COMPONENT
# ==============================================================================

# Register Navigator class as a composable ROS2 component
# This allows navigator to be loaded into a component container at runtime
#
# What this does:
# - Generates component registration code
# - Adds navigator to component registry
# - Enables dynamic loading via composition API
#
# Usage:
# 1. Standalone: Run navigator_main executable
# 2. Composed: Load via `ros2 component load` or launch file
#
# Composition benefits:
# - Multiple nodes in one process (lower overhead)
# - Intra-process communication (zero-copy)
# - Dynamic node loading/unloading
rclcpp_components_register_nodes(navigator "uam_navigator::Navigator")

# ==============================================================================
# SECTION 6: BUILD EXECUTABLE (Layer 3)
# ==============================================================================

# Create standalone executable
# Thin wrapper that instantiates navigator component
add_executable(navigator_main src/navigator_main.cpp)

# Link against dependencies
ament_target_dependencies(navigator_main ${navigator_dependencies})

# Set include directories for executable
target_include_directories(navigator_main PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

# Link against navigator library
# Executable → navigator → navigator_modes (dependency chain)
target_link_libraries(navigator_main navigator)

# Install executable to lib/<package_name>/
# Standard location for ROS2 executables
install(TARGETS navigator_main DESTINATION lib/${PROJECT_NAME})

# ==============================================================================
# SECTION 7: INSTALL LIBRARIES
# ==============================================================================

# Install both libraries
# These can be linked by other packages or loaded as components
install(TARGETS navigator navigator_modes
  ARCHIVE DESTINATION lib    # Static libraries (.a)
  LIBRARY DESTINATION lib    # Shared libraries (.so) - USED
  RUNTIME DESTINATION bin    # Windows DLLs (not used on Linux)
)

# ==============================================================================
# SECTION 8: INSTALL HEADERS
# ==============================================================================

# Install all header files to install/include/
# Allows other packages to #include our headers
# Pattern: include/ directory contains subdirectory structure
install(DIRECTORY include/
  DESTINATION include/
)

# ==============================================================================
# SECTION 9: EXPORT PACKAGE INFORMATION
# ==============================================================================

# Export include directories for downstream packages
# Other packages depending on us will automatically get these includes
ament_export_include_directories(include)

# Export navigator library
# Other packages can link against libnavigator.so
# Note: navigator_modes is not exported (internal implementation detail)
ament_export_libraries(navigator)

# Export dependencies (transitive dependencies)
# Packages depending on us will automatically depend on our dependencies
ament_export_dependencies(${navigator_dependencies})

# Finalize package
ament_package()
