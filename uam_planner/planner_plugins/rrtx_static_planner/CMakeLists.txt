# ==============================================================================
# RRT* Static Planner Plugin - CMake Build Configuration
# ==============================================================================
#
# This CMakeLists.txt builds the RRT* planner as a ROS2 PLUGIN, not a standalone
# executable. The plugin is dynamically loaded at runtime by the planner server.
#
# Key Concepts:
# -------------
#
# 1. PLUGIN ARCHITECTURE
# ----------------------
# Instead of hardcoding planners into the planner server executable, we use
# pluginlib to enable dynamic loading of planner algorithms at runtime.
#
# Benefits:
# - Add new planners without recompiling planner server
# - Switch planners via configuration files (no code changes)
# - Distribute planners in separate packages
# - Easy A/B testing of different algorithms
#
# Plugin Flow:
#   CMake Build → Shared Library (.so) → Plugin XML → Runtime Loading
#
# 2. SHARED LIBRARY vs EXECUTABLE
# --------------------------------
# This package builds a SHARED LIBRARY (.so file), not an executable.
#
# Why shared library?
# - Plugins must be dynamically loadable at runtime
# - Multiple plugins can exist in the same process
# - Reduces memory footprint (code shared across instances)
#
# File output:
#   - Executable: /install/lib/package_name/executable_name
#   - Shared lib: /install/lib/librrtx_static.so
#
# 3. PLUGINLIB MECHANISM
# ----------------------
# pluginlib uses XML files to describe available plugins:
#
# Plugin XML (rrtx_static_planner_plugin.xml):
# ```xml
# <library path="librrtx_static">
#   <class type="rrtx_static_planner::RrtxStatic"
#          base_class_type="uam_planner::PlannerBase">
#     <description>RRT* planner for static environments</description>
#   </class>
# </library>
# ```
#
# At runtime:
# 1. Planner server calls pluginlib::ClassLoader
# 2. ClassLoader reads XML to find available plugins
# 3. dlopen() loads the .so file
# 4. Factory creates instance of RrtxStatic class
#
# 4. DEPENDENCY MANAGEMENT
# ------------------------
# Two types of dependencies:
#
# a) ament dependencies (ROS2 packages):
#    - Automatically sets include paths, link libraries, compile flags
#    - Use ament_target_dependencies() - it handles everything
#
# b) Non-ament dependencies (system libraries):
#    - Manually find with find_package()
#    - Manually link with target_link_libraries()
#    - Example: OMPL (not a ROS2 package)
#
# ==============================================================================

# Minimum CMake version (ROS2 Humble requirement)
cmake_minimum_required(VERSION 3.8)

# Project name - becomes ROS2 package name
project(rrtx_static_planner)

# ==============================================================================
# SECTION 1: FIND DEPENDENCIES
# ==============================================================================
# Find all required packages before building.
# find_package() searches CMake's search paths for package configuration files.

# --- ROS2 Core ---
find_package(ament_cmake REQUIRED)  # ROS2 build system (required for all ROS2 packages)

# --- ROS2 Libraries ---
find_package(rclcpp REQUIRED)       # ROS2 C++ client library (nodes, timers, publishers, etc.)
find_package(pluginlib REQUIRED)    # Plugin loading system (dynamic library loading)

# --- Math & Geometry ---
find_package(eigen3_cmake_module REQUIRED)  # Helper for finding Eigen3 in ROS2
find_package(Eigen3 REQUIRED)               # Linear algebra library (matrices, vectors, quaternions)
find_package(geometric_shapes REQUIRED)     # Collision detection primitives (Box, Sphere, Cylinder)

# --- Planning Libraries ---
find_package(ompl REQUIRED)                 # Open Motion Planning Library (RRT*, PRM*, etc.)
                                            # Note: OMPL is a system library, NOT a ROS2 package

# --- Custom Messages ---
find_package(uam_mapping_msgs REQUIRED)     # Custom messages for obstacle representation

# --- Base Plugin Interface ---
find_package(uam_planner REQUIRED)          # Provides PlannerBase abstract class
                                            # Our plugin inherits from this interface

# ==============================================================================
# SECTION 2: INCLUDE DIRECTORIES
# ==============================================================================
# Tell compiler where to find header files (.hpp, .h)

include_directories(
  include                    # Our headers: include/rrtx_static_planner/rrtx_static.hpp
  ${OMPL_INCLUDE_DIRS}       # OMPL headers (non-ament package, must add manually)
)

# Note: ament packages (rclcpp, Eigen3, etc.) automatically add their includes
# via ament_target_dependencies() later. No need to manually include them here.

# ==============================================================================
# SECTION 3: DEFINE LIBRARY NAME AND DEPENDENCIES
# ==============================================================================

# Library name (without 'lib' prefix and '.so' suffix)
# CMake will create: librrtx_static.so
set(library_name rrtx_static)

# List of ROS2 (ament) dependencies
# These will be handled by ament_target_dependencies()
set(dependencies
  rclcpp              # ROS2 C++ API
  Eigen3              # Linear algebra
  geometric_shapes    # Collision primitives
  uam_mapping_msgs    # Obstacle messages
  uam_planner         # PlannerBase interface
  pluginlib           # Plugin system
)

# ==============================================================================
# SECTION 4: BUILD SHARED LIBRARY
# ==============================================================================

# Create shared library (.so file) from source code
# SHARED = dynamic library (vs STATIC = archive)
add_library(${library_name} SHARED
  src/rrtx_static.cpp          # Implementation file
)

# Link against OMPL libraries
# OMPL is not an ament package, so we must manually link it
# ${OMPL_LIBRARIES} expands to: -lompl
target_link_libraries(${library_name} ${OMPL_LIBRARIES})

# Link against all ROS2 (ament) dependencies
# This single command:
# - Adds include directories for all dependencies
# - Links against dependency libraries
# - Adds compiler flags from dependencies
# - Sets up dependency ordering
ament_target_dependencies(${library_name}
  ${dependencies}
)

# ==============================================================================
# SECTION 5: EXPORT PLUGIN DESCRIPTION
# ==============================================================================

# Register this plugin with pluginlib
# Arguments:
#   1. Base class package name: uam_planner (provides PlannerBase)
#   2. Plugin XML file: rrtx_static_planner_plugin.xml (describes our plugin)
#
# What this does:
# - Installs plugin XML to share directory
# - Adds XML to ament index (makes it discoverable)
# - Allows ClassLoader to find plugin at runtime
#
# Runtime discovery:
# 1. Planner server calls pluginlib::ClassLoader<uam_planner::PlannerBase>
# 2. ClassLoader searches ament index for "uam_planner" plugins
# 3. Finds our XML in share/rrtx_static_planner/
# 4. Reads XML to learn about RrtxStatic class
# 5. Loads librrtx_static.so and instantiates RrtxStatic
pluginlib_export_plugin_description_file(uam_planner rrtx_static_planner_plugin.xml)

# ==============================================================================
# SECTION 6: INSTALL RULES
# ==============================================================================
# Specify which files to copy to install directory during `colcon build`

# Install compiled library to lib directory
# The library will be copied to: install/lib/librrtx_static.so
install(TARGETS ${library_name}
  ARCHIVE DESTINATION lib    # Static libraries (.a) - not used for SHARED
  LIBRARY DESTINATION lib    # Shared libraries (.so) - THIS IS USED
  RUNTIME DESTINATION bin    # Executables - not used for libraries
)

# Install plugin XML to share directory
# Allows pluginlib to find plugin description at runtime
# Destination: install/share/rrtx_static_planner/rrtx_static_planner_plugin.xml
install(FILES rrtx_static_planner_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

# ==============================================================================
# SECTION 7: EXPORT PACKAGE INFORMATION
# ==============================================================================
# Export information so other packages can use this package

# Export include directories
# Other packages that depend on us will automatically get these includes
ament_export_include_directories(include ${OMPL_INCLUDE_DIRS})

# Export our library
# Other packages can link against librrtx_static.so
ament_export_libraries(${library_name})

# Export dependencies
# Packages depending on us will automatically depend on our dependencies
# (transitive dependencies)
ament_export_dependencies(${dependencies})

# Finalize package configuration
# Must be last command in CMakeLists.txt
# Generates ament index files, package.xml processing, etc.
ament_package()
