/**
 * @file obstacle_advertiser.cpp
 * @brief Static Obstacle Publisher for UAV Navigation Environment
 *
 * This node publishes static obstacle information for path planning and
 * collision avoidance. Obstacles are configured via ROS2 parameters and
 * published periodically for consumption by the planning system.
 *
 * Obstacle Representation:
 * ------------------------
 *
 * Obstacles are represented as 3D geometric primitives:
 * - Type: BOX (rectangular prism)
 * - Dimensions: [length_x, length_y, length_z] in meters
 * - Pose: Position (x, y, z) and orientation (quaternion)
 * - Frame: "map" (global ENU coordinate system)
 *
 * Current Configuration:
 * ----------------------
 * All obstacles are identical boxes representing vertical posts/pillars:
 * - Width (X): 0.4191 m (~16.5 inches)
 * - Depth (Y): 0.4191 m (~16.5 inches)
 * - Height (Z): 0.9779 m (~38.5 inches)
 *
 * These dimensions likely represent:
 * - Indoor vertical obstacles (columns, posts)
 * - Urban infrastructure (bollards, signposts)
 * - Test environment markers
 *
 * Obstacle Positioning:
 * ---------------------
 * - X, Y coordinates: Loaded from ROS2 parameters
 * - Z coordinate: Automatically set to half-height (obstacle sits on ground)
 * - Orientation: Identity (axis-aligned boxes)
 *
 * Mathematical Representation:
 * ----------------------------
 * Each obstacle occupies a 3D volume in configuration space:
 *
 *   Obstacle_i = {p ∈ ℝ³ | ||p - c_i||_∞ ≤ d_i/2}
 *
 * Where:
 * - p = point in 3D space
 * - c_i = obstacle center position
 * - d_i = [d_x, d_y, d_z]^T = obstacle dimensions
 * - ||·||_∞ = infinity norm (max absolute component)
 *
 * For axis-aligned box:
 *   c_i.x - d_x/2 ≤ p.x ≤ c_i.x + d_x/2
 *   c_i.y - d_y/2 ≤ p.y ≤ c_i.y + d_y/2
 *   c_i.z - d_z/2 ≤ p.z ≤ c_i.z + d_z/2
 *
 * Configuration Space Obstacle (C-obstacle):
 * ------------------------------------------
 * For a point robot (UAV treated as point mass):
 *   C-obs_i = Obstacle_i ⊕ Robot_radius
 *
 * The planner implements obstacle inflation via scaling factor:
 *   Inflated_obstacle = scale_factor × Obstacle_dimensions
 *
 * This effectively creates a safety margin around obstacles.
 *
 * Parameter Configuration:
 * ------------------------
 * Obstacles are defined via three parallel arrays in ROS2 parameters:
 *
 * obstacle_ids:  ["1", "2", "3", ...]  (unique identifiers)
 * obstacles_x:   [x1, x2, x3, ...]     (X positions in map frame)
 * obstacles_y:   [y1, y2, y3, ...]     (Y positions in map frame)
 *
 * Example YAML configuration:
 * ```yaml
 * static_obstacle_advertiser:
 *   ros__parameters:
 *     obstacle_ids: ["1", "2", "3"]
 *     obstacles_x: [5.0, 10.0, 15.0]
 *     obstacles_y: [5.0, 10.0, 5.0]
 * ```
 *
 * This creates 3 obstacles at:
 * - Obstacle 1: (5.0, 5.0) m
 * - Obstacle 2: (10.0, 10.0) m
 * - Obstacle 3: (15.0, 5.0) m
 *
 * Update Rate:
 * ------------
 * Obstacles are published at 10 Hz (every 100ms).
 *
 * Why periodic publishing for static obstacles?
 * - Late-joining subscribers: Nodes that start after launch receive obstacle data
 * - Message reliability: Compensates for potential message loss in best-effort QoS
 * - State synchronization: Ensures all planning instances have consistent obstacle data
 *
 * For truly static environments, a latched topic (QoS transient_local) would be
 * more efficient, but periodic publishing is simpler and more robust.
 *
 * Integration with Planning:
 * --------------------------
 * The RRT* planner subscribes to this topic and:
 * 1. Receives obstacle array messages
 * 2. Converts to geometric_shapes::Body collision primitives
 * 3. Applies obstacle scaling for safety margins
 * 4. Uses for collision checking during planning
 *
 * Data Flow:
 * ----------
 * Parameters (YAML) → ObstacleAdvertiser → /uam_mapping/obstacles
 *                                         → RRT* Planner → Collision Checker
 *
 * Future Enhancements:
 * --------------------
 * - Support for dynamic obstacles (moving obstacles)
 * - Multiple primitive types (cylinders, spheres)
 * - Obstacle orientation (non-axis-aligned boxes)
 * - Obstacle metadata (type, material, hazard level)
 * - Octomap integration for dense 3D environment representation
 *
 * References:
 * -----------
 * 1. LaValle, S. M. (2006). "Planning Algorithms." Cambridge University Press.
 *    Chapter 3.2: Configuration Space Obstacles.
 *    URL: http://lavalle.pl/planning/
 *
 * 2. Lozano-Pérez, T. (1983). "Spatial Planning: A Configuration Space Approach."
 *    IEEE Transactions on Computers, C-32(2), 108-120.
 *    DOI: 10.1109/TC.1983.1676196
 *    (Foundational paper on C-space obstacle representation)
 */

#include <uam_mapping/obstacle_advertiser.hpp>

using namespace uam_mapping;
using namespace std::chrono;

/**
 * @brief Constructor - Initialize obstacle advertiser node
 *
 * Loads obstacle configuration from ROS2 parameters and sets up
 * periodic publishing timer.
 *
 * Initialization Sequence:
 * ------------------------
 * 1. Create obstacle array publisher
 * 2. Declare and load parameters
 * 3. Validate parameter consistency
 * 4. Build obstacle map from parameters
 * 5. Start periodic publishing timer
 *
 * Parameter Validation:
 * ---------------------
 * Ensures all parameter arrays have matching lengths:
 *   length(obstacle_ids) = length(obstacles_x) = length(obstacles_y)
 *
 * If lengths mismatch, logs error and continues with empty obstacle map
 * (no obstacles will be published).
 *
 * Data Structure:
 * ---------------
 * Obstacles stored in map: obstacle_id → (x, y) position
 *   std::unordered_map<std::string, std::pair<double, double>>
 *
 * This allows:
 * - O(1) lookup by ID
 * - Efficient iteration for publishing
 * - Future support for obstacle updates/removal by ID
 */
ObstacleAdvertiser::ObstacleAdvertiser() : Node("static_obstacle_advertiser")
{
	// ============================================================================
	// STEP 1: Create Publisher for Obstacle Array
	// ============================================================================

	// Publisher for obstacle information consumed by path planners
	// Topic: /uam_mapping/obstacles
	// Message: ObstacleArray (contains multiple Obstacle messages)
	// QoS: Default (reliable, volatile, depth 10)
	obstacle_array_pub_ =
			this->create_publisher<uam_mapping_msgs::msg::ObstacleArray>("uam_mapping/obstacles", 10);

	// ============================================================================
	// STEP 2: Declare ROS2 Parameters for Obstacle Configuration
	// ============================================================================

	// Three parallel arrays define obstacles:
	// - obstacle_ids: Unique string identifiers for each obstacle
	// - obstacles_x: X coordinates in map frame (East in ENU)
	// - obstacles_y: Y coordinates in map frame (North in ENU)
	//
	// Note: Z coordinates not parameterized - automatically calculated
	// from obstacle height (obstacles sit on ground plane z=0)
	declare_parameter("obstacle_ids", rclcpp::ParameterValue(std::vector<std::string>()));
	declare_parameter("obstacles_x", rclcpp::ParameterValue(std::vector<double>()));
	declare_parameter("obstacles_y", rclcpp::ParameterValue(std::vector<double>()));

	// ============================================================================
	// STEP 3: Retrieve Parameter Values
	// ============================================================================

	// Load parameters from ROS2 parameter server
	// Parameters may come from:
	// - Launch file arguments
	// - YAML configuration files
	// - Command-line arguments (--ros-args -p ...)
	get_parameter("obstacle_ids", obstacle_ids_);
	get_parameter("obstacles_x", obstacles_x_);
	get_parameter("obstacles_y", obstacles_y_);

	// ============================================================================
	// STEP 4: Validate Parameters and Build Obstacle Map
	// ============================================================================

	// Ensure all parameter arrays have consistent lengths
	// Array lengths must match for proper obstacle construction
	if (obstacle_ids_.size() == obstacles_x_.size() && obstacles_x_.size() == obstacles_y_.size()){
		// Build obstacle map from parameter arrays
		// Maps obstacle ID (string) → (x, y) position pair
		for (size_t i = 0; i < obstacle_ids_.size(); i++) {
			obstacles_map_.insert({obstacle_ids_[i], std::make_pair(obstacles_x_[i], obstacles_y_[i])});
		}

		RCLCPP_INFO(get_logger(), "Loaded %zu obstacles from parameters", obstacle_ids_.size());
	} else {
		// Parameter array length mismatch - cannot build obstacle map
		// Log error and continue with empty map (no obstacles published)
		RCLCPP_ERROR(get_logger(), "Invalid size of obstacle parameter vectors");
		RCLCPP_ERROR(get_logger(), "  obstacle_ids: %zu elements", obstacle_ids_.size());
		RCLCPP_ERROR(get_logger(), "  obstacles_x:  %zu elements", obstacles_x_.size());
		RCLCPP_ERROR(get_logger(), "  obstacles_y:  %zu elements", obstacles_y_.size());
	}

	// ============================================================================
	// STEP 5: Start Periodic Publishing Timer
	// ============================================================================

	// Lambda callback for timer - calls publishObstacles() every 100ms
	auto timer_callback = [this]() -> void
	{
		publishObstacles();
	};

	// Create wall timer at 10 Hz (100ms period)
	// Wall timer uses real-time clock (not affected by simulation time)
	timer_ = this->create_wall_timer(100ms, timer_callback);

	RCLCPP_INFO(get_logger(), "Obstacle advertiser initialized - publishing at 10 Hz");
}

/**
 * @brief Publish obstacle array to topic
 *
 * Constructs and publishes ObstacleArray message containing all configured
 * obstacles with their geometric properties and poses.
 *
 * Called at 10 Hz by wall timer.
 *
 * Message Construction:
 * ---------------------
 * For each obstacle in obstacles_map_:
 * 1. Create Obstacle message
 * 2. Set primitive type (BOX)
 * 3. Set dimensions (width, depth, height)
 * 4. Set pose (position from parameters, orientation = identity)
 * 5. Add to obstacle array
 *
 * Geometric Properties:
 * ---------------------
 * All obstacles are currently identical boxes:
 * - Type: BOX (rectangular prism)
 * - Dimensions: [0.4191, 0.4191, 0.9779] meters
 *   - X (width):  0.4191 m (16.5 inches)
 *   - Y (depth):  0.4191 m (16.5 inches)
 *   - Z (height): 0.9779 m (38.5 inches)
 *
 * Pose Convention:
 * ----------------
 * - Position: Center of obstacle
 * - X, Y: From parameters (horizontal position)
 * - Z: height/2 (obstacle sits on ground, extends upward)
 * - Orientation: Identity quaternion (axis-aligned)
 *
 * Coordinate Frame:
 * -----------------
 * - Frame ID: "map" (global ENU frame)
 * - All obstacle positions relative to map origin
 *
 * Performance Notes:
 * ------------------
 * - Message construction time: O(n) where n = number of obstacles
 * - Typical n: 5-50 obstacles for test environments
 * - Publication rate: 10 Hz (100ms period)
 * - Message size: ~100 bytes per obstacle + header
 */
void ObstacleAdvertiser::publishObstacles()
{
	// Create obstacle array message
	uam_mapping_msgs::msg::ObstacleArray obstacles;

	// Set message header
	obstacles.header.frame_id = "map";  // Global ENU coordinate frame
	obstacles.header.stamp = this->get_clock()->now();

	// ============================================================================
	// Build Obstacle Array from Map
	// ============================================================================

	// Iterate through obstacle map and construct individual obstacle messages
	for (auto const & obstacle : obstacles_map_) {
		uam_mapping_msgs::msg::Obstacle obstacle_msg;

		// -----------------------------------------------------------------------
		// Set Geometric Primitive Type
		// -----------------------------------------------------------------------
		// shape_msgs supports: BOX, SPHERE, CYLINDER, CONE
		// Currently only BOX primitives are used
		obstacle_msg.obstacle.type = shape_msgs::msg::SolidPrimitive::BOX;

		// -----------------------------------------------------------------------
		// Set Obstacle ID
		// -----------------------------------------------------------------------
		// Convert string ID to integer for message
		// String IDs used in parameters for readability
		// Integer IDs used in messages for efficiency and planner indexing
		obstacle_msg.obstacle_id = std::stoi(obstacle.first);

		// -----------------------------------------------------------------------
		// Set Obstacle Dimensions
		// -----------------------------------------------------------------------
		// BOX dimensions: [size_x, size_y, size_z] in meters
		// Currently all obstacles have identical dimensions
		// TODO: Make dimensions configurable per-obstacle via parameters
		obstacle_msg.obstacle.dimensions = {0.4191, 0.4191, 0.9779};

		// -----------------------------------------------------------------------
		// Set Obstacle Position
		// -----------------------------------------------------------------------
		// X, Y from parameters (horizontal position in map frame)
		obstacle_msg.pose.position.x = obstacle.second.first;   // X (East)
		obstacle_msg.pose.position.y = obstacle.second.second;  // Y (North)

		// Z calculated as half-height (obstacle center, sits on ground)
		// Ground plane at z=0, obstacle extends from 0 to height
		// Pose represents center, so z = height/2
		obstacle_msg.pose.position.z = obstacle_msg.obstacle.dimensions[2] / 2.0;

		// Orientation defaults to identity (axis-aligned box)
		// pose.orientation.w = 1, x = 0, y = 0, z = 0 (no rotation)

		// -----------------------------------------------------------------------
		// Add to Obstacle Array
		// -----------------------------------------------------------------------
		obstacles.obstacles.push_back(obstacle_msg);
	}

	// ============================================================================
	// Publish Obstacle Array
	// ============================================================================

	// Publish to /uam_mapping/obstacles topic
	// Consumed by path planners (RRT*, PRM, etc.) for collision checking
	obstacle_array_pub_->publish(obstacles);
}

int main(int argc, char *argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<uam_mapping::ObstacleAdvertiser>());
	rclcpp::shutdown();
	return 0;
}