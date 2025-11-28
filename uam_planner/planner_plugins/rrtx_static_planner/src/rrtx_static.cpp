/**
 * @file rrtx_static.cpp
 * @brief RRT*-static Path Planning Plugin for UAV Navigation
 *
 * This file implements the RRT*-static algorithm for computing collision-free
 * paths in 3D space using the Open Motion Planning Library (OMPL).
 *
 * Algorithm: RRT*-static (Rapidly-exploring Random Tree Star - Static Environment)
 * ---------------------------------------------------------------------------------
 *
 * RRT* is an asymptotically optimal sampling-based motion planning algorithm that
 * incrementally builds a tree of feasible trajectories by random sampling and
 * tree rewiring to minimize path cost.
 *
 * Key Mathematical Foundation:
 * ---------------------------
 *
 * Configuration Space: C = SE(3) = ℝ³ × SO(3)
 *   - Position: p ∈ ℝ³ (3D Euclidean space)
 *   - Orientation: q ∈ SO(3) (Special Orthogonal group, rotations)
 *   - Currently: Orientation fixed to identity (position-only planning)
 *
 * Cost Function: c(path) = ∫ ||ṗ(s)|| ds (path length)
 *
 * Optimality Guarantee (Karaman & Frazzoli 2011):
 *   lim_{n→∞} P(c(path_n) = c*) = 1
 *   where n = number of samples, c* = optimal path cost
 *
 * RRT* Algorithm:
 * ---------------
 * 1. Initialize tree T with start state x_init
 * 2. For i = 1 to n iterations:
 *    a. Sample random state x_rand ~ Uniform(C_free)
 *    b. Find nearest node in tree: x_near = argmin_{x∈T} d(x, x_rand)
 *    c. Steer toward sample: x_new = Steer(x_near, x_rand, η)
 *    d. Check collision: if ¬CollisionFree(x_near, x_new) continue
 *    e. Find nearby nodes: X_near = Near(T, x_new, r_n)
 *    f. Choose best parent:
 *       x_min = argmin_{x∈X_near} Cost(x) + c(Line(x, x_new))
 *    g. Add x_new to tree with parent x_min
 *    h. Rewire tree (optimize nearby connections):
 *       for x_near ∈ X_near:
 *         if Cost(x_new) + c(Line(x_new, x_near)) < Cost(x_near):
 *           Update parent of x_near to x_new
 *    i. If x_new connects to goal region, update best path
 * 3. Return best path found
 *
 * Near radius calculation: r_n = min(γ(log(n)/n)^(1/d), η)
 *   where d = dimension (3 for position), γ = scaling constant
 *
 * Key References:
 * ---------------
 * 1. Karaman, S., & Frazzoli, E. (2011). "Sampling-based algorithms for optimal
 *    motion planning." The International Journal of Robotics Research, 30(7), 846-894.
 *    DOI: 10.1177/0278364911406761
 *    - Proves asymptotic optimality of RRT*
 *    - Defines near radius and convergence rate
 *
 * 2. LaValle, S. M. (2006). "Planning Algorithms." Cambridge University Press.
 *    URL: http://lavalle.pl/planning/
 *    - Chapter 5: Sampling-based motion planning foundations
 *
 * 3. Şucan, I. A., Moll, M., & Kavraki, L. E. (2012). "The Open Motion Planning
 *    Library." IEEE Robotics & Automation Magazine, 19(4), 72-82.
 *    DOI: 10.1109/MRA.2012.2205651
 *    - OMPL implementation details
 *
 * Implementation Notes:
 * ---------------------
 * - Uses OMPL's geometric::RRTXstatic planner (variant optimized for static obstacles)
 * - Collision checking via geometric_shapes library (Box primitives)
 * - Discrete motion validation (edges checked at 1cm intervals)
 * - Obstacle inflation for safety margins (configurable scaling factor)
 * - Coordinate frame handling: ENU (map) and NED (map_ned) supported
 */

#include "rrtx_static_planner/rrtx_static.hpp"
#include "ompl/base/spaces/SE3StateSpace.h"


namespace rrtx_static_planner
{

/**
 * @brief Default constructor for RRT* planner plugin
 */
RrtxStatic::RrtxStatic()
{
	// Empty - initialization done in configure()
}

/**
 * @brief Destructor - logs cleanup for debugging
 */
RrtxStatic::~RrtxStatic() noexcept
{
	RCLCPP_INFO(
			logger_, "Destroying plugin %s of type RrtxStatic",
			name_.c_str());
}

/**
 * @brief Configure the RRT* planner plugin
 *
 * This method performs one-time initialization:
 * 1. Loads parameters (bounds, obstacle scaling, solve time)
 * 2. Sets up obstacle subscription for dynamic updates
 * 3. Initializes OMPL state space and planner
 *
 * @param parent Weak pointer to lifecycle node for accessing ROS2 functionality
 * @param name Plugin name for parameter namespacing
 *
 * OMPL Configuration Chain:
 * -------------------------
 * StateSpace → SpaceInformation → ProblemDefinition → Planner
 *
 * StateSpace (SE3):
 *   Defines the configuration space C = ℝ³ × SO(3)
 *   Sets bounds on position subspace: [x_min, x_max] × [y_min, y_max] × [z_min, z_max]
 *
 * SpaceInformation:
 *   Associates state space with validity checker and motion validator
 *   - Validity Checker: Determines if single state is collision-free
 *   - Motion Validator: Checks if edge between states is collision-free
 *
 * ProblemDefinition:
 *   Holds start state, goal state, and optimization objective
 *
 * Planner (RRTXstatic):
 *   The actual sampling-based algorithm implementation
 */
void RrtxStatic::configure(
		const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent,
		std::string name)
{
	// Store references to parent node
	name_ = name;
	global_frame_ = "map";  // Planning happens in ENU frame
	node_ = parent;
	auto node = parent.lock();
	clock_ = node->get_clock();
	logger_ = node->get_logger();

	RCLCPP_INFO(
			logger_, "Configuring plugin %s of type RrtxStatic",
			name_.c_str());

	// ============================================================================
	// STEP 1: Load Parameters
	// ============================================================================

	try {
		// Obstacle Scaling Factor:
		// Inflates obstacle dimensions by this factor for safety margin
		// Example: scaling=2.0 doubles obstacle size (adds 50% clearance on each side)
		// Higher values = safer but may make narrow passages impassable
		node->declare_parameter(name + ".obstacle_scaling", rclcpp::ParameterValue(1.3));
		node->get_parameter(name + ".obstacle_scaling", obstacle_scaling_);

		// Planning Bounds (3D bounding box):
		// Restricts sampling region to prevent planning in unreachable areas
		// Format: [x_min, y_min, z_min] and [x_max, y_max, z_max] in meters
		// Frame: ENU (East-North-Up)
		node->declare_parameter(name + ".bounds_low",
		                        rclcpp::ParameterValue(std::vector<double>RRTX_ARENA_BOUNDS_LOW_DEFAULT));
		node->get_parameter(name + ".bounds_low", planner_map_bounds_low_);
		node->declare_parameter(name + ".bounds_high",
		                        rclcpp::ParameterValue(std::vector<double>RRTX_ARENA_BOUNDS_HIGH_DEFAULT));
		node->get_parameter(name + ".bounds_high", planner_map_bounds_high_);

		// Solve Time (seconds):
		// Maximum time allowed for planning before timeout
		// RRT* is an anytime algorithm: longer time → better path quality
		// Typical values: 0.5-2.0 seconds for UAV navigation
		node->declare_parameter(name + ".solve_time", rclcpp::ParameterValue((double)RRTX_PLANNER_MAX_SOLVE_TIME_S));
		node->get_parameter(name + ".solve_time", planner_solve_time_);
	} catch (const rclcpp::ParameterTypeException & ex) {
		RCLCPP_ERROR(node->get_logger(), "Plugin RrtxStatic parameter type exception:  %s", ex.what());
	}


	// ============================================================================
	// STEP 2: Subscribe to Obstacle Updates
	// ============================================================================

	/**
	 * Obstacle Subscription Callback
	 *
	 * Maintains dynamic obstacle map for collision checking. The geometric_shapes
	 * library provides efficient collision detection primitives.
	 *
	 * Collision Detection Background:
	 * -------------------------------
	 * Uses Gilbert-Johnson-Keerthi (GJK) algorithm for convex collision detection:
	 *
	 * GJK Algorithm (Gilbert et al. 1988):
	 *   Determines if two convex shapes intersect by iteratively constructing the
	 *   Minkowski difference support function until either:
	 *   1. Origin is found inside (collision detected)
	 *   2. Simplex cannot contain origin (no collision)
	 *
	 * Computational Complexity: O(k) where k = iterations (typically < 10)
	 *
	 * Reference:
	 * Gilbert, E. G., Johnson, D. W., & Keerthi, S. S. (1988).
	 * "A fast procedure for computing the distance between complex objects
	 * in three-dimensional space." IEEE Journal on Robotics and Automation, 4(2).
	 * DOI: 10.1109/56.2083
	 */
	obstacle_sub_ = node->create_subscription<uam_mapping_msgs::msg::ObstacleArray>(
			"uam_mapping/obstacles", 10,
			[this](const uam_mapping_msgs::msg::ObstacleArray::UniquePtr msg) {
				// Remove obstacles no longer in the scene
				// (Handles dynamic obstacle removal - though currently all obstacles are static)
				for (const auto& obstacle : obstacles_) {
					if (std::find(std::begin(msg->obstacle_ids), std::end(msg->obstacle_ids), obstacle.first) == std::end(msg->obstacle_ids))
						obstacles_.erase(obstacle.first);
				}

				// Add or update obstacles
				for (const auto& msg_obstacle : msg->obstacles) {
					bodies::BodyPtr body;
					shapes::ShapeConstPtr shape;

					switch (msg_obstacle.obstacle.type) {
						case shape_msgs::msg::SolidPrimitive::BOX:
							// Create Box collision primitive
							body = std::make_shared<bodies::Box>();

							// Construct SE(3) pose: T = Translation · Rotation
							// Isometry3d represents rigid body transformation in ℝ³
							const Eigen::Isometry3d pose =
									Eigen::Translation3d(
											msg_obstacle.pose.position.x,
											msg_obstacle.pose.position.y,
											msg_obstacle.pose.position.z)
									* Eigen::Quaterniond(
											msg_obstacle.pose.orientation.w,
											msg_obstacle.pose.orientation.x,
											msg_obstacle.pose.orientation.y,
											msg_obstacle.pose.orientation.z);

							// Box dimensions: [length_x, length_y, length_z]
							shape = std::make_shared<const shapes::Box>(
									msg_obstacle.obstacle.dimensions[shape_msgs::msg::SolidPrimitive::BOX_X],
									msg_obstacle.obstacle.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Y],
									msg_obstacle.obstacle.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Z]);

							// Apply safety scaling (obstacle inflation)
							// Example: scaling=2.0 → obstacle appears 2x larger
							// This provides safety margin for planning
							body->setScaleDirty(obstacle_scaling_);
							body->setPoseDirty(pose);
							body->setDimensionsDirty(shape.get());

							// Recompute internal collision detection structures
							body->updateInternalData();
							break;

						// TODO: Add SPHERE and CYLINDER support
						// Currently only BOX primitives are fully implemented
					}

					// Update existing obstacle or insert new one
					if (auto search = obstacles_.find(msg_obstacle.obstacle_id); search != obstacles_.end()) {
							search->second = body;  // Update existing
					} else {
						obstacles_.insert({msg_obstacle.obstacle_id, body});  // Add new
					}
				}
			});

	// ============================================================================
	// STEP 3: Initialize OMPL State Space and Planner
	// ============================================================================

	// Disable OMPL console output (avoid spam in logs)
	ompl::msg::noOutputHandler();

	// Create SE(3) state space: C = ℝ³ × SO(3)
	// SE(3) = Special Euclidean group in 3D (rigid body transformations)
	//       = {(p, R) | p ∈ ℝ³, R ∈ SO(3)}
	state_space_ptr_ = std::make_shared<ompl::base::SE3StateSpace>();

	// Set 3D position bounds (restricts sampling region)
	// Bounds define rectangular prism: [x_min, x_max] × [y_min, y_max] × [z_min, z_max]
	ompl::base::RealVectorBounds bounds(3);
	bounds.setLow(0, planner_map_bounds_low_[0]);   // x_min
	bounds.setHigh(0, planner_map_bounds_high_[0]); // x_max
	bounds.setLow(1, planner_map_bounds_low_[1]);   // y_min
	bounds.setHigh(1, planner_map_bounds_high_[1]); // y_max
	bounds.setLow(2, planner_map_bounds_low_[2]);   // z_min
	bounds.setHigh(2, planner_map_bounds_high_[2]); // z_max
	state_space_ptr_->as<ompl::base::SE3StateSpace>()->setBounds(bounds);

	// Valid Segment Count Factor:
	// Controls edge discretization for collision checking
	// Higher values = more samples along edge = more accurate but slower
	// Formula: num_segments = factor * distance / max_extent
	// Example: For 1m edge with factor=100, checks ~100 points
	state_space_ptr_->setValidSegmentCountFactor(100);

	// Create SpaceInformation (associates state space with validity checking)
	space_info_ptr_ = std::make_shared<ompl::base::SpaceInformation>(state_space_ptr_);

	// Set state validity checker (point collision detection)
	// Lambda calls isStateValid() for each sampled state
	space_info_ptr_->setStateValidityChecker([this](const ompl::base::State *state)
	                                         {
		                                         return isStateValid(state);
	                                         });

	// Set motion validator (edge collision detection)
	// DiscreteMotionValidator checks edges by discretizing into small segments
	// Each segment checked using state validity checker
	space_info_ptr_->setMotionValidator(std::make_shared<ompl::base::DiscreteMotionValidator>(space_info_ptr_));

	// State validity checking resolution:
	// Maximum distance between collision checks along an edge
	// Resolution = 0.01 means checks every 1cm
	// Lower values = more checks = slower but safer
	space_info_ptr_->setStateValidityCheckingResolution(0.01);

	// Create problem definition (will hold start/goal states)
	prob_def_ptr_ = std::make_shared<ompl::base::ProblemDefinition>(space_info_ptr_);

	// Instantiate RRT*-static planner
	// RRTXstatic is optimized for static environments (obstacles don't move)
	// Alternative planners available: RRTConnect, PRM*, BIT*, etc.
	planner_ptr_ = std::make_shared<ompl::geometric::RRTXstatic>(space_info_ptr_);
	planner_ptr_->setProblemDefinition(prob_def_ptr_);
}

/**
 * @brief Activate the RRT* planner plugin
 *
 * Lifecycle callback when node transitions to active state.
 * All initialization is done in configure(), so this is currently a no-op.
 *
 * ROS2 Lifecycle States:
 * ----------------------
 * Unconfigured → configure() → Inactive → activate() → Active
 *
 * This design allows:
 * - Resource allocation in configure() (one-time setup)
 * - Runtime start/stop without reallocation (activate/deactivate)
 * - Clean shutdown (cleanup)
 */
void RrtxStatic::activate()
{
	RCLCPP_INFO(
			logger_, "Activating plugin %s of type RrtxStatic",
			name_.c_str());
}

/**
 * @brief Deactivate the RRT* planner plugin
 *
 * Lifecycle callback when node transitions from active to inactive state.
 * Planning requests will not be processed when inactive.
 *
 * Note: Resources (OMPL objects, subscribers) remain allocated.
 * Use cleanup() for full resource deallocation.
 */
void RrtxStatic::deactivate()
{
	RCLCPP_INFO(
			logger_, "Deactivating plugin %s of type RrtxStatic",
			name_.c_str());
}

/**
 * @brief Clean up the RRT* planner plugin and release all resources
 *
 * Lifecycle callback when node transitions to finalized state.
 * Deallocates all OMPL data structures and resets shared pointers.
 *
 * Resource Cleanup Order:
 * -----------------------
 * 1. State space (SE3StateSpace) - defines configuration space
 * 2. Space information (validity checker, motion validator)
 * 3. Problem definition (start/goal states, solutions)
 * 4. Planner (RRT* algorithm instance)
 *
 * Memory Management:
 * All members use shared_ptr, so reset() decrements reference count.
 * Actual deallocation occurs when reference count reaches zero.
 */
void RrtxStatic::cleanup()
{
	RCLCPP_INFO(
			logger_, "Cleaning up plugin %s of type RrtxStatic",
			name_.c_str());
	state_space_ptr_.reset();
	space_info_ptr_.reset();
	prob_def_ptr_.reset();
	planner_ptr_.reset();

}

/**
 * @brief Compute collision-free path from start to goal using RRT*
 *
 * This is the main planning interface called by the planner server.
 * Executes the RRT* algorithm to find an asymptotically optimal path.
 *
 * @param start Starting pose (position + orientation)
 * @param goal Goal pose (position + orientation)
 * @return nav_msgs::msg::Path Sequence of waypoints from start to goal
 * @throws uam_planner::NoValidPathCouldBeFound if planning fails
 *
 * Algorithm Execution Flow:
 * -------------------------
 * 1. Clear previous planning problem (start, goal, solution, tree)
 * 2. Convert ROS poses to OMPL states with frame transformation
 * 3. Set start and goal states in problem definition
 * 4. Execute RRT* planning for specified solve time
 * 5. Extract solution path and convert back to ROS format
 *
 * Coordinate Frame Transformations:
 * ---------------------------------
 * The planner operates in ENU (East-North-Up) frame internally,
 * but supports input/output in two coordinate systems:
 *
 * 1. ENU (map frame):
 *    - X: East, Y: North, Z: Up
 *    - Right-handed coordinate system
 *    - Common in ROS navigation and geographic applications
 *    - Transform: (x, y, z)_ENU → (x, y, z)_planner (identity)
 *
 * 2. NED (map_ned frame):
 *    - X: North, Y: East, Z: Down
 *    - Right-handed coordinate system
 *    - Common in aerospace and PX4 autopilots
 *    - Transform: (x, y, z)_NED → (y, x, -z)_ENU
 *
 * Mathematical Representation:
 *   ENU: p_ENU = [x_east, y_north, z_up]^T
 *   NED: p_NED = [x_north, y_east, z_down]^T
 *   Conversion: p_ENU = [p_NED[1], p_NED[0], -p_NED[2]]^T
 *
 * Reference:
 * International Organization for Standardization (ISO). (2010).
 * "ISO 8855:2011 - Road vehicles — Vehicle dynamics and road-holding ability —
 * Vocabulary." Defines coordinate system conventions for vehicle dynamics.
 *
 * Planner Status Codes:
 * ---------------------
 * - EXACT_SOLUTION: Goal reached within tolerance, path found
 * - APPROXIMATE_SOLUTION: Closest state to goal found (goal unreachable)
 * - TIMEOUT: Planning time exceeded, no solution found
 * - INVALID_START: Start state in collision or out of bounds
 * - INVALID_GOAL: Goal state in collision or out of bounds
 * - CRASH: Internal planner error
 *
 * Note: Currently accepts APPROXIMATE_SOLUTION as valid.
 * TODO: Add tolerance threshold for approximate solutions
 */
nav_msgs::msg::Path RrtxStatic::createPath(
		const geometry_msgs::msg::PoseStamped & start,
        const geometry_msgs::msg::PoseStamped & goal)
{
	// ============================================================================
	// STEP 1: Clear Previous Planning Problem
	// ============================================================================
	// RRT* builds a search tree incrementally. Must clear previous tree
	// to avoid reusing stale data from different start/goal configurations.
	prob_def_ptr_->clearStartStates();     // Remove previous start state
	prob_def_ptr_->clearGoal();            // Remove previous goal state
	prob_def_ptr_->clearSolutionPaths();   // Remove previous solution paths
	planner_ptr_->clear();                 // Clear search tree and internal data structures

	// ============================================================================
	// STEP 2: Convert Start Pose to OMPL State
	// ============================================================================
	// ScopedState automatically manages memory for OMPL state
	// SE3StateSpace::StateType = (position: ℝ³, rotation: SO(3))
	ompl::base::ScopedState<ompl::base::SE3StateSpace> start_state(state_space_ptr_);

	// Handle coordinate frame transformation for start pose
	if (start.header.frame_id == "map") {
		// ENU frame: Direct mapping (identity transform)
		start_state->setXYZ(
				start.pose.position.x,
				start.pose.position.y,
				start.pose.position.z);
	} else if (start.header.frame_id == "map_ned") {
		// NED frame: Apply NED→ENU transformation
		// (North, East, Down) → (East, North, Up)
		start_state->setXYZ(
				start.pose.position.y,   // East = NED_y
				start.pose.position.x,   // North = NED_x
				-start.pose.position.z); // Up = -NED_z
	}

	// Set rotation to identity (orientation-free planning)
	// Current implementation only plans for position (x, y, z)
	// Orientation is fixed at identity quaternion: (w=1, x=0, y=0, z=0)
	start_state->rotation().setIdentity();

	// ============================================================================
	// STEP 3: Convert Goal Pose to OMPL State
	// ============================================================================
	ompl::base::ScopedState<ompl::base::SE3StateSpace> goal_state(state_space_ptr_);

	// Handle coordinate frame transformation for goal pose
	if (goal.header.frame_id == "map") {
		// ENU frame: Direct mapping
		goal_state->setXYZ(
				goal.pose.position.x,
				goal.pose.position.y,
				goal.pose.position.z);
	} else if (goal.header.frame_id == "map_ned") {
		// NED frame: Apply NED→ENU transformation
		goal_state->setXYZ(
				goal.pose.position.y,
				goal.pose.position.x,
				-goal.pose.position.z);
	}

	// Set rotation to identity (orientation-free planning)
	goal_state->rotation().setIdentity();

	// ============================================================================
	// STEP 4: Configure Problem and Execute RRT* Planning
	// ============================================================================

	// Set start and goal in problem definition
	prob_def_ptr_->setStartAndGoalStates(start_state, goal_state);

	// Setup planner (initializes internal data structures)
	// Checks validity of start/goal states, initializes nearest-neighbor data structure
	planner_ptr_->setup();

	// Debug output (disabled by default)
	// Uncomment to print state space settings and problem definition
	//	space_info_ptr_->printSettings(std::cout);
	//	prob_def_ptr_->print(std::cout);

	// Execute RRT* planning algorithm
	// Returns status: EXACT_SOLUTION, APPROXIMATE_SOLUTION, TIMEOUT, etc.
	// solve() blocks until solution found or timeout reached
	auto planner_status = planner_ptr_->ompl::base::Planner::solve(planner_solve_time_);

	// Debug output (disabled by default)
	//	prob_def_ptr_->print(std::cout);

	// ============================================================================
	// STEP 5: Validate Planning Result
	// ============================================================================
	// TODO: add more exception checking
	// - Check if start/goal are in collision
	// - Add tolerance threshold for approximate solutions
	// - Log planning statistics (tree size, iterations, path cost)
	if (planner_status != ompl::base::PlannerStatus::EXACT_SOLUTION &&
		planner_status != ompl::base::PlannerStatus::APPROXIMATE_SOLUTION) {
		throw uam_planner::NoValidPathCouldBeFound("Failed to create a valid path");
	}

	// ============================================================================
	// STEP 6: Extract Solution Path and Convert to ROS Format
	// ============================================================================
	nav_msgs::msg::Path path;

	path.poses.clear();
	path.header.stamp = clock_->now();
	path.header.frame_id = global_frame_;  // Output in ENU ("map") frame

	// Cast solution to geometric path (sequence of SE3 states)
	auto path_ptr = prob_def_ptr_->getSolutionPath()->as<ompl::geometric::PathGeometric>();

	// Convert each OMPL state to ROS PoseStamped message
	for(size_t i = 0; i < path_ptr->getStateCount(); i++) {
		auto state = path_ptr->getState(i)->as<ompl::base::SE3StateSpace::StateType>();
		geometry_msgs::msg::PoseStamped pose;
		pose.header.frame_id = global_frame_;

		// Extract position from SE3 state (already in ENU frame)
		pose.pose.position.x = state->getX();
		pose.pose.position.y = state->getY();
		pose.pose.position.z = state->getZ();

		// Note: Orientation is not set (defaults to identity quaternion)
		// Controller is responsible for computing heading between waypoints

		path.poses.push_back(pose);
	}

	return path;
}

/**
 * @brief Check if a state is valid (collision-free and within bounds)
 *
 * This function is called by OMPL during:
 * 1. Random state sampling (validity check before adding to tree)
 * 2. Edge validation (discretized motion checking between states)
 * 3. Start/goal validation (before planning begins)
 *
 * @param state OMPL state to validate (SE3: position + rotation)
 * @return true if state is valid (no collision and within bounds)
 * @return false if state is invalid (collision detected or out of bounds)
 *
 * Collision Detection Theory:
 * ---------------------------
 * Point-in-convex-shape test using geometric_shapes library.
 *
 * For Box primitives:
 * A point p ∈ ℝ³ is inside an axis-aligned box B if:
 *   p_min ≤ p ≤ p_max  (component-wise)
 *
 * For oriented (rotated) boxes:
 * 1. Transform point to box's local frame: p_local = R^T(p - t)
 *    where R ∈ SO(3) is rotation matrix, t ∈ ℝ³ is translation
 * 2. Apply axis-aligned test in local frame
 *
 * Mathematical Formulation:
 *   Let B = {p | ||R^T(p - t)||_∞ ≤ d/2}
 *   where d = [d_x, d_y, d_z]^T are box dimensions
 *   ||·||_∞ is the infinity norm (max absolute component)
 *
 * Computational Complexity: O(n) where n = number of obstacles
 *   - Each containsPoint() call: O(1) for convex shapes
 *   - Total: Linear scan through obstacle list
 *
 * Optimization Opportunities:
 * - Spatial hashing for large obstacle counts
 * - Bounding volume hierarchies (BVH) for complex scenes
 * - Early termination on first collision (already implemented via |=)
 *
 * Reference:
 * Ericson, C. (2004). "Real-Time Collision Detection."
 * Morgan Kaufmann. Chapter 4: Bounding Volumes.
 * ISBN: 978-1558607323
 *
 * Bounds Checking:
 * ----------------
 * Verifies state satisfies configuration space bounds:
 *   x_min ≤ x ≤ x_max
 *   y_min ≤ y ≤ y_max
 *   z_min ≤ z ≤ z_max
 *
 * These bounds were set in configure() to restrict sampling region.
 */
bool RrtxStatic::isStateValid(const ompl::base::State *state) const
{
	// Cast generic OMPL state to SE3 state type
	// SE3StateSpace::StateType contains:
	//   - Position: (x, y, z) ∈ ℝ³
	//   - Rotation: quaternion (w, x, y, z) ∈ SO(3)
	const auto *se3state = state->as<ompl::base::SE3StateSpace::StateType>();

	// Extract 3D position as Eigen vector for collision checking
	// Note: Rotation is ignored (orientation-free collision checking)
	Eigen::Vector3d node(se3state->getX(), se3state->getY(), se3state->getZ());

	// Variable for collision status (unused variable corner1 removed)
	bool collision = false;

	// Check collision with all obstacles in the scene
	// Iterates through obstacle map: {obstacle_id → Body (collision geometry)}
	for (const auto& obstacle : obstacles_) {
		// containsPoint() performs point-in-shape test
		// Returns true if point p is inside the obstacle
		// Uses |= operator for early termination: once collision=true, stays true
		collision |= obstacle.second->containsPoint(node);
	}

	// State is valid if BOTH conditions are met:
	// 1. Within configuration space bounds (bounding box check)
	// 2. No collision with any obstacle (collision = false)
	return (space_info_ptr_->satisfiesBounds(se3state) && (!collision));
}

} // namespace rrtx_static_planner

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(rrtx_static_planner::RrtxStatic, uam_planner::PlannerBase)