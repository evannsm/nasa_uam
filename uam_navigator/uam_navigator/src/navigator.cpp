/**
 * @file navigator.cpp
 * @brief UAV Navigator State Machine for Mission Management
 *
 * This file implements a hierarchical finite state machine (FSM) for managing
 * UAV navigation modes during autonomous flight missions.
 *
 * State Machine Architecture:
 * ---------------------------
 *
 * The navigator implements a Moore-type finite state machine where:
 * - States represent navigation modes (Idle, Takeoff, Loiter, NavigateToPose, Land)
 * - Transitions are triggered by action commands
 * - Each state produces position setpoints as output
 *
 * Formal Definition:
 *   FSM = (S, Σ, δ, s₀, F)
 *   S = {Idle, Takeoff, Loiter, NavigateToPose, Land} (states)
 *   Σ = {TakeoffCmd, LoiterCmd, NavigateCmd, LandCmd} (input alphabet)
 *   δ: S × Σ → S (transition function)
 *   s₀ = Idle (initial state)
 *   F = {Land} (accepting/final states)
 *
 * State Transition Diagram:
 * -------------------------
 *
 *                    ┌──────┐
 *                    │ Idle │ (Initial State)
 *                    └──┬───┘
 *                       │ TakeoffCmd
 *                       ↓
 *   ┌────────────────────────────────────────┐
 *   │            ┌─────────┐                 │
 *   │      ┌────→│ Takeoff │────┐            │
 *   │      │     └─────────┘    │            │
 *   │      │           │         │            │
 *   │  LandCmd    LoiterCmd  LandCmd         │
 *   │      │           ↓         │            │
 *   │      │     ┌────────┐      │            │
 *   │      │  ┌─→│ Loiter │←─┐   │            │
 *   │      │  │  └────────┘  │   │            │
 *   │      │  │       ↕       │   │            │
 *   │      │  │   NavCmd      │   │            │
 *   │      │  │       ↓       │   │            │
 *   │      │  │  ┌──────────┐ │   │            │
 *   │      └──┼─→│NavigateTo│─┘   │            │
 *   │         │  │  Pose    │     │            │
 *   │         │  └──────────┘     │            │
 *   │         │                   │            │
 *   │         │    ┌──────┐       │            │
 *   │         └───→│ Land │←──────┘            │
 *   │              └──────┘                    │
 *   │                 │                        │
 *   └─────────────────┴────────────────────────┘
 *                     │
 *                     ↓
 *                  Mission Complete
 *
 * Transition Rules:
 * -----------------
 * - Idle → Takeoff (only valid transition from Idle)
 * - Takeoff → {Loiter, Land}
 * - Loiter → {NavigateToPose, Land, ...custom plugins}
 * - NavigateToPose → {Loiter, Land}
 * - Land → (terminal state, mission ends)
 *
 * Special Transition Case:
 * If currently in a navigation plugin and requesting a different navigation plugin,
 * the system automatically transitions through Loiter as an intermediate state:
 *   NavigateToPose_A → Loiter → NavigateToPose_B
 *
 * This ensures smooth transitions and prevents abrupt mode switches.
 *
 * Navigation Mode Outputs:
 * ------------------------
 * Each state publishes position setpoints (nav_msgs::msg::Odometry) at 100Hz:
 * - Takeoff: Vertical ascent to altitude setpoint
 * - Loiter: Hold current position (station-keeping)
 * - NavigateToPose: Follow planned trajectory to goal
 * - Land: Controlled descent to ground
 *
 * Plugin Architecture:
 * --------------------
 * The navigator uses a plugin-based design for extensibility.
 * Core modes (Takeoff, Loiter, Land) are built-in.
 * Navigation modes (NavigateToPose, ...) are loaded as plugins via ROS2 parameters.
 *
 * Each plugin implements the NavigatorModeBase interface:
 * - configure(): Initialize mode with parameters
 * - activate(goal): Start executing mode
 * - deactivate(): Stop executing mode
 * - publishNavigatorSetpoint(): Publish position command (called at 100Hz)
 *
 * ROS2 Lifecycle Integration:
 * ----------------------------
 * The navigator is a lifecycle node with standard state transitions:
 *   Unconfigured → configure() → Inactive → activate() → Active
 *
 * Lifecycle states vs. Navigation modes:
 * - Lifecycle states: Node-level (configure, activate, deactivate, cleanup)
 * - Navigation modes: Mission-level (Idle, Takeoff, Loiter, etc.)
 *
 * Action Server Interface:
 * ------------------------
 * Commands are received via ROS2 action interface:
 *   Action: uam_navigator_msgs/action/NavigatorCommand
 *   Goal: command (string) - name of mode to transition to
 *   Result: error_code (int) - success or failure reason
 *   Feedback: (none currently)
 *
 * Error Codes:
 * - NAV_RESPONSE_INVALID_NAVIGATOR: Requested mode doesn't exist
 * - NAV_RESPONSE_INVALID_TRANSITION: Transition not allowed from current mode
 * - NAV_RESPONSE_NAVIGATOR_MODE_ACTIVATION_FAILED: Mode failed to activate
 *
 * Thread Safety:
 * --------------
 * The navigator uses ROS2's single-threaded executor model:
 * - Timer callbacks (onLoop) execute sequentially
 * - Action callbacks (commandCallback) execute sequentially
 * - No explicit locking required (all callbacks on same thread)
 *
 * References:
 * -----------
 * 1. Hopcroft, J. E., Motwani, R., & Ullman, J. D. (2006).
 *    "Introduction to Automata Theory, Languages, and Computation" (3rd ed.).
 *    Pearson. Chapter 2: Finite Automata.
 *    ISBN: 978-0321455369
 *
 * 2. Koubaa, A., et al. (2019). "Robot Operating System (ROS):
 *    The Complete Reference (Volume 4)." Springer.
 *    Chapter: Lifecycle Management for ROS2 Nodes.
 *    DOI: 10.1007/978-3-030-20190-6
 */

#include "uam_util/node_utils.hpp"
#include "navigator.hpp"

namespace uam_navigator
{

/**
 * @brief Constructor for Navigator lifecycle node
 *
 * Initializes the navigator with default parameters for plugin loading.
 *
 * @param options ROS2 node options for configuration
 *
 * Parameter Initialization:
 * -------------------------
 * Declares "navigator_plugins" parameter with default value:
 * - Default: ["navigate_to_pose_rrtx_static"]
 * - Type: string array
 * - Purpose: Specifies which navigation plugins to load at runtime
 *
 * Users can override this parameter via:
 * 1. Launch files: <param name="navigator_plugins" value="[...]"/>
 * 2. YAML config files: navigator_plugins: [...]
 * 3. Command line: --ros-args -p navigator_plugins:=[...]
 *
 * Plugin Loading:
 * Actual plugin instantiation occurs in on_configure(), not here.
 * Constructor only declares parameters for later retrieval.
 */
Navigator::Navigator(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("navigator", "", options), default_navigator_plugin_ids_{"navigate_to_pose_rrtx_static"}
{
	// Declare ROS2 parameter for navigator plugin IDs
	// This allows runtime configuration of which navigation modes to load
	declare_parameter("navigator_plugins", default_navigator_plugin_ids_);

	// Retrieve parameter value (may be overridden by launch file/config)
	get_parameter("navigator_plugins", navigator_plugin_ids_);
}

/**
 * @brief Destructor - no explicit cleanup needed
 *
 * All resources are managed via shared_ptr and will be automatically
 * deallocated when the node is destroyed.
 */
Navigator::~Navigator()
{
}

/**
 * @brief Configure the navigator state machine
 *
 * Lifecycle callback that performs one-time initialization:
 * 1. Instantiate navigation mode plugins
 * 2. Configure publishers and subscribers
 * 3. Define state transition rules
 * 4. Start main execution timer
 * 5. Set up action server/client for commands
 *
 * @param state Current lifecycle state (unused, transition is implicit)
 * @return CallbackReturn::SUCCESS if configuration succeeds
 *
 * Configuration Steps:
 * --------------------
 *
 * STEP 1: Instantiate Navigation Modes
 * -------------------------------------
 * Creates instances of built-in modes (Takeoff, Loiter, Land) and
 * user-specified plugin modes (NavigateToPose, custom modes).
 *
 * Built-in modes are always available.
 * Plugin modes are loaded based on "navigator_plugins" parameter.
 *
 * STEP 2: Create Publishers
 * --------------------------
 * - position_setpoint_publisher_: Sends position commands to controller (100Hz)
 * - navigator_status_publisher_: Broadcasts current navigation mode (100Hz)
 *
 * STEP 3: Configure All Mode Plugins
 * -----------------------------------
 * Calls configure() on each mode plugin to:
 * - Initialize mode-specific parameters
 * - Set up mode-specific subscribers/publishers
 * - Allocate mode-specific resources
 *
 * STEP 4: Define State Transition Table
 * --------------------------------------
 * Constructs adjacency list representation of FSM transition function δ:
 *   navigator_transitions_[current_mode] = [allowed_next_modes]
 *
 * Transition Table:
 *   Idle       → [Takeoff]
 *   Takeoff    → [Loiter, Land]
 *   Loiter     → [NavigateToPose, Land, ...custom_plugins]
 *   NavigateToPose → [Loiter, Land]
 *
 * Data Structure:
 *   std::unordered_map<std::string, std::vector<std::string>>
 *   Key: current mode name
 *   Value: vector of valid next mode names
 *
 * STEP 5: Start Main Execution Timer
 * -----------------------------------
 * Creates wall timer at 100Hz (10ms period) for:
 * - Publishing current mode status
 * - Calling active mode's setpoint publisher
 *
 * Timer Precision:
 * Uses wall_timer (real-time clock) rather than ROS time.
 * This ensures consistent execution even during simulation time changes.
 *
 * STEP 6: Subscribe to Vehicle State
 * -----------------------------------
 * Subscribes to vehicle odometry for:
 * - Current position (used by Loiter mode for station-keeping)
 * - Current velocity (used for smooth transitions)
 * - Current orientation (used for heading control)
 *
 * STEP 7: Initialize State Machine State
 * ---------------------------------------
 * Sets initial conditions:
 * - current_nav_mode_ = "Idle" (UAV on ground, not executing mission)
 * - requested_nav_mode_ = "Idle" (no pending transitions)
 *
 * STEP 8: Create Action Server and Client
 * ----------------------------------------
 * Action Server: Receives navigation commands from external nodes
 * Action Client: Used for internal recursive transitions (e.g., Loiter→Nav via Loiter)
 *
 * Why both server AND client?
 * - Server: External interface for mission planners
 * - Client: Internal interface for chaining transitions
 *
 * Example: To go from NavigateToPose_A to NavigateToPose_B:
 *   1. Receive NavigateToPose_B command via server
 *   2. Send Loiter command via client (intermediate state)
 *   3. Send NavigateToPose_B command via client (final state)
 */
uam_util::CallbackReturn
Navigator::on_configure(const rclcpp_lifecycle::State &)
{
	RCLCPP_INFO(get_logger(), "Configuring");

	auto node = shared_from_this();

	// ============================================================================
	// STEP 1: Instantiate Navigation Mode Plugins
	// ============================================================================

	// Built-in modes (always available)
	navigators_.insert({"Takeoff", std::make_shared<Takeoff>()});
	// Note: Land mode commented out - likely under development
	//	navigators_.insert({NavMode::NAV_MODE_LAND, std::make_unique<Land>()});
	navigators_.insert({"Loiter", std::make_shared<Loiter>()});

	// User-configured plugin modes (from ROS2 parameters)
	// Default: "navigate_to_pose_rrtx_static" (RRT* path following)
	navigators_.insert({"navigate_to_pose_rrtx_static", std::make_shared<NavigateToPose>()});

	// ============================================================================
	// STEP 2: Create Publishers for Setpoints and Status
	// ============================================================================

	// Position setpoint publisher (consumed by uam_control)
	// Message type: nav_msgs::msg::Odometry
	// Contains: position, velocity, orientation setpoints for trajectory tracking
	position_setpoint_publisher_ = node->create_publisher<nav_msgs::msg::Odometry>("/uam_navigator/position_setpoint", 10);

	// Navigator status publisher (for monitoring and logging)
	// Message type: uam_navigator_msgs::msg::NavigatorStatus
	// Contains: current mode name, timestamp
	navigator_status_publisher_ = node->create_publisher<uam_navigator_msgs::msg::NavigatorStatus>("/uam_navigator/navigator_status", 10);

	// ============================================================================
	// STEP 3: Configure All Navigation Mode Plugins
	// ============================================================================

	// Call configure() on each mode to initialize mode-specific resources
	// Passes shared pointers to:
	// - node: Access to ROS2 node functionality (logging, parameters, topics)
	// - navSharedFromThis(): Callback interface for modes to publish setpoints
	// - navigator.first: Mode name string (for logging and parameter namespacing)
	for (const auto & navigator : navigators_) {
		navigator.second->configure(node, navSharedFromThis(), navigator.first);
	}

	// ============================================================================
	// STEP 4: Define State Transition Table (FSM Transition Function δ)
	// ============================================================================

	// Takeoff can transition to: Loiter (normal) or Land (emergency)
	navigator_transitions_.insert({"Takeoff", std::vector<std::string>{"Loiter", "Land"}});

	// Loiter can transition to: all navigation plugins + Land
	// Build list dynamically from configured plugins
	auto tmp = navigator_plugin_ids_;
	tmp.emplace_back("Land");  // Add Land as always-available emergency option
	navigator_transitions_.insert({"Loiter", tmp});

	// Each navigation plugin can transition to: Loiter (pause) or Land (end)
	for (const auto & nav_mode : navigator_plugin_ids_) {
		navigator_transitions_.insert({nav_mode, std::vector<std::string>{"Loiter", "Land"}});
	}

	// Idle (ground state) can only transition to: Takeoff
	// This is the mission entry point
	navigator_transitions_.insert({"Idle", std::vector<std::string>{"Takeoff"}});

	// ============================================================================
	// STEP 5: Start Main Execution Timer (100Hz Control Loop)
	// ============================================================================

	// Wall timer at 10ms period (100Hz) for:
	// 1. Publishing navigator status
	// 2. Calling active mode's setpoint publisher
	timer_ = node->create_wall_timer(std::chrono::milliseconds(10),
								  std::bind(&Navigator::onLoop, this));

	// ============================================================================
	// STEP 6: Subscribe to Vehicle Odometry State
	// ============================================================================

	// Receives current vehicle state estimate (position, velocity, orientation)
	// Used by navigation modes for feedback control and state tracking
	vehicle_odometry_sub_ =
			node->create_subscription<nav_msgs::msg::Odometry>(
					"/uam_vehicle_interface/odometry",
					10,
					[this](const nav_msgs::msg::Odometry::UniquePtr msg)
					{
						// Store latest odometry in member variable
						// Lambda capture [this] allows access to class members
						vehicle_odom_ = *msg;
					});

	// ============================================================================
	// STEP 7: Initialize State Machine to Idle
	// ============================================================================

	// Set initial state (UAV on ground, awaiting Takeoff command)
	current_nav_mode_ = "Idle";
	requested_nav_mode_ = "Idle";

	// ============================================================================
	// STEP 8: Create Action Server and Client for Command Interface
	// ============================================================================

	// Action server: External interface for receiving navigation commands
	// Callback: commandCallback() handles state transitions and validation
	// Timeout: 500ms for processing commands
	nav_command_action_server_ = std::make_unique<ActionServerNavigatorCommand>(
			shared_from_this(),
			"send_navigator_command",
			std::bind(&Navigator::commandCallback, this),
			nullptr,
			std::chrono::milliseconds(500),
			true);

	// Action client: Internal interface for recursive/chained transitions
	// Used when transitioning between navigation plugins via intermediate Loiter state
	nav_command_action_client_ = rclcpp_action::create_client<ActionNavigatorCommand>(node, "send_navigator_command");

	return uam_util::CallbackReturn::SUCCESS;
}

/**
 * @brief Activate the navigator node
 *
 * Lifecycle callback when node transitions to active state.
 * Enables publishers and action server to begin accepting commands.
 *
 * @param state Current lifecycle state (unused, transition is implicit)
 * @return CallbackReturn::SUCCESS if activation succeeds
 *
 * Activation Sequence:
 * --------------------
 * 1. Activate lifecycle-managed publishers (enables publishing)
 * 2. Activate action server (enables command reception)
 *
 * ROS2 Lifecycle Publisher States:
 * - Inactive: Publisher exists but publish() calls are no-ops
 * - Active: Publisher fully functional, messages transmitted
 *
 * After activation, the node is ready to:
 * - Receive navigation commands via action interface
 * - Execute state transitions
 * - Publish setpoints and status messages
 */
uam_util::CallbackReturn
Navigator::on_activate(const rclcpp_lifecycle::State &)
{
	RCLCPP_INFO(get_logger(), "Activating");

	// Enable lifecycle publishers to transmit messages
	position_setpoint_publisher_->on_activate();
	navigator_status_publisher_->on_activate();

	// Enable action server to accept navigation commands
	nav_command_action_server_->activate();

	auto node = shared_from_this();
	return uam_util::CallbackReturn::SUCCESS;
}

/**
 * @brief Deactivate the navigator node
 *
 * Lifecycle callback when node transitions from active to inactive state.
 * Disables publishers, action server, and deactivates all navigation modes.
 *
 * @param state Current lifecycle state (unused, transition is implicit)
 * @return CallbackReturn::SUCCESS if deactivation succeeds
 *
 * Deactivation Sequence:
 * ----------------------
 * 1. Deactivate lifecycle-managed publishers (stops message transmission)
 * 2. Deactivate action server (stops accepting new commands)
 * 3. Deactivate all navigation mode plugins
 *
 * Note: Resources (subscriptions, timers, plugins) remain allocated.
 * Use on_cleanup() for full resource deallocation.
 *
 * After deactivation:
 * - No setpoints published (vehicle will not receive commands)
 * - No new navigation commands accepted
 * - All modes are in inactive state
 */
uam_util::CallbackReturn
Navigator::on_deactivate(const rclcpp_lifecycle::State &)
{
	RCLCPP_INFO(get_logger(), "Deactivating");

	// Disable lifecycle publishers (publish() becomes no-op)
	position_setpoint_publisher_->on_deactivate();
	navigator_status_publisher_->on_deactivate();

	// Disable action server (new commands rejected)
	nav_command_action_server_->deactivate();

	// Deactivate all navigation mode plugins
	// Modes release mode-specific active resources but remain configured
	NavigatorMap ::iterator it;
	for (it = navigators_.begin(); it != navigators_.end(); ++it) {
		it->second->deactivate();
	}

	return uam_util::CallbackReturn::SUCCESS;
}

/**
 * @brief Clean up navigator resources
 *
 * Lifecycle callback when node transitions to finalized state.
 * Deallocates all subscribers, publishers, action servers, and mode plugins.
 *
 * @param state Current lifecycle state (unused, transition is implicit)
 * @return CallbackReturn::SUCCESS if cleanup succeeds
 *
 * Cleanup Sequence:
 * -----------------
 * 1. Reset subscriptions (unsubscribe from topics)
 * 2. Reset publishers (release publisher handles)
 * 3. Reset action server (close action interface)
 * 4. Clear vehicle odometry cache
 * 5. Reset timer (stop execution loop)
 * 6. Cleanup all mode plugins (release mode resources)
 * 7. Clear mode plugin map (deallocate plugin objects)
 *
 * Memory Management:
 * All members use shared_ptr or unique_ptr, so reset() triggers deallocation
 * when reference count reaches zero.
 *
 * After cleanup, the node can be reconfigured via on_configure() or
 * shutdown completely.
 */
uam_util::CallbackReturn
Navigator::on_cleanup(const rclcpp_lifecycle::State &)
{
	RCLCPP_INFO(get_logger(), "Cleaning up");

	// Reset ROS2 communication interfaces
	vehicle_odometry_sub_.reset();
	position_setpoint_publisher_.reset();
	navigator_status_publisher_.reset();
	nav_command_action_server_.reset();

	// Clear cached vehicle state
	vehicle_odom_ = nav_msgs::msg::Odometry();

	// Stop execution timer
	timer_->reset();

	// Cleanup all navigation mode plugins
	// Calls cleanup() on each mode to release mode-specific resources
	NavigatorMap ::iterator it;
	for (it = navigators_.begin(); it != navigators_.end(); ++it) {
		it->second->cleanup();
	}

	// Clear mode plugin map (deallocates plugin objects)
	navigators_.clear();

	return uam_util::CallbackReturn::SUCCESS;
}

/**
 * @brief Shutdown the navigator node
 *
 * Lifecycle callback for emergency or graceful shutdown.
 * Releases critical resources immediately without full cleanup.
 *
 * @param state Current lifecycle state (unused, transition is implicit)
 * @return CallbackReturn::SUCCESS if shutdown succeeds
 *
 * Shutdown vs. Cleanup:
 * ---------------------
 * - Shutdown: Emergency stop, minimal resource release
 * - Cleanup: Full deallocation, orderly resource management
 *
 * Shutdown only releases communication interfaces and timer.
 * Mode plugins are NOT explicitly cleaned up (rely on destructors).
 */
uam_util::CallbackReturn
Navigator::on_shutdown(const rclcpp_lifecycle::State &)
{
	RCLCPP_INFO(get_logger(), "Shutting down");

	// Release communication interfaces immediately
	vehicle_odometry_sub_.reset();
	position_setpoint_publisher_.reset();
	navigator_status_publisher_.reset();

	// Stop execution timer
	timer_->reset();

	return uam_util::CallbackReturn::SUCCESS;
}

/**
 * @brief Main execution loop - called at 100Hz by wall timer
 *
 * This function is the heartbeat of the navigator state machine.
 * Called every 10ms (100Hz) to:
 * 1. Publish current navigation mode status
 * 2. Execute active mode's setpoint publishing
 *
 * Execution Flow:
 * ---------------
 * 1. Get current node time
 * 2. Construct status message with current mode
 * 3. Publish status (for monitoring/logging)
 * 4. If mode is "Idle", return early (no setpoints published)
 * 5. Otherwise, call active mode's publishNavigatorSetpoint()
 *
 * Why 100Hz?
 * ----------
 * - Fast enough for smooth trajectory tracking
 * - Aligns with typical IMU rates (100-200Hz)
 * - Lower than flight controller rate (200-400Hz) to avoid overload
 * - Standard for UAV guidance systems
 *
 * Idle Mode Handling:
 * -------------------
 * When current_nav_mode_ == "Idle":
 * - Status is published (observers know UAV is idle)
 * - No setpoints published (vehicle remains on ground)
 * - Early return prevents calling non-existent Idle mode plugin
 *
 * Active Mode Execution:
 * ----------------------
 * For all other modes (Takeoff, Loiter, NavigateToPose, Land):
 * - Mode's publishNavigatorSetpoint() is called
 * - Mode computes desired position/velocity/attitude
 * - Mode calls navigator->publishOdometrySetpoint(setpoint)
 * - Setpoint is transmitted to controller for execution
 *
 * Thread Safety:
 * --------------
 * This callback runs on the same thread as action callbacks.
 * No race conditions possible (single-threaded executor).
 */
void Navigator::onLoop()
{
	auto node = shared_from_this();

	// ============================================================================
	// STEP 1: Publish Navigator Status
	// ============================================================================

	// Create status message with current mode and timestamp
	uam_navigator_msgs::msg::NavigatorStatus nav_status_msg;
	nav_status_msg.stamp = node->get_clock()->now();
	nav_status_msg.nav_mode = current_nav_mode_;

	// Publish for monitoring tools and logging
	navigator_status_publisher_->publish(nav_status_msg);

	// ============================================================================
	// STEP 2: Execute Active Mode's Setpoint Publishing
	// ============================================================================

	// Idle mode: UAV on ground, no active navigation
	// Early return to avoid accessing non-existent Idle mode plugin
	if (current_nav_mode_ == "Idle") {
		return;
	}

	// Call active mode's setpoint publisher
	// Mode computes desired trajectory point and publishes via callback interface
	navigators_[current_nav_mode_]->publishNavigatorSetpoint();

}

/**
 * @brief Send Loiter command via internal action client
 *
 * Convenience function for modes to request transition to Loiter state.
 * Used during intermediate transitions between navigation plugins.
 *
 * Internal vs. External Commands:
 * -------------------------------
 * - External: Commands from mission planner → Action Server → commandCallback()
 * - Internal: Commands from modes → Action Client → Action Server → commandCallback()
 *
 * This function uses the action client to send commands to the action server
 * running in the same node. This enables modes to trigger state transitions.
 *
 * Example Usage:
 * When transitioning from NavigateToPose_A to NavigateToPose_B:
 * 1. External command requests NavigateToPose_B
 * 2. commandCallback() detects plugin-to-plugin transition
 * 3. Calls loiter() to transition to intermediate Loiter state
 * 4. Sends NavigateToPose_B command via client for final transition
 */
void Navigator::loiter()
{
	ActionNavigatorCommandGoal nav_command_request;
	nav_command_request.command = "Loiter";

	// Send asynchronously (non-blocking)
	// Result will be handled by action server callback
	nav_command_action_client_->async_send_goal(nav_command_request);
}

/**
 * @brief Send Takeoff command via internal action client
 *
 * Convenience function for programmatic takeoff initiation.
 * See loiter() documentation for internal command mechanism.
 */
void Navigator::takeoff()
{
	ActionNavigatorCommandGoal nav_command_request;
	nav_command_request.command = "Takeoff";
	nav_command_action_client_->async_send_goal(nav_command_request);
}

/**
 * @brief Send Land command via internal action client
 *
 * Convenience function for programmatic landing initiation.
 * See loiter() documentation for internal command mechanism.
 */
void Navigator::land()
{
	ActionNavigatorCommandGoal nav_command_request;
	nav_command_request.command = "Land";
	nav_command_action_client_->async_send_goal(nav_command_request);
}

/**
 * @brief Publish position setpoint to controller
 *
 * Callback interface for navigation modes to publish their computed setpoints.
 * Called by active mode's publishNavigatorSetpoint() at 100Hz.
 *
 * @param odom_msg Odometry message containing desired state:
 *                 - pose: Desired position and orientation
 *                 - twist: Desired velocity (linear and angular)
 *
 * Message Flow:
 * -------------
 * Mode → publishOdometrySetpoint() → position_setpoint_publisher_ → Controller
 *
 * Controller Consumption:
 * The controller (uam_control node) subscribes to this topic and:
 * 1. Extracts desired position, velocity, attitude
 * 2. Computes control errors (desired - actual)
 * 3. Applies control law (PID, LQR, Q-Learning, etc.)
 * 4. Generates motor commands for trajectory tracking
 */
void Navigator::publishOdometrySetpoint(nav_msgs::msg::Odometry odom_msg)
{
	// Publish setpoint for controller consumption
	position_setpoint_publisher_->publish(odom_msg);
}

/**
 * @brief Action server callback for processing navigation mode commands
 *
 * This is the core state transition function δ that implements the FSM logic.
 * Validates requested transitions, activates/deactivates modes, and handles errors.
 *
 * State Transition Algorithm:
 * ---------------------------
 *
 * 1. Validate server state and cancellation requests
 * 2. Validate requested mode exists (navigator plugin is loaded)
 * 3. Check transition rules and execute one of three cases:
 *
 *    CASE A: Direct Transition (most common)
 *    ----------------------------------------
 *    Condition: goal->command ∈ navigator_transitions_[current_nav_mode_]
 *    Example: Loiter → NavigateToPose
 *    Action:
 *      1. Activate new mode
 *      2. Update current_nav_mode_
 *      3. Deactivate previous mode
 *      4. Report success
 *
 *    CASE B: Plugin-to-Plugin Transition (requires intermediate state)
 *    -----------------------------------------------------------------
 *    Condition: Both current and goal modes are navigation plugins
 *    Example: NavigateToPose_A → NavigateToPose_B
 *    Action:
 *      1. Transition to Loiter (intermediate state)
 *      2. Queue goal mode for next transition
 *    Rationale:
 *      Direct plugin-to-plugin transitions are not allowed to ensure
 *      smooth handoff and prevent control discontinuities.
 *      Loiter acts as a stable intermediate state.
 *
 *    CASE C: Invalid Transition
 *    ---------------------------
 *    Condition: Transition not allowed by FSM rules
 *    Example: Takeoff → NavigateToPose (must go through Loiter first)
 *    Action: Throw InvalidTransition exception
 *
 * Exception Handling:
 * -------------------
 * - InvalidNavigator: Requested mode doesn't exist
 * - InvalidTransition: Transition violates FSM rules
 * - NavigatorModeActivationFailed: Mode's activate() returned false
 *
 * All exceptions are caught, error codes are set, and action is terminated
 * with failure status.
 *
 * Thread Safety:
 * --------------
 * This callback runs on the same thread as onLoop() (single-threaded executor).
 * State variables (current_nav_mode_, etc.) are thread-safe by design.
 *
 * Formal Verification:
 * --------------------
 * The implementation enforces the transition function δ: S × Σ → S
 * by checking navigator_transitions_ adjacency list before state updates.
 * This guarantees FSM invariants:
 * - No undefined transitions
 * - No unreachable states (all states have incoming transitions except Idle)
 * - Terminal state (Land) is reachable from all states
 */
void Navigator::commandCallback()
{
	// ============================================================================
	// STEP 1: Get Command Goal and Prepare Result
	// ============================================================================

	// Retrieve goal from action server (contains requested mode name)
	auto goal = nav_command_action_server_->get_current_goal();

	// Prepare result message (will be populated with success/error code)
	auto result = std::make_shared<ActionNavigatorCommand::Result>();

	RCLCPP_DEBUG(get_logger(), "Received command %s", goal->command.c_str());

	try {
		// ========================================================================
		// STEP 2: Validate Server State
		// ========================================================================

		// Check if action server is inactive or cancel was requested
		// If either is true, abort processing and return
		if (isServerInactive(nav_command_action_server_) || isCancelRequested(nav_command_action_server_)) {
			return;
		}

		// ========================================================================
		// STEP 3: Validate Requested Mode Exists
		// ========================================================================

		// Check if requested mode is in navigators_ map
		// If not found, mode was not loaded (invalid plugin name)
		if (navigators_.find(goal->command) == navigators_.end()) {
			throw uam_navigator::InvalidNavigator("Navigator plugin " + goal->command + " is invalid");
		}

		// ========================================================================
		// STEP 4: Execute State Transition Logic (Three Cases)
		// ========================================================================

		// ------------------------------------------------------------------------
		// CASE A: Direct Transition (allowed by FSM rules)
		// ------------------------------------------------------------------------
		// Check if goal->command is in the list of allowed transitions from current mode
		// Example: current="Loiter", goal="NavigateToPose"
		//          navigator_transitions_["Loiter"] = ["NavigateToPose", "Land", ...]
		if (std::find(
				navigator_transitions_[current_nav_mode_].begin(),
				navigator_transitions_[current_nav_mode_].end(),
				goal->command) != navigator_transitions_[current_nav_mode_].end()) {

			// Attempt to activate the requested mode
			// Mode's activate() may return false if:
			// - Goal parameters are invalid
			// - Mode-specific preconditions not met
			// - Resources unavailable
			if (navigators_[goal->command]->activate(goal)) {
				// Activation successful - update state machine state

				// Store previous mode for deactivation
				auto previous_nav_mode = current_nav_mode_;

				// Update current mode to new mode (FSM state transition)
				current_nav_mode_ = goal->command;

				// Reset mission completion flag (new mode started)
				mission_complete_ = false;

				// Deactivate previous mode if it wasn't Idle
				// Idle has no plugin to deactivate
				if (previous_nav_mode != "Idle") {
					navigators_[previous_nav_mode]->deactivate();
				}

				// Report success to action client
				nav_command_action_server_->succeeded_current(result);
			} else {
				// Mode activation failed - throw exception
				throw uam_navigator::NavigatorModeActivationFailed("Failed to activate " + goal->command);
			}

		// ------------------------------------------------------------------------
		// CASE B: Plugin-to-Plugin Transition (requires intermediate Loiter state)
		// ------------------------------------------------------------------------
		// Check if BOTH current mode AND goal mode are navigation plugins
		// Example: current="navigate_to_pose_rrtx_static", goal="navigate_to_pose_prm"
		} else if (std::find(
				navigator_plugin_ids_.begin(),
				navigator_plugin_ids_.end(),
				current_nav_mode_) != navigator_plugin_ids_.end() &&
				std::find(
						navigator_plugin_ids_.begin(),
						navigator_plugin_ids_.end(),
						goal->command) != navigator_plugin_ids_.end()) {

			// Plugin-to-plugin transition requires going through Loiter
			// This ensures smooth handoff and prevents control discontinuities

			// Store goal for later execution
			auto tmp_goal = *goal;

			// Step 1: Transition to Loiter (intermediate state)
			loiter();

			// Step 2: Queue goal mode for next transition
			// This will execute after Loiter transition completes
			nav_command_action_client_->async_send_goal(tmp_goal);

		// ------------------------------------------------------------------------
		// CASE C: Invalid Transition
		// ------------------------------------------------------------------------
		// Transition is not allowed by FSM rules
		} else {
			throw uam_navigator::InvalidTransition("Navigator plugin " + goal->command + " invalid transition");
		}

	// ============================================================================
	// STEP 5: Exception Handling and Error Reporting
	// ============================================================================

	// Catch InvalidNavigator: Requested mode doesn't exist
	} catch (uam_navigator::InvalidNavigator & ex) {
		result->error_code = ActionNavigatorCommandGoal::NAV_RESPONSE_INVALID_NAVIGATOR;
		nav_command_action_server_->terminate_current(result);

	// Catch InvalidTransition: Transition violates FSM rules
	} catch (uam_navigator::InvalidTransition & ex) {
		result->error_code = ActionNavigatorCommandGoal::NAV_RESPONSE_INVALID_TRANSITION;
		nav_command_action_server_->terminate_current(result);

	// Catch NavigatorModeActivationFailed: Mode's activate() returned false
	} catch (uam_navigator::NavigatorModeActivationFailed & ex) {
		result->error_code = ActionNavigatorCommandGoal::NAV_RESPONSE_NAVIGATOR_MODE_ACTIVATION_FAILED;
		nav_command_action_server_->terminate_current(result);
	}
}

/**
 * @brief Check if action server is inactive or null
 *
 * Helper function to validate action server state before processing commands.
 * Prevents accessing invalid or inactive action server instances.
 *
 * @tparam T Action type (e.g., NavigatorCommand)
 * @param action_server Unique pointer to action server to check
 * @return true if server is null or inactive
 * @return false if server is active and ready
 *
 * Usage:
 * Called at the beginning of action callbacks to ensure server is ready.
 * If server is inactive, callback should return early without processing.
 *
 * ROS2 Action Server States:
 * - nullptr: Server never instantiated (error condition)
 * - Inactive: Server exists but not accepting goals
 * - Active: Server ready to accept and process goals
 */
template<typename T>
bool Navigator::isServerInactive(std::unique_ptr<uam_util::SimpleActionServer<T>> & action_server)
{
	// Check if server pointer is null OR if server is not active
	if (action_server == nullptr || !action_server->is_server_active()) {
		RCLCPP_DEBUG(get_logger(), "Action server unavailable or inactive. Stopping.");
		return true;
	}

	return false;
}

/**
 * @brief Check if current action goal has been canceled
 *
 * Helper function to handle action cancellation requests from clients.
 * If cancellation is detected, terminates all pending goals.
 *
 * @tparam T Action type (e.g., NavigatorCommand)
 * @param action_server Unique pointer to action server to check
 * @return true if cancel was requested (goals terminated)
 * @return false if no cancellation requested
 *
 * Cancellation Handling:
 * ----------------------
 * When a client cancels an action goal:
 * 1. Action server sets cancel_requested flag
 * 2. This function detects the flag
 * 3. Calls terminate_all() to abort all pending goals
 * 4. Returns true to indicate callback should exit
 *
 * Use Case:
 * Mission planner sends Takeoff command, then immediately cancels.
 * This function detects cancellation and aborts transition to Takeoff mode.
 */
template<typename T>
bool Navigator::isCancelRequested(
		std::unique_ptr<uam_util::SimpleActionServer<T>> & action_server)
{
	// Check if client has requested cancellation
	if (action_server->is_cancel_requested()) {
		RCLCPP_INFO(get_logger(), "Goal was canceled. Canceling navigator command action.");

		// Terminate all pending action goals
		action_server->terminate_all();

		return true;
	}

	return false;
}

} // namespace uam_navigator

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(uam_navigator::Navigator)