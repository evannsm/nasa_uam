/**
 * @file px4_interface.cpp
 * @brief Interface between ROS2 navigation stack and PX4 autopilot
 *
 * This node serves as a bidirectional translator between:
 * - ROS2 UAM stack (uses ENU frames and ROS conventions)
 * - PX4 autopilot firmware (uses NED frames and MAVLink conventions)
 *
 * Coordinate Frame Transformations:
 * ---------------------------------
 *
 * The interface handles multiple coordinate system transformations:
 *
 * 1. POSITION FRAMES (Translation Coordinate Systems)
 * ===================================================
 *
 * ENU (East-North-Up) - ROS/ROS2 Standard:
 * ----------------------------------------
 * - X-axis: Points East
 * - Y-axis: Points North
 * - Z-axis: Points Up (against gravity)
 * - Right-handed coordinate system
 * - Used by: ROS navigation stack, RVIZ, most robotics applications
 * - Frame name: "map"
 *
 * NED (North-East-Down) - Aerospace/PX4 Standard:
 * -----------------------------------------------
 * - X-axis: Points North
 * - Y-axis: Points East
 * - Z-axis: Points Down (with gravity)
 * - Right-handed coordinate system
 * - Used by: PX4, APM, most autopilots, aviation industry
 * - Frame name: "map_ned"
 *
 * Transformation: NED ↔ ENU
 * -------------------------
 * Position transformation (translation vector):
 *   p_ENU = [p_NED.y, p_NED.x, -p_NED.z]^T
 *   p_ENU.x (East)  = p_NED.y (East)
 *   p_ENU.y (North) = p_NED.x (North)
 *   p_ENU.z (Up)    = -p_NED.z (-Down)
 *
 * Rotation matrix R_NED_to_ENU:
 *   ┌  0  1  0 ┐
 *   │  1  0  0 │
 *   └  0  0 -1 ┘
 *
 * Quaternion q_NED_to_ENU (90° rotation about Z, then 180° about X):
 *   q = [0.7071, 0.7071, 0, 0]  (w, x, y, z)
 *
 * 2. BODY FRAMES (Vehicle-Fixed Coordinate Systems)
 * ==================================================
 *
 * FLU (Forward-Left-Up) - ROS Standard:
 * -------------------------------------
 * - X-axis: Points Forward (vehicle's nose)
 * - Y-axis: Points Left (vehicle's left wing/side)
 * - Z-axis: Points Up (vehicle's top)
 * - Right-handed coordinate system
 * - Frame name: "base_link"
 *
 * FRD (Forward-Right-Down) - Aerospace/PX4 Standard:
 * --------------------------------------------------
 * - X-axis: Points Forward (vehicle's nose)
 * - Y-axis: Points Right (vehicle's right wing/side)
 * - Z-axis: Points Down (vehicle's bottom)
 * - Right-handed coordinate system
 * - Frame name: "base_link_frd"
 *
 * Transformation: FRD ↔ FLU
 * -------------------------
 * Body-frame transformation:
 *   v_FLU = [v_FRD.x, -v_FRD.y, -v_FRD.z]^T
 *   v_FLU.x (Forward) = v_FRD.x (Forward)
 *   v_FLU.y (Left)    = -v_FRD.y (-Right)
 *   v_FLU.z (Up)      = -v_FRD.z (-Down)
 *
 * Rotation matrix R_FRD_to_FLU:
 *   ┌  1  0  0 ┐
 *   │  0 -1  0 │
 *   └  0  0 -1 ┘
 *
 * Quaternion q_FRD_to_FLU (180° rotation about X-axis):
 *   q = [0, 1, 0, 0]  (w, x, y, z)
 *
 * 3. COMPLETE TRANSFORMATION CHAIN
 * =================================
 *
 * PX4 → ROS2 (Telemetry):
 * -----------------------
 * Vehicle state in PX4 (NED/FRD) → Publish in ROS2 (ENU/FLU)
 *
 * Step 1: Position NED → ENU
 *   p_map = R_NED_to_ENU * p_map_ned
 *
 * Step 2: Orientation NED → ENU
 *   q_map = q_NED_to_ENU ⊗ q_map_ned  (quaternion multiplication)
 *
 * Step 3: Body frame FRD → FLU
 *   q_base_link = q_FRD_to_FLU ⊗ q_map
 *
 * ROS2 → PX4 (Commands):
 * ----------------------
 * Control commands in ROS2 (ENU/FLU) → Send to PX4 (NED/FRD)
 * Inverse transformations applied.
 *
 * TF2 Transform Tree:
 * -------------------
 *
 *          map (ENU)
 *          │
 *          ├─── map_ned (NED) [static]
 *          │
 *          └─── base_link (FLU) [dynamic odometry]
 *               │
 *               └─── base_link_frd (FRD) [static]
 *
 * - "map" is the global ENU frame (ROS convention)
 * - "map_ned" is the global NED frame (PX4 convention) - static transform from map
 * - "base_link" is the vehicle FLU frame (ROS convention) - dynamic from odometry
 * - "base_link_frd" is the vehicle FRD frame (PX4 convention) - static from base_link
 *
 * Mathematical Conventions:
 * -------------------------
 * - Quaternions: Hamilton convention (w, x, y, z)
 * - Rotations: Active (vector) rotation, not passive (frame) rotation
 * - Quaternion multiplication: q1 ⊗ q2 (applies q2 first, then q1)
 * - Rotation matrices: Right-multiplication R*v (column vector)
 *
 * PX4 Communication:
 * ------------------
 * This node communicates with PX4 via uXRCE-DDS:
 * - Input topics: /fmu/in/* (commands TO PX4)
 * - Output topics: /fmu/out/* (telemetry FROM PX4)
 * - QoS: Best-effort for most topics (matches PX4 DDS configuration)
 *
 * References:
 * -----------
 * 1. PX4 Coordinate Frames:
 *    https://docs.px4.io/main/en/ros/external_position_estimation.html
 *
 * 2. ROS REP 103 - Standard Units of Measure and Coordinate Conventions:
 *    https://www.ros.org/reps/rep-0103.html
 *
 * 3. Stevens, B. L., & Lewis, F. L. (2015). "Aircraft Control and Simulation"
 *    (3rd ed.). Wiley. Chapter 1: Aircraft Equations of Motion.
 *    ISBN: 978-1118870983
 *
 * 4. Diebel, J. (2006). "Representing Attitude: Euler Angles, Unit Quaternions,
 *    and Rotation Vectors." Stanford University.
 *    https://www.astro.rug.nl/software/kapteyn/_downloads/attitude.pdf
 */

#include "uam_vehicle_interface/px4_interface.hpp"
#include "uam_util/qos_profiles.hpp"

namespace uam_vehicle_interface
{

Px4Interface::Px4Interface()
		: rclcpp::Node("uam_vehicle_interface")
{
	setupParameters();
	// ----------------------- Publishers --------------------------
	vehicle_command_pub_ =
			this->create_publisher<px4_msgs::msg::VehicleCommand>(
					"/fmu/in/vehicle_command", uam_util::px4_qos_pub);
	offboard_control_mode_pub_ =
			this->create_publisher<px4_msgs::msg::OffboardControlMode>(
					"/fmu/in/offboard_control_mode", uam_util::px4_qos_pub);
	vehicle_rates_setpoint_pub_ =
			this->create_publisher<px4_msgs::msg::VehicleRatesSetpoint>(
					"/fmu/in/vehicle_rates_setpoint", uam_util::px4_qos_pub);
	vehicle_attitude_setpoint_pub_ =
			this->create_publisher<px4_msgs::msg::VehicleAttitudeSetpoint>(
					"/fmu/in/vehicle_attitude_setpoint", uam_util::px4_qos_pub);
	vehicle_local_position_setpoint_pub_ =
			this->create_publisher<px4_msgs::msg::VehicleLocalPositionSetpoint>(
					"/fmu/in/vehicle_local_position_setpoint", uam_util::px4_qos_pub);
	vehicle_odometry_pub_ =
			this->create_publisher<nav_msgs::msg::Odometry>(
					"/uam_vehicle_interface/odometry", 10);
	vehicle_interface_status_pub_ =
			this->create_publisher<uam_vehicle_interface_msgs::msg::VehicleInterfaceStatus>(
					"/uam_vehicle_interface/vehicle_interface_status", 10);

	tf_dynamic_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
	tf_static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
	// ----------------------- Subscribers --------------------------
	battery_status_sub_ =
			this->create_subscription<px4_msgs::msg::BatteryStatus>(
					"/fmu/out/battery_status", uam_util::px4_qos_sub,
					[this](const px4_msgs::msg::BatteryStatus::UniquePtr msg)
					{
						battery_status_ = *msg;
					});
	attitude_setpoint_sub_ =
			this->create_subscription<uam_control_msgs::msg::AttitudeSetpoint>(
					"/uam_control/attitude_setpoint",
					10,
					std::bind(&Px4Interface::publishAttitudeSetpoint, this, std::placeholders::_1));
	vehicle_status_sub_ =
			this->create_subscription<px4_msgs::msg::VehicleStatus>(
					"/fmu/out/vehicle_status",
					uam_util::px4_qos_sub,
					[this](const px4_msgs::msg::VehicleStatus::UniquePtr msg)
					{
						vehicle_status_ = *msg;
					});

	vehicle_odometry_sub_ =
			this->create_subscription<px4_msgs::msg::VehicleOdometry>(
					"/fmu/out/vehicle_odometry",
					uam_util::px4_qos_sub,
					std::bind(&Px4Interface::vehicleOdometryCallback, this, std::placeholders::_1));
	channels_sub_ =
			this->create_subscription<px4_msgs::msg::RcChannels>(
					"/fmu/out/rc_channels",
					uam_util::px4_qos_sub,
					[this](const px4_msgs::msg::RcChannels::UniquePtr msg)
					{
						channels_ = *msg;
					});
	vehicle_interface_command_sub_ =
			this->create_subscription<uam_vehicle_interface_msgs::msg::VehicleInterfaceCommand>(
					"/uam_vehicle_interface/vehicle_interface_commands",
					10,
					[this](const uam_vehicle_interface_msgs::msg::VehicleInterfaceCommand::UniquePtr msg)
					{
						switch (msg->command) {
							case uam_vehicle_interface_msgs::msg::VehicleInterfaceCommand::VEHICLE_KILL_SWITCH_ENABLE:
								kill_switch_enabled_ = true;
								break;
							case uam_vehicle_interface_msgs::msg::VehicleInterfaceCommand::VEHICLE_KILL_SWITCH_DISABLE:
								kill_switch_enabled_ = false;
								break;
						}
					});

	timer_ = this->create_wall_timer(
			std::chrono::milliseconds(100),
			[this]()
			{
				uam_vehicle_interface_msgs::msg::VehicleInterfaceStatus status_msg;
				status_msg.stamp = this->get_clock()->now();
				status_msg.vehicle_kill_switch_enabled = kill_switch_enabled_;
				vehicle_interface_status_pub_->publish(status_msg);
			});

	setupStaticTransforms();
}

void Px4Interface::setupParameters()
{
	try {
		declare_parameter("environment", rclcpp::ParameterValue("Vehicle"));
		declare_parameter("vehicle_mass", rclcpp::ParameterValue(0.73));
		declare_parameter("motor_thrust_max", rclcpp::ParameterValue(12.5));
		declare_parameter("motor_thrust_armed", rclcpp::ParameterValue(0.026975));
		declare_parameter("motor_constant", rclcpp::ParameterValue(0.00000584));
		declare_parameter("motor_input_scaling", rclcpp::ParameterValue(1000.0));

		get_parameter("environment", environment_);
		get_parameter("vehicle_mass", vehicle_mass_);
		get_parameter("motor_thrust_max", motor_thrust_max_);
		get_parameter("motor_thrust_armed", motor_thrust_armed_);
		get_parameter("motor_constant", motor_constant_);
		get_parameter("motor_input_scaling", motor_input_scaling_);

		RCLCPP_INFO(get_logger(), "Loading parameters");
		RCLCPP_INFO(get_logger(), "Environment is %s", environment_.c_str());

	} catch (const rclcpp::ParameterTypeException & ex) {
		RCLCPP_ERROR(get_logger(), "Parameter type exception:  %s", ex.what());
	}
}


/**
* @brief Publish the offboard control mode.
*        For this example, only position and altitude controls are active.
*/
void Px4Interface::publishOffboardControlMode()
{
	px4_msgs::msg::OffboardControlMode msg{};
	msg.timestamp = int(get_clock()->now().nanoseconds() / 1000);
	msg.position = false;
	msg.velocity = false;
	msg.acceleration = false;
	msg.attitude = true;
	msg.body_rate = false;

	offboard_control_mode_pub_->publish(msg);
}


/**
 * @brief Send a command to Arm the vehicle
 */
void Px4Interface::arm()
{
	publishVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
	RCLCPP_INFO(this->get_logger(), "Arm command send");
}

/**
 * @brief Send a command to Disarm the vehicle
 */
void Px4Interface::disarm()
{
	publishVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);
	RCLCPP_INFO(this->get_logger(), "Disarm command send");
}


/**
 * @brief Publish vehicle commands
 * @param command   Command code (matches VehicleCommand and MAVLink CMD codes)
 * @param param1    Command parameter 1
 * @param param2    Command parameter 2
 */
void Px4Interface::publishVehicleCommand(uint16_t command, float param1, float param2)
{
	px4_msgs::msg::VehicleCommand msg{};
	msg.timestamp = int(get_clock()->now().nanoseconds() / 1000);
	msg.param1 = param1;
	msg.param2 = param2;
	msg.command = command;
	msg.target_system = 1;
	msg.target_component = 1;
	msg.source_system = 1;
	msg.source_component = 1;
	msg.from_external = true;

	vehicle_command_pub_->publish(msg);
}

//void Px4Interface::publish_rate_control(mavRateControl control) {
//	px4_msgs::msg::VehicleRatesSetpoint vehicle_rates;
//	if (channels_.channels[OFFBOARD_ENABLE_CHANNEL - 1] >= 0.75
//			&& vehicle_status_.arming_state == px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED) {
//		if (vehicle_status_.nav_state != px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD && offboard_counter_ == 10) {
//			publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
//		} else if (offboard_counter_ < 11) {
//			offboard_counter_++;
//		}
//
//		vehicle_rates.timestamp = int(get_clock()->now().nanoseconds() / 1000);
//		vehicle_rates.roll = control.roll_rate;
//		vehicle_rates.pitch = control.pitch_rate;
//		vehicle_rates.yaw = control.yaw_rate;
//		vehicle_rates.thrust_body[2] = fmin(-compute_relative_thrust(control.thrust_normalized * mass_),0.0);
//
//		publish_offboard_control_mode();
//		vehicle_rates_setpoint_pub_->publish(vehicle_rates);
//	} else {
//		offboard_counter_ = 0;
//	}
//}

void Px4Interface::publishAttitudeSetpoint(const uam_control_msgs::msg::AttitudeSetpoint& msg)
{
	px4_msgs::msg::VehicleAttitudeSetpoint vehicle_attitude;

	if (!kill_switch_enabled_) {
		if (vehicle_status_.nav_state != px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD && offboard_counter_ == 10) {
			if (vehicle_status_.arming_state != px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED) {
				arm();
			}
			publishVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
		} else if (offboard_counter_ < 11) {
			offboard_counter_++;
		}
		vehicle_attitude.timestamp = static_cast<int>(get_clock()->now().nanoseconds() / 1000);
		vehicle_attitude.roll_body = static_cast<float>(msg.roll_body);
		vehicle_attitude.pitch_body = static_cast<float>(msg.pitch_body);
		vehicle_attitude.yaw_body = static_cast<float>(msg.yaw_body);
		vehicle_attitude.q_d[0] = static_cast<float>(msg.q_d[0]);
		vehicle_attitude.q_d[1] = static_cast<float>(msg.q_d[1]);
		vehicle_attitude.q_d[2] = static_cast<float>(msg.q_d[2]);
		vehicle_attitude.q_d[3] = static_cast<float>(msg.q_d[3]);
		vehicle_attitude.thrust_body[2] = static_cast<float>(fmin(-computeRelativeThrust(msg.thrust_body_normalized * vehicle_mass_), 0.0));

		publishOffboardControlMode();
		vehicle_attitude_setpoint_pub_->publish(vehicle_attitude);
	} else {
		offboard_counter_ = 0;
	}
}

/**
 * @brief Process vehicle odometry from PX4 and publish in ROS conventions
 *
 * This callback is the primary interface for receiving vehicle state from PX4.
 * It performs coordinate frame transformations and publishes:
 * 1. Odometry in NED/FRD frames (for debugging/logging)
 * 2. TF transform in ENU/FLU frames (for ROS navigation stack)
 *
 * @param msg VehicleOdometry message from PX4 (NED frame, FRD body frame)
 *
 * Data Flow:
 * ----------
 * PX4 Estimator (EKF2) → uXRCE-DDS → /fmu/out/vehicle_odometry
 *                                   → vehicleOdometryCallback()
 *                                   → [NED→ENU transform]
 *                                   → TF2 broadcast
 *                                   → ROS navigation stack
 *
 * Transformation Pipeline:
 * ------------------------
 *
 * STEP 1: Publish Raw Odometry (NED/FRD Frames)
 * ----------------------------------------------
 * Republishes PX4 odometry in ROS message format without coordinate changes.
 * Frame IDs:
 * - header.frame_id = "map_ned" (global NED frame)
 * - child_frame_id = "base_link_frd" (vehicle FRD frame)
 *
 * Purpose: Allows debugging and comparison with PX4-native tools.
 *
 * STEP 2: Transform Position (NED → ENU)
 * ---------------------------------------
 * Converts global position from NED to ENU coordinates.
 *
 * Input:  p_NED = [North, East, Down]^T (from PX4)
 * Output: p_ENU = [East, North, Up]^T (for ROS)
 *
 * Transformation:
 *   x_ENU = y_NED  (East = East)
 *   y_ENU = x_NED  (North = North)
 *   z_ENU = -z_NED (Up = -Down)
 *
 * STEP 3: Transform Orientation (NED → ENU)
 * ------------------------------------------
 * Converts vehicle orientation quaternion from NED to ENU reference frame.
 *
 * Input:  q_NED = vehicle attitude in NED frame (from PX4)
 * Output: q_ENU = vehicle attitude in ENU frame (for ROS)
 *
 * Mathematical Operation:
 *   q_ENU = q_NED_to_ENU_transform ⊗ q_NED
 *
 * Where q_NED_to_ENU_transform represents:
 * - 90° rotation about Z-axis (swap X and Y)
 * - 180° rotation about X-axis (flip Z)
 *
 * STEP 4: Transform Body Frame (FRD → FLU)
 * -----------------------------------------
 * Converts vehicle body frame from aerospace (FRD) to ROS (FLU) convention.
 *
 * Input:  q_ENU (vehicle orientation in ENU frame, still FRD body)
 * Output: q_base_link (vehicle orientation in ENU frame, FLU body)
 *
 * Transformation:
 *   q_base_link = q_FRD_to_FLU ⊗ q_ENU
 *
 * Where q_FRD_to_FLU = [0, 1, 0, 0] (180° rotation about X-axis)
 *
 * STEP 5: Construct and Broadcast TF Transform
 * ---------------------------------------------
 * Combines position and orientation into SE(3) transformation:
 *   T_map_to_base_link = (p_ENU, q_base_link)
 *
 * Broadcast as TF2 transform:
 * - Parent frame: "map" (global ENU frame)
 * - Child frame: "base_link" (vehicle FLU frame)
 *
 * Update Rate:
 * ------------
 * This callback fires at PX4's odometry publication rate:
 * - SITL: 250 Hz (fast simulation)
 * - Hardware: 50-250 Hz (depends on estimator configuration)
 *
 * The TF transform is updated at the same rate, providing high-frequency
 * vehicle pose updates for the navigation stack.
 */
void Px4Interface::vehicleOdometryCallback(px4_msgs::msg::VehicleOdometry::UniquePtr msg)
{
	// ============================================================================
	// STEP 1: Publish Odometry in Native PX4 Frames (NED/FRD)
	// ============================================================================

	// Create ROS odometry message in PX4's native coordinate frames
	// This is useful for debugging and comparing with PX4 logs/tools
	nav_msgs::msg::Odometry vehicle_odom;
	vehicle_odom.header.stamp = get_clock()->now();
	vehicle_odom.header.frame_id = "map_ned";      // Global NED frame
	vehicle_odom.child_frame_id = "base_link_frd"; // Vehicle FRD frame

	// Position in NED frame (directly from PX4, no transformation)
	// [0] = North, [1] = East, [2] = Down
	vehicle_odom.pose.pose.position.x = msg->position[0];
	vehicle_odom.pose.pose.position.y = msg->position[1];
	vehicle_odom.pose.pose.position.z = msg->position[2];

	// Orientation quaternion in NED frame (w, x, y, z)
	vehicle_odom.pose.pose.orientation.w = msg->q[0];
	vehicle_odom.pose.pose.orientation.x = msg->q[1];
	vehicle_odom.pose.pose.orientation.y = msg->q[2];
	vehicle_odom.pose.pose.orientation.z = msg->q[3];

	// Linear velocity in FRD body frame
	// [0] = Forward, [1] = Right, [2] = Down
	vehicle_odom.twist.twist.linear.x = msg->velocity[0];
	vehicle_odom.twist.twist.linear.y = msg->velocity[1];
	vehicle_odom.twist.twist.linear.z = msg->velocity[2];

	// Angular velocity in FRD body frame (rad/s)
	// [0] = Roll rate, [1] = Pitch rate, [2] = Yaw rate
	vehicle_odom.twist.twist.angular.x = msg->angular_velocity[0];
	vehicle_odom.twist.twist.angular.y = msg->angular_velocity[1];
	vehicle_odom.twist.twist.angular.z = msg->angular_velocity[2];

	// Publish raw PX4 odometry (NED/FRD frames)
	vehicle_odometry_pub_->publish(vehicle_odom);

	// ============================================================================
	// STEP 2: Transform Position from NED to ENU
	// ============================================================================

	// Extract position vector from PX4 message (NED coordinates)
	Eigen::Vector3d pose_ned(msg->position[0], msg->position[1], msg->position[2]);

	// Convert NED position to ENU position using px4_ros_com utilities
	// This function applies the rotation: [East, North, Up] = [NED_y, NED_x, -NED_z]
	Eigen::Translation3d pose_enu(px4_ros_com::frame_transforms::ned_to_enu_local_frame(pose_ned));

	// ============================================================================
	// STEP 3: Transform Orientation from NED to ENU
	// ============================================================================

	// Extract orientation quaternion from PX4 message (NED frame reference)
	Eigen::Quaterniond q(msg->q[0], msg->q[1], msg->q[2], msg->q[3]);

	// Convert orientation from NED to ENU reference frame
	// Then convert body frame from FRD to FLU
	// Combined transformation: q_base_link = q_FRD_to_FLU ⊗ q_NED_to_ENU ⊗ q
	Eigen::Quaterniond q_base_link = px4_ros_com::frame_transforms::aircraft_to_baselink_orientation(
			px4_ros_com::frame_transforms::ned_to_enu_orientation(q));

	// ============================================================================
	// STEP 4: Construct SE(3) Transform (Position + Orientation)
	// ============================================================================

	// Combine ENU position and FLU orientation into affine transformation
	// Affine3d = Eigen representation of SE(3) (Special Euclidean group in 3D)
	// SE(3) = {(R, t) | R ∈ SO(3), t ∈ ℝ³}
	Eigen::Affine3d base_link = pose_enu * q_base_link;

	// ============================================================================
	// STEP 5: Broadcast TF Transform (map → base_link)
	// ============================================================================

	// Convert Eigen transformation to ROS TF2 message format
	geometry_msgs::msg::TransformStamped tf = tf2::eigenToTransform(base_link);

	// Set transform metadata
	tf.header.stamp = this->get_clock()->now();
	tf.header.frame_id = "map";        // Parent frame: global ENU
	tf.child_frame_id = "base_link";   // Child frame: vehicle FLU

	// Broadcast dynamic transform to TF2 tree
	// This transform updates at odometry rate (50-250 Hz)
	// Navigation stack listens to this transform for vehicle pose
	tf_dynamic_broadcaster_->sendTransform(tf);
}

/**
 * @brief Set up static coordinate frame transforms
 *
 * Publishes static TF transforms that define relationships between
 * coordinate system conventions (NED↔ENU, FRD↔FLU).
 *
 * Called once during node initialization.
 *
 * Static Transforms Published:
 * ----------------------------
 *
 * 1. map → map_ned
 * -----------------
 * Defines transformation between ROS global frame (ENU) and PX4 global frame (NED).
 *
 * Purpose:
 * - Allows visualization of PX4-native data in RVIZ (ENU viewer)
 * - Enables coordinate conversion for debugging
 * - Supports dual-frame workflows (ROS + PX4 tools)
 *
 * Rotation: 90° about Z (swap X,Y), then 180° about X (flip Z)
 * Quaternion: q_ENU_to_NED
 *
 * 2. base_link → base_link_frd
 * -----------------------------
 * Defines transformation between ROS body frame (FLU) and PX4 body frame (FRD).
 *
 * Purpose:
 * - Allows expression of sensor data in PX4 conventions
 * - Enables visualization of FRD-frame vectors in ROS
 * - Supports IMU/sensor frame conversions
 *
 * Rotation: 180° about X-axis (flip Y and Z)
 * Quaternion: q_FLU_to_FRD = [0, 1, 0, 0]
 *
 * Why Static Transforms?
 * ----------------------
 * These transforms represent fixed relationships between coordinate conventions.
 * They never change during flight, so we publish them as static transforms:
 * - More efficient (published once, cached by TF2)
 * - Lower bandwidth (no repeated transmission)
 * - Guaranteed consistency (cannot be overwritten accidentally)
 *
 * Static vs. Dynamic:
 * - Static: Fixed coordinate convention transforms (map↔map_ned, base_link↔base_link_frd)
 * - Dynamic: Vehicle pose that changes over time (map→base_link from odometry)
 */
void Px4Interface::setupStaticTransforms()
{
	// ============================================================================
	// STATIC TRANSFORM 1: map (ENU) → map_ned (NED)
	// ============================================================================

	// Create identity quaternion as starting point (no rotation)
	// Then apply ENU→NED rotation transformation
	Eigen::Quaterniond q_enu_to_ned = px4_ros_com::frame_transforms::enu_to_ned_orientation(
			Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0)  // Identity: w=1, x=0, y=0, z=0
	);

	// Create affine transformation (pure rotation, no translation)
	// ENU and NED share the same origin, just different axis orientations
	Eigen::Affine3d enu_to_ned_transform(q_enu_to_ned);

	// Convert Eigen transform to ROS TF2 message
	geometry_msgs::msg::TransformStamped tf_map_ned = tf2::eigenToTransform(enu_to_ned_transform);

	// Set transform metadata
	tf_map_ned.header.stamp = this->get_clock()->now();
	tf_map_ned.header.frame_id = "map";       // Parent: ROS global frame (ENU)
	tf_map_ned.child_frame_id = "map_ned";    // Child: PX4 global frame (NED)

	// Publish as static transform (broadcasted once, cached by TF2)
	tf_static_broadcaster_->sendTransform(tf_map_ned);

	// ============================================================================
	// STATIC TRANSFORM 2: base_link (FLU) → base_link_frd (FRD)
	// ============================================================================

	// Create identity quaternion, then apply FLU→FRD rotation
	Eigen::Quaterniond q_base_link_to_aircraft = px4_ros_com::frame_transforms::baselink_to_aircraft_orientation(
			Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0)  // Identity quaternion
	);

	// Create affine transformation (180° rotation about X-axis)
	// This flips Y and Z axes: FLU → FRD
	Eigen::Affine3d base_link_aircraft_transform(q_base_link_to_aircraft);

	// Convert Eigen transform to ROS TF2 message
	geometry_msgs::msg::TransformStamped tf_base_link_frd = tf2::eigenToTransform(base_link_aircraft_transform);

	// Set transform metadata
	tf_base_link_frd.header.stamp = this->get_clock()->now();
	tf_base_link_frd.header.frame_id = "base_link";      // Parent: ROS body frame (FLU)
	tf_base_link_frd.child_frame_id = "base_link_frd";   // Child: PX4 body frame (FRD)

	// Publish as static transform
	tf_static_broadcaster_->sendTransform(tf_base_link_frd);
}

//void Px4Interface::publish_local_position_setpoint(mavLocalPositionSetpoint control)
//{
//	px4_msgs::msg::VehicleLocalPositionSetpoint vehicle_local_position_setpoint;
//	if (channels_.channels[OFFBOARD_ENABLE_CHANNEL - 1] >= 0.75
//			&& vehicle_status_.arming_state == px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED) {
//		if (vehicle_status_.nav_state != px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD && offboard_counter_ == 10) {
//			publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
//		} else if (offboard_counter_ < 11) {
//			offboard_counter_++;
//		}
//
//		vehicle_local_position_setpoint.timestamp = int(get_clock()->now().nanoseconds() / 1000);
//		vehicle_local_position_setpoint.x = control.x;
//		vehicle_local_position_setpoint.y = control.y;
//		vehicle_local_position_setpoint.z = control.z;
//		vehicle_local_position_setpoint.vx = control.vx;
//		vehicle_local_position_setpoint.vy = control.vy;
//		vehicle_local_position_setpoint.vz = control.vz;
//		vehicle_local_position_setpoint.acceleration[0] = control.ax;
//		vehicle_local_position_setpoint.acceleration[1] = control.ay;
//		vehicle_local_position_setpoint.acceleration[2] = control.az;
//		vehicle_local_position_setpoint.thrust[2] = fmin(-compute_relative_thrust(control.thrust_normalized * mass_),0.0);
//
//		publish_offboard_control_mode();
//		vehicle_local_position_setpoint_pub_->publish(vehicle_local_position_setpoint);
//	} else {
//		offboard_counter_ = 0;
//	}
//}

double Px4Interface::computeRelativeThrust(const double &collective_thrust) const
{
	if (environment_ == "SITL") {
		double motor_speed = sqrt(collective_thrust / (4.0 * motor_constant_));
		double thrust_command = (motor_speed - motor_velocity_armed_) / motor_input_scaling_;
//	float rel_thrust = (collective_thrust - min_thrust_) / (max_thrust_ - min_thrust_);
//	return (0.54358075f * rel_thrust + 0.f * sqrtf(3.6484f * rel_thrust + 0.00772641f) - 0.021992793f);
		return thrust_command;

	} else if (environment_ == "Vehicle") {
		double rel_thrust = ((collective_thrust / 4.0) - motor_thrust_armed_) / (motor_thrust_max_ - motor_thrust_armed_);

		if (battery_status_.voltage_filtered_v > 14.0) {
			return (0.54358075 * rel_thrust + 0.25020242 * sqrt(3.6484 * rel_thrust + 0.00772641) - 0.021992793) *
					(1.0 - 0.0779 * (battery_status_.voltage_filtered_v - 16.0));
		} else {
			return (0.54358075 * rel_thrust + 0.25020242 * sqrt(3.6484 * rel_thrust + 0.00772641) - 0.021992793);
		}
	}
}
} // namespace uam_vehicle_interface

int main(int argc, char *argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<uam_vehicle_interface::Px4Interface>());
	rclcpp::shutdown();
	return 0;
}


