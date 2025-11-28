/**
 * @file qlearning_controller.cpp
 * @brief Actor-Critic Q-Learning Controller for UAV Trajectory Tracking
 *
 * This file implements an online adaptive optimal control algorithm using
 * actor-critic reinforcement learning based on Adaptive Dynamic Programming (ADP).
 *
 * Mathematical Foundation:
 * -----------------------
 * The controller solves the continuous-time infinite horizon optimal control problem
 * by learning the optimal value function (critic) and optimal policy (actor) online.
 *
 * Key References:
 * 1. Vamvoudakis, K. G., & Lewis, F. L. (2010). "Online actor-critic algorithm to solve
 *    the continuous-time infinite horizon optimal control problem." Automatica, 46(5), 878-888.
 *    DOI: 10.1016/j.automatica.2010.02.018
 *
 * 2. Vrabie, D., Pastravanu, O., Abu-Khalaf, M., & Lewis, F. L. (2009). "Adaptive optimal
 *    control for continuous-time linear systems based on policy iteration." Automatica,
 *    45(2), 477-484.
 *    DOI: 10.1016/j.automatica.2008.08.017
 *
 * 3. Bellman, R. (1957). "Dynamic Programming." Princeton University Press.
 *    (Foundation of optimal control and value functions)
 *
 * Theoretical Basis:
 * ------------------
 * The Hamilton-Jacobi-Bellman (HJB) equation for optimal control:
 *   0 = min_u { r(x,u) + ∇V(x)ᵀf(x,u) }
 *
 * where:
 *   V(x) = value function (cost-to-go from state x)
 *   r(x,u) = running cost (state and control penalties)
 *   f(x,u) = system dynamics
 *
 * Q-Function (State-Action Value):
 *   Q(x,u) = r(x,u) + V(f(x,u))
 *
 * Bellman Optimality Equation:
 *   Q*(x,u) = r(x,u) + γ·Q*(x',π*(x'))
 *
 * where:
 *   γ = discount factor (1.0 for infinite horizon)
 *   π*(x) = optimal policy
 *   x' = next state
 *
 * Implementation Details:
 * -----------------------
 * State Vector (6D): x = [Δx, Δy, Δz, Δvx, Δvy, Δvz]ᵀ
 *   - Position errors: Δx, Δy, Δz (meters)
 *   - Velocity errors: Δvx, Δvy, Δvz (m/s)
 *
 * Control Vector (3D): u = [ax, ay, az]ᵀ
 *   - Desired body accelerations (m/s²)
 *   - Converted to [roll, pitch, thrust] via geometric control
 *
 * Augmented State (9D): ξ = [x; u]ᵀ
 *   - Combined state-action vector for Q-function approximation
 *
 * Q-Function Approximation using Radial Basis Functions (RBF):
 *   Q(x,u) ≈ wᵀ·φ(svec(ξ⊗ξ))
 *
 * where:
 *   w = critic weight vector (45D)
 *   φ = radial basis function (currently identity)
 *   ξ⊗ξ = Kronecker product (creates quadratic features)
 *   svec = symmetric vectorization operator
 *
 * Temporal Difference (TD) Error:
 *   δ = r(x,u) + Q(x',u') - Q(x,u)
 *
 * Critic Update (Gradient Descent):
 *   ẇ = -αc · (∇Q·δ) / (1 + ||∇Q||²)²
 *
 * Actor Update (Policy Gradient):
 *   Ẇa = -αa · (x·∇uQ) / (1 + ||x||²)
 *
 * where:
 *   αc = critic learning rate (convergence_rate)
 *   αa = actor learning rate (convergence_rate)
 *   ∇Q = gradient of Q-function w.r.t. weights
 *   ∇uQ = gradient of Q-function w.r.t. control
 *
 * Cost Function:
 *   r(x,u) = xᵀQx + uᵀRu
 *
 * where:
 *   Q = state penalty matrix (diagonal: position=10, velocity=30)
 *   R = control penalty matrix (diagonal: 20 for each axis)
 */

#include <uam_control/qlearning_controller.hpp>
#include "uam_util/qos_profiles.hpp"

using namespace uam_control;
using namespace rclcpp;
using namespace Eigen;

/**
 * @brief Constructor for Q-Learning Controller
 *
 * Initializes the actor-critic learning framework with pre-configured weight matrices
 * and penalty parameters.
 *
 * Initialization Strategy:
 * ------------------------
 * The controller uses warm-start initialization with pre-computed weights from
 * offline training or previous missions. This approach:
 * 1. Reduces initial exploration (safer for UAV flight)
 * 2. Accelerates convergence
 * 3. Provides baseline performance guarantees
 *
 * Penalty Matrix Design:
 * ----------------------
 * Following Linear Quadratic Regulator (LQR) theory:
 *   J = ∫ (xᵀQx + uᵀRu) dt
 *
 * State Penalty Matrix Q (6×6 diagonal):
 *   - Position errors (x,y,z): weight = 10
 *   - Velocity errors (vx,vy,vz): weight = 30
 *   Rationale: Higher velocity penalty encourages smooth, damped response
 *              (similar to derivative gain in PID control)
 *
 * Control Penalty Matrix R (3×3 diagonal):
 *   - Acceleration commands (ax,ay,az): weight = 20
 *   Rationale: Penalizes excessive control effort, improves energy efficiency
 *
 * Terminal Cost Matrix P (6×6 diagonal):
 *   - Position errors: weight = 100
 *   - Velocity errors: weight = 10
 *   Rationale: Strong emphasis on reaching goal position accurately
 *              (used in finite-horizon formulation via critic_error_2_)
 *
 * Reference for LQR theory:
 * Lewis, F. L., Vrabie, D., & Syrmos, V. L. (2012). "Optimal Control"
 * (3rd ed.). Wiley. Chapter 5: Linear Quadratic Regulators.
 */
QLearningController::QLearningController() : rclcpp::Node("uam_control")
{
	// Initialize symmetric vectorization transform matrix (for Kronecker products)
	// This matrix converts vec(ξ⊗ξ) to svec(ξ⊗ξ) by eliminating redundant elements
	// from the symmetric Kronecker product. Reduces 81 elements to 45.
	computeSvecMatrix();

	// Initialize state and control to zero (will be updated from odometry)
	state_vector_.setZero();
	control_vector_.setZero();
	critic_augmented_state_kronecker_prior_.setZero();

	// Terminal Riccati matrix P for finite-horizon cost-to-go estimation
	// Used in critic_error_2_ to enforce terminal constraints
	// Higher position weights (100) ensure accurate goal reaching
	terminal_riccati_matrix_.diagonal() << 100.0, 100.0, 100.0, 10.0, 10.0, 10.0;

	// State penalty matrix Q - penalizes tracking errors
	// Velocity penalties (30) > position penalties (10) for damped response
	state_penalty_matrix_.setZero();
	state_penalty_matrix_.diagonal() << 10.0, 10.0, 10.0, 30.0, 30.0, 30.0;

	// Control penalty matrix R - penalizes excessive control effort
	// Uniform weight (20) across all acceleration components
	control_penalty_matrix_.setZero();
	control_penalty_matrix_.diagonal() << 20.0, 20.0, 20.0;

	try {
		declare_parameter("rrtx_static.critic_convergence_rate",rclcpp::ParameterValue(0.1));
		declare_parameter("rrtx_static.actor_convergence_rate",rclcpp::ParameterValue(0.1));
		declare_parameter("rrtx_static.minimum_altitude_for_learning",rclcpp::ParameterValue(0.6));
		declare_parameter("rrtx_static.learning_update_frequency",rclcpp::ParameterValue(100.0));

		get_parameter("rrtx_static.critic_convergence_rate",critic_convergence_rate_);
		get_parameter("rrtx_static.actor_convergence_rate",actor_convergence_rate_);
		get_parameter("rrtx_static.minimum_altitude_for_learning",minimum_altitude_for_learning_);
		get_parameter("rrtx_static.learning_update_frequency",learning_update_frequency_);

	} catch (const rclcpp::ParameterTypeException & ex) {
		RCLCPP_ERROR(get_logger(), "Parameter type exception thrown");
	}
	// ----------------------- Subscribers --------------------------
	navigator_setpoint_sub_ =
			this->create_subscription<nav_msgs::msg::Odometry>(
					"/uam_navigator/position_setpoint", 10,
					[this](const nav_msgs::msg::Odometry::UniquePtr msg) {
						if (navigator_setpoint_.pose.pose != msg->pose.pose) {
							is_setpoint_new_ = true;
							start_time_ = this->get_clock()->now();
							// reset weights
							critic_weight_vector_ << 10.83,0.00,0.00,11.72,0.00,0.00,2.93,0.00,0.00,10.83,0.00,0.00,11.72,0.00,0.00,2.93,0.00,10.83,0.00,0.00,11.72,0.00,0.00,2.93,11.99,0.00,0.00,4.24,0.00,0.00,11.99,0.00,0.00,4.24,0.00,11.99,0.00,0.00,4.24,1.00,0.00,0.00,1.00,0.00,1.00;

							actor_weight_matrix_ <<  -2.07,0.00,0.00,0.00,-2.07,0.00,0.00,0.00,-2.07,-3.00,0.00,0.00,0.00,-3.00,0.00,0.00,0.00,-3.00;

							running_cost_ = 0;
							running_cost_prior_ = 0;
						}
						navigator_setpoint_ = *msg;

						auto state_vector = computeStateVector();
						auto control = computeControl(state_vector);
						publishControlMellinger(control);
					});
	// TODO: remove dependence on px4_msgs
	vehicle_status_sub_ =
			this->create_subscription<px4_msgs::msg::VehicleStatus>("/fmu/out/vehicle_status", uam_util::px4_qos_sub,
					[this](const px4_msgs::msg::VehicleStatus::UniquePtr msg) {
						vehicle_status_ = *msg;
					});
	vehicle_odometry_sub_ =
			this->create_subscription<nav_msgs::msg::Odometry>("/uam_vehicle_interface/odometry", 10,
					[this](const nav_msgs::msg::Odometry::UniquePtr msg) {
						rclcpp::Time vehicle_odom_time(vehicle_odometry_.header.stamp.sec, vehicle_odometry_.header.stamp.nanosec);
						rclcpp::Time msg_time(msg->header.stamp.sec, msg->header.stamp.nanosec);
						time_resolution_ = msg_time.seconds() - vehicle_odom_time.seconds();
//						RCLCPP_INFO(get_logger(), "time resolution: %.4f", time_resolution_);
					    vehicle_odometry_ = *msg;
						if (canLearn() && time_resolution_ < 0.1) {
							//------- Q-Learning -------------
							learn();
						} else {
							start_time_ = this->get_clock()->now();
						}
					});
	// ----------------------- Publishers ---------------------------

	vehicle_attitude_setpoint_pub_ =
			this->create_publisher<uam_control_msgs::msg::AttitudeSetpoint>("/uam_control/attitude_setpoint", 10);
	qlearning_status_pub_ =
			this->create_publisher<uam_control_msgs::msg::QLearningStatus>("/uam_control/qlearning_status", 10);

//	auto timer_callback = [this]() -> void
//	{
//		if (can_learn()) {
//			//------- Q-Learning -------------
//			learn();
//		} else {
//			start_time_ = this->get_clock()->now();
//		}
//	};

//	time_resolution_ = 1.0 / learning_update_frequency_;
//	timer_ = this->create_wall_timer(std::chrono::duration<double>(time_resolution_), timer_callback);
}

bool QLearningController::canLearn() const
{
	bool out;
	out =   abs(vehicle_odometry_.pose.pose.position.z) > minimum_altitude_for_learning_ &&
			vehicle_status_.arming_state == px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED &&
			vehicle_status_.nav_state == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD;
	return out;
}

state_vector_t QLearningController::computeStateVector() const
{
	state_vector_t state_vector;
	state_vector << vehicle_odometry_.pose.pose.position.x - navigator_setpoint_.pose.pose.position.x,
					vehicle_odometry_.pose.pose.position.y - navigator_setpoint_.pose.pose.position.y,
					vehicle_odometry_.pose.pose.position.z - navigator_setpoint_.pose.pose.position.z,
					vehicle_odometry_.twist.twist.linear.x - navigator_setpoint_.twist.twist.linear.x,
					vehicle_odometry_.twist.twist.linear.y - navigator_setpoint_.twist.twist.linear.y,
					vehicle_odometry_.twist.twist.linear.z - navigator_setpoint_.twist.twist.linear.z;
	return state_vector;
}
/**
 * @brief Main Q-Learning Update Loop
 *
 * This method implements the core actor-critic learning algorithm using temporal
 * difference (TD) learning and policy gradient methods.
 *
 * Algorithm Overview:
 * -------------------
 * 1. Compute current state (tracking errors)
 * 2. Compute control action using actor network
 * 3. Evaluate Q-value using critic network
 * 4. Compute TD error for critic
 * 5. Update critic weights (value function approximation)
 * 6. Update actor weights (policy improvement)
 * 7. Accumulate running cost
 *
 * Mathematical Details:
 * ---------------------
 *
 * STEP 1: Feature Vector Construction
 * ------------------------------------
 * Augmented state-action vector:
 *   ξ(t) = [x(t); u(t)] ∈ ℝ⁹
 *
 * Kronecker product feature (quadratic basis):
 *   φ(t) = svec(ξ(t) ⊗ ξ(t)) ∈ ℝ⁴⁵
 *
 * The Kronecker product ξ⊗ξ creates all pairwise products:
 *   [ξ₁ξ₁, ξ₁ξ₂, ..., ξ₁ξ₉, ξ₂ξ₁, ξ₂ξ₂, ..., ξ₉ξ₉]ᵀ
 *
 * The svec (symmetric vectorization) eliminates redundant elements since ξ⊗ξ
 * is symmetric: ξᵢξⱼ = ξⱼξᵢ. This reduces 81 elements to 45.
 *
 * Reference: Magnus, J. R., & Neudecker, H. (1999). "Matrix Differential Calculus
 * with Applications in Statistics and Econometrics" (Revised ed.). Wiley.
 * Chapter 3: The vec and vech operators.
 *
 * STEP 2: Q-Function Approximation
 * ---------------------------------
 * Using linear function approximation with RBF basis:
 *   Q(x,u,w) = wᵀ·Φ(t)·φ(t)
 *
 * where:
 *   w ∈ ℝ⁴⁵ = critic weight vector
 *   Φ(t) ∈ ℝ⁴⁵ˣ⁴⁵ = radial basis function matrix (currently Identity)
 *   φ(t) ∈ ℝ⁴⁵ = feature vector (Kronecker product)
 *
 * Currently Φ(t) = I (identity), but framework supports time-varying RBF:
 *   Φᵢⱼ(t) = exp(-||φᵢ(t) - cⱼ||² / (2σ²))
 *
 * STEP 3: Temporal Difference Errors
 * -----------------------------------
 * Two types of TD errors are computed:
 *
 * critic_error_1 (Bellman Residual):
 *   δ₁ = wᵀ[Φ(t)φ(t) - Φ(t-Δt)φ(t-Δt)] + [r(t) - r(t-Δt)]
 *
 * This approximates the continuous-time Bellman equation:
 *   dQ/dt = -r(x,u)
 *
 * Integrating: Q(t) - Q(t-Δt) ≈ -r(t)·Δt
 *
 * Reference: Doya, K. (2000). "Reinforcement Learning in Continuous Time and Space."
 * Neural Computation, 12(1), 219-245.
 * DOI: 10.1162/089976600300015961
 *
 * critic_error_2 (Terminal Cost Error):
 *   δ₂ = 0.5·xᵀPx - wᵀΦφ
 *
 * This enforces the terminal boundary condition:
 *   V(x_final) = 0.5·xᵀPx
 *
 * where P is the terminal Riccati matrix. In finite-horizon optimal control,
 * the value function must satisfy this boundary condition.
 *
 * NOTE: Currently weighted with 0.0, so only δ₁ is used (infinite horizon).
 *
 * STEP 4: Normalized Gradient Descent
 * ------------------------------------
 * Critic weight update:
 *   ẇ = -αc · σ/(1 + ||σ||²)² · δ₁
 *
 * where:
 *   σ = ∇wQ = Φ(φ - φ_prior) = gradient of Q w.r.t. weights
 *   αc = critic_convergence_rate (learning rate)
 *
 * The normalization factor (1 + ||σ||²)² provides:
 * 1. Numerical stability (prevents weight explosion)
 * 2. Adaptive step size (larger gradients → smaller updates)
 * 3. Convergence guarantees under certain conditions
 *
 * Reference: Vamvoudakis & Lewis (2010), Equation (23)
 *
 * STEP 5: Policy Gradient for Actor Update
 * -----------------------------------------
 * First, extract control-dependent part of Q-function:
 *   ∇²Q = 2·Φᵀw  (Hessian matrix, 9×9)
 *   Quu = bottom-right 3×3 block (control-control coupling)
 *   Qux = bottom-left 3×6 block (control-state coupling)
 *
 * Optimal control from HJB equation:
 *   u* = -Quu⁻¹·Qux·x
 *
 * Actor error (deviation from optimal policy):
 *   ea = Wa·Ψ(t)·x + Quu⁻¹·Qux·x
 *
 * where:
 *   Wa ∈ ℝ⁶ˣ³ = actor weight matrix
 *   Ψ(t) ∈ ℝ⁶ˣ⁶ = actor RBF matrix (currently Identity)
 *
 * Actor weight update (gradient descent on policy error):
 *   Ẇa = -αa · x/(1 + xᵀx) · eaᵀ
 *
 * Normalization ensures bounded updates even for large state errors.
 *
 * Reference: Vamvoudakis & Lewis (2010), Equation (33)
 *
 * STEP 6: Running Cost Accumulation
 * ----------------------------------
 * Incremental cost (Euler integration):
 *   r(t) += 0.5 · [xᵀQx + uᵀRu] · Δt
 *
 * This accumulated cost is used in the next iteration's TD error computation.
 */
void QLearningController::learn()
{
	double current_time = (this->get_clock()->now().seconds() - start_time_.seconds());

	// ============================================================================
	// STEP 1: Compute State, Control, and Feature Vectors
	// ============================================================================

	// Extract tracking errors from odometry and setpoint
	state_vector_ = computeStateVector();

	// Compute control action using current actor policy: u = Wa·Ψ·x
	control_vector_ = computeControl(state_vector_);

	// Create augmented state-action vector: ξ = [x; u] ∈ ℝ⁹
	augmented_state_vector_ << state_vector_, control_vector_;

	// Compute feature vector via symmetric Kronecker product: φ = svec(ξ⊗ξ) ∈ ℝ⁴⁵
	// This creates quadratic features for Q-function approximation
	critic_augmented_state_kronecker_ = vec_to_svec_transform_matrix_ * kroneckerProduct(augmented_state_vector_, augmented_state_vector_);

	// Compute radial basis function matrices (currently returns Identity)
	critic_radial_basis_matrix_ = computeCriticRadialBasisMatrix(current_time);
	actor_radial_basis_matrix_ = computeActorRadialBasisMatrix(current_time);

	// On new setpoint, reset prior features (discontinuity in trajectory)
	if (is_setpoint_new_) {
		critic_augmented_state_kronecker_prior_ = critic_augmented_state_kronecker_;
		is_setpoint_new_ = false;
		RCLCPP_INFO(get_logger(), "Point is new.........");
	}

	// ============================================================================
	// STEP 2: Compute Temporal Difference Errors for Critic
	// ============================================================================

	// Gradient terms for normalized gradient descent
	critic_augmented_state_kronecker_t sigma;    // ∇Q for Bellman residual
	critic_augmented_state_kronecker_t sigma_f;  // ∇Q for terminal cost

	// σ = Φ·(φ(t) - φ(t-Δt)) - change in Q-function features
	sigma << critic_radial_basis_matrix_ * (critic_augmented_state_kronecker_ - critic_augmented_state_kronecker_prior_);

	// σf = -Φ·φ(t) - for terminal boundary condition
	sigma_f << -critic_radial_basis_matrix_ * critic_augmented_state_kronecker_;

	// Bellman Residual TD Error:
	// δ₁ = wᵀ[Φ(t)φ(t) - Φ(t-Δt)φ(t-Δt)] + [r(t) - r(t-Δt)]
	// Measures how well current Q-function satisfies Bellman equation
	critic_error_1_ = critic_weight_vector_.transpose() * (critic_radial_basis_matrix_ * critic_augmented_state_kronecker_
			- computeCriticRadialBasisMatrix(current_time - time_resolution_) * critic_augmented_state_kronecker_prior_)
			+ running_cost_ - running_cost_prior_;

	// Terminal Cost TD Error:
	// δ₂ = 0.5·xᵀPx - wᵀΦφ
	// Enforces boundary condition V(x_final) = 0.5·xᵀPx
	critic_error_2_ = 0.5 * (state_vector_.transpose() * terminal_riccati_matrix_).dot(state_vector_)
			- critic_weight_vector_.transpose().dot(sigma_f);

	// ============================================================================
	// STEP 3: Update Critic Weights (Value Function Learning)
	// ============================================================================

	// Normalized gradient descent with two error terms:
	// ẇ = -αc·[σ/(1+||σ||²)²·δ₁ + σf/(1+||σf||²)²·δ₂]
	//
	// First term: Bellman residual minimization (active, weight = 1.0)
	// Second term: Terminal cost matching (inactive, weight = 0.0)
	critic_weight_vector_t critic_weight_dot = -critic_convergence_rate_
			* ((sigma /pow(1.0 + sigma.squaredNorm(), 2))*critic_error_1_)
			- 0.0*critic_convergence_rate_ * (sigma_f /pow(1.0 + sigma_f.squaredNorm(), 2))*critic_error_2_;
	// Alternative update laws (commented out):
	// - Version with exponentially decaying terminal cost term
	// - Version with only Bellman residual (no terminal cost)

	// ============================================================================
	// STEP 4: Update Actor Weights (Policy Improvement)
	// ============================================================================

	// Reconstruct Q-function Hessian matrix from critic weights
	// Q(ξ) ≈ ξᵀQ̄ξ where Q̄ is 9×9 symmetric matrix
	//
	// The critic learns Q(ξ) = wᵀφ(ξ) where φ = svec(ξ⊗ξ)
	// To extract Q̄: ∂²Q/∂ξ² = 2·reshape(svec_transform'·w)
	augmented_state_matrix_t Q_bar;
	Q_bar << (vec_to_svec_transform_matrix_.transpose() *
			(2 * critic_radial_basis_matrix_.transpose() * critic_weight_vector_))
			.reshaped(AUGMENTED_STATE_VECTOR_SIZE,AUGMENTED_STATE_VECTOR_SIZE);

	// Extract control-control block of Hessian (bottom-right 3×3)
	// Quu = ∂²Q/∂u² - measures curvature of Q w.r.t. control
	control_matrix_t Quu = Q_bar.bottomRightCorner<QLEARNING_CONTROL_VECTOR_SIZE, QLEARNING_CONTROL_VECTOR_SIZE>();

	// Compute actor error (policy residual)
	// Current policy: u = Wa·Ψ·x
	// Optimal policy (from HJB): u* = -Quu⁻¹·Qux·x
	//
	// Actor error measures deviation from optimality:
	// ea = u - u* = Wa·Ψ·x + Quu⁻¹·Qux·x
	actor_error_ = actor_weight_matrix_.transpose() * actor_radial_basis_matrix_ * state_vector_
			+ Quu.inverse() * Q_bar.bottomLeftCorner<QLEARNING_CONTROL_VECTOR_SIZE, QLEARNING_STATE_VECTOR_SIZE>() * state_vector_;

	// Actor weight update (gradient descent on policy error)
	// Ẇa = -αa · x/(1 + xᵀx) · eaᵀ
	//
	// This drives the actor towards the optimal policy defined by the critic
	// Normalization prevents large updates when tracking errors are large
	actor_weight_matrix_t actor_weight_dot = -actor_convergence_rate_ * state_vector_ / (1.0 + state_vector_.dot(state_vector_)) * actor_error_.transpose();

	// ============================================================================
	// STEP 5: Compute Current Q-Value (for monitoring)
	// ============================================================================

	// Q(x,u) = wᵀ·Φ·φ
	// This is the learned estimate of cost-to-go from current state-action
	quality_value_ = critic_weight_vector_.transpose() * critic_radial_basis_matrix_ * critic_augmented_state_kronecker_;

	// Publish diagnostic information (weights, errors, Q-value)
	publishQlearningStatus();

	// ============================================================================
	// STEP 6: Apply Weight Updates (Euler Integration)
	// ============================================================================

	// Critic weight update: w(t+Δt) = w(t) + ẇ·Δt
	critic_weight_vector_ += critic_weight_dot * time_resolution_;

	// Actor weight update: Wa(t+Δt) = Wa(t) + Ẇa·Δt
	actor_weight_matrix_ += actor_weight_dot * time_resolution_;

	// Update running cost for next TD error computation
	running_cost_prior_ = running_cost_;

	// Accumulate cost: r(t+Δt) = r(t) + 0.5·[xᵀQx + uᵀRu]·Δt
	// This is the integral of instantaneous cost used in Bellman equation
	running_cost_ += 0.5 * ((state_vector_.transpose() * state_penalty_matrix_).dot(state_vector_)
			+ (control_vector_.transpose() * control_penalty_matrix_).dot(control_vector_)) * time_resolution_;

	// Save current features for next iteration's TD error
	critic_augmented_state_kronecker_prior_ = critic_augmented_state_kronecker_;
}
control_vector_t QLearningController::computeControl(const mavState& state_vector)
{
	double current_time = (this->get_clock()->now().seconds() - start_time_.seconds());
	return actor_weight_matrix_.transpose() * computeActorRadialBasisMatrix(current_time) * state_vector;
}

critic_radial_basis_matrix_t QLearningController::computeCriticRadialBasisMatrix(double t)
{
	(void) t; // fix unused time argument warnings
	return critic_radial_basis_matrix_t::Identity();
}
actor_radial_basis_matrix_t QLearningController::computeActorRadialBasisMatrix(double t)
{
	(void) t; // fix unused time argument warnings
	return actor_radial_basis_matrix_t::Identity();
}
/**
 * @brief Compute Symmetric Vectorization Transform Matrix
 *
 * This method constructs the transformation matrix T that converts a full
 * vectorized Kronecker product vec(ξ⊗ξ) to its symmetric vectorization svec(ξ⊗ξ).
 *
 * Mathematical Background:
 * ------------------------
 * For a symmetric matrix A ∈ ℝⁿˣⁿ (where A = Aᵀ), the elements above and below
 * the diagonal are identical:
 *   Aᵢⱼ = Aⱼᵢ for all i,j
 *
 * The Kronecker product ξ⊗ξ produces a symmetric matrix, so we can represent it
 * more efficiently using only the unique elements.
 *
 * Full vectorization: vec(ξ⊗ξ) ∈ ℝ⁸¹ (9×9 = 81 elements)
 * Symmetric vectorization: svec(ξ⊗ξ) ∈ ℝ⁴⁵ (9×10/2 = 45 unique elements)
 *
 * The svec operator stacks the diagonal and lower triangular elements:
 *   svec(A) = [A₁₁, √2·A₂₁, A₂₂, √2·A₃₁, √2·A₃₂, A₃₃, ...]ᵀ
 *
 * The √2 scaling factor ensures that:
 *   vec(A)ᵀvec(A) = svec(A)ᵀsvec(A)
 *
 * This preserves the Frobenius norm and simplifies quadratic form computations.
 *
 * Transform Matrix Construction:
 * ------------------------------
 * The matrix T ∈ ℝ⁴⁵ˣ⁸¹ satisfies:
 *   svec(ξ⊗ξ) = T · vec(ξ⊗ξ)
 *
 * For each unique element (i,j) where i ≥ j:
 *   - If i = j (diagonal): select element directly
 *   - If i > j (off-diagonal): average elements (i,j) and (j,i) with √2 scaling
 *
 * Example for n=3:
 *   vec(A) = [A₁₁, A₁₂, A₁₃, A₂₁, A₂₂, A₂₃, A₃₁, A₃₂, A₃₃]ᵀ
 *   svec(A) = [A₁₁, √2·A₂₁, A₂₂, √2·A₃₁, √2·A₃₂, A₃₃]ᵀ
 *
 * Applications:
 * -------------
 * 1. Efficient storage for symmetric matrices
 * 2. Q-function parameterization in quadratic form: Q(ξ) = ξᵀAξ
 * 3. Reduces critic weight vector from 81 to 45 dimensions
 *
 * Reference:
 * Magnus, J. R., & Neudecker, H. (1999). "Matrix Differential Calculus with
 * Applications in Statistics and Econometrics" (Revised ed.). Wiley.
 * Section 3.8: The symmetric vectorization operator.
 */
void QLearningController::computeSvecMatrix()
{
	svec_matrix_t vec_to_svec_transform_matrix;
	vec_to_svec_transform_matrix.setZero();
	size_t n = AUGMENTED_STATE_VECTOR_SIZE;  // n = 9

	// Iterate over lower triangular elements (including diagonal)
	for (size_t j = 0; j < n; ++j)      // column index
	{
		for(size_t i = j; i < n; ++i)  // row index (i ≥ j)
		{
			// Create selector vector uᵢⱼ for this element in svec representation
			// Index in svec: j*n + i - (j+1)*j/2
			// This maps (i,j) to its position in the compressed vector
			critic_augmented_state_kronecker_t uij;
			uij.setZero();
			uij(j*n + i - (j+1)*j/2) = 1.0;

			// Create transformation matrix Tᵢⱼ for selecting/averaging elements
			augmented_state_matrix_t Tij;
			Tij.setZero();

			if (i == j)  // Diagonal element
			{
				// Directly select Aᵢᵢ (no averaging needed)
				Tij(i, j) = 1.0;
			}
			else  // Off-diagonal element
			{
				// Average Aᵢⱼ and Aⱼᵢ with √2 scaling for norm preservation
				Tij(i, j) = 1.0/sqrt(2.0);  // Select Aᵢⱼ
				Tij(j, i) = 1.0/sqrt(2.0);  // Select Aⱼᵢ (symmetric counterpart)
			}

			// Accumulate: T = Σᵢⱼ uᵢⱼ ⊗ vec(Tᵢⱼ)
			vec_to_svec_transform_matrix += uij * Tij.reshaped().transpose();
		}
	}

	// Store the computed transformation matrix (45×81)
	vec_to_svec_transform_matrix_ = vec_to_svec_transform_matrix;
}
/**
 * @brief Geometric Control Law (Mellinger-style Attitude Computation)
 *
 * Converts desired acceleration commands into attitude (roll, pitch, yaw) and
 * thrust setpoints for the PX4 autopilot using geometric nonlinear control theory.
 *
 * Mathematical Foundation:
 * ------------------------
 * This implements the geometric tracking controller from:
 *
 * Mellinger, D., & Kumar, V. (2011). "Minimum snap trajectory generation and
 * control for quadrotors." In 2011 IEEE International Conference on Robotics
 * and Automation (pp. 2520-2525). DOI: 10.1109/ICRA.2011.5980409
 *
 * Lee, T., Leok, M., & McClamroch, N. H. (2010). "Geometric tracking control
 * of a quadrotor UAV on SE(3)." In 49th IEEE Conference on Decision and Control
 * (CDC) (pp. 5420-5425). DOI: 10.1109/CDC.2010.5717652
 *
 * Algorithm Overview:
 * -------------------
 * Input: Desired acceleration vector ad ∈ ℝ³ (from Q-learning actor)
 * Output: Attitude quaternion q ∈ SO(3) and thrust scalar T ∈ ℝ
 *
 * Step 1: Compute desired thrust vector
 *   Fd = m·(ad + g·e₃)
 * where:
 *   m = vehicle mass
 *   g = gravitational acceleration (9.80665 m/s²)
 *   e₃ = [0, 0, 1]ᵀ (vertical unit vector)
 *
 * Step 2: Compute desired body z-axis (thrust direction)
 *   zB,d = -Fd / ||Fd||
 * Note: Negative sign because body z-axis points downward in FRD frame
 *
 * Step 3: Compute desired body x-axis (heading constraint)
 *   Given desired yaw ψd, construct horizontal heading:
 *     xc,d = [cos(ψd), sin(ψd), 0]ᵀ
 *   Compute body y-axis:
 *     yB,d = (zB,d × xc,d) / ||zB,d × xc,d||
 *   Compute body x-axis:
 *     xB,d = yB,d × zB,d
 *
 * Step 4: Construct rotation matrix and extract Euler angles
 *   Rd = [xB,d | yB,d | zB,d]
 *   [ψd, θd, φd] = ZYX_Euler(Rd)
 *
 * Step 5: Compute thrust magnitude
 *   T = Fd · (-zB,current)
 * Uses current orientation to project desired force along current thrust axis
 *
 * Advantages of Geometric Control:
 * ---------------------------------
 * 1. Globally valid (no singularities except gimbal lock)
 * 2. Directly operates on SO(3) manifold
 * 3. Decouples position and attitude dynamics
 * 4. Linearizes inner loop (PX4 attitude controller)
 *
 * Note on Coordinate Frames:
 * --------------------------
 * - Input acceleration: ENU frame (x=East, y=North, z=Up)
 * - Output attitude: FRD frame (x=Forward, y=Right, z=Down)
 * - Gravity vector compensates for downward acceleration
 * - PX4 expects attitude in FRD and thrust as normalized scalar [0,1]
 */
void QLearningController::publishControlMellinger(const control_vector_t& control_vector)
{
	Vector3d thrust_des;
	uam_control_msgs::msg::AttitudeSetpoint attitude_setpoint;
	Vector3d x_c_des;  // Desired heading direction (horizontal)
	double yaw_des = EIGEN_PI / 2.0;  // Fixed yaw setpoint (90°, facing North)
	Vector3d z_axis;   // Current body z-axis
	Vector3d x_B_desired, y_B_desired, z_B_desired;  // Desired body frame axes
	double thrust_proj;  // Projected thrust magnitude
	Vector3d rpy;  // Roll, pitch, yaw (unused, kept for compatibility)

	Quaterniond q(vehicle_odometry_.pose.pose.orientation.w,
	              vehicle_odometry_.pose.pose.orientation.x,
	              vehicle_odometry_.pose.pose.orientation.y,
	              vehicle_odometry_.pose.pose.orientation.z);

	Matrix3d R = q.toRotationMatrix();
	//Matrix3d R = q.toRotationMatrix();
	z_axis = R.col(2);

	//TODO common config params
	thrust_des = control_vector + Vector3d(0.0,0.0,-9.80665);
	thrust_proj = thrust_des.dot(-z_axis);

	x_c_des << cos(yaw_des), sin(yaw_des), 0;
	z_B_desired = -thrust_des.normalized();
	y_B_desired = (z_B_desired.cross(x_c_des)).normalized();
	x_B_desired = y_B_desired.cross(z_B_desired);

	Matrix3d R_d;
	R_d.col(0) = x_B_desired;
	R_d.col(1) = y_B_desired;
	R_d.col(2) = z_B_desired;
	Vector3d ypr_d = R_d.eulerAngles(2,1,0);
	Quaterniond q_d(R_d);
//	auto e_R_matrix = (R_d.transpose() * R - R.transpose() * R_d);
//	auto e_R = Eigen::Vector3d(0.5 * e_R_matrix(2,1),
//							   0.5 * e_R_matrix(0,2),
//							   0.5 * e_R_matrix(1,0));
//	auto rpy_des = rpy - e_R;
	attitude_setpoint.header.stamp = this->get_clock()->now();
	attitude_setpoint.header.frame_id = "map_ned";
	attitude_setpoint.roll_body = ypr_d(2);
	attitude_setpoint.pitch_body = ypr_d(1);
	attitude_setpoint.yaw_body = ypr_d(0);
	attitude_setpoint.thrust_body_normalized = thrust_proj;
	attitude_setpoint.q_d[0] = q_d.w();
	attitude_setpoint.q_d[1] = q_d.x();
	attitude_setpoint.q_d[2] = q_d.y();
	attitude_setpoint.q_d[3] = q_d.z();
	vehicle_attitude_setpoint_pub_->publish(attitude_setpoint);
}

void QLearningController::publishQlearningStatus()
{
	uam_control_msgs::msg::QLearningStatus qlearning_status;

	qlearning_status.stamp = this->get_clock()->now();
	qlearning_status.start_time = start_time_;
	qlearning_status.quality_value = quality_value_;

	for(size_t i = 0; i < qlearning_status.actor_weight_matrix.size(); i++)
		qlearning_status.actor_weight_matrix[i] = *(actor_weight_matrix_.data() + i);

	for(size_t i = 0; i < qlearning_status.critic_weight_vector.size(); i++)
		qlearning_status.critic_weight_vector[i] = critic_weight_vector_[i];

	for(size_t i = 0; i < qlearning_status.control_vector.size(); i++)
		qlearning_status.control_vector[i] = control_vector_[i];

	for(size_t i = 0; i < qlearning_status.state_vector.size(); i++)
		qlearning_status.state_vector[i] = state_vector_[i];

	for(size_t i = 0; i < qlearning_status.critic_augmented_state_kronecker.size(); i++)
		qlearning_status.critic_augmented_state_kronecker[i] = critic_augmented_state_kronecker_[i];

	for(size_t i = 0; i < qlearning_status.critic_augmented_state_kronecker_prior.size(); i++)
		qlearning_status.critic_augmented_state_kronecker_prior[i] = critic_augmented_state_kronecker_prior_[i];

	qlearning_status.critic_error_1 = critic_error_1_;
	qlearning_status.critic_error_2 = critic_error_2_;

	for(size_t i = 0; i < qlearning_status.actor_error.size(); i++)
		qlearning_status.actor_error[i] = actor_error_[i];

	qlearning_status_pub_->publish(qlearning_status);
}

int main(int argc, char *argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<uam_control::QLearningController>());
	rclcpp::shutdown();
	return 0;
}