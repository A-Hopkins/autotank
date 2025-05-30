/**
 * @file localization_task.h
 * @brief Defines the LocalizationTask class, responsible for handling localization operations
 *
 * The LocalizationTask class extends BaseTask and determines the robot's pose and velocity by
 * fusing IMU and odometry data, using an Extended Kalman Filter (EKF) based on kfplusplus.
 * It also processes incoming control commands to predict state evolution over time.
 */

#pragma once
#include "kfplusplus/include/kfplusplus.h"
#include "msg/cmdvel_msg.h"
#include "msg/imu_msg.h"
#include "msg/lidar_msg.h"
#include "msg/localization_estimate_msg.h"
#include "msg/odom_msg.h"
#include "protocore/include/task/task.h"
#include <chrono>

#ifdef UNIT_TESTING
#include <gtest/gtest_prod.h>
#endif

/**
 * @class LocalizationTask
 * @brief A task that handles localization operations for a robot.
 *
 * The LocalizationTask class is responsible for estimating the robot's state (pose and velocity)
 * by fusing data from various sensors (IMU, Odometry, Lidar) and considering the commanded
 * velocity. It operates in a predict-update cycle:
 * 1. **Predict**: Uses the latest command velocity (`CmdVelMsg`) to predict the state change over
 * time.
 * 2. **Update**: Corrects the predicted state using incoming sensor measurements (`IMUDataMsg`,
 * `OdomDataMsg`, `LidarDataMsg`).
 * 3. **Publish**: Publishes the fused state estimate as a `LocalizationEstimateMsg` for other tasks
 * (e.g., MotionControlTask) to consume.
 *
 * This task subscribes to sensor data messages and command velocity messages, and publishes
 * localization estimates. The interaction loop with motion control typically follows:
 * - t₀:  LocalizationTask publishes state estimate x̂₀ (`LocalizationEstimateMsg`).
 * - t₀+: MotionControlTask consumes x̂₀, computes control command u₁, publishes `CmdVelMsg`.
 * - t₁:  LocalizationTask consumes u₁, predicts state to t₁, consumes sensor data, updates state →
 * publishes x̂₁.
 * - t₁+: MotionControlTask consumes x̂₁ ... (cycle repeats)
 */
class LocalizationTask : public task::Task
{
public:
  static std::shared_ptr<LocalizationTask> create()
  {
    auto instance = std::shared_ptr<LocalizationTask>(new LocalizationTask("LocalizationTask"));
    instance->on_initialize();
    return instance;
  }

  /**
   * @brief Destructor for LocalizationTask, ensuring proper resource cleanup.
   */
  ~LocalizationTask();

protected:
  /**
   * @brief Constructs a LocalizationTask instance.
   * @param name The name assigned to this task instance. Defaults to "LocalizationTask".
   */
  LocalizationTask(const std::string& name = "LocalizationTask") : task::Task(name) {}

  /**
   * @brief Processes incoming messages relevant to localization.
   *
   * This method handles messages such as state transitions (`StateMsg`),
   * sensor data (`IMUDataMsg`, `OdomDataMsg`, `LidarDataMsg`), and command velocity (`CmdVelMsg`).
   * Sensor data and command velocity are used in the predict-update cycle of the state estimation
   * algorithm.
   * @param msg The message to process.
   */
  void process_message(const msg::Msg& msg) override;

  /**
   * @brief Transitions the task to a new state.
   *
   * Manages the internal state of the localization task.
   * @param new_state The target state for the task.
   */
  void transition_to_state(task::TaskState new_state) override;

  /**
   * @brief Performs initialization steps for the LocalizationTask.
   *
   * Subscribes to necessary message types required for localization,
   * including state control, sensor data, and command velocity.
   */
  void on_initialize() override
  {
    safe_subscribe(msg::Type::StateMsg);
    safe_subscribe(msg::Type::OdomDataMsg);
    safe_subscribe(msg::Type::IMUDataMsg);
    safe_subscribe(msg::Type::CmdVelMsg);

    // initial process noise (Q):
    // position variance ~ (0.01 m)^2, heading ~(0.01 rad)^2, velocity ~(0.1 m/s)^2, yaw-rate ~(0.1
    // rad/s)^2
    Q       = linalg::Matrix<STATE_DIM, STATE_DIM>();
    Q(0, 0) = 1e-4;
    Q(1, 1) = 1e-4;
    Q(2, 2) = 1e-4;
    Q(3, 3) = 1e-2;
    Q(4, 4) = 1e-2;
    ekf.set_process_noise(Q);

    // initial covariance P:
    // position ~1 m^2, heading ~(0.1 rad)^2, velocity ~(0.1 m/s)^2, yaw-rate ~(0.1 rad/s)^2
    auto P0  = linalg::Matrix<STATE_DIM, STATE_DIM>::identity();
    P0(0, 0) = 1.0;
    P0(1, 1) = 1.0;
    P0(2, 2) = 1e-2;
    P0(3, 3) = 1e-2;
    P0(4, 4) = 1e-2;
    ekf.set_covariance(P0);

    // default measurement noise (can override later)
    R_imu  = linalg::Matrix<IMU_MEASUREMENT_DIM, IMU_MEASUREMENT_DIM>::identity() * 1e-2;
    R_odom = linalg::Matrix<ODOM_MEASUREMENT_DIM, ODOM_MEASUREMENT_DIM>::identity() * 1e-2;
  }

private:
#ifdef UNIT_TESTING
  FRIEND_TEST(LocalizationTaskTest, IMUUpdateSetsCorrectPosterior);
  FRIEND_TEST(LocalizationTaskTest, OdomUpdateSetsCorrectPosterior);
  FRIEND_TEST(LocalizationTaskTest, SequentialFusionUpdatesState);
#endif

  msg::LocalizationEstimateMsg current_state_est{}; ///< Current state of the localization task
  std::chrono::steady_clock::time_point last_time;  ///< Timestamp for prediction dt

  // EKF dimensions
  static constexpr size_t STATE_DIM            = 5; ///< [x, y, θ, v, ω]
  static constexpr size_t CONTROL_DIM          = 6; ///< [vx, vy, vz, wx, wy, wz]
  static constexpr size_t IMU_MEASUREMENT_DIM  = 2; ///< [θ, ω]
  static constexpr size_t ODOM_MEASUREMENT_DIM = 5; ///< [x, y, θ, v, ω]

  /**
   * @brief Extended Kalman Filter instance for state estimation.
   *
   * This filter is used to predict and update the robot's state based on sensor data
   * and command velocity. The filter is now configured for a 13D state space (3D position,
   * 4D orientation, 3D linear velocity, 3D angular velocity) and a 6D control space (3D linear,
   * 3D angular).
   */
  kfplusplus::ExtendedKalmanFilter<STATE_DIM, CONTROL_DIM> ekf;
  // Process noise covariance (Q)
  linalg::Matrix<STATE_DIM, STATE_DIM> Q;

  // Measurement noise covariances
  linalg::Matrix<IMU_MEASUREMENT_DIM, IMU_MEASUREMENT_DIM>   R_imu;
  linalg::Matrix<ODOM_MEASUREMENT_DIM, ODOM_MEASUREMENT_DIM> R_odom;

  void handle_sensor_data(const msg::Msg& sensor_msg);
  void handle_cmd_vel_data(const msg::CmdVelMsg* cmd_vel_data);
  void publish_estimate();
};
