/**
 * @file localization_task.h
 * @brief Defines the LocalizationTask class, responsible for handling localization operations
 *
 * The LocalizationTask class extends BaseTask and determines the robots pose and velocity given
 * sensor data.
 */

#pragma once
#include "protocore/include/task.h"

#include "msg/cmdvel_msg.h"
#include "msg/odom_msg.h"
#include "msg/imu_msg.h"
#include "msg/lidar_msg.h"
#include "msg/localization_estimate_msg.h"

/**
 * @class LocalizationTask
 * @brief A task that handles localization operations for a robot.
 *
 * The LocalizationTask class is responsible for estimating the robot's state (pose and velocity)
 * by fusing data from various sensors (IMU, Odometry, Lidar) and considering the commanded velocity.
 * It operates in a predict-update cycle:
 * 1. **Predict**: Uses the latest command velocity (`CmdVelMsg`) to predict the state change over time.
 * 2. **Update**: Corrects the predicted state using incoming sensor measurements (`IMUDataMsg`, `OdomDataMsg`, `LidarDataMsg`).
 * 3. **Publish**: Publishes the fused state estimate as a `LocalizationEstimateMsg` for other tasks (e.g., MotionControlTask) to consume.
 *
 * This task subscribes to sensor data messages and command velocity messages, and publishes localization estimates.
 * The interaction loop with motion control typically follows:
 * - t₀:  LocalizationTask publishes state estimate x̂₀ (`LocalizationEstimateMsg`).
 * - t₀+: MotionControlTask consumes x̂₀, computes control command u₁, publishes `CmdVelMsg`.
 * - t₁:  LocalizationTask consumes u₁, predicts state to t₁, consumes sensor data, updates state → publishes x̂₁.
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

  /**
  * @brief Processes incoming messages relevant to localization.
  *
  * This method handles messages such as state transitions (`StateMsg`),
  * sensor data (`IMUDataMsg`, `OdomDataMsg`, `LidarDataMsg`), and command velocity (`CmdVelMsg`).
  * Sensor data and command velocity are used in the predict-update cycle of the state estimation algorithm.
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


protected:
  /**
   * @brief Constructs a LocalizationTask instance.
   * @param name The name assigned to this task instance. Defaults to "LocalizationTask".
   */
  LocalizationTask(const std::string& name = "LocalizationTask") : task::Task(name) { }

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
    safe_subscribe(msg::Type::LidarDataMsg);
    safe_subscribe(msg::Type::CmdVelMsg);
  }

private:
  // TODO: Add private members for storing the robot's state estimate (pose, velocity),
  // sensor data buffers, and potentially the state estimation filter (e.g., EKF).

};
