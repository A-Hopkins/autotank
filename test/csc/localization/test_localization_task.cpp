#include <gtest/gtest.h>
#include "protocore/include/broker.h"
#include "csc/localization/localization_task.h"


class LocalizationTaskTest : public ::testing::Test
{
protected:
  std::shared_ptr<LocalizationTask> task;

  void SetUp() override
  {
    // Initialize the publisher and subscriber broker
    Broker::initialize();
    task = LocalizationTask::create();
  }
};

// Helper to build a minimal IMUDataMsg
static msg::IMUDataMsg make_imu(double yaw, double wz)
{
  msg::IMUDataMsg imu{};
  // quaternion [0,0,sin(yaw/2),cos(yaw/2)]
  double half = yaw * 0.5;
  imu.orientation = {0.0, 0.0, std::sin(half), std::cos(half)};
  imu.angular_velocity = {0.0, 0.0, wz};
  return imu;
}

// Helper to build a minimal OdomDataMsg
static msg::OdomDataMsg make_odom(double x, double y, double yaw, double vx, double wz)
{
  msg::OdomDataMsg odom{};
  odom.pose.pose.point    = {x, y, 0.0};
  double half = yaw * 0.5;
  odom.pose.pose.orientation = {0.0, 0.0, std::sin(half), std::cos(half)};
  odom.twist.twist.linear   = {vx, 0.0, 0.0};
  odom.twist.twist.angular  = {0.0, 0.0, wz};
  return odom;
}

TEST_F(LocalizationTaskTest, IMUUpdateSetsCorrectPosterior)
{
  // measurement:
  double meas_yaw = 0.9;
  double meas_wz = 1.5;
  auto imu = make_imu(meas_yaw, meas_wz);

  // do the update
  task->handle_sensor_data(msg::Msg(task.get(), imu));

  // pull out the private state
  auto &est = task->current_state_est;

  // expected K = P0/(P0+R) = 0.01/(0.01+0.01) = 0.5
  double post_yaw = 0.5 * meas_yaw;
  EXPECT_NEAR(est.est_pose.pose.orientation(2), std::sin(post_yaw/2), 1e-9);
  EXPECT_NEAR(est.est_twist.twist.angular(2), 0.5 * meas_wz, 1e-9);
}

TEST_F(LocalizationTaskTest, OdomUpdateSetsCorrectPosterior)
{
  // P0(x)=1.0, R_odom(x)=0.01 => Kx = 1/(1+0.01) ~0.990099
  // P0(θ)=0.01, R_odom(θ)=0.01 => Kθ = 0.01/(0.01+0.01)=0.5
  // P0(v)=0.01, R_odom(v)=0.01 => Kv = 0.01/(0.01+0.01)=0.5
  double mx = 1.1;
  double my = -2.2;
  double myaw = 1.5;
  double mvx = 0.7;
  double mwz = 0.02;
  auto odom = make_odom(mx, my, myaw, mvx, mwz);

  task->handle_sensor_data(msg::Msg(task.get(), odom));
  auto &est = task->current_state_est;

  double Kx = 1.0 / 1.01;
  double Ktheta = 0.5;
  double Kv = 0.5;

  // x, y
  EXPECT_NEAR(est.est_pose.pose.point(0), Kx*mx, 1e-9);
  EXPECT_NEAR(est.est_pose.pose.point(1), Kx*my, 1e-9);

  // yaw -> quaternion
  double post_yaw = Ktheta * myaw;
  EXPECT_NEAR(est.est_pose.pose.orientation(2), std::sin(post_yaw/2), 1e-9);

  // forward velocity
  EXPECT_NEAR(est.est_twist.twist.linear(0), Kv * mvx, 1e-9);

  // angular velocity
  EXPECT_NEAR(est.est_twist.twist.angular(2), Kv * mwz, 1e-9);
}


TEST_F(LocalizationTaskTest, SequentialFusionUpdatesState)
{
  // 1) IMU update
  double imu_yaw = 0.4;
  double imu_wz = 0.1;

  task->handle_sensor_data(msg::Msg(task.get(), make_imu(imu_yaw, imu_wz)));

  // 2) Odom update (uses same yaw and velocity dims)
  double ox = 0.25;
  double oy = 0.0;
  double oyaw = imu_yaw;
  double ovx = 0.5;
  double owz = imu_wz;
  auto od = make_odom(ox, oy, oyaw, ovx, owz);
  task->handle_sensor_data(msg::Msg(task.get(), od));

  auto &est = task->current_state_est;

  // Gains after IMU and odom on x,y dims remain Kx~0.990099
  double Kx = 1.0 / 1.01;
  EXPECT_NEAR(est.est_pose.pose.point(0), Kx * ox, 1e-9);

  // Pθ after IMU update = 0.005 => Kθ2 = 0.005/(0.005+0.01)=1/3
  double Ktheta2 = 0.005 / (0.015);
  // New posterior yaw = Ktheta2*odom_yaw + (1-Ktheta2)*(IMU_posterior_yaw)
  double imu_post = 0.5 * imu_yaw;
  double final_yaw = Ktheta2 * oyaw + (1 - Ktheta2) * imu_post;
  EXPECT_NEAR(est.est_pose.pose.orientation(2), std::sin(final_yaw / 2), 1e-9);

  // Similarly for angular velocity:
  double Kw2 = Ktheta2; // same P0,R
  double final_wz = Kw2 * owz + (1-Kw2) * (0.5*imu_wz);
  EXPECT_NEAR(est.est_twist.twist.angular(2), final_wz, 1e-9);

  // Forward velocity: P0=0.01 unmodified by IMU, so Kv=0.5
  double Kv = 0.5;
  EXPECT_NEAR(est.est_twist.twist.linear(0), Kv * ovx, 1e-9);
}
