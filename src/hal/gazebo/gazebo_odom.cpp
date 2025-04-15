#include <iostream>
#include <string>
#include <gz/msgs.hh>
#include <gz/transport.hh>

#include "csc/sensors/odom/odom.h"
#include "msg/odom_msg.h"
#include "gazebo_helpers.h"

static gz::transport::Node node;
static std::function<void(const msg::OdomDataMsg&)> odom_callback;
static bool running = false;

Odom::Odom() { }

void Odom::start(std::function<void(const msg::OdomDataMsg&)> callback)
{
  odom_callback = callback;
  running = true;

  node.Subscribe<gz::msgs::Odometry>("/odom", [this](const gz::msgs::Odometry &msg)
  {
    if (!running) return;

    msg::Header extracted_header = gazebo_helper::extract_header(msg);

    msg::OdomDataMsg odom_data = {
      .header = {
        .seq = extracted_header.seq,
        .stamp = {
          .sec = extracted_header.stamp.sec,
          .nsec = extracted_header.stamp.nsec
        },
        .frame_id = extracted_header.frame_id
      },
      
      .pose = {
        .point = {
          msg.pose().position().x(),
          msg.pose().position().y(),
          msg.pose().position().z()
        },
        .orientation = {
          msg.pose().orientation().x(),
          msg.pose().orientation().y(),
          msg.pose().orientation().z(),
          msg.pose().orientation().w()
        },
        // The covariance matrix is initialized to zero.
        .covariance = linalg::Matrix<6, 6>()
      },
      .twist = {
        .twist = {
          .linear = {
            msg.twist().linear().x(),
            msg.twist().linear().y(),
            msg.twist().linear().z()
          },
          .angular = {
            msg.twist().angular().x(),
            msg.twist().angular().y(),
            msg.twist().angular().z()
          }
        },
        // The covariance matrix is initialized to zero.
        .covariance = linalg::Matrix<6, 6>()
      }
    };

    if (odom_callback)
    odom_callback(odom_data);
  });
}

void Odom::stop()
{
  running = false;
}
