#include <gz/transport.hh>
#include <gz/msgs.hh>

#include "csc/control/motion_control/diff_drive.h"
#include "msg/cmdvel_msg.h"
#include "gazebo_helpers.h"

static gz::transport::Node node;
static gz::transport::Node::Publisher pub;
static bool running = false;

DiffDrive::DiffDrive() { }


void DiffDrive::start()
{
  running = true;
  if (!pub)
  {
    pub = node.Advertise<gz::msgs::Twist>("/cmd_vel");
  }
}

void DiffDrive::stop()
{
  running = false;
}

void DiffDrive::send_cmd_vel(const msg::CmdVelMsg& cmdvel)
{
  if (!running || !pub) return;

  gz::msgs::Twist msg;
  msg.mutable_linear()->set_x(cmdvel.twist.linear(0));
  msg.mutable_linear()->set_y(cmdvel.twist.linear(1));
  msg.mutable_linear()->set_z(cmdvel.twist.linear(2));
  msg.mutable_angular()->set_x(cmdvel.twist.angular(0));
  msg.mutable_angular()->set_y(cmdvel.twist.angular(1));
  msg.mutable_angular()->set_z(cmdvel.twist.angular(2));

  pub.Publish(msg);
}
