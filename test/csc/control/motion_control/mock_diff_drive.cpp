#include "csc/control/motion_control/diff_drive.h"
#include "msg/cmdvel_msg.h"

// Provide a dummy implementation for DiffDrive methods

DiffDrive::DiffDrive() {}

void DiffDrive::start() {}
void DiffDrive::stop() {}
void DiffDrive::send_cmd_vel(const msg::CmdVelMsg& cmd) {}
