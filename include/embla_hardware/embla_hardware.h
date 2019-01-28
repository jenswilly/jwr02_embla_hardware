/*
* Copyright 2019 Jens Willy Johannsen <jens@jwrobotics.com>, JW Robotics
*
* Permission is hereby granted, free of charge, to any person obtaining a copy
* of this software and associated documentation files (the "Software"), to deal
* in the Software without restriction, including without limitation the rights
* to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
* copies of the Software, and to permit persons to whom the Software is
* furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included in
* all copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
* OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
* THE SOFTWARE.
*
* JWR-01 Embla Hardware driver/controller
*/

#ifndef EMBLA_HARDWARE_H
#define EMBLA_HARDWARE_H

//#include "husky_base/husky_diagnostics.h"
#include "diagnostic_updater/diagnostic_updater.h"
#include "hardware_interface/joint_state_interface.h"
#include "hardware_interface/joint_command_interface.h"
#include "hardware_interface/robot_hw.h"
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
//#include "husky_msgs/HuskyStatus.h"
#include <string>

namespace embla_hardware
{

  /**
  * Class representing Husky hardware, allows for ros_control to modify internal state via joint interfaces
  */
  class EmblaHardware :
    public hardware_interface::RobotHW
  {
  public:
    EmblaHardware(ros::NodeHandle nh, ros::NodeHandle private_nh, double target_control_freq);

    void updateJointsFromHardware();

    void writeCommandsToHardware();

    void updateDiagnostics();

    void reportLoopDuration(const ros::Duration &duration);

  private:

    // void initializeDiagnostics();

    void resetTravelOffset();

    void registerControlInterfaces();

    double linearToAngular(const double &travel) const;

    double angularToLinear(const double &angle) const;

    void limitDifferentialSpeed(double &travel_speed_left, double &travel_speed_right);

    ros::NodeHandle nh_, private_nh_;
	
	RoboClaw _roboclaw;

    // ROS Control interfaces
    hardware_interface::JointStateInterface joint_state_interface_;
    hardware_interface::VelocityJointInterface velocity_joint_interface_;

    // Diagnostics
	/*
    ros::Publisher diagnostic_publisher_;
    husky_msgs::HuskyStatus husky_status_msg_;
    diagnostic_updater::Updater diagnostic_updater_;
    HuskyHardwareDiagnosticTask<clearpath::DataSystemStatus> system_status_task_;
    HuskyHardwareDiagnosticTask<clearpath::DataPowerSystem> power_status_task_;
    HuskyHardwareDiagnosticTask<clearpath::DataSafetySystemStatus> safety_status_task_;
    HuskySoftwareDiagnosticTask software_status_task_;
	*/

    // ROS Parameters
    double wheel_diameter_, max_accel_, max_speed_;

    double polling_timeout_;

    /**
    * Joint structure that is hooked to ros_control's InterfaceManager, to allow control via diff_drive_controller
    */
    struct Joint
    {
      double position;
      double position_offset;
      double velocity;
      double effort;
      double velocity_command;

      Joint() :
        position(0), velocity(0), effort(0), velocity_command(0)
      { }
    } joints_[2];	// Two joints: left and right
  };

}  // namespace embla_hardware
#endif  // EMBLA_HARDWARE_H
