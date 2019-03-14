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

#include "embla_diagnostics.h"
#include "diagnostic_updater/diagnostic_updater.h"
#include "hardware_interface/joint_state_interface.h"
#include "hardware_interface/joint_command_interface.h"
#include "hardware_interface/robot_hw.h"
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "embla_hardware/EmblaEMCUStatus.h"
#include <robot_state_publisher/robot_state_publisher.h>
#include <string>
#include "roboclaw/roboclaw_driver.h"
#include <map>

namespace embla_hardware
{

	/**
	* Hardware class containing controller interfaces (velocity joint handler and joint state updater) and diagnostics publisher and updater
	*/
	class EmblaHardware :
		public hardware_interface::RobotHW
	{
	  public:
		EmblaHardware( ros::NodeHandle nh, ros::NodeHandle private_nh, double target_control_freq, robot_state_publisher::RobotStatePublisher robotStatePublisher );

		void updateJointsFromHardware();

		void writeCommandsToHardware();

		void updateDiagnostics();

		void reportLoopDuration( const ros::Duration &duration );

	  private:

		void initializeDiagnostics();
		void resetTravelOffset();
		void registerControlInterfaces();

		double encoderPulsesToAngular( const int &encoder ) const;
		int angularToEncoderPulses( const double &angle ) const;
		void limitDifferentialSpeed( double &travel_speed_left, double &travel_speed_right, double maxSpeed ) const;

		ros::NodeHandle nh_, private_nh_;
		roboclaw::driver roboclaw_;
		robot_state_publisher::RobotStatePublisher robotStatePublisher_;

		// ROS Control interfaces
		hardware_interface::JointStateInterface joint_state_interface_;
		hardware_interface::VelocityJointInterface velocity_joint_interface_;

		// Diagnostics
		ros::Publisher diagnostic_publisher_;
		EmblaEMCUStatus embla_emcu_status_msg_;
		diagnostic_updater::Updater diagnostic_updater_;
		EmblaEMCUDiagnosticTask emcu_status_task_;

		// ROS Parameters
		double wheel_diameter_, max_accel_, max_speed_, pulsesPerRev_;

		double polling_timeout_;

		/**
		* Joint structure that is hooked to ros_control's InterfaceManager, to allow control via diff_drive_controller
		*/
		struct Joint
		{
			double position;            // In radians
			double position_offset;     // Radians
			double velocity;            // Radians/sec
			double effort;              // (not used)
			double velocity_command;    // ?

			Joint() :
				position( 0 ), velocity( 0 ), effort( 0 ), velocity_command( 0 )
			{ }
		} joints_[2]; // Two joints: left and right
	};

}  // namespace embla_hardware
#endif  // EMBLA_HARDWARE_H
