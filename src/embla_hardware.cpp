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

#include "embla_hardware/embla_hardware.h"
#include "roboclaw/roboclaw.h"
#include <boost/assign/list_of.hpp>
#include <math.h>

namespace
{
	const uint8_t LEFT = 0, RIGHT = 1;
	const uint8_t ROBOCLAW_ADDRESS = 128;
};

namespace embla_hardware
{

	/**
	* Initialize Husky hardware
	*/
	EmblaHardware::EmblaHardware( ros::NodeHandle nh, ros::NodeHandle private_nh, double target_control_freq ) :
		nh_( nh ),
		private_nh_( private_nh ),
		roboclaw_( "/dev/ttyACM0", 460800 ) /*,
	system_status_task_(husky_status_msg_),
	power_status_task_(husky_status_msg_),
	safety_status_task_(husky_status_msg_),
	software_status_task_(husky_status_msg_, target_control_freq) */
	{
		private_nh_.param<double>( "wheel_diameter", wheel_diameter_, 0.08 );
		private_nh_.param<double>( "max_accel", max_accel_, 2.0 );
		private_nh_.param<double>( "max_speed", max_speed_, 1.8 );    // Note: this is in m/s and is used _after_ converting from rad/s -> m/s
		private_nh_.param<double>( "polling_timeout_", polling_timeout_, 10.0 );
		private_nh_.param<double>( "pulses_per_rev", pulsesPerRev_, 990 );    // 990 pulses per rev. is default for the Embla motors

		std::string port;
		private_nh_.param<std::string>( "port", port, "/dev/prolific" );

//    horizon_legacy::connect(port);
//    horizon_legacy::configureLimits(max_speed_, max_accel_);
		resetTravelOffset();
//    initializeDiagnostics();
		registerControlInterfaces();
		
//		ROS_INFO( "Opening Roboclaw port" );
//		roboclaw_.openPort();
	}

	/**
	* Get current encoder travel offsets from MCU and bias future encoder readings against them
	*/
	void EmblaHardware::resetTravelOffset()
	{
		uint32_t encoders[ 2 ];  // Only two encoders. We'll %2 to update all fours joints since order is front_left, front_right, rear_left, rear_right.
		if( roboclaw_.ReadEncoders( ROBOCLAW_ADDRESS, encoders[0], encoders[1] ))
		{
			for( int i = 0; i < 4; i++ )
				joints_[ i ].position_offset = encoderPulsesToAngular( encoders[ i % 2 ] );
		}
		else
		{
			ROS_ERROR( "Could not get encoder data to calibrate travel offset" );
		}
	}

	/**
	* Register diagnostic tasks with updater class
	*/
	/*
	void EmblaHardware::initializeDiagnostics()
	{
	  horizon_legacy::Channel<clearpath::DataPlatformInfo>::Ptr info = horizon_legacy::Channel<clearpath::DataPlatformInfo>::requestData(polling_timeout_);
	  std::ostringstream hardware_id_stream;
	  hardware_id_stream << "Husky " << info->getModel() << "-" << info->getSerial();

	  diagnostic_updater_.setHardwareID(hardware_id_stream.str());
	  diagnostic_updater_.add(system_status_task_);
	  diagnostic_updater_.add(power_status_task_);
	  diagnostic_updater_.add(safety_status_task_);
	  diagnostic_updater_.add(software_status_task_);
	  diagnostic_publisher_ = nh_.advertise<husky_msgs::HuskyStatus>("status", 10);
	}
	*/


	/**
	* Register interfaces with the RobotHW interface manager, allowing ros_control operation
	*/
	void EmblaHardware::registerControlInterfaces()
	{
		ros::V_string joint_names = boost::assign::list_of( "left_front_wheel_joint" )
						    ( "right_front_wheel_joint" ) ( "left_rear_wheel_joint" ) ( "right_rear_wheel_joint" );
		for( unsigned int i = 0; i < joint_names.size(); i++ )
		{
			// Create and register joint state (output to controller)
			hardware_interface::JointStateHandle joint_state_handle( joint_names[i],
										 &joints_[i].position,
										 &joints_[i].velocity,
										 &joints_[i].effort );
			joint_state_interface_.registerHandle( joint_state_handle );

			// Register velocity joint (input from controller)
			hardware_interface::JointHandle joint_handle( joint_state_handle, &joints_[i].velocity_command );
			velocity_joint_interface_.registerHandle( joint_handle );
			ROS_INFO_STREAM( "Control interface registered:" << joint_names[i] );
		}
		registerInterface( &joint_state_interface_ );
		registerInterface( &velocity_joint_interface_ );

	}

	/**
	* External hook to trigger diagnostic update
	*/
	/*
	void EmblaHardware::updateDiagnostics()
	{
	  diagnostic_updater_.force_update();
	  husky_status_msg_.header.stamp = ros::Time::now();
	  diagnostic_publisher_.publish(husky_status_msg_);
	}
	*/

	/**
	 * @brief Read function: Pull latest speed and travel measurements from EMCU, and update joint structure for ros_control
	 */
	void EmblaHardware::updateJointsFromHardware()
	{
		uint32_t encoders[ 2 ];  // Only two encoders. We'll %2 to update all fours joints since order is front_left, front_right, rear_left, rear_right.
	 	if( roboclaw_.ReadEncoders( ROBOCLAW_ADDRESS, encoders[0], encoders[1] ))
		{
        	ROS_WARN_STREAM( "Received encoder information (pulses) L:" << encoders[ LEFT ] << " R:" << encoders[ RIGHT ] );
			for( int i = 0; i < 4; i++ ) 
			{
				double delta = encoderPulsesToAngular( encoders[ i % 2 ] ) - joints_[ i ].position - joints_[ i ].position_offset;

				// 1.0 radians delta is deemed ok. Anything larger that that is "suspiciously large" and might be from encoder rollover
				if( std::abs( delta ) < 1.0 )
					joints_[ i ].position += delta;
				else
				{
					// Suspicious! Drop this measurement and only update the offset for subsequent readings
					joints_[i].position_offset += delta;
					ROS_DEBUG( "Dropping overflow measurement from encoder" );				
				}
			}
		}

		uint32_t speeds[ 2 ];
		if( roboclaw_.ReadISpeeds( ROBOCLAW_ADDRESS, speeds[0], speeds[1] ))
		{
        	ROS_WARN_STREAM( "Received speed information (pulses/sec) L:" << speeds[ LEFT ] << " R:" << speeds[ RIGHT ] );
			for( int i = 0; i < 4; i++ )
				joints_[ i ].velocity = encoderPulsesToAngular( speeds[ i % 2 ] );
		}
	}

	/**
	 * @brief Write function: Get latest velocity commands from ros_control from joint structure's command, and send to EMCU.
	 */
	void EmblaHardware::writeCommandsToHardware()
	{
		double speedLeft = angularToEncoderPulses( joints_[LEFT].velocity_command );
		double speedRight = angularToEncoderPulses( joints_[RIGHT].velocity_command );

		limitDifferentialSpeed( speedLeft, speedRight, max_speed_ * pulsesPerRev_ );



if( speedLeft > 0 )
{
		roboclaw_.SpeedM1( 128, (uint32_t)990 );
//		roboclaw_.SpeedAccelM2( 128, max_accel_ * pulsesPerRev_, speedRight );
		ROS_WARN_STREAM( "Writing to Roboclaw. M1: 990" );
}
else
{
		ROS_WARN_STREAM( "Writing to Roboclaw. L: " << speedLeft << ", R: " << speedRight );
		roboclaw_.SpeedAccelM1( ROBOCLAW_ADDRESS, max_accel_ * pulsesPerRev_, speedLeft );
		roboclaw_.SpeedAccelM2( ROBOCLAW_ADDRESS, max_accel_ * pulsesPerRev_, speedRight );
}
//		roboclaw_.SpeedAccelM1M2( ROBOCLAW_ADDRESS, max_accel_ * pulsesPerRev_, speedLeft, speedRight );
	}

	/**
	* Update diagnostics with control loop timing information
	*/
	void EmblaHardware::reportLoopDuration( const ros::Duration &duration )
	{
		// software_status_task_.updateControlFrequency( 1 / duration.toSec());
	}

	/**
	 * @brief Scale left and right speed outputs to maintain ros_control's desired trajectory. Units are not specified; use same units for all parameters.
	 * @param diff_speed_left  Left speed.
	 * @param diff_speed_right  Right speed.
	 * @param maxSpeed  The maximum allowable speed in the same units as left/right speeds.
	 */
	void EmblaHardware::limitDifferentialSpeed( double &diff_speed_left, double &diff_speed_right, double maxSpeed ) const
	{
		double large_speed = std::max( std::abs( diff_speed_left ), std::abs( diff_speed_right ));

		if( large_speed > maxSpeed )
		{
			diff_speed_left *= maxSpeed / large_speed;
			diff_speed_right *= maxSpeed / large_speed;
		}
	}

	/**
	 * @brief Converts encoder pulses to radians.
	 * @param encoder Encoder value in pulses.
	 * @return Returns the corresponding radian value.
	 */
	double EmblaHardware::encoderPulsesToAngular( const double &encoder ) const
	{
		return encoder / pulsesPerRev_ * 2 * M_PI;
	}

	/**
	 * @brief Converts radians to encoder pulses.
	 * @param angle Number of radians.
	 * @return The corresponding number of encoder pulses.
	 */
	double EmblaHardware::angularToEncoderPulses( const double &angle ) const
	{
		return angle / (2 * M_PI) * pulsesPerRev_;
	}

}  // namespace embla_hardware
