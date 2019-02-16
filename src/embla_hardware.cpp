/*
* Copyright 2019 Jens Willy Johannsen <jens@jwrobotics.com>, JW Robotics
*
* License: MIT
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
#include "roboclaw/roboclaw_driver.h"
#include <boost/assign/list_of.hpp>
#include <math.h>
#include <string>

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
		roboclaw_( "/dev/ttyACM0", 460800 ),
		emcu_status_task_( embla_emcu_status_msg_, roboclaw_, ROBOCLAW_ADDRESS )
	{
		private_nh_.param<double>( "wheel_diameter", wheel_diameter_, 0.08 );
		private_nh_.param<double>( "max_accel", max_accel_, 1 );
		private_nh_.param<double>( "max_speed", max_speed_, 1 );    // Note: this is in m/s and is used _after_ converting from rad/s -> m/s
		private_nh_.param<double>( "polling_timeout_", polling_timeout_, 10.0 );
		private_nh_.param<double>( "pulses_per_rev", pulsesPerRev_, 990 );    // 990 pulses per rev. is default for the Embla motors

		// Wait until Roboclaw driver is ready
		if( !roboclaw_.serial->isOpen() )
		{
			ROS_WARN( "Roboclaw port not open - waiting" );
			while( !roboclaw_.serial->isOpen() )
				;
		}

		resetTravelOffset();
		registerControlInterfaces();
		initializeDiagnostics();
	}

	/**
	* Get current encoder travel offsets from MCU and bias future encoder readings against them
	*/
	void EmblaHardware::resetTravelOffset()
	{
		roboclaw_.reset_encoders( ROBOCLAW_ADDRESS );
		std::pair<int, int> encoders = roboclaw_.get_encoders( ROBOCLAW_ADDRESS );

		joints_[ 0 ].position_offset = encoderPulsesToAngular( encoders.first );
		joints_[ 1 ].position_offset = encoderPulsesToAngular( encoders.second );
		joints_[ 2 ].position_offset = encoderPulsesToAngular( encoders.first );
		joints_[ 3 ].position_offset = encoderPulsesToAngular( encoders.second );
		
		ROS_INFO_STREAM( "Initial encoder values: L=" << encoders.first << ", R=" << encoders.second << ". radL=" << joints_[0].position_offset << ", radR=" << joints_[1].position_offset );
	}

	/**
	* Register diagnostic tasks with updater class
	*/
	void EmblaHardware::initializeDiagnostics()
	{
		std::string hardwareID = roboclaw_.get_version( ROBOCLAW_ADDRESS );
		ROS_INFO_STREAM( "EMCU diagnostics hardware ID: '" << hardwareID << "'" );

		diagnostic_updater_.setHardwareID( hardwareID );
		diagnostic_updater_.add( emcu_status_task_ );
		diagnostic_publisher_ = nh_.advertise<EmblaEMCUStatus>( "status", 10 );   // TODO: Do we need the publisher? -JWJ
	}


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
	void EmblaHardware::updateDiagnostics()
	{
		diagnostic_updater_.force_update();
		embla_emcu_status_msg_.header.stamp = ros::Time::now();
		diagnostic_publisher_.publish( embla_emcu_status_msg_ );
	}

	/**
	 * @brief Read function: Pull latest speed and travel measurements from EMCU, and update joint structure for ros_control
	 */
	void EmblaHardware::updateJointsFromHardware()
	{
		try {
			std::pair<int, int> encoders = roboclaw_.get_encoders( ROBOCLAW_ADDRESS );

			for( int i = 0; i < 4; i++ )
			{
				double delta = encoderPulsesToAngular( (i % 2 == 0 ? encoders.first : encoders.second) ) - joints_[ i ].position - joints_[ i ].position_offset;

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

			std::pair<int, int> speeds = roboclaw_.get_velocity( ROBOCLAW_ADDRESS );
			for( int i = 0; i < 4; i++ )
				joints_[ i ].velocity = encoderPulsesToAngular( (i % 2 == 0 ? speeds.first : speeds.second) );
				
			// ROS_INFO_STREAM( "Updated encoders: L=" << encoders.first << ", R=" << encoders.second << ". radL=" << joints_[0].position << ", radR=" << joints_[1].position << ". vL=" << speeds.first << ", vR=" << speeds.second << ". vLrad=" << joints_[0].velocity << ", vRrad=" << joints_[1].velocity );

		} catch( timeout_exception ex ) {
			ROS_ERROR_STREAM( "Roboclaw timeout error in updateJointsFromHardware: " << ex.what() );
			throw ex;
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

		roboclaw_.set_velocity( ROBOCLAW_ADDRESS, std::pair<int,int>( speedLeft, speedRight ) );
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
	double EmblaHardware::encoderPulsesToAngular( const int &encoder ) const
	{
		return (double)encoder / pulsesPerRev_ * 2 * M_PI;
	}

	/**
	 * @brief Converts radians to encoder pulses.
	 * @param angle Number of radians.
	 * @return The corresponding number of encoder pulses.
	 */
	int EmblaHardware::angularToEncoderPulses( const double &angle ) const
	{
		return (int)(angle / (2 * M_PI) * pulsesPerRev_);
	}

}  // namespace embla_hardware
