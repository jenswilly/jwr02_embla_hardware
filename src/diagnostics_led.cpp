/*
* Copyright 2018 Jens Willy Johannsen <jens@jwrobotics.com>, JW Robotics
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
* Diagnostics -> LEDs node
*
* Subscribes to:
*	/sbus (Sbus)
*	/diagnostics (?)
* Calls services:
*	i2c_service
*/

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <cstdlib>

#include <sbus_serial/Sbus.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <i2c_service/I2CWriteRegisterByte.h>

#define LED_ORANGE 0x01
#define LED_BLUE 0x02
#define LED_RED 0x04
#define LED_GREEN 0x08
#define MAX_SBUS_INTERVAL 1.0     // Seconds before SBUS link will be considered inactive

class DiagnosticLEDUpdater
{
  public:
	DiagnosticLEDUpdater() :
		lastSbusTimestamp_( 0 ), sbusIsActive_( false ), ledValue_( LED_RED | LED_ORANGE )
	{
		diagnostics_sub_ = nh_.subscribe( "diagnostics_toplevel_state", 1, &DiagnosticLEDUpdater::diagnosticsCallback, this );
		sbus_sub_ = nh_.subscribe( "sbus", 1, &DiagnosticLEDUpdater::sbusCallback, this );


		// Configure MCP23017 inline - should probably be moved. -JWJ
		i2c_client_ = nh_.serviceClient<i2c_service::I2CWriteRegisterByte>( "/i2c_write_byte" );
		i2c_service::I2CWriteRegisterByte i2c_write_srv;
		i2c_write_srv.request.address = 0x20;
		i2c_write_srv.request.reg = 0x00;
		i2c_write_srv.request.value = 0x00;
		if( !i2c_client_.call( i2c_write_srv ))
			ROS_ERROR( "Unable to send configuration request to MCP23017" );

		// Set initial LED state
		updateLEDs();
	}

	void diagnosticsCallback( const diagnostic_msgs::DiagnosticStatus::ConstPtr& msg )
	{
		// Preserve the curent status of everything except orange, red and green LEDs
		uint8_t value = ledValue_ & 0xF2;

		switch( msg->level ) {
		case diagnostic_msgs::DiagnosticStatus::STALE:
			// ROS_WARN( "Diagnostics STALE" );
			value |= LED_ORANGE;
			break;

		case diagnostic_msgs::DiagnosticStatus::WARN:
			// ROS_WARN( "Diagnostics WARNING" );
			value |= LED_ORANGE;
			break;

		case diagnostic_msgs::DiagnosticStatus::ERROR:
			// ROS_ERROR( "Diagnostics ERROR" );
			value |= LED_RED;
			break;

		case diagnostic_msgs::DiagnosticStatus::OK:
			value |= LED_GREEN;
			break;

		default:
			// ( "Other status..." );
			value |= LED_RED;
			break;
		}

		// Only update if different from current
		if( value != ledValue_ )
		{
			ledValue_ = value;
			updateLEDs();
		}
	}

	void sbusCallback( const sbus_serial::Sbus::ConstPtr& msg )
	{
		// Update timestamp
		lastSbusTimestamp_ = msg->header.stamp;
	}

	/**
	 * @brief Sets SBUS link active or inactive.
	 * @param isActive Specify whether the SBUS link is currently active or not.
	 * This will update LEDs if the value is different from the current value.
	 */
	void setSBUSActive( bool isActive )
	{
		if( isActive != sbusIsActive_ )
		{
			sbusIsActive_ = isActive;
			if( sbusIsActive_ )
				ledValue_ |= LED_BLUE;
			else
				ledValue_ &= ~LED_BLUE;

			updateLEDs();
		}
	}

	ros::Time lastSBUSTimestamp()
	{
		return lastSbusTimestamp_;
	}

  private:
	ros::NodeHandle nh_;
	ros::Subscriber diagnostics_sub_;
	ros::Subscriber sbus_sub_;
	ros::ServiceClient i2c_client_;
	uint8_t ledValue_;
	ros::Time lastSbusTimestamp_;
	bool sbusIsActive_;

	/**
	 * @brief Updates LEDs with the current value of `ledValue_`.
	 * @return Returns true if the operation succeeded; false if not.
	 */
	bool updateLEDs()
	{
		i2c_service::I2CWriteRegisterByte i2c_write_srv;
		i2c_write_srv.request.address = 0x20;
		i2c_write_srv.request.reg = 0x12;
		i2c_write_srv.request.value = ledValue_;
		if( !i2c_client_.call( i2c_write_srv ))
		{
			ROS_ERROR( "Unable to send LED update request to MCP23017" );
			return false;
		}

		return true;
	}
};

int main( int argc, char **argv )
{
	ros::init( argc, argv, "diagnostics_led" );

	DiagnosticLEDUpdater diagnosticsLEDUpdater;
	ROS_INFO( "Embla LED diagnostics ready..." );

	ros::Rate rate( 25 );   // 25 Hz hardcoded
	while( ros::ok() )
	{
		// Check SBUS timeout
		ros::Duration interval = ros::Time::now() - diagnosticsLEDUpdater.lastSBUSTimestamp();
		diagnosticsLEDUpdater.setSBUSActive( (interval.toSec() < MAX_SBUS_INTERVAL) );

		ros::spinOnce();
		rate.sleep();
	}

	return 0;
}
