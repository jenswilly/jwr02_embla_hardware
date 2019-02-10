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
#include "controller_manager/controller_manager.h"
#include "ros/callback_queue.h"

#include <boost/chrono.hpp>

typedef boost::chrono::steady_clock time_source;

/**
* Control loop, not realtime safe
*/
void controlLoop( embla_hardware::EmblaHardware &embla,
		  controller_manager::ControllerManager &cm,
		  time_source::time_point &last_time )
{

	// Calculate monotonic time difference
	time_source::time_point this_time = time_source::now();
	boost::chrono::duration<double> elapsed_duration = this_time - last_time;
	ros::Duration elapsed( elapsed_duration.count());
	last_time = this_time;

	// Process control loop
//	husky.reportLoopDuration( elapsed );
	embla.updateJointsFromHardware();
	cm.update( ros::Time::now(), elapsed );
	embla.writeCommandsToHardware();
}

/**
* Diagnostics loop (not realtime safe because of Roboclaw access)
*/
void diagnosticLoop( embla_hardware::EmblaHardware &embla )
{
	embla.updateDiagnostics();
}

int main( int argc, char *argv[] )
{
	ros::init( argc, argv, "embla_hardware" );
	ros::NodeHandle nh, private_nh( "~" );

	double control_frequency, diagnostic_frequency;
	private_nh.param<double>( "control_frequency", control_frequency, 10.0 );
	private_nh.param<double>( "diagnostic_frequency", diagnostic_frequency, 1.0 );

	// Initialize robot hardware and link to controller manager
	embla_hardware::EmblaHardware embla( nh, private_nh, control_frequency );
	controller_manager::ControllerManager cm( &embla, nh );

	// Setup separate queue and single-threaded spinner to process timer callbacks
	// that interface with Husky hardware - libhorizon_legacy not threadsafe. This
	// avoids having to lock around hardware access, but precludes realtime safety
	// in the control loop.
	ros::CallbackQueue embla_queue;
	ros::AsyncSpinner embla_spinner( 1, &embla_queue );

	time_source::time_point last_time = time_source::now();
	ros::TimerOptions control_timer(
		ros::Duration( 1 / control_frequency ),
		boost::bind( controlLoop, boost::ref( embla ), boost::ref( cm ), boost::ref( last_time )),
		&embla_queue );
	ros::Timer control_loop = nh.createTimer( control_timer );

	ros::TimerOptions diagnostic_timer(
		ros::Duration( 1 / diagnostic_frequency ),
		boost::bind( diagnosticLoop, boost::ref( embla )),
		&embla_queue );
	ros::Timer diagnostic_loop = nh.createTimer( diagnostic_timer );

	embla_spinner.start();

	// Process remainder of ROS callbacks separately, mainly ControlManager related
	ros::spin();

	return 0;
}
