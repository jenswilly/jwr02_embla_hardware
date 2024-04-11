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

// #include "embla_hardware/embla_hardware.h"
#include "rclcpp/rclcpp.hpp"
// #include "controller_manager/controller_manager.h"
// #include "ros/callback_queue.h"
// #include <robot_state_publisher/robot_state_publisher.h>
// #include <kdl_parser/kdl_parser.hpp>
#include <boost/chrono.hpp>

typedef boost::chrono::steady_clock time_source;
#if 0
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

/**
 * Publish static transforms (part of robot_state_publisher)
 * */
void staticTfLoop( robot_state_publisher::RobotStatePublisher &robotStatePublisher, std::string &tf_prefix )
{
	// Reading KDL tree: http://wiki.ros.org/kdl_parser/Tutorials/Start%20using%20the%20KDL%20parser
	// Calling robot_state_publisher as library: http://wiki.ros.org/robot_state_publisher/Tutorials/Using%20the%20robot%20state%20publisher%20on%20your%20own%20robot
	robotStatePublisher.publishFixedTransforms( tf_prefix, true );	// Experiment: use_tf_static=true
}

int main( int argc, char *argv[] )
{
	ros::init( argc, argv, "embla_hardware" );
	ros::NodeHandle nh, private_nh( "~" );

	double control_frequency, diagnostic_frequency, static_tf_frequency;
	std::string tf_prefix;
	bool publish_tf, static_only;

	private_nh.param<double>( "control_frequency", control_frequency, 10.0 );
	private_nh.param<double>( "diagnostic_frequency", diagnostic_frequency, 1.0 );
	private_nh.param<double>( "static_tf_frequency", static_tf_frequency, 5.0 );
	private_nh.param<std::string>( "tf_prefix", tf_prefix, std::string("") );
	private_nh.param<bool>( "publish_tf", publish_tf, false );
	private_nh.param<bool>( "static_only", static_only, false );

	// Initialize robot state publisher
	KDL::Tree tree;
	std::string robot_desc_string;
	nh.param( "robot_description", robot_desc_string, std::string() );
	if( !kdl_parser::treeFromString( robot_desc_string, tree )) {
		ROS_ERROR("Failed to construct KDL tree");
		return false;
	}
	robot_state_publisher::RobotStatePublisher robotStatePublisher( tree );

	// Setup separate queue and single-threaded spinner to process timer callbacks
	// that interface with Husky hardware - libhorizon_legacy not threadsafe. This
	// avoids having to lock around hardware access, but precludes realtime safety
	// in the control loop.
	ros::CallbackQueue embla_queue;
	ros::AsyncSpinner embla_spinner( 1, &embla_queue );
	time_source::time_point last_time = time_source::now();

	// Initialize robot hardware and link to controller manager. They are only instantiated if !static_only
	embla_hardware::EmblaHardware *embla;
	controller_manager::ControllerManager *cm;
	ros::Timer control_loop;
	ros::Timer diagnostic_loop;

	if( !static_only )
	{
		embla = new embla_hardware::EmblaHardware( nh, private_nh, control_frequency, robotStatePublisher );
		cm = new controller_manager::ControllerManager( embla, nh );

		// Control manager timer loop
		ros::TimerOptions control_timer(
			ros::Duration( 1 / control_frequency ),
			boost::bind( controlLoop, boost::ref( *embla ), boost::ref( *cm ), boost::ref( last_time )),
			&embla_queue );
		control_loop = nh.createTimer( control_timer );

		// Diagnostics timer loop
		ros::TimerOptions diagnostic_timer(
			ros::Duration( 1 / diagnostic_frequency ),
			boost::bind( diagnosticLoop, boost::ref( *embla )),
			&embla_queue );
		diagnostic_loop = nh.createTimer( diagnostic_timer );
	}

	// Loop for robot_state_publisher static tf publishing
	ros::Timer static_tf_loop;
	if( publish_tf ) {
		ros::TimerOptions static_tf_timer(
			ros::Duration( 1 / static_tf_frequency ),
			boost::bind( staticTfLoop, boost::ref( robotStatePublisher ), boost::ref( tf_prefix )),
			&embla_queue );
		static_tf_loop = nh.createTimer( static_tf_timer );
	}

	embla_spinner.start();

	// Process remainder of ROS callbacks separately, mainly ControlManager related
	ros::spin();

	return 0;
}
#endif

class EmblaHardwareNode : public rclcpp::Node
{
public:
	EmblaHardwareNode() : Node("embla_hardware")
	{
		RCLCPP_INFO(this->get_logger(), "EmblaHardware constructor");
	}
};

int main(int argc, char *argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<EmblaHardwareNode>());
	rclcpp::shutdown();
	return 0;
}