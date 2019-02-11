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

#include "embla_hardware/embla_diagnostics.h"

namespace embla_hardware
{
	EmblaEMCUDiagnosticTask::EmblaEMCUDiagnosticTask( EmblaEMCUStatus &msg, roboclaw::driver &roboclaw, uint8_t roboclawAddress ) :
		DiagnosticTask( "emcu_status" ),
		msg_( msg ),
		roboclaw_( roboclaw ),
		roboclawAddress_( roboclawAddress )
	{ }

	void EmblaEMCUDiagnosticTask::run( diagnostic_updater::DiagnosticStatusWrapper &stat )
	{
		// Update from hardware
		try {
			msg_.status = roboclaw_.get_status( roboclawAddress_ );
			msg_.battery_voltage = roboclaw_.get_battery_voltage( roboclawAddress_ );
			msg_.logic_voltage = roboclaw_.get_logic_voltage( roboclawAddress_ );
			msg_.temperature = roboclaw_.get_temperature1( roboclawAddress_ );      // TODO: temp
			std::pair<double, double> currents = roboclaw_.get_currents( roboclawAddress_ );        // TODO: temp
			msg_.motor1_current = currents.first;
			msg_.motor2_current = currents.second;
		}
		catch( roboclaw::crc_exception ) {
			ROS_ERROR( "CRC error from Roboclaw – ignoring diagnostics" );
			stat.summary( diagnostic_msgs::DiagnosticStatus::WARN, "EMCU CRC error" );
			return;
		}
		// Other exceptions will be thrown

//		msg_.status = 0x0000;
//		msg_.battery_voltage = 10.0;
//		msg_.logic_voltage = 3.0;
//		msg_.temperature = 20.0;
//		std::pair<double, double> currents( 1, 1 );

		stat.add( "Status", msg_.status );
		stat.add( "Temperature", msg_.temperature );
		stat.add( "Main battery voltage", msg_.battery_voltage );
		stat.add( "Logic voltage", msg_.logic_voltage );
		stat.add( "Motor 1 current", msg_.motor1_current );
		stat.add( "Motor 2 current", msg_.motor2_current );

		// Status codes from the manual
		stat.summary( diagnostic_msgs::DiagnosticStatus::OK, "EMCU Status OK" );
		if( msg_.status & 0x0001 )
			stat.mergeSummary( diagnostic_msgs::DiagnosticStatus::WARN, "M1 OverCurrent" );
		if( msg_.status & 0x0002 )
			stat.mergeSummary( diagnostic_msgs::DiagnosticStatus::WARN, "M2 OverCurrent" );
		if( msg_.status & 0x0004 )
			stat.mergeSummary( diagnostic_msgs::DiagnosticStatus::ERROR, "E-Stop" );
		if( msg_.status & 0x0008 )
			stat.mergeSummary( diagnostic_msgs::DiagnosticStatus::ERROR, "Temperatur 1" );
		if( msg_.status & 0x0010 )
			stat.mergeSummary( diagnostic_msgs::DiagnosticStatus::ERROR, "Temperature 2" );
		if( msg_.status & 0x0020 )
			stat.mergeSummary( diagnostic_msgs::DiagnosticStatus::ERROR, "Main battery high" );
		if( msg_.status & 0x0040 )
			stat.mergeSummary( diagnostic_msgs::DiagnosticStatus::ERROR, "Logic battery high" );
		if( msg_.status & 0x0080 )
			stat.mergeSummary( diagnostic_msgs::DiagnosticStatus::ERROR, "Logic battery low" );
		if( msg_.status & 0x0100 )
			stat.mergeSummary( diagnostic_msgs::DiagnosticStatus::ERROR, "M1 Driver fault" );
		if( msg_.status & 0x0200 )
			stat.mergeSummary( diagnostic_msgs::DiagnosticStatus::ERROR, "M2 Driver fault" );
		if( msg_.status & 0x0400 )
			stat.mergeSummary( diagnostic_msgs::DiagnosticStatus::WARN, "Main battery high warning" );
		if( msg_.status & 0x0800 )
			stat.mergeSummary( diagnostic_msgs::DiagnosticStatus::WARN, "Main battery low warning" );
		if( msg_.status & 0x1000 )
			stat.mergeSummary( diagnostic_msgs::DiagnosticStatus::WARN, "Temperature 1 warning" );
		if( msg_.status & 0x2000 )
			stat.mergeSummary( diagnostic_msgs::DiagnosticStatus::WARN, "Temperature 2 warning" );
		if( msg_.status & 0x4000 )
			stat.mergeSummary( diagnostic_msgs::DiagnosticStatus::WARN, "M1 Home" );
		if( msg_.status & 0x8000 )
			stat.mergeSummary( diagnostic_msgs::DiagnosticStatus::WARN, "M2 Home" );
	}
}       // namespace embla_hardware
