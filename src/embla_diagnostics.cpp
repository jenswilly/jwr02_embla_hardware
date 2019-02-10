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
	EmblaEMCUDiagnosticTask::EmblaEMCUDiagnosticTask( EmblaEMCUStatus &msg, roboclaw::driver roboclaw ) :
		DiagnosticTask( "emcu_status" ),
		msg_( msg ),
		roboclaw_( roboclaw )
	{ }

	void EmblaEMCUDiagnosticTask::run( diagnostic_updater::DiagnosticStatusWrapper &stat )
	{
		// Update from hardware
		msg_.status = roboclaw_.get_status( ROBOCLAW_ADDRESS );
		msg_.battery_voltage = roboclaw_.get_battery_voltage( ROBOCLAW_ADDRESS );
		msg_.logic_voltage = roboclaw_.get_logic_voltage( ROBOCLAW_ADDRESS );
		msg_.temperature = roboclaw_.get_temperature1( ROBOCLAW_ADDRESS );	// TODO: temp
		std::pair<double, double> currents = roboclaw_.get_currents( ROBOCLAW_ADDRESS );	// TODO: temp
		msg_.motor1_current = currents.first;
		msg_.motor2_current = currents.second;

		stat.add( "Status", msg_.status );
		stat.add( "Temperature", msg_.temperature );
		stat.add( "Main battery voltage", msg_.battery_voltage );
		stat.add( "Logic voltage", msg_.logic_voltage );
		stat.add( "Motor 1 current", msg_.motor1_current );
		stat.add( "Motor 2 current", msg_.motor2_current );

		stat.summary( diagnostic_msgs::DiagnosticStatus::OK, "EMCU Status OK" );
		if( msg_.status != 0x0000 )
			stat.mergeSummaryf( diagnostic_msgs::DiagnosticStatus::ERROR, "Unexpected status: %04x", msg_.status );
	}
}       // namespace embla_hardware
