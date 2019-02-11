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
			msg_.temperature = roboclaw_.get_temperature1( roboclawAddress_ );
			std::pair<double, double> currents = roboclaw_.get_currents( roboclawAddress_ );
			msg_.motor1_current = currents.first;
			msg_.motor2_current = currents.second;
		}
		catch( roboclaw::crc_exception ) {
			ROS_ERROR( "CRC error from Roboclaw - ignoring diagnostics" );
			stat.summary( diagnostic_msgs::DiagnosticStatus::WARN, "EMCU CRC error" );
			return;
		}
		// Other exceptions will be thrown

		stat.add( "Status", msg_.status );
		stat.add( "Temperature", msg_.temperature );
		stat.add( "Main battery voltage", msg_.battery_voltage );
		stat.add( "Logic voltage", msg_.logic_voltage );
		stat.add( "Motor 1 current", msg_.motor1_current );
		stat.add( "Motor 2 current", msg_.motor2_current );
		stat.addf( "Raw status", "0x%08x", msg_.status );

		stat.summary( diagnostic_msgs::DiagnosticStatus::OK, "EMCU OK" );

		/*
		Status codes from the Forum (http://forums.basicmicro.com/viewtopic.php?f=2&t=806):

		The first two bytes are now the warning status bits(16bit word)
		#define WARN_NONE	0x00
		#define WARN_OVERCURRENTM1	0x01
		#define WARN_OVERCURRENTM2	0x02
		#define WARN_MBATHIGH	0x04
		#define WARN_MBATLOW	0x08
		#define WARN_TEMP	0x10
		#define WARN_TEMP2	0x20
		#define WARN_S4	0x40	//HOME, LIMITF, LIMITR
		#define WARN_S5	0x80	//HOME, LIMITF, LIMITR

		The second two bytes are now the error status bits(16bit word):
		#define ERROR_NONE	0x0000
		#define ERROR_ESTOP	0x0001
		#define ERROR_TEMP	0x0002
		#define ERROR_TEMP2	0x0004
		#define ERROR_MBATHIGH	0x0008
		#define ERROR_LBATHIGH	0x0010
		#define ERROR_LBATLOW	0x0020
		#define ERROR_FAULTM1	0x0040
		#define ERROR_FAULTM2	0x0080
		#define ERROR_SPEED1	0x0100 //M1 speed limit error
		#define ERROR_SPEED2	0x0200 //M2 speed limit error
		#define ERROR_POS1	0x0400 //M1 position limit error
		#define ERROR_POS2	0x0800 //M2 position limit error
		#define ERROR_MBATLOW	0x4000 //Flag indicates motors are free wheeling due to under voltage but is not a latching error
		#define ERROR_MBATHIGH_HYST	0x8000 //Flag indicates motors are in braking state due to an over voltage, but is not a latching error
		*/

		// Warnings
		if( msg_.status & 0x00010000 )
			stat.mergeSummary( diagnostic_msgs::DiagnosticStatus::WARN, "M1 OverCurrent" );
		if( msg_.status & 0x00020000 )
			stat.mergeSummary( diagnostic_msgs::DiagnosticStatus::WARN, "M1 OverCurrent" );
		if( msg_.status & 0x00040000 )
			stat.mergeSummary( diagnostic_msgs::DiagnosticStatus::WARN, "Main battery high warning" );
		if( msg_.status & 0x00080000 )
			stat.mergeSummary( diagnostic_msgs::DiagnosticStatus::WARN, "Main battery low warning" );
		if( msg_.status & 0x00100000 )
			stat.mergeSummary( diagnostic_msgs::DiagnosticStatus::WARN, "Temperature 1 warning" );
		if( msg_.status & 0x00200000 )
			stat.mergeSummary( diagnostic_msgs::DiagnosticStatus::WARN, "Temperature 2 warning" );
		if( msg_.status & 0x00400000 )
			stat.mergeSummary( diagnostic_msgs::DiagnosticStatus::WARN, "S4 warning" );
		if( msg_.status & 0x00800000 )
			stat.mergeSummary( diagnostic_msgs::DiagnosticStatus::WARN, "S5 warning" );

		// Errors
		if( msg_.status & 0x00000001 )
			stat.mergeSummary( diagnostic_msgs::DiagnosticStatus::ERROR, "E-Stop" );
		if( msg_.status & 0x00000002 )
			stat.mergeSummary( diagnostic_msgs::DiagnosticStatus::ERROR, "Temperature 1" );
		if( msg_.status & 0x00000004 )
			stat.mergeSummary( diagnostic_msgs::DiagnosticStatus::ERROR, "Temperature 2" );
		if( msg_.status & 0x00000008 )
			stat.mergeSummary( diagnostic_msgs::DiagnosticStatus::ERROR, "Main battery high" );
		if( msg_.status & 0x00000010 )
			stat.mergeSummary( diagnostic_msgs::DiagnosticStatus::ERROR, "Logic battery high" );
		if( msg_.status & 0x00000020 )
			stat.mergeSummary( diagnostic_msgs::DiagnosticStatus::ERROR, "Logic battery low" );
		if( msg_.status & 0x00000040 )
			stat.mergeSummary( diagnostic_msgs::DiagnosticStatus::ERROR, "M1 Driver fault" );
		if( msg_.status & 0x00000080 )
			stat.mergeSummary( diagnostic_msgs::DiagnosticStatus::ERROR, "M2 Driver fault" );
		if( msg_.status & 0x00000100 )
			stat.mergeSummary( diagnostic_msgs::DiagnosticStatus::ERROR, "M1 overspeed" );
		if( msg_.status & 0x00000200 )
			stat.mergeSummary( diagnostic_msgs::DiagnosticStatus::ERROR, "M2 overspeed" );
		if( msg_.status & 0x00000400 )
			stat.mergeSummary( diagnostic_msgs::DiagnosticStatus::ERROR, "M1 position" );
		if( msg_.status & 0x00000800 )
			stat.mergeSummary( diagnostic_msgs::DiagnosticStatus::ERROR, "M2 position" );
		if( msg_.status & 0x00004000 )
			stat.mergeSummary( diagnostic_msgs::DiagnosticStatus::WARN, "Non-latching undervoltage" );
		if( msg_.status & 0x00004000 )
			stat.mergeSummary( diagnostic_msgs::DiagnosticStatus::WARN, "Non-latching overvoltage" );
	}
}       // namespace embla_hardware
