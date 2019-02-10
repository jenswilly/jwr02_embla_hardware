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
	EmblaEMCUDiagnosticTask::EmblaEMCUDiagnosticTask( EmblaEMCUStatus &msg ) :
		DiagnosticTask( "emcu_status" ),
		msg_( msg )
	{ }

	void EmblaEMCUDiagnosticTask::run( diagnostic_updater::DiagnosticStatusWrapper &stat )
	{
		// Update from hardware
		// TEMP: Dummy values
		msg_.status = 0x0000;
		msg_.temperature = 21.2;
		msg_.battery_voltage = 11.9;
		msg_.logic_voltage = 3.0;
		msg_.motor1_current = 0;
		msg_.motor2_current = 0;

		stat.add( "Status", msg_.status );
		stat.add( "Temperature", msg_.temperature );
		stat.add( "Main battery voltage", msg_.battery_voltage );
		stat.add( "Logic voltage", msg_.logic_voltage );
		stat.add( "Motor 1 current", msg_.motor1_current );
		stat.add( "Motor 2 current", msg_.motor2_current );

	}
}       // namespace embla_hardware
