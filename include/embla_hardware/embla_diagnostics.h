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

#ifndef EMBLA_DIAGNOSTICS_H
#define EMBLA_DIAGNOSTICS_H

#include "ros/ros.h"
#include <diagnostic_updater/diagnostic_updater.h>
#include "embla_hardware/EmblaEMCUStatus.h"
#include "roboclaw/roboclaw_driver.h"

namespace embla_hardware
{
	class EmblaEMCUDiagnosticTask : public diagnostic_updater::DiagnosticTask
	{
	  public:
		explicit EmblaEMCUDiagnosticTask( EmblaEMCUStatus &msg, roboclaw::driver &roboclaw, uint8_t roboclawAddress );
		void run( diagnostic_updater::DiagnosticStatusWrapper &stat );

	  private:
		EmblaEMCUStatus &msg_;  // TODO: This is used to also publish the status on the "status" topic. Maybe not needed? -JWJ
		roboclaw::driver &roboclaw_;
		uint8_t roboclawAddress_;
	};
}       // namespace embla_hardware
#endif
