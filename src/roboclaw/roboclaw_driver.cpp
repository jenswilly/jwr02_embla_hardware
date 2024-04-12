/**
 *
 * Copyright (c) 2018 Carroll Vance.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 */

#include "roboclaw/roboclaw_driver.h"

#include <boost/thread/mutex.hpp>
#include <boost/array.hpp>
#include <iomanip>

namespace roboclaw
{

	unsigned char driver::BASE_ADDRESS = 128;
	unsigned int driver::DEFAULT_BAUDRATE = 115200;

	driver::driver(std::string port, unsigned int baudrate)
	{
		serial = std::shared_ptr<TimeoutSerial>(new TimeoutSerial(port, baudrate));
		//		serial->setTimeout(boost::posix_time::milliseconds(200));
		serial->setTimeout(boost::posix_time::milliseconds(10000));
	}

	void driver::crc16_reset()
	{
		crc = 0;
	}

	uint16_t driver::crc16(uint8_t *packet, size_t nBytes)
	{

		for (size_t byte = 0; byte < nBytes; byte++)
		{

			crc = crc ^ ((uint16_t)packet[byte] << 8);

			for (uint8_t bit = 0; bit < 8; bit++)
			{
				if (crc & 0x8000)
					crc = (crc << 1) ^ 0x1021;
				else
					crc = crc << 1;
			}
		}

		return crc;
	}

	size_t driver::txrx(unsigned char address,
						unsigned char command,
						unsigned char *tx_data,
						size_t tx_length,
						unsigned char *rx_data,
						size_t rx_length,
						bool tx_crc, bool rx_crc)
	{

		boost::mutex::scoped_lock lock(serial_mutex);

		std::vector<unsigned char> packet;

		if (tx_crc)
			packet.resize(tx_length + 4);
		else
			packet.resize(tx_length + 2);

		// Header
		packet[0] = address;
		packet[1] = command;

		crc16_reset();
		crc16(&packet[0], 2);

		// Data
		if (tx_length > 0 && tx_data != nullptr)
			memcpy(&packet[2], tx_data, tx_length);

		// CRC
		if (tx_crc)
		{
			unsigned int crc = crc16(&packet[2], tx_length);

			// RoboClaw expects big endian / MSB first
			packet[tx_length + 2] = (unsigned char)((crc >> 8) & 0xFF);
			packet[tx_length + 2 + 1] = (unsigned char)(crc & 0xFF);
		}

		serial->write((char *)&packet[0], packet.size());

		size_t want_bytes;
		if (rx_crc)
			want_bytes = rx_length + 2;
		else
			want_bytes = rx_length;

		std::vector<char> response_vector;

		response_vector = serial->read(want_bytes);

		size_t bytes_received = response_vector.size();

		unsigned char *response = (unsigned char *)&response_vector[0];

		if (bytes_received != want_bytes)
		{
			std::ostringstream oss;
			oss << "Timeout reading from RoboClaw. Expected " << want_bytes << ", got " << bytes_received;
			throw timeout_exception(oss.str());
		}

		// Check CRC
		if (rx_crc)
		{
			unsigned int crc_calculated = crc16(&response[0], bytes_received - 2);
			unsigned int crc_received = 0;

			// RoboClaw generates big endian / MSB first
			crc_received += response[bytes_received - 2] << 8;
			crc_received += response[bytes_received - 1];

			if (crc_calculated != crc_received)
			{
				throw roboclaw::crc_exception("CRC error in read data.");
			}

			memcpy(rx_data, &response[0], bytes_received - 2);
		}
		else
		{
			memcpy(rx_data, &response[0], bytes_received);
		}

		if (!rx_crc)
			return bytes_received;
		else
			return bytes_received - 2;
	}

	/**
	 * @brief Transmits specifed TX data and reads string until a \0 is received. RX CRC _is_ performed.
	 * @param rx_length is the MAX length to receive. If a \0 has not been received when this many bytes have been read, a timeout exception will be thrown.
	 **/
	std::string driver::txrx(unsigned char address,
							 unsigned char command,
							 unsigned char *tx_data,
							 size_t tx_length,
							 size_t rx_length,
							 bool tx_crc)
	{

		boost::mutex::scoped_lock lock(serial_mutex);

		std::vector<unsigned char> packet;

		if (tx_crc)
			packet.resize(tx_length + 4);
		else
			packet.resize(tx_length + 2);

		// Header
		packet[0] = address;
		packet[1] = command;

		crc16_reset();
		crc16(&packet[0], 2);

		// Data
		if (tx_length > 0 && tx_data != nullptr)
			memcpy(&packet[2], tx_data, tx_length);

		// CRC
		if (tx_crc)
		{
			unsigned int crc = crc16(&packet[2], tx_length);

			// RoboClaw expects big endian / MSB first
			packet[tx_length + 2] = (unsigned char)((crc >> 8) & 0xFF);
			packet[tx_length + 2 + 1] = (unsigned char)(crc & 0xFF);
		}

		serial->write((char *)&packet[0], packet.size());

		// Read data one byte at a time
		std::vector<char> response_vector;
		char response_byte;

		do
		{
			// Read one byte and add it to the response vector
			serial->read(&response_byte, 1);
			response_vector.push_back(response_byte);
		} while (response_vector.back() != '\0' && response_vector.size() <= rx_length); // We can safely use .back() since the vector will never be empty here

		// Length check
		if (response_vector.size() > rx_length)
			throw timeout_exception("Didn't get string terminator with specified rx_length");

		// Read two more bytes for CRC
		serial->read(&response_byte, 1);
		response_vector.push_back(response_byte);
		serial->read(&response_byte, 1);
		response_vector.push_back(response_byte);

		// The response_vector now contains string including \0. Perform CRC check
		unsigned char *response = (unsigned char *)&response_vector[0];
		size_t bytes_received = response_vector.size();

		unsigned int crc_calculated = crc16(&response[0], bytes_received - 2);
		unsigned int crc_received = 0;

		// RoboClaw generates big endian / MSB first
		crc_received += response[bytes_received - 2] << 8;
		crc_received += response[bytes_received - 1];

		if (crc_calculated != crc_received)
		{
			throw roboclaw::crc_exception("Roboclaw CRC mismatch in reading string");
		}

		// We're good: copy to string. This will strip off the \0
		return std::string((char *)&response_vector[0]);
	}

	std::string driver::get_version(unsigned char address)
	{
		std::string firmware_version = txrx(address, 21, nullptr, 0, 50, false);
		trim(firmware_version); // This will strip off the terminating newline character

		return firmware_version;
	}

	/**
	 * @brief Read Status (command 90)
	 * @param address Roboclaw address
	 * @return Returns the status bytes (shifted LSB -> MSB) with current status. Refer to manual for bit mask values.
	 */
	uint32_t driver::get_status(unsigned char address)
	{
		// Documentation just says response is: [Status, CRC(2 bytes)].
		// But real info is here: http://forums.basicmicro.com/viewtopic.php?f=2&t=806
		unsigned char rx_buffer[4];

		txrx(address, 90, nullptr, 0, rx_buffer, sizeof(rx_buffer), false, true);

		uint32_t value = 0;
		value |= rx_buffer[0] << 24;
		value |= rx_buffer[1] << 16;
		value |= rx_buffer[2] << 8;
		value |= rx_buffer[3];

		return value;
	}

	std::pair<int, int> driver::get_encoders(unsigned char address)
	{

		unsigned char rx_buffer[5];

		txrx(address, 16, nullptr, 0, rx_buffer, sizeof(rx_buffer), false, true);

		uint32_t e1 = 0;

		e1 += rx_buffer[0] << 24;
		e1 += rx_buffer[1] << 16;
		e1 += rx_buffer[2] << 8;
		e1 += rx_buffer[3];

		txrx(address, 17, nullptr, 0, rx_buffer, sizeof(rx_buffer), false, true);

		uint32_t e2 = 0;

		e2 += rx_buffer[0] << 24;
		e2 += rx_buffer[1] << 16;
		e2 += rx_buffer[2] << 8;
		e2 += rx_buffer[3];

		return std::pair<int, int>((int)(int32_t)e1, (int)(int32_t)e2);
	}

	std::pair<int, int> driver::get_velocity(unsigned char address)
	{

		unsigned char rx_buffer[5];

		txrx(address, 18, nullptr, 0, rx_buffer, sizeof(rx_buffer), false, true);

		uint32_t e1 = 0;

		e1 += rx_buffer[0] << 24;
		e1 += rx_buffer[1] << 16;
		e1 += rx_buffer[2] << 8;
		e1 += rx_buffer[3];

		txrx(address, 19, nullptr, 0, rx_buffer, sizeof(rx_buffer), false, true);

		uint32_t e2 = 0;

		e2 += rx_buffer[0] << 24;
		e2 += rx_buffer[1] << 16;
		e2 += rx_buffer[2] << 8;
		e2 += rx_buffer[3];

		return std::pair<int, int>((int)(int32_t)e1, (int)(int32_t)e2);
	}

	void driver::reset_encoders(unsigned char address)
	{
		unsigned char rx_buffer[1];
		txrx(address, 20, nullptr, 0, rx_buffer, sizeof(rx_buffer), true, false);
	}

	void driver::set_velocity(unsigned char address, std::pair<int, int> speed)
	{
		unsigned char rx_buffer[1];
		unsigned char tx_buffer[8];

		// RoboClaw expects big endian / MSB first
		tx_buffer[0] = (unsigned char)((speed.first >> 24) & 0xFF);
		tx_buffer[1] = (unsigned char)((speed.first >> 16) & 0xFF);
		tx_buffer[2] = (unsigned char)((speed.first >> 8) & 0xFF);
		tx_buffer[3] = (unsigned char)(speed.first & 0xFF);

		tx_buffer[4] = (unsigned char)((speed.second >> 24) & 0xFF);
		tx_buffer[5] = (unsigned char)((speed.second >> 16) & 0xFF);
		tx_buffer[6] = (unsigned char)((speed.second >> 8) & 0xFF);
		tx_buffer[7] = (unsigned char)(speed.second & 0xFF);

		txrx(address, 37, tx_buffer, sizeof(tx_buffer), rx_buffer, sizeof(rx_buffer), true, false);
	}

	// -------- TESTING
	void driver::drive_M1_position(unsigned char address, uint32_t position)
	{
		unsigned char rx_buffer[1];
		unsigned char tx_buffer[5];

		// RoboClaw expects big endian / MSB first
		tx_buffer[0] = (unsigned char)((position >> 24) & 0xFF);
		tx_buffer[1] = (unsigned char)((position >> 16) & 0xFF);
		tx_buffer[2] = (unsigned char)((position >> 8) & 0xFF);
		tx_buffer[3] = (unsigned char)(position & 0xFF);
		tx_buffer[4] = (unsigned char)1; // Buffer: 0=add command to buffer, 1=execute immediately

		txrx(address, 119, tx_buffer, sizeof(tx_buffer), rx_buffer, sizeof(rx_buffer), true, false);
	}
	// -------- end-TESTING

	void driver::set_duty(unsigned char address, std::pair<int, int> duty)
	{
		unsigned char rx_buffer[1];
		unsigned char tx_buffer[4];

		// RoboClaw expects big endian / MSB first
		tx_buffer[0] = (unsigned char)((duty.first >> 8) & 0xFF);
		tx_buffer[1] = (unsigned char)(duty.first & 0xFF);

		tx_buffer[2] = (unsigned char)((duty.second >> 8) & 0xFF);
		tx_buffer[3] = (unsigned char)(duty.second & 0xFF);

		txrx(address, 34, tx_buffer, sizeof(tx_buffer), rx_buffer, sizeof(rx_buffer), true, false);
	}

	double driver::get_battery_voltage(unsigned char address)
	{
		unsigned char rx_buffer[2];

		txrx(address, 24, nullptr, 0, rx_buffer, sizeof(rx_buffer), false, true);

		// Convert from MSB first
		uint16_t value = 0;
		value += rx_buffer[0] << 8;
		value += rx_buffer[1];

		// Divide by 10 and return as double
		return (double)value / 10.0;
	}

	double driver::get_logic_voltage(unsigned char address)
	{
		unsigned char rx_buffer[2];

		txrx(address, 25, nullptr, 0, rx_buffer, sizeof(rx_buffer), false, true);

		// Convert from MSB first
		uint16_t value = 0;
		value += rx_buffer[0] << 8;
		value += rx_buffer[1];

		// Divide by 10 and return as double
		return (double)value / 10.0;
	}

	double driver::get_temperature1(unsigned char address)
	{
		unsigned char rx_buffer[2];

		txrx(address, 82, nullptr, 0, rx_buffer, sizeof(rx_buffer), false, true);

		// Convert from MSB first
		uint16_t value = 0;
		value += rx_buffer[0] << 8;
		value += rx_buffer[1];

		// Divide by 10 and return as double
		return (double)value / 10.0;
	}

	std::pair<double, double> driver::get_currents(unsigned char address)
	{
		unsigned char rx_buffer[4];

		txrx(address, 49, nullptr, 0, rx_buffer, sizeof(rx_buffer), false, true);

		double i1, i2;

		// Convert from MSB first
		uint16_t value = 0;
		value += rx_buffer[0] << 8;
		value += rx_buffer[1];
		i1 = (double)value / 100.0;

		value = rx_buffer[2] << 8;
		value += rx_buffer[3];
		i2 = (double)value / 100.0;

		// Divide by 10 and return as double
		return std::pair<double, double>(i1, i2);
	}
}
