/****************************************************************************
 *
 *   Copyright (c) 2017-2019 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include "TFMINI.hpp"

#include <fcntl.h>

TFMINI::TFMINI(const char *port, uint8_t rotation) :
	ScheduledWorkItem(MODULE_NAME, px4::serial_port_to_wq(port)),
	_px4_rangefinder(0 /* TODO: device id */, rotation)
{
	// store port name
	strncpy(_port, port, sizeof(_port) - 1);

	// enforce null termination
	_port[sizeof(_port) - 1] = '\0';
}

TFMINI::~TFMINI()
{
	// make sure we are truly inactive
	stop();

	perf_free(_sample_perf);
	perf_free(_comms_errors);
}

int TFMINI::collect()
{
	perf_begin(_sample_perf);

	int distance_mm = -1;

	const hrt_abstime read_elapsed = hrt_elapsed_time(&_last_read);
	const hrt_abstime timestamp_sample = hrt_absolute_time();

	int bytes_read = ::read(_file_descriptor, &_linebuf[0], sizeof(_linebuf) - 1);

	if (bytes_read > 0) {

		_last_read = hrt_absolute_time();

		// parse buffer
		for (size_t i = 0; i < sizeof(_linebuf) - 1; i++) {
			data_parser(_linebuf[i], _linebuf, _parse_state, _linebuf_index, distance_mm);
		}

	} else {
		if (read_elapsed > static_cast<hrt_abstime>(20_ms)) {
			// clear the input buffer if last read was too long ago.
			tcflush(_file_descriptor, TCIFLUSH);
		}

		PX4_INFO("read error: %d", bytes_read);
		perf_count(_comms_errors);
		perf_end(_sample_perf);
		return PX4_ERROR;
	}

	// no valid measurement after parsing buffer
	if (distance_mm < 0) {
		PX4_INFO("invalid distance");
		perf_count(_comms_errors);
		perf_end(_sample_perf);
		return -EAGAIN;
	}

	PX4_INFO("tfmini distance_m: %f", static_cast<double>(distance_mm));

	const float distance_m = static_cast<float>(distance_mm) * 1e-3f;

	// publish most recent valid measurement from buffer
	_px4_rangefinder.update(timestamp_sample, distance_m);

	perf_end(_sample_perf);

	return PX4_OK;
}

int
TFMINI::data_parser(const uint8_t check_byte, uint8_t parserbuf[PARSER_BUF_LENGTH], PARSE_STATE &state,
		    unsigned int parserbuf_index, int &distance)
{
	switch (state) {
	case PARSE_STATE::CHECKSUM:
		if (check_byte == 'Y') {
			state = PARSE_STATE::SYNC_1;
			parserbuf[parserbuf_index] = check_byte;
			(parserbuf_index)++;

		} else {
			state = PARSE_STATE::UNSYNC;
		}

		break;

	case PARSE_STATE::UNSYNC:
		if (check_byte == 'Y') {
			state = PARSE_STATE::SYNC_1;
			parserbuf[parserbuf_index] = check_byte;
			(parserbuf_index)++;
		}

		break;

	case PARSE_STATE::SYNC_1:
		if (check_byte == 'Y') {
			state = PARSE_STATE::SYNC_2;
			parserbuf[parserbuf_index] = check_byte;
			(parserbuf_index)++;

		} else {
			state = PARSE_STATE::UNSYNC;
			parserbuf_index = 0;
		}

		break;

	case PARSE_STATE::SYNC_2:
		state = PARSE_STATE::DIST_L;
		parserbuf[parserbuf_index] = check_byte;
		(parserbuf_index)++;
		break;

	case PARSE_STATE::DIST_L:
		state = PARSE_STATE::DIST_H;
		parserbuf[parserbuf_index] = check_byte;
		(parserbuf_index)++;
		break;

	case PARSE_STATE::DIST_H:
		state = PARSE_STATE::STRENGTH_L;
		parserbuf[parserbuf_index] = check_byte;
		(parserbuf_index)++;
		break;

	case PARSE_STATE::STRENGTH_L:
		state = PARSE_STATE::STRENGTH_H;
		parserbuf[parserbuf_index] = check_byte;
		(parserbuf_index)++;
		break;

	case PARSE_STATE::STRENGTH_H:
		state = PARSE_STATE::RESERVED;
		parserbuf[parserbuf_index] = check_byte;
		(parserbuf_index)++;
		break;

	case PARSE_STATE::RESERVED:
		state = PARSE_STATE::QUALITY;
		parserbuf[parserbuf_index] = check_byte;
		(parserbuf_index)++;
		break;

	case PARSE_STATE::QUALITY:
		// Find the checksum
		unsigned char checksum = 0;

		for (int i = 0; i < 8; i++) {
			checksum += parserbuf[i];
		}

		if (check_byte == checksum) {
			parserbuf[parserbuf_index] = '\0';
			unsigned int t1 = parserbuf[2];
			unsigned int t2 = parserbuf[3];
			t2 <<= 8;
			t2 += t1;
			distance = t2;
			state = PARSE_STATE::CHECKSUM;
			parserbuf_index = 0;

		} else {
			state = PARSE_STATE::UNSYNC;
			parserbuf_index = 0;
		}

		break;
	}

#ifdef TFMINI_DEBUG
	printf("state: TFMINI_PARSE_STATE%s\n", parser_state[state]);
#endif

	return PX4_OK;
}

int
TFMINI::init()
{
	int32_t hw_model = 1; // only one model so far...

	switch (hw_model) {
	case 1: // TFMINI (12m, 100 Hz)
		// Note: Sensor specification shows 0.3m as minimum, but in practice
		// 0.3 is too close to minimum so chattering of invalid sensor decision
		// is happening sometimes. this cause EKF to believe inconsistent range readings.
		// So we set 0.4 as valid minimum.
		_px4_rangefinder.set_min_distance(0.4f);
		_px4_rangefinder.set_max_distance(12.0f);
		_px4_rangefinder.set_fov(math::radians(1.15f));
		break;

	default:
		PX4_ERR("invalid HW model %d.", hw_model);
		return PX4_ERROR;
	}

	start();

	return PX4_OK;
}

int
TFMINI::open_serial_port(const speed_t speed)
{
	// File descriptor initialized?
	if (_file_descriptor > 0) {
		// PX4_INFO("serial port already open");
		return PX4_OK;
	}

	// Configure port flags for read/write, non-controlling, non-blocking.
	int flags = (O_RDWR | O_NOCTTY | O_NONBLOCK);

	// Open the serial port.
	_file_descriptor = ::open(_port, flags);

	if (_file_descriptor < 0) {
		PX4_ERR("open failed (%i)", errno);
		return PX4_ERROR;
	}

	termios uart_config = {};

	// Store the current port configuration. attributes.
	tcgetattr(_file_descriptor, &uart_config);

	// setup for non-canonical mode
	uart_config.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
	uart_config.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);

	// Clear ONLCR flag (which appends a CR for every LF).
	uart_config.c_oflag &= ~ONLCR;
	uart_config.c_oflag &= ~OPOST;

	// fetch bytes as they become available
	uart_config.c_cc[VMIN] = 1;
	uart_config.c_cc[VTIME] = 1;

	// No hardware flowcontrol, one stop bit, no parity bit.
	uart_config.c_cflag &= ~(CRTSCTS | CSIZE | CSTOPB | PARENB);

	// Ignore modem controls, 8-bit characters.
	uart_config.c_cflag |= (CLOCAL | CREAD | CS8);

	// Set the input baud rate in the uart_config struct.
	int termios_state = cfsetispeed(&uart_config, speed);

	if (termios_state < 0) {
		PX4_ERR("CFG: %d ISPD", termios_state);
		::close(_file_descriptor);
		return PX4_ERROR;
	}

	// Set the output baud rate in the uart_config struct.
	termios_state = cfsetospeed(&uart_config, speed);

	if (termios_state < 0) {
		PX4_ERR("CFG: %d OSPD", termios_state);
		::close(_file_descriptor);
		return PX4_ERROR;
	}

	// Apply the modified port attributes.
	termios_state = tcsetattr(_file_descriptor, TCSANOW, &uart_config);

	if (termios_state < 0) {
		PX4_ERR("baud %d ATTR", termios_state);
		::close(_file_descriptor);
		return PX4_ERROR;
	}

	PX4_INFO("successfully opened UART port %s", _port);
	return PX4_OK;
}

void TFMINI::print_info()
{
	printf("Using port '%s'\n", _port);
	perf_print_counter(_sample_perf);
	perf_print_counter(_comms_errors);
}

void
TFMINI::Run()
{
	// Ensure the serial port is open.
	open_serial_port();

	// perform collection
	if (collect() == -EAGAIN) {
		// reschedule to grab the missing bits, time to transmit 9 bytes @ 115200 bps
		ScheduleClear();
		ScheduleOnInterval(TFMINI_MEASURE_INTERVAL, 87 * 9);
		return;
	}
}

void
TFMINI::start()
{
	// schedule a cycle to start things
	ScheduleOnInterval(TFMINI_MEASURE_INTERVAL);
}

void
TFMINI::stop()
{
	ScheduleClear();

	::close(_file_descriptor);
	_file_descriptor = -1;
}