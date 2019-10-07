/****************************************************************************
 *
 *   Copyright (c) 2014-2019 PX4 Development Team. All rights reserved.
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


/**
 * @file LidarLiteI2C.cpp
 * @author Allyson Kreft
 *
 * Driver for the PulsedLight Lidar-Lite range finders connected via I2C.
 */

#include "LidarLiteI2C.h"

LidarLiteI2C::LidarLiteI2C(const int bus, const uint8_t rotation, const int address) :
	LidarLite(rotation),
	I2C("LL40LS", nullptr, bus, address, 100000),
	ScheduledWorkItem(MODULE_NAME, px4::device_bus_to_wq(get_device_id()))
{
	// up the retries since the device misses the first measure attempts
	_retries = 3;
}

LidarLiteI2C::~LidarLiteI2C()
{
	stop();
}

int
LidarLiteI2C::collect()
{
	perf_begin(_sample_perf);
	uint8_t val[2] {};

	// Increment the sensor index, (limited to the number of sensors connected).
	for (size_t index = 0; index < _sensor_count; index++) {

		// Set the sensor i2c adress for the active cycle.
		set_device_address(_sensor_addresses[index]);

		// read the high and low byte distance registers
		uint8_t distance_reg = LL40LS_DISTHIGH_REG | LL40LS_AUTO_INCREMENT;
		int ret = transfer(&distance_reg, 1, &val[0], sizeof(val));

		// if the transfer failed or if the high bit of distance is
		// set then the distance is invalid
		if (ret < 0 || (val[0] & 0x80)) {
			if (hrt_absolute_time() - _acquire_time_usec > LL40LS_MEASUREMENT_TIMEOUT) {
				/*
				  NACKs from the sensor are expected when we
				  read before it is ready, so only consider it
				  an error if more than 100ms has elapsed.
				 */
				PX4_DEBUG("error reading from sensor: %d", ret);
				perf_count(_comms_errors);

				if (perf_event_count(_comms_errors) % 10 == 0) {
					perf_count(_sensor_resets);
					reset_sensor();
				}
			}

			perf_end(_sample_perf);

			// If the measurement could not be acquired, request a new measurement.
			measure();
			return ret;
		}

		uint16_t distance_cm = (val[0] << 8) | val[1];
		const float distance_m = float(distance_cm) * 1e-2f;

		if (distance_cm == 0) {
			_zero_counter++;

			if (_zero_counter == 20) {
				/*
				   we have had 20 zeros in a row - reset the
				   sensor. This is a known bad state of the
				   sensor where it returns 16 bits of zero for
				   the distance with a trailing NACK, and
				   keeps doing that even when the target comes
				   into a valid range.
				 */
				_zero_counter = 0;
				perf_end(_sample_perf);
				perf_count(_sensor_resets);
				reset_sensor();
			}

		} else {
			_zero_counter = 0;
		}

		// This should be fairly close to the end of the measurement, so the best approximation of the time
		const hrt_abstime timestamp_sample = hrt_absolute_time();

		// Relative signal strength measurement, i.e. the strength of
		// the main signal peak compared to the general noise level.
		uint8_t signal_strength_reg = LL40LS_SIGNAL_STRENGTH_REG;
		ret = transfer(&signal_strength_reg, 1, &val[0], 1);

		// check if the transfer failed
		if (ret < 0) {
			if (hrt_elapsed_time(&_acquire_time_usec) > LL40LS_MEASUREMENT_TIMEOUT) {
				/*
				  NACKs from the sensor are expected when we
				  read before it is ready, so only consider it
				  an error if more than 100ms has elapsed.
				 */
				PX4_INFO("signal strength read failed");

				DEVICE_DEBUG("error reading signal strength from sensor: %d", ret);
				perf_count(_comms_errors);

				if (perf_event_count(_comms_errors) % 10 == 0) {
					perf_count(_sensor_resets);
					reset_sensor();
				}
			}

			perf_end(_sample_perf);
			// If the signal strength measurement could not be acquired, request a new measurement.
			return measure();
		}

		uint8_t ll40ls_signal_strength = val[0];
		uint8_t signal_min = 0;
		uint8_t signal_max = 0;
		uint8_t signal_value = 0;

		// We detect if V3HP is being used.
		if (_is_v3hp) {
			signal_min = LL40LS_SIGNAL_STRENGTH_MIN_V3HP;
			signal_max = LL40LS_SIGNAL_STRENGTH_MAX_V3HP;
			signal_value = ll40ls_signal_strength;

		} else {
			// Absolute peak strength measurement, i.e. absolute strength of main signal peak.
			uint8_t peak_strength_reg = LL40LS_PEAK_STRENGTH_REG;
			ret = transfer(&peak_strength_reg, 1, &val[0], 1);

			// Check if the transfer failed.
			if (ret < 0) {
				if (hrt_elapsed_time(&_acquire_time_usec) > LL40LS_MEASUREMENT_TIMEOUT) {
					/*
					  NACKs from the sensor are expected when we
					  read before it is ready, so only consider it
					  an error if more than 100ms has elapsed.
					 */
					PX4_INFO("peak strength read failed");

					DEVICE_DEBUG("error reading peak strength from sensor: %d", ret);
					perf_count(_comms_errors);

					if (perf_event_count(_comms_errors) % 10 == 0) {
						perf_count(_sensor_resets);
						reset_sensor();
					}
				}

				perf_end(_sample_perf);
				// If the peak strength measurement could not be acquired, request a new measurement.
				return measure();
			}

			uint8_t ll40ls_peak_strength = val[0];
			signal_min = LL40LS_PEAK_STRENGTH_LOW;
			signal_max = LL40LS_PEAK_STRENGTH_HIGH;

			// For v2 and v3 use ll40ls_signal_strength (a relative measure, i.e. peak strength to noise!) to reject potentially ambiguous measurements
			if (ll40ls_signal_strength <= LL40LS_SIGNAL_STRENGTH_LOW) {
				signal_value = 0;

			} else {
				signal_value = ll40ls_peak_strength;
			}
		}

		// Final data quality evaluation. This is based on the datasheet and simple heuristics retrieved from experiments
		// Step 1: Normalize signal strength to 0...100 percent using the absolute signal peak strength.
		uint8_t signal_quality = 100 * math::max(signal_value - signal_min, 0) / (signal_max - signal_min);

		// Step 2: Filter physically impossible measurements, which removes some crazy outliers that appear on LL40LS.
		if (distance_m < LL40LS_MIN_DISTANCE) {
			signal_quality = 0;
		}

		_px4_rangefinder.update(timestamp_sample, distance_m, signal_quality);
	}

	perf_end(_sample_perf);

	// The measurement was successfully acquired, request a new measurement.
	return measure();
}

int
LidarLiteI2C::get_sensor_orientation(const size_t index)
{
	switch (index) {
	case 0: return _p_sensor0_rot.get();

	case 1: return _p_sensor1_rot.get();

	case 2: return _p_sensor2_rot.get();

	case 3: return _p_sensor3_rot.get();

	case 4: return _p_sensor4_rot.get();

	case 5: return _p_sensor5_rot.get();

	case 6: return _p_sensor6_rot.get();

	case 7: return _p_sensor7_rot.get();

	case 8: return _p_sensor8_rot.get();

	case 9: return _p_sensor9_rot.get();

	case 10: return _p_sensor10_rot.get();

	case 11: return _p_sensor11_rot.get();

	default: return PX4_ERROR;
	}
}

int
LidarLiteI2C::init()
{
	// Perform I2C init (and probe) first.
	if (I2C::init() != PX4_OK) {
		return PX4_ERROR;
	}

	// Base addresses are essentially 6 bit addresses, so 8 bit addresses must be divisible by 4.
	size_t address_increment = 4;
	size_t max_address = LL40LS_BASE_ADDR + RANGE_FINDER_MAX_SENSORS * address_increment;

	// Check for connected rangefinders on each i2c port by incrementing from the base address. LL40LS_BASE_ADDR = 0x62 (7-bit).
	for (uint8_t address = LL40LS_BASE_ADDR + address_increment; address <= max_address; address += address_increment) {

		if (set_address(address) != PX4_OK) {
			break;
		}

		if (measure() != PX4_OK) {
			PX4_ERR("measurement error");
			break;
		}

		// Store I2C address
		_sensor_addresses[_sensor_count] = address;
		_sensor_rotations[_sensor_count] = get_sensor_orientation(_sensor_count);
		_sensor_count++;

		PX4_INFO("sensor %i at address 0x%02X added", _sensor_count, get_device_address());
		px4_usleep(_measure_interval);
	}

	// Return an error if no sensors were detected.
	if (_sensor_count == 0) {
		PX4_ERR("no sensors discovered");
		return PX4_ERROR;
	}

	PX4_INFO("Total sensors connected: %i", _sensor_count);
	return PX4_OK;
}

int
LidarLiteI2C::lidar_lite_transfer(const uint8_t *send, const unsigned send_len, uint8_t *recv, const unsigned recv_len)
{
	if (send != nullptr && send_len > 0) {
		int ret = transfer(send, send_len, nullptr, 0);

		if (ret != PX4_OK) {
			return ret;
		}
	}

	if (recv != nullptr && recv_len > 0) {
		return transfer(nullptr, 0, recv, recv_len);
	}

	return PX4_ERROR;
}

int
LidarLiteI2C::measure()
{
	// Send the command to begin a measurement.
	int ret = write_reg(LL40LS_MEASURE_REG, LL40LS_MSRREG_ACQUIRE);

	if (ret != PX4_OK) {
		perf_count(_comms_errors);
		PX4_DEBUG("I2C::transfer returned %d", ret);

		// If we are getting lots of I2C transfer errors try resetting the sensor.
		if (perf_event_count(_comms_errors) % 10 == 0) {
			perf_count(_sensor_resets);
			reset_sensor();
		}

		return ret;
	}

	// Store the acquire time so we can know when the acquisition has timed out.
	_acquire_time_usec = hrt_absolute_time();

	return PX4_OK;
}

void
LidarLiteI2C::print_registers()
{
	// Stop the normal driver cycle to read registers.
	ScheduleClear();

	// Wait for a while to ensure the sensors are in a ready state.
	px4_usleep(50000);

	// Increment the sensor index, (limited to the number of sensors connected).
	for (size_t index = 0; index < _sensor_count; index++) {

		set_device_address(_sensor_addresses[index]);
		PX4_INFO("Register values for sensor 0x%02X", get_device_address());

		for (uint8_t reg = 0; reg <= 0x67; reg++) {
			uint8_t val = 0;
			int ret = transfer(&reg, 1, &val, 1);

			if (ret != OK) {
				PX4_INFO("%02x:XX ", static_cast<unsigned>(reg));

			} else {
				PX4_INFO("%02x:%02x ", static_cast<unsigned>(reg), static_cast<unsigned>(val));
			}

			if (reg % 16 == 15) {
				printf("\n");
			}
		}

		printf("\n");
	}

	// Reschedule the normal driver cycle.
	ScheduleOnInterval(_measure_interval);
}

int
LidarLiteI2C::probe()
{
	// cope with both old and new I2C bus address
	const uint8_t addresses[2] = { LL40LS_BASE_ADDR, LL40LS_BASE_ADDR_OLD };

	uint8_t id_high = 0;
	uint8_t id_low = 0;

	// more retries for detection
	_retries = 10;

	for (uint8_t i = 0; i < sizeof(addresses); i++) {

		set_device_address(addresses[i]);

		/**
		 * The probing is divided into different cases. One to detect v2, one for v3 and v1 and one for v3HP.
		 * The reason for this is that registers are not consistent between different versions.
		 * The v3HP doesn't have the HW VERSION (or at least no version is specified),
		 * v1 and v3 don't have the unit id register while v2 has both.
		 * It would be better if we had a proper WHOAMI register.
		 */
		if ((read_reg(LL40LS_HW_VERSION, _hw_version) == PX4_OK) &&
		    (read_reg(LL40LS_SW_VERSION, _sw_version) == PX4_OK)) {

			if (read_reg(LL40LS_UNIT_ID_HIGH, id_high) == PX4_OK &&
			    read_reg(LL40LS_UNIT_ID_LOW, id_low) == PX4_OK) {
				_unit_id = (static_cast<uint16_t>(id_high << 8) | id_low) & 0xFFFF;
			}

			if (_hw_version > 0) {

				if (_unit_id > 0) {
					// v2
					PX4_INFO("probe success - hw: %u, sw:%u, id: %u", _hw_version, _sw_version, _unit_id);
					_px4_rangefinder.set_max_distance(LL40LS_MAX_DISTANCE_V2);

				} else {
					// v1 and v3
					PX4_INFO("probe success - hw: %u, sw:%u", _hw_version, _sw_version);
				}

			} else {

				if (_unit_id > 0) {
					// v3hp
					_is_v3hp = true;
					PX4_INFO("probe success - id: %u", _unit_id);
				}
			}

			_retries = 3;
			return reset_sensor();
		}

		PX4_ERR("probe failed unit_id=0x%02x hw_version=0x%02x sw_version=0x%02x",
			(unsigned)_unit_id, (unsigned)_hw_version, (unsigned)_sw_version);
	}

	// not found on any address
	return -EIO;
}

int
LidarLiteI2C::read_reg(const uint8_t reg, uint8_t &val)
{
	return lidar_lite_transfer(&reg, 1, &val, 1);
}

int
LidarLiteI2C::write_reg(const uint8_t reg, const uint8_t &val)
{
	const uint8_t cmd[2] = { reg, val };
	return transfer(&cmd[0], 2, nullptr, 0);
}

int
LidarLiteI2C::reset_sensor()
{
	px4_usleep(15000);

	int ret = write_reg(LL40LS_SIG_COUNT_VAL_REG, LL40LS_SIG_COUNT_VAL_MAX);

	if (ret != PX4_OK) {
		return ret;
	}

	px4_usleep(15000);
	ret = write_reg(LL40LS_MEASURE_REG, LL40LS_MSRREG_RESET);


	if (ret != PX4_OK) {
		uint8_t sig_cnt;

		px4_usleep(15000);
		ret = read_reg(LL40LS_SIG_COUNT_VAL_REG, sig_cnt);

		if ((ret != PX4_OK) || (sig_cnt != LL40LS_SIG_COUNT_VAL_DEFAULT)) {
			PX4_INFO("Error: ll40ls reset failure. Exiting!\n");
			return ret;

		}
	}

	// wait for sensor reset to complete
	px4_usleep(50000);
	ret = write_reg(LL40LS_SIG_COUNT_VAL_REG, LL40LS_SIG_COUNT_VAL_MAX);

	if (ret != PX4_OK) {
		return ret;
	}

	// wait for register write to complete
	px4_usleep(1000);

	return OK;
}

void
LidarLiteI2C::Run()
{
	// Collect the sensor data.
	if (collect() != PX4_OK) {
		PX4_INFO("collection error");

		/* if we've been waiting more than 200ms then
		   send a new acquire */
		if (hrt_elapsed_time(&_acquire_time_usec) > (LL40LS_MEASUREMENT_TIMEOUT * 2)) {
			_collect_phase = false;
		}
	}
}

int
LidarLiteI2C::set_address(const uint8_t address)
{
	if (address < LL40LS_BASE_ADDR &&
	    address != LL40LS_BASE_ADDR_OLD) {
		PX4_ERR("incompatible address requested");
		return PX4_ERROR;
	}

	set_device_address(LL40LS_BASE_ADDR);
	PX4_INFO("current address: 0x%02X, requested address: 0x%02X", get_device_address(), address);

	uint8_t serial_number[2] {};

	// Obtain the device serial number.
	if (lidar_lite_transfer(&LL40LS_SERIAL_NUMBER_REG, 1, &serial_number[0], 2) != PX4_OK) {
		PX4_ERR("could not read a device serial number");
		return PX4_ERROR;
	}

	// Store the serial number values.
	_sensor_serial_numbers[_sensor_count] = static_cast<uint16_t>(serial_number[0]) << 8 | serial_number[1];

	PX4_INFO("serial number %u", _sensor_serial_numbers[_sensor_count]);

	// Unlock the LL40LS_I2C_SEC_ADDR register
	if (write_reg(LL40LS_I2C_ID_HIGH, serial_number[0]) != PX4_OK ||
	    write_reg(LL40LS_I2C_ID_LOW,  serial_number[1]) != PX4_OK) {
		PX4_ERR("failed to unlock sec_addr register");
		return PX4_ERROR;
	}

	uint8_t shifted_address = (address << 1);

	// Write the new (8-bit) address.
	if (write_reg(LL40LS_I2C_SEC_ADDR, shifted_address) != PX4_OK) {
		PX4_ERR("failed to set new address");
		return PX4_ERROR;
	}

	// Enable use of the new address.
	if (write_reg(LL40LS_I2C_CONFIG, LL40LS_ENABLE_NEW_ADDR) != PX4_OK)  {
		PX4_ERR("failed to enable new address");
		return PX4_ERROR;
	}

	set_device_address(address);
	PX4_INFO("new address 0x%02X ", address);

	uint8_t config{0};

	// Obtain the current config settings.
	if (read_reg(LL40LS_I2C_CONFIG, config) != PX4_OK) {
		PX4_ERR("failed to read config: 0x%02X", config);
		return PX4_ERROR;
	}

	// Set the disable default address bit of the config register.
	if (write_reg(LL40LS_I2C_CONFIG, config | LL40LS_DISABLE_DEFAULT_ADDR) != PX4_OK) {
		PX4_ERR("failed to enable new address");
		return PX4_ERROR;
	}

	// Reset the serial number so that if can be validated.
	serial_number[0] = 0;
	serial_number[1] = 0;

	// Verify the device answers at the new address.
	if (lidar_lite_transfer(&LL40LS_SERIAL_NUMBER_REG, 1, &serial_number[0], 2) != PX4_OK) {
		PX4_ERR("serial number request error");
		return PX4_ERROR;
	}

	uint16_t validate_serial_number = (static_cast<uint16_t>(serial_number[0]) << 8) | serial_number[1];

	if (_sensor_serial_numbers[_sensor_count] != validate_serial_number) {
		PX4_ERR("serial number read back is 0x%02X, address was not set", validate_serial_number);
		return PX4_ERROR;
	}

	return PX4_OK;
}

void
LidarLiteI2C::start()
{
	// Fetch parameter values.
	ModuleParams::updateParams();

	// Schedule the driver cycle.
	ScheduleOnInterval(_measure_interval);
}

void
LidarLiteI2C::stop()
{
	ScheduleClear();
}
