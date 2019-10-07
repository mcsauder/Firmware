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
 * @file LidarLiteI2C.h
 * @author Allyson Kreft
 *
 * Driver for the PulsedLight Lidar-Lite range finders connected via I2C.
 */

#pragma once

#include "LidarLite.h"

#include <drivers/device/i2c.h>
#include <drivers/drv_hrt.h>
#include <mathlib/mathlib.h>
#include <px4_defines.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

/* Configuration Constants */
static constexpr uint8_t LL40LS_SIG_COUNT_VAL_DEFAULT = 0x80; /* Default maximum acquisition count */

/* LL40LS Registers addresses */
static constexpr uint8_t LL40LS_MEASURE_REG           = 0x00; /* Measure range register */
static constexpr uint8_t LL40LS_MSRREG_RESET          = 0x00; /* reset to power on defaults */
static constexpr uint8_t LL40LS_SYSTEM_STATUS         = 0x01; /* System status register */
static constexpr uint8_t LL40LS_MSRREG_ACQUIRE        = 0x04; /* Value to acquire a measurement, version specific */
static constexpr uint8_t LL40LS_DISABLE_DEFAULT_ADDR  = 0x08; /* Disables use of the default device address. */
static constexpr uint8_t LL40LS_ENABLE_NEW_ADDR       = 0x10; /* Enables use of a non-default device address. */
static constexpr uint8_t LL40LS_DISTHIGH_REG          = 0x0F; /* High byte of distance register, auto increment */
static constexpr uint8_t LL40LS_AUTO_INCREMENT        = 0x80;
static constexpr uint8_t LL40LS_HW_VERSION            = 0x41;
static constexpr uint8_t LL40LS_SW_VERSION            = 0x4f;
static constexpr uint8_t LL40LS_SIGNAL_STRENGTH_REG   = 0x0e;
static constexpr uint8_t LL40LS_PEAK_STRENGTH_REG     = 0x0c;

static constexpr uint8_t LL40LS_UNIT_ID_HIGH          = 0x16; /* Serial number high byte - Unique. */
static constexpr uint8_t LL40LS_UNIT_ID_LOW           = 0x17; /* Serial number low byte - Unique. */
static constexpr uint8_t LL40LS_I2C_ID_HIGH           = 0x18; /* Write serial number high byte for address unlock. */
static constexpr uint8_t LL40LS_I2C_ID_LOW            = 0x19; /* Write serial number low byte for address unlock. */
static constexpr uint8_t LL40LS_I2C_SEC_ADDR          = 0x1a; /* Write new I2C address after unlock */
static constexpr uint8_t LL40LS_I2C_CONFIG            = 0x1e; /* Default address response control 0x00 */

static constexpr uint8_t LL40LS_SIG_COUNT_VAL_REG     = 0x02; /* Maximum acquisition count register */
static constexpr uint8_t LL40LS_SIG_COUNT_VAL_MAX     = 0xFF; /* Maximum acquisition count max value */

static constexpr uint8_t LL40LS_POWER_CONTROL         = 0x65; /* Activate/deactive sleep mode. */
static constexpr uint8_t LL40LS_SERIAL_NUMBER_REG     = 0x96; /* Serial number register. */

static constexpr int LL40LS_SIGNAL_STRENGTH_MIN_V3HP  = 70;  /* Min signal strength for V3HP */
static constexpr int LL40LS_SIGNAL_STRENGTH_MAX_V3HP  = 255; /* Max signal strength for V3HP */

static constexpr int LL40LS_SIGNAL_STRENGTH_LOW       = 24;  /* Minimum signal strength for a valid measurement */
static constexpr int LL40LS_PEAK_STRENGTH_LOW         = 135; /* Minimum peak strength for accepting a measurement */
static constexpr int LL40LS_PEAK_STRENGTH_HIGH        = 234; /* Max peak strength raw value */


class LidarLiteI2C : public LidarLite, public device::I2C, public px4::ScheduledWorkItem
{
public:
	LidarLiteI2C(const int bus, const uint8_t rotation = distance_sensor_s::ROTATION_DOWNWARD_FACING,
		     const int address = LL40LS_BASE_ADDR);
	virtual ~LidarLiteI2C();

	int init() override;

	/**
	 * Print sensor registers to console for debugging.
	 */
	void print_registers() override;

	int set_address(const uint8_t address = LL40LS_BASE_ADDR) override;

	/**
	 * Initialise the automatic measurement state machine and start it.
	 *
	 * @note This function is called at open and error time.  It might make sense
	 *       to make it more aggressive about resetting the bus in case of errors.
	 */
	void start() override;

	/**
	 * Stop the automatic measurement state machine.
	 */
	void stop() override;

protected:

	int measure() override;

	int probe() override;

	/**
	 * Reset the sensor to power on defaults plus additional configurations.
	 */
	int reset_sensor() override;

	int read_reg(const uint8_t reg, uint8_t &val);

	int write_reg(const uint8_t reg, const uint8_t &val);

private:

	int collect() override;

	/**
	 * Gets the current sensor orientation value.
	 */
	int get_sensor_orientation(const size_t index);

	/**
	 * LidarLite specific transfer function. This is needed
	 * to avoid a stop transition with SCL high
	 *
	 * @param send          Pointer to bytes to send.
	 * @param send_len      Number of bytes to send.
	 * @param recv          Pointer to buffer for bytes received.
	 * @param recv_len      Number of bytes to receive.
	 * @return              OK if the transfer was successful, -errno
	 *                      otherwise.
	 */
	int lidar_lite_transfer(const uint8_t *send, const unsigned send_len, uint8_t *recv, const unsigned recv_len);

	/**
	 * Test whether the device supported by the driver is present at a
	 * specific address.
	 *
	 * @param address The I2C bus address to probe.
	 * @return        True if the device is present.
	 */
	int probe_address(const uint8_t address);

	/**
	 * Perform a poll cycle; collect from the previous measurement
	 * and start a new one.
	 */
	void Run() override;

	bool _collect_phase{false};
	bool _is_v3hp{false};

	size_t _sensor_index{0};	// Initialize counter for cycling i2c adresses to zero.
	size_t _sensor_count{0};	// Number of sensors connected.

	uint8_t _hw_version{0};
	uint8_t _sw_version{0};

	uint16_t _unit_id{0};
	uint16_t _zero_counter{0};

	uint64_t _acquire_time_usec{0};

	px4::Array<uint8_t, RANGE_FINDER_MAX_SENSORS> _sensor_addresses {};
	px4::Array<uint8_t, RANGE_FINDER_MAX_SENSORS> _sensor_rotations {};

	px4::Array<uint16_t, RANGE_FINDER_MAX_SENSORS> _sensor_serial_numbers {};

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::SENS_EN_LL40LS>) _p_sensor_enabled,
		(ParamInt<px4::params::LL40LS_0_ROT>)   _p_sensor0_rot,
		(ParamInt<px4::params::LL40LS_1_ROT>)   _p_sensor1_rot,
		(ParamInt<px4::params::LL40LS_2_ROT>)   _p_sensor2_rot,
		(ParamInt<px4::params::LL40LS_3_ROT>)   _p_sensor3_rot,
		(ParamInt<px4::params::LL40LS_4_ROT>)   _p_sensor4_rot,
		(ParamInt<px4::params::LL40LS_5_ROT>)   _p_sensor5_rot,
		(ParamInt<px4::params::LL40LS_6_ROT>)   _p_sensor6_rot,
		(ParamInt<px4::params::LL40LS_7_ROT>)   _p_sensor7_rot,
		(ParamInt<px4::params::LL40LS_8_ROT>)   _p_sensor8_rot,
		(ParamInt<px4::params::LL40LS_9_ROT>)   _p_sensor9_rot,
		(ParamInt<px4::params::LL40LS_10_ROT>)  _p_sensor10_rot,
		(ParamInt<px4::params::LL40LS_11_ROT>)  _p_sensor11_rot
	);
};
