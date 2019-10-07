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
 * @file LidarLite.h
 * @author Johan Jansen <jnsn.johan@gmail.com>
 *
 * Generic interface driver for the PulsedLight Lidar-Lite range finders.
 */
#pragma once

#include <containers/Array.hpp>
#include <drivers/device/device.h>
#include <lib/drivers/rangefinder/PX4Rangefinder.hpp>
#include <perf/perf_counter.h>
#include <px4_module_params.h>

using namespace time_literals;

// Device limits
static constexpr float LL40LS_MIN_DISTANCE{0.05f};
static constexpr float LL40LS_MAX_DISTANCE{25.00f};
static constexpr float LL40LS_MAX_DISTANCE_V2{35.00f};

// Normal conversion wait time.
static constexpr uint32_t LL40LS_MEASUREMENT_INTERVAL{50_ms};

// Maximum time to wait for a conversion to complete.
static constexpr uint32_t LL40LS_MEASUREMENT_TIMEOUT{100_ms};

// Sensor base addresses.
static constexpr uint8_t LL40LS_BASE_ADDR{0x62};        /* 7-bit address */
static constexpr uint8_t LL40LS_BASE_ADDR_8_BIT{0xc4};  /* 8-bit address */
static constexpr uint8_t LL40LS_BASE_ADDR_OLD{0x42};    /* previous 7-bit address */

class LidarLite : public ModuleParams
{
public:
	LidarLite(const uint8_t rotation = distance_sensor_s::ROTATION_DOWNWARD_FACING);
	virtual ~LidarLite();

	virtual int init() = 0;

	/**
	 * @brief Diagnostics - print some basic information about the driver.
	 */
	void print_info();

	/**
	 * @brief print registers to console.
	 */
	virtual void print_registers() {};

	/**
	 * Sets a new I2C device address, (always returns PX4_OK for PWM sensors).
	 * @param address The new I2C sensor address value.
	 * @return Returns PX4_OK iff successful, PX4_ERROR otherwise.
	 */
	virtual int set_address(const uint8_t address = LL40LS_BASE_ADDR) { return PX4_OK; };

	virtual void start() = 0;
	virtual void stop() = 0;

protected:

	virtual int collect() = 0;

	virtual int measure() = 0;

	virtual int reset_sensor() { return PX4_ERROR; };

	uint32_t _measure_interval{LL40LS_MEASUREMENT_INTERVAL};

	perf_counter_t _comms_errors{perf_alloc(PC_COUNT, "ll40ls: comms errors")};
	perf_counter_t _sample_perf{perf_alloc(PC_ELAPSED, "ll40ls: read")};
	perf_counter_t _sensor_resets{perf_alloc(PC_COUNT, "ll40ls: resets")};

	PX4Rangefinder	_px4_rangefinder;

private:

};
