/****************************************************************************
 *
 *   Copyright (c) 2019 PX4 Development Team. All rights reserved.
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
 * @file FlightTaskAutoManuever.cpp
 */

#include "FlightTaskAutoManuever.hpp"

bool FlightTaskAutoManuever::activate(const vehicle_local_position_setpoint_s &last_setpoint)
{
	bool ret = FlightTask::activate(last_setpoint);

	_position_setpoint(0) = _position(0);
	_position_setpoint(1) = _position(1);
	_position_setpoint(2) = _position(2);

	_velocity_setpoint(0) = 0.f;
	_velocity_setpoint(1) = 0.f;
	_velocity_setpoint(2) = 0.f;

	_acceleration_setpoint(0) = 0.f;
	_acceleration_setpoint(1) = 0.f;
	_acceleration_setpoint(2) = 0.f;

	_manuever_frequency = _param_mpc_automan_freq.get();

	_manuever_acceleration = _param_mpc_acc_hor_max.get() * 5.f;
	_manuever_velocity     = _param_mpc_xy_vel_max.get();

	_manuever_time_us = static_cast<int>(1000000.f / _manuever_frequency);
	_manuever_count = 0;

	// keep heading
	_yaw_setpoint = _yaw;

	PX4_INFO("Auto Manuever Activated");
	PX4_INFO("_manuever_frequency %f", static_cast<double>(_manuever_frequency));
	PX4_INFO("_manuever_time_s %f", static_cast<double>(_manuever_time_us) / 1000000.0);

	_manuever_start_time = hrt_absolute_time();

	return ret;
}

bool FlightTaskAutoManuever::update()
{
	bool ret = FlightTask::update();

	hrt_abstime time_now = hrt_absolute_time();
	hrt_abstime time_delta = time_now - _manuever_start_time;

	if (time_delta > _manuever_time_us &&
	    _manuever_count <= _manuever_iterations) {

		_manuever_count++;
		_manuever_start_time = time_now;

		_manuever_velocity *= -1.f;
		_manuever_acceleration *= -1.f;
	}

	if (_manuever_count > _manuever_iterations) {
		_manuever_velocity = 0.f;
		_manuever_acceleration = 0.f;
	}

	_velocity_setpoint(0) = 0.f;
	_velocity_setpoint(1) = _manuever_velocity;
	_velocity_setpoint(2) = 0.f;

	_acceleration_setpoint(0) = 0.f;
	_acceleration_setpoint(1) = _manuever_acceleration;
	_acceleration_setpoint(2) = 0.f;

	return ret;
}
