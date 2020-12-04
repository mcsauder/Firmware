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
 * @file FlightTaskAutoManuever.hpp
 */

#pragma once

#include "FlightTask.hpp"

class FlightTaskAutoManuever : public FlightTask
{
public:
	FlightTaskAutoManuever() = default;
	virtual ~FlightTaskAutoManuever() = default;

	bool update() override;
	bool activate(const vehicle_local_position_setpoint_s &last_setpoint) override;

private:

	hrt_abstime _manuever_start_time {0};
	hrt_abstime _manuever_time_us {100000};

	size_t _manuever_iterations {10};
	size_t _manuever_count {0};

	float _manuever_frequency {1.f};

	float _manuever_acceleration{10.f};
	float _manuever_velocity {10.f};

	DEFINE_PARAMETERS_CUSTOM_PARENT(
		FlightTask, (ParamFloat<px4::params::MPC_AUTOMAN_FREQ>) _param_mpc_automan_freq, ///< Frequency for controlled manuever
		(ParamFloat<px4::params::MPC_ACC_HOR_MAX>) _param_mpc_acc_hor_max, 		 ///< horizontal acceleration
		(ParamFloat<px4::params::MPC_XY_VEL_MAX>) _param_mpc_xy_vel_max 		 ///< horizontal velocity
	)
};
