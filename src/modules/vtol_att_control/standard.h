/****************************************************************************
 *
 *   Copyright (c) 2015-2022 PX4 Development Team. All rights reserved.
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
* @file standard.h
* VTOL with fixed multirotor motor configurations (such as quad) and a pusher
* (or puller aka tractor) motor for forward flight.
*
* @author Simon Wilks 		<simon@uaventure.com>
* @author Roman Bapst 		<bapstroman@gmail.com>
* @author Andreas Antener	<andreas@uaventure.com>
* @author Sander Smeets 	<sander@droneslab.com>
*
*/

#ifndef STANDARD_H
#define STANDARD_H
#include "vtol_type.h"
#include <uORB/Subscription.hpp>
#include <uORB/topics/manual_control_setpoint.h>

class Standard : public VtolType
{

public:

	Standard(VtolAttitudeControl *_att_controller);
	~Standard() override = default;

	void update_vtol_state() override;
	void update_transition_state() override;
	void update_fw_state() override;
	void update_mc_state() override;
	void fill_actuator_outputs() override;
	void waiting_on_tecs() override;
	void blendThrottleAfterFrontTransition(float scale) override;

	// DROBOT: 로버 휠 명령 getter
	float get_wheel_left() const override { return _wheel_left; }
	float get_wheel_right() const override { return _wheel_right; }

	// DROBOT: 서보/리니어 액추에이터 명령 getter
	float get_servo_arm_cmd() const override { return _servo_arm_cmd; }
	float get_linear_act_cmd() const override { return _linear_act_cmd; }

private:

	enum class vtol_mode {
		MC_MODE = 0,
		TRANSITION_TO_FW,
		TRANSITION_TO_MC,
		FW_MODE
	};

	vtol_mode _vtol_mode{vtol_mode::MC_MODE};			/**< vtol flight mode, defined by enum vtol_mode */

	float _pusher_throttle{0.0f};
	float _airspeed_trans_blend_margin{0.0f};
	hrt_abstime _last_time_pusher_transition_update{0};

	// DROBOT: 로버 휠 명령
	float _wheel_left{0.f};
	float _wheel_right{0.f};
	uORB::Subscription _manual_control_setpoint_sub{ORB_ID(manual_control_setpoint)};

	// DROBOT: 서보/리니어 액추에이터 명령 (정규화 [-1, 1])
	float _servo_arm_cmd{1.0f};    // 1.0=팔 펴짐(MC), -1.0=팔 접힘(rover)
	float _linear_act_cmd{-1.0f};  // -1.0=수축(MC), 1.0=확장(rover)

	// DROBOT: 휠 ramp up 추적
	hrt_abstime _fw_mode_enter_time{0};
	bool _fw_mode_entered{false};

	void parameters_update() override;

	DEFINE_PARAMETERS_CUSTOM_PARENT(VtolType,
					(ParamFloat<px4::params::VT_PSHER_SLEW>) _param_vt_psher_slew,
					(ParamFloat<px4::params::VT_B_TRANS_RAMP>) _param_vt_b_trans_ramp,
					(ParamFloat<px4::params::FW_PSP_OFF>) _param_fw_psp_off,
					(ParamFloat<px4::params::VT_D_SETTLE_T>) _param_vt_d_settle_t,
					(ParamFloat<px4::params::VT_D_SERVO_DUR>) _param_vt_d_servo_dur,
					(ParamFloat<px4::params::VT_D_WHL_RAMP>) _param_vt_d_whl_ramp
				       )
};
#endif
