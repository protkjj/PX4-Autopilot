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
 * @file standard.cpp
 *
 * @author Simon Wilks		<simon@uaventure.com>
 * @author Roman Bapst		<bapstroman@gmail.com>
 * @author Andreas Antener	<andreas@uaventure.com>
 * @author Sander Smeets	<sander@droneslab.com>
 *
*/

#include "standard.h"
#include "vtol_att_control_main.h"

#include <float.h>
#include <uORB/topics/manual_control_setpoint.h>

using namespace matrix;

Standard::Standard(VtolAttitudeControl *attc) :
	VtolType(attc)
{
}

void
Standard::parameters_update()
{
	VtolType::updateParams();

	// make sure that pusher ramp in backtransition is smaller than back transition (max) duration
	_param_vt_b_trans_ramp.set(math::min(_param_vt_b_trans_ramp.get(), _param_vt_b_trans_dur.get()));
}

void Standard::update_vtol_state()
{
	/* After flipping the switch the vehicle will start the pusher (or tractor) motor, picking up
	 * forward speed. After the vehicle has picked up enough speed the rotors shutdown.
	 * For the back transition the pusher motor is immediately stopped and rotors reactivated.
	 */

	float mc_weight = _mc_roll_weight;

	if (_vtol_vehicle_status->fixed_wing_system_failure) {
		// Failsafe event, engage mc motors immediately
		_vtol_mode = vtol_mode::MC_MODE;
		_pusher_throttle = 0.0f;

	} else if (!_attc->is_fixed_wing_requested()) {

		// the transition to fw mode switch is off
		if (_vtol_mode == vtol_mode::MC_MODE) {
			// in mc mode
			_vtol_mode = vtol_mode::MC_MODE;
			mc_weight = 1.0f;

		} else if (_vtol_mode == vtol_mode::FW_MODE) {
			// Regular backtransition
			resetTransitionStates();
			_vtol_mode = vtol_mode::TRANSITION_TO_MC;

		} else if (_vtol_mode == vtol_mode::TRANSITION_TO_FW) {
			// failsafe back to mc mode
			_vtol_mode = vtol_mode::MC_MODE;
			mc_weight = 1.0f;
			_pusher_throttle = 0.0f;

		} else if (_vtol_mode == vtol_mode::TRANSITION_TO_MC) {
			// DROBOT: 정지 확인 + 변환 시간 경과로 변경
			const bool exit_backtransition_time_condition =
				_time_since_trans_start > _param_vt_b_trans_dur.get();

			if (can_transition_on_ground() || exit_backtransition_time_condition) {
				_vtol_mode = vtol_mode::MC_MODE;
			}
		}

	} else {
		// the transition to fw mode switch is on
		if (_vtol_mode == vtol_mode::MC_MODE || _vtol_mode == vtol_mode::TRANSITION_TO_MC) {
			// start transition to fw mode
			/* NOTE: The failsafe transition to fixed-wing was removed because it can result in an
			 * unsafe flying state. */
			resetTransitionStates();
			_vtol_mode = vtol_mode::TRANSITION_TO_FW;

		} else if (_vtol_mode == vtol_mode::FW_MODE) {
			// in fw mode
			_vtol_mode = vtol_mode::FW_MODE;
			mc_weight = 0.0f;

		} else if (_vtol_mode == vtol_mode::TRANSITION_TO_FW) {

			if (isFrontTransitionCompleted()) {
				_vtol_mode = vtol_mode::FW_MODE;

				// don't set pusher throttle here as it's being ramped up elsewhere
				_trans_finished_ts = hrt_absolute_time();
			}
		}
	}

	_mc_roll_weight = mc_weight;
	_mc_pitch_weight = mc_weight;
	_mc_yaw_weight = mc_weight;
	_mc_throttle_weight = mc_weight;

	// map specific control phases to simple control modes
	switch (_vtol_mode) {
	case vtol_mode::MC_MODE:
		_common_vtol_mode = mode::ROTARY_WING;
		_fw_mode_entered = false; // 휠 ramp up 리셋
		break;

	case vtol_mode::FW_MODE:
		_common_vtol_mode = mode::FIXED_WING;
		break;

	case vtol_mode::TRANSITION_TO_FW:
		_common_vtol_mode = mode::TRANSITION_TO_FW;
		break;

	case vtol_mode::TRANSITION_TO_MC:
		_common_vtol_mode = mode::TRANSITION_TO_MC;
		break;
	}
}

void Standard::update_transition_state()
{
	// DROBOT: 단계별 전환 로직
	// MC→로버: 정지대기 → 서보 팔접기 → 리니어 확장 → 휠 ramp up
	// 로버→MC: 휠정지 → 리니어 수축 → 서보 팔펴기 → 드론 활성화
	const float SETTLE_TIME = _param_vt_d_settle_t.get();
	const float SERVO_DURATION = _param_vt_d_servo_dur.get();
	const float LINEAR_DURATION = SERVO_DURATION; // 리니어도 서보와 동일 기간
	const float TOTAL_TRANSITION_TIME = SETTLE_TIME + SERVO_DURATION + LINEAR_DURATION;

	VtolType::update_transition_state();

	float mc_weight = 0.0f;

	const float WHEEL_RAMP = _param_vt_d_whl_ramp.get();

	if (_vtol_mode == vtol_mode::TRANSITION_TO_FW) {
		// MC→로버: 정지대기 → 서보 팔접기 → 리니어 확장 → 휠 ramp up
		mc_weight = 0.0f;
		_pusher_throttle = 0.0f;

		if (_time_since_trans_start < SETTLE_TIME) {
			// Phase 0: 정지 대기 — 모터 끄고 안정화
			_servo_arm_cmd = 1.0f;
			_linear_act_cmd = -1.0f;

		} else if (_time_since_trans_start < SETTLE_TIME + SERVO_DURATION) {
			// Phase 1: 서보 팔 접기, 1.0→-1.0 램프
			float progress = (_time_since_trans_start - SETTLE_TIME) / SERVO_DURATION;
			_servo_arm_cmd = 1.0f - 2.0f * progress;
			_linear_act_cmd = -1.0f;

		} else if (_time_since_trans_start < TOTAL_TRANSITION_TIME) {
			// Phase 2: 리니어 확장, -1.0→1.0 램프
			float progress = (_time_since_trans_start - SETTLE_TIME - SERVO_DURATION) / LINEAR_DURATION;
			_servo_arm_cmd = -1.0f;
			_linear_act_cmd = -1.0f + 2.0f * progress;
		}
		// Phase 3: TOTAL_TRANSITION_TIME 경과 → isFrontTransitionCompletedBase()에서 FW_MODE 전환

	} else if (_vtol_mode == vtol_mode::TRANSITION_TO_MC) {
		// 로버→MC: 휠정지 → 리니어 수축 → 서보 팔펴기 → 드론 활성화
		_wheel_left = 0.0f;
		_wheel_right = 0.0f;

		if (_time_since_trans_start < LINEAR_DURATION) {
			// Phase 1: 리니어 수축, 1.0→-1.0 램프
			float progress = _time_since_trans_start / LINEAR_DURATION;
			_servo_arm_cmd = -1.0f;
			_linear_act_cmd = 1.0f - 2.0f * progress;
			mc_weight = 0.0f;

		} else if (_time_since_trans_start < LINEAR_DURATION + SERVO_DURATION) {
			// Phase 2: 서보 팔 펴기, -1.0→1.0 램프
			float progress = (_time_since_trans_start - LINEAR_DURATION) / SERVO_DURATION;
			_servo_arm_cmd = -1.0f + 2.0f * progress;
			_linear_act_cmd = -1.0f;
			mc_weight = 0.0f;

		} else {
			// Phase 3: 전환 완료, MC 준비
			_servo_arm_cmd = 1.0f;
			_linear_act_cmd = -1.0f;
			mc_weight = 1.0f;
		}
	}

	_mc_roll_weight = mc_weight;
	_mc_pitch_weight = mc_weight;
	_mc_yaw_weight = mc_weight;
	_mc_throttle_weight = mc_weight;
}

void Standard::update_mc_state()
{
	VtolType::update_mc_state();

	_pusher_throttle = VtolType::pusher_assist();
}

void Standard::update_fw_state()
{
	// DROBOT: 로버 모드 — FW 컨트롤러(TECS/quadchute) 우회
	_mc_roll_weight = 0.0f;
	_mc_pitch_weight = 0.0f;
	_mc_yaw_weight = 0.0f;
	_mc_throttle_weight = 0.0f;

	// 휠 ramp up 타이머 초기화
	if (!_fw_mode_entered) {
		_fw_mode_enter_time = hrt_absolute_time();
		_fw_mode_entered = true;
	}

	// RC 스틱 → 휠 명령
	manual_control_setpoint_s manual_sp;

	if (_manual_control_setpoint_sub.update(&manual_sp)) {
		const float throttle = manual_sp.throttle;  // [-1, 1]
		const float steering = manual_sp.roll;       // [-1, 1]

		// 역기구학: differential drive
		float raw_left  = math::constrain(throttle - steering, -1.f, 1.f);
		float raw_right = math::constrain(throttle + steering, -1.f, 1.f);

		// 휠 ramp up 적용
		const float whl_ramp = _param_vt_d_whl_ramp.get();
		float ramp_scale = 1.0f;

		if (whl_ramp > 0.f) {
			float elapsed = (hrt_absolute_time() - _fw_mode_enter_time) * 1e-6f;
			ramp_scale = math::constrain(elapsed / whl_ramp, 0.f, 1.f);
		}

		_wheel_left  = raw_left * ramp_scale;
		_wheel_right = raw_right * ramp_scale;
	}
}

/**
 * Prepare message to actuators with data from mc and fw attitude controllers. An mc attitude weighting will determine
 * what proportion of control should be applied to each of the control groups (mc and fw).
 */
void Standard::fill_actuator_outputs()
{
	_torque_setpoint_0->timestamp = hrt_absolute_time();
	_torque_setpoint_0->timestamp_sample = _vehicle_torque_setpoint_virtual_mc->timestamp_sample;
	_torque_setpoint_0->xyz[0] = 0.f;
	_torque_setpoint_0->xyz[1] = 0.f;
	_torque_setpoint_0->xyz[2] = 0.f;

	_torque_setpoint_1->timestamp = hrt_absolute_time();
	_torque_setpoint_1->timestamp_sample = _vehicle_torque_setpoint_virtual_fw->timestamp_sample;
	_torque_setpoint_1->xyz[0] = 0.f;
	_torque_setpoint_1->xyz[1] = 0.f;
	_torque_setpoint_1->xyz[2] = 0.f;

	_thrust_setpoint_0->timestamp = hrt_absolute_time();
	_thrust_setpoint_0->timestamp_sample = _vehicle_thrust_setpoint_virtual_mc->timestamp_sample;
	_thrust_setpoint_0->xyz[0] = 0.f;
	_thrust_setpoint_0->xyz[1] = 0.f;
	_thrust_setpoint_0->xyz[2] = 0.f;

	_thrust_setpoint_1->timestamp = hrt_absolute_time();
	_thrust_setpoint_1->timestamp_sample = _vehicle_thrust_setpoint_virtual_fw->timestamp_sample;
	_thrust_setpoint_1->xyz[0] = 0.f;
	_thrust_setpoint_1->xyz[1] = 0.f;
	_thrust_setpoint_1->xyz[2] = 0.f;

	switch (_vtol_mode) {
	case vtol_mode::MC_MODE:

		// MC actuators:
		_torque_setpoint_0->xyz[0] = _vehicle_torque_setpoint_virtual_mc->xyz[0];
		_torque_setpoint_0->xyz[1] = _vehicle_torque_setpoint_virtual_mc->xyz[1];
		_torque_setpoint_0->xyz[2] = _vehicle_torque_setpoint_virtual_mc->xyz[2];
		_thrust_setpoint_0->xyz[2] = _vehicle_thrust_setpoint_virtual_mc->xyz[2];

		// DROBOT: 서보/리니어 명령 (CA가 roll→서보, pitch→리니어로 매핑)
		_torque_setpoint_1->xyz[0] = _servo_arm_cmd;   // 팔 펴짐 유지
		_torque_setpoint_1->xyz[1] = _linear_act_cmd;  // 수축 유지
		break;

	case vtol_mode::TRANSITION_TO_FW:
	// FALLTHROUGH
	case vtol_mode::TRANSITION_TO_MC:
		// DROBOT: 전환 중 MC 모터는 가중치 적용
		_torque_setpoint_0->xyz[0] = _vehicle_torque_setpoint_virtual_mc->xyz[0] * _mc_roll_weight;
		_torque_setpoint_0->xyz[1] = _vehicle_torque_setpoint_virtual_mc->xyz[1] * _mc_pitch_weight;
		_torque_setpoint_0->xyz[2] = _vehicle_torque_setpoint_virtual_mc->xyz[2] * _mc_yaw_weight;
		_thrust_setpoint_0->xyz[2] = _vehicle_thrust_setpoint_virtual_mc->xyz[2] * _mc_throttle_weight;
		// DROBOT: 서보/리니어 명령 (CA가 roll→서보, pitch→리니어로 매핑)
		_torque_setpoint_1->xyz[0] = _servo_arm_cmd;
		_torque_setpoint_1->xyz[1] = _linear_act_cmd;
		break;

	case vtol_mode::FW_MODE:
		// DROBOT: 로버 모드 — MC 출력 0 (휠/서보는 직접 발행)
		_servo_arm_cmd = -1.0f;   // 팔 접힘 유지
		_linear_act_cmd = 1.0f;   // 확장 유지
		break;
	}
}

void
Standard::waiting_on_tecs()
{
	// keep thrust from transition
	_v_att_sp->thrust_body[0] = _pusher_throttle;
};

void Standard::blendThrottleAfterFrontTransition(float scale)
{
	const float tecs_throttle = _v_att_sp->thrust_body[0];
	_v_att_sp->thrust_body[0] = scale * tecs_throttle + (1.0f - scale) * _pusher_throttle;
}
