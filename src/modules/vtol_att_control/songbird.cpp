/****************************************************************************
 *
 *   Copyright (c) 2015 PX4 Development Team. All rights reserved.
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
 * @file songbird.cpp
 *
 * @author Maximilian Laiacker 		<m.laiacker@germandrones.com>
 *
*/

#include "songbird.h"
#include "vtol_att_control_main.h"


Songbird::Songbird(VtolAttitudeControl *attc) :
	Tiltrotor(attc),
	_tilt_yaw_lp_pitch(250, 1.0f)
{
}

/**
* Write data to actuator output topic.
*/
void Songbird::fill_actuator_outputs()
{
	//float tilt_pitch_rad = 0.0f; // tilt angle offset to fly forwards in mc mode
	float tilt_angle_rad = 0.0f; // combined angle from transition and attitude control, used for mc mixing
	// Multirotor output
	_actuators_out_0->timestamp = hrt_absolute_time();
	_actuators_out_0->timestamp_sample = _actuators_mc_in->timestamp_sample;


	_mc_yaw_weight = _mc_roll_weight;

	if (_vtol_schedule.flight_mode == vtol_mode::FW_MODE) {
		_actuators_out_0->control[actuator_controls_s::INDEX_THROTTLE] =
			_actuators_fw_in->control[actuator_controls_s::INDEX_THROTTLE];

		/* allow differential thrust if enabled */
		if (_params->diff_thrust == 1) {
			_actuators_out_0->control[actuator_controls_s::INDEX_ROLL] =
				_actuators_fw_in->control[actuator_controls_s::INDEX_YAW] * _params->diff_thrust_scale;
		}

	} else {
		_actuators_out_0->control[actuator_controls_s::INDEX_THROTTLE] =
			_actuators_mc_in->control[actuator_controls_s::INDEX_THROTTLE] * _mc_throttle_weight;
		/* use output of  mc_att control for _tilt based on pitch cmd*/
	//	tilt_pitch_rad = (PX4_ISFINITE(_actuators_mc_in->control[4])) ? _actuators_mc_in->control[4] : 0.0f;
	}

	// Fixed wing output
	_actuators_out_1->timestamp = hrt_absolute_time();
	_actuators_out_1->timestamp_sample = _actuators_fw_in->timestamp_sample;
	// _tilt_control is between 0 and 1 so mapping to 0 and 90 degrees
	float tilt_cos = 1.0f;
	float tilt_sin = 0.0f;
	if(PX4_ISFINITE(_tilt_control)){
		if(_tilt_control>1.0f) {
			_tilt_control = 1.0f;
		}
		if(_tilt_control<0.0f) {
			_tilt_control = 0.0f;
		}
		tilt_angle_rad = _tilt_control *M_PI_F*0.5f;// + tilt_pitch_rad; // 0 : up pi/2 : forward

		_actuators_out_1->control[4] = acosf(1.0f - (tilt_angle_rad) * 4.0f / M_PI_F) /
						   M_PI_F; /* nonlinear map from tilt angle to servo angle*/;

		tilt_cos = 1.0f / cosf(fminf(M_PI_F * 0.5f, tilt_angle_rad)); // max 45deg tilt compensation
		tilt_sin = sinf(fminf(M_PI_F * 0.5f, tilt_angle_rad)) * 0.5f; // compensation for roll <-> yaw coupling
	}
	// we have to mix
	_actuators_out_0->control[actuator_controls_s::INDEX_ROLL] = (_actuators_mc_in->control[actuator_controls_s::INDEX_ROLL]
			* tilt_cos + _actuators_mc_in->control[actuator_controls_s::INDEX_YAW] * tilt_sin) * _mc_roll_weight;
	_actuators_out_0->control[actuator_controls_s::INDEX_PITCH] =
		_actuators_mc_in->control[actuator_controls_s::INDEX_PITCH] * tilt_cos * _mc_pitch_weight;
	_actuators_out_0->control[actuator_controls_s::INDEX_YAW] = (_actuators_mc_in->control[actuator_controls_s::INDEX_YAW] *
			tilt_cos  - _actuators_mc_in->control[actuator_controls_s::INDEX_ROLL] * tilt_sin) * _mc_yaw_weight;
	/* filter yaw control to use for differential tilt */
	_actuators_out_0->control[actuator_controls_s::INDEX_AIRBRAKES] = _tilt_yaw_lp_pitch.apply(
				_actuators_out_0->control[actuator_controls_s::INDEX_YAW]);


	if (_params->elevons_mc_lock && _vtol_schedule.flight_mode == vtol_mode::MC_MODE) {
		_actuators_out_1->control[actuator_controls_s::INDEX_ROLL] = 0.0f;
		_actuators_out_1->control[actuator_controls_s::INDEX_PITCH] = 0.0f;
		_actuators_out_1->control[actuator_controls_s::INDEX_YAW] = 0.0f;

	} else {
		_actuators_out_1->control[actuator_controls_s::INDEX_ROLL] =
			_actuators_fw_in->control[actuator_controls_s::INDEX_ROLL];
		_actuators_out_1->control[actuator_controls_s::INDEX_PITCH] =
			_actuators_fw_in->control[actuator_controls_s::INDEX_PITCH];
		_actuators_out_1->control[actuator_controls_s::INDEX_YAW] =
			_actuators_fw_in->control[actuator_controls_s::INDEX_YAW];
	}
}
