/****************************************************************************
 *
 *   Copyright (c) 2018 PX4 Development Team. All rights reserved.
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

#pragma once

#include <px4_module.h>
#include <px4_module_params.h>
#include <px4_posix.h>

#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/esc_status.h>
#include <uORB/topics/actuator_outputs.h>
#include <uORB/topics/actuator_controls.h>

#include <mixer/mixer.h>


extern "C" __EXPORT int can_vesc_main(int argc, char *argv[]);

#define VESC_CAN_ID_START	1
#define VESC_CAN_NUM		4


class CanVESC : public ModuleBase<CanVESC>, public ModuleParams
{
public:
	CanVESC(bool debug_flag);

	virtual ~CanVESC() = default;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static CanVESC *instantiate(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	/** @see ModuleBase::run() */
	void run() override;

	/** @see ModuleBase::print_status() */
	int print_status() override;
private:
	int 		_can_fd; //
	bool 		_is_armed = false;
	int			_armed_sub = -1;
	int			_actuator_outputs_sub=-1;	///< actuator outputs topic
	int			_esc_update_count = 0;
	int			_can_write_error = 0;
	int			_esc_update_freq = 0;
	actuator_armed_s	_armed = {};
	orb_advert_t      _esc_feedback_pub = nullptr;
	esc_status_s      _esc_feedback = {};
	bool _debug_flag = false;

	static const uint8_t 	_device_mux_map[4];
	static const uint8_t 	_device_dir_map[4];
	int			_control_subs[actuator_controls_s::NUM_ACTUATOR_CONTROL_GROUPS];
	actuator_controls_s 	_controls[actuator_controls_s::NUM_ACTUATOR_CONTROL_GROUPS];
	orb_id_t		_control_topics[actuator_controls_s::NUM_ACTUATOR_CONTROL_GROUPS];
	px4_pollfd_struct_t	_poll_fds[actuator_controls_s::NUM_ACTUATOR_CONTROL_GROUPS];
	unsigned		_poll_fds_num = 0;
	MixerGroup	*_mixers = nullptr;
	actuator_outputs_s      _outputs = {};
	orb_advert_t      _to_mixer_status = nullptr; 	///< mixer status flags

	bool init();
	//! Byte swap unsigned short
	static uint16_t swap_uint16(uint16_t val)
	{
		return (val << 8) | (val >> 8);
	}

	//! Byte swap short
	static int16_t swap_int16(int16_t val)
	{
		return (val << 8) | ((val >> 8) & 0xFF);
	}

	//! Byte swap unsigned int
	static uint32_t swap_uint32(uint32_t val)
	{
		val = ((val << 8) & 0xFF00FF00) | ((val >> 8) & 0xFF00FF);
		return (val << 16) | (val >> 16);
	}

	//! Byte swap int
	static int32_t swap_int32(int32_t val)
	{
		val = ((val << 8) & 0xFF00FF00) | ((val >> 8) & 0xFF00FF);
		return (val << 16) | ((val >> 16) & 0xFFFF);
	}


	void readStatus();
	void writeDuty(uint8_t vescAddr, float duty);
	/**
	 * Check for parameter changes and update them if needed.
	 * @param parameter_update_sub uorb subscription to parameter_update
	 * @param force for a parameter update
	 */
	void parameters_update(int parameter_update_sub, bool force = false);


};

