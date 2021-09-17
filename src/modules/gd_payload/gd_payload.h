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

#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <battery/battery.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/adc_report.h>
#include <uORB/Subscription.hpp>
#include <uORB/topics/debug_key_value.h>


extern "C" __EXPORT int gd_payload_main(int argc, char *argv[]);

#define PX4IO_DEV "/dev/px4io"

class GDPayload : public ModuleBase<GDPayload>, public ModuleParams
{
public:
	GDPayload(char const *const device, bool debug_flag);

	virtual ~GDPayload() = default;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static GDPayload *instantiate(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	/** @see ModuleBase::run() */
	void run() override;

	/** @see ModuleBase::print_status() */
	int print_status() override;

	static 	void writePayloadPower(bool on);

	static  void printTrip2Report();
	static  void printBatteryReport();
	static  void batGetAll(battery_status_s *lion, battery_status_s *lipo);
private:
	struct {
		float battery_v_div{15.468f};
		float battery_a_per_v{1.0f};
		float payload_current_warn_a{3.0f}; // current above we should send a warning message
	} _parameters;
	struct {
		param_t battery_v_div{PARAM_INVALID};
		param_t battery_a_per_v{PARAM_INVALID};
		param_t payload_current_warn_a{PARAM_INVALID};
	} _parameters_handles;
	int 	_parameter_update_sub{-1};

#ifdef __PX4_NUTTX
	adc_report_s _adc_report;
	uORB::Subscription	_adc_report_sub{ORB_ID(adc_report)};
	int 	_px4io_fd = -1; // needed for payload on/off
#else
	// stuff needed for simulation
	Battery				_battery_sim;
	uORB::Subscription	_sub_vehicle_status{ORB_ID(vehicle_status)};
	struct vehicle_status_s _vstatus {};
	uORB::Subscription 	_sub_vtol_status{ORB_ID(vtol_vehicle_status)}; // needed for battery switchover
	uORB::Subscription	_sub_actuator_ctrl_0{ORB_ID(actuator_controls_0)};		/**< attitude controls sub */
	bool				_sim_was_armed{false};
	uORB::Publication<debug_key_value_s>	_sim_pub_pdb_temp{ORB_ID(debug_key_value)}; // generate PDB temperature in simulation
	struct debug_key_value_s _sim_temp_pdb;
	uORB::Subscription	_sub_global_pos{ORB_ID(vehicle_global_position)};
#endif

	uORB::Publication<vehicle_command_ack_s>	_command_ack_pub{ORB_ID(vehicle_command_ack)};

	int 				_instance{0}; // for battery (payload) publish
	battery_status_s	_battery_status;
	hrt_abstime 		_payload_last_warn_time{0};
	orb_advert_t		_pub_battery{nullptr};
	float 				_voltage_v{0.0f};
	float 				_current_a{0.0f};
	float 				_used_mAh{0.0f};

	// for reading temperature of PDB
	uORB::Subscription  _sub_debug_key{ORB_ID(debug_key_value)};
	float _temp_last_report_c{0.0f}; // last temperature value in degC
	hrt_abstime _temp_last_warn_time{0};
	orb_advert_t _pub_mavlink_log{nullptr}; // for sending messages to GCS

	bool	_debug_flag = false;
	uORB::Subscription 	_sub_vehicle_cmd{ORB_ID(vehicle_command)};

	bool  readPayloadAdc();
	void updateBatteryDisconnect();
	void vehicleCommand(const vehicle_command_s *vcmd);
	void vehicleCommandAck(const vehicle_command_s *cmd);

	bool init();

	void vehicle_control_mode_poll();
	/**
	 * Check for parameter changes and update them if needed.
	 * @param parameter_update_sub uorb subscription to parameter_update
	 * @param force for a parameter update
	 */
	void parameters_update(int parameter_update_sub, bool force = false);

};

