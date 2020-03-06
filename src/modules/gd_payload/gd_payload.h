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
#include <battery/battery.h>
#include <uORB/topics/battery_status.h>
#include <DevMgr.hpp>
#include <battery/battery.h>


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
	static  int  switchStatus();
	static  int  switchSet(int);
	static  void batGetAll(battery_status_s* lion, battery_status_s* lipo);
private:
	struct {
		float battery_v_div;
		float battery_a_per_v;
	} _parameters;
	struct {
		param_t battery_v_div;
		param_t battery_a_per_v;
	} _parameters_handles;
	int 	_parameter_update_sub{-1};

#ifdef __PX4_NUTTX
	DriverFramework::DevHandle 	_h_adc;				/**< ADC driver handle */

	char 	_device[32];
	px4_adc_msg_t _buf_adc[PX4_MAX_ADC_CHANNELS];
	int 	_px4io_fd = -1;
#else
	orb_advert_t		_pub_battery_sim;
	Battery				_battery_sim;
	int					_sub_actuator_ctrl_0{-1};		/**< attitude controls sub */
	int 				_instance_sim={-1};
	battery_status_s 	_batt_sim;
#endif

	orb_advert_t		_pub_battery;
	orb_advert_t		_vehicle_command_ack_pub;

	battery_status_s	_battery_status;
	float _voltage_v, _current_a, _used_mAh;
	bool  readPayloadAdc();
	void updateBatteryDisconnect();
	void vehicleCommand(const vehicle_command_s *vcmd);
	void vehicleCommandAck(const vehicle_command_s *cmd);
	bool cmdTripCordinate(double lat, double lon, float alt);
	bool cmdTripRecord(bool on);
	bool cmdTripSnapshot();
	bool cmdTripMode(int mode);

	bool	_debug_flag = false;
	int 	_instance;
	int		_sub_vcontrol_mode{-1};		/**< vehicle control mode subscription */
	int		_sub_trip2{-1};
	int 	_sub_vehicle_cmd{-1};
	int		_sub_vehicle_status{-1};

	bool		_armed{false};
	bool		_auto_mode{false};
	uint16_t _cmdold{0};

	struct vehicle_status_s _vstatus{};

	bool init();

	void vehicle_control_mode_poll();
	/**
	 * Check for parameter changes and update them if needed.
	 * @param parameter_update_sub uorb subscription to parameter_update
	 * @param force for a parameter update
	 */
	void parameters_update(int parameter_update_sub, bool force = false);

};

