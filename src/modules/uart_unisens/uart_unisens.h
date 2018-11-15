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


extern "C" __EXPORT int uart_unisens_main(int argc, char *argv[]);

#define UART_UNISENS_UART "/dev/ttyS6"

class UartUnisens : public ModuleBase<UartUnisens>, public ModuleParams
{
public:
	UartUnisens(char const *const device, bool debug_flag);

	virtual ~UartUnisens() = default;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static UartUnisens *instantiate(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	/** @see ModuleBase::run() */
	void run() override;

	/** @see ModuleBase::print_status() */
	int print_status() override;
private:
	char 	_device[32];
	int 	_uart_fd = -1;
	int 	_rate = 0;
	int		_timeout = 0;
	bool	_debug_flag = false;
	orb_advert_t		_pub_battery;
	battery_status_s	_battery_status;
	Battery				_battery;			/**< Helper lib to publish battery_status topic. */
	float _voltage_v, _current_a, _used_mAh, _temp_c,_baro_hPa, _voltage_rc_v ;
	int		_actuator_ctrl_0_sub{-1};		/**< attitude controls sub */
	int		_vcontrol_mode_sub{-1};		/**< vehicle control mode subscription */
	bool		_armed{false};

	bool init();

	bool readPoll(uint32_t tout=5000);
	bool  readStatusInit();
	bool  readStatusData();
	void writeRequest();
	void writeInit();
	void vehicle_control_mode_poll();
	int8_t calcCheckSum(uint8_t *data, ssize_t length);
	void updateBatteryDisconnect();
	/**
	 * Check for parameter changes and update them if needed.
	 * @param parameter_update_sub uorb subscription to parameter_update
	 * @param force for a parameter update
	 */
	void parameters_update(int parameter_update_sub, bool force = false);
};

