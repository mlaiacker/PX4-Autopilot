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

/**
 * @file uart_unisens.cpp
 *
 * Read data from SM-Modellbau Unisens over uart and publish battery_status in uOrb
 *
 * @author Maximilian Laiacker <post@mlaiacker.de>
 */

#include <px4_getopt.h>
#include <px4_log.h>
#include <px4_posix.h>

#include <sys/types.h>
#include <sys/ioctl.h>

#include <termios.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <arch/board/board.h>

#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/trip2_sys_report.h>
#include <uORB/topics/trip2_los_report.h>
#include <uORB/topics/trip2_gnd_report.h>

#include <drivers/drv_hrt.h>
#include <drivers/drv_pwm_output.h>
#include <drivers/drv_adc.h>
#include "gd_payload.h"

int GDPayload::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
	driver for gd_payload
### Examples
CLI usage example:
$ gd_payload start
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("gd_payload", "modules");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_COMMAND("on");
	PRINT_MODULE_USAGE_COMMAND("off");
	PRINT_MODULE_USAGE_COMMAND("trip2");
	PRINT_MODULE_USAGE_PARAM_FLAG('v', "debug flag", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

int GDPayload::print_status()
{
	// print additional runtime information about the state of the module
	PX4_INFO("inst: %i", _instance);
//	PX4_INFO("rate: %i", _rate);
	PX4_INFO("voltage: %.2fV (%f)", (double)_voltage_v, double(_parameters.battery_v_div));
	PX4_INFO("current: %.3fA (%f)", (double)_current_a, double(_parameters.battery_a_per_v));
	PX4_INFO("bat used: %.1fmAh", (double)_used_mAh);
	for (unsigned i = 0; i < PX4_MAX_ADC_CHANNELS; i++)
	{
		PX4_INFO("ADC:%i, %i=%i",i,_buf_adc[i].am_channel, _buf_adc[i].am_data);
	}
	return 0;
}

int GDPayload::custom_command(int argc, char *argv[])
{
	const char *verb = argv[0];

	if (!strcmp(verb, "on")) {
		writePayloadPower(true);
		return 0;
	}

	if (!strcmp(verb, "off")) {
		writePayloadPower(false);
		return 0;
	}

	if (!strcmp(verb, "trip2")) {
		printTrip2Report();
		return 0;
	}

	return print_usage("unknown command");
}

void GDPayload::writePayloadPower(bool on)
{
	int fd = open(PX4IO_DEV, O_RDWR);

	if (fd < 0) {
		PX4_ERR("open fail");
		return;
	}

//	if(_debug_flag)
	{
		PX4_INFO("power %i", (int)on);
	}

	if(on)
	{
		ioctl(fd, DSM_BIND_POWER_UP, 0);
	} else
	{
		ioctl(fd, DSM_BIND_START,0);
	}
	close(fd);
}

void GDPayload::printTrip2Report()
{
	int sub_trip2_sys = orb_subscribe(ORB_ID(trip2_sys_report));
	if(sub_trip2_sys>0)
	{
		char modes[11][16]={"Stow","Pilot","Retract","Retract Lock","Observation","GRR","Hold Coord","Point Coord","Local Pos","Global Pos","Track"};
		trip2_sys_report_s uorb_trip2_sys;
		orb_copy(ORB_ID(trip2_sys_report), sub_trip2_sys, &uorb_trip2_sys);
		if(uorb_trip2_sys.system_mode<=10)
		{
			PX4_INFO("Mode:%s",modes[uorb_trip2_sys.system_mode]);
		}
		else
		{
			PX4_INFO("Mode:%i",uorb_trip2_sys.system_mode);
		}
		print_message(uorb_trip2_sys);
		orb_unsubscribe(sub_trip2_sys);
	} else
	{
		PX4_ERR("failed to subscribe to trip2_sys topic");
	}
	int sub_trip2_los = orb_subscribe(ORB_ID(trip2_los_report));
	if(sub_trip2_los>0)
	{
		trip2_los_report_s uorb_trip2_los;
		orb_copy(ORB_ID(trip2_los_report), sub_trip2_los, &uorb_trip2_los);
		print_message(uorb_trip2_los);
		orb_unsubscribe(sub_trip2_los);
	} else
	{
		PX4_ERR("failed to subscribe to trip2 los topic");
	}
	int sub_trip2_gnd = orb_subscribe(ORB_ID(trip2_gnd_report));
	if(sub_trip2_gnd>0)
	{
		trip2_gnd_report_s uorb_trip2_gnd;
		orb_copy(ORB_ID(trip2_gnd_report), sub_trip2_gnd, &uorb_trip2_gnd);
		print_message(uorb_trip2_gnd);
		orb_unsubscribe(sub_trip2_gnd);
	} else
	{
		PX4_ERR("failed to subscribe to trip2 gnd topic");
	}
}

int GDPayload::task_spawn(int argc, char *argv[])
{
	_task_id = px4_task_spawn_cmd("gd_payload",
				      SCHED_DEFAULT,
				      SCHED_PRIORITY_DEFAULT+11, /* reduced pritority */
				      1512,
				      (px4_main_t)&run_trampoline,
				      (char *const *)argv);

	if (_task_id < 0) {
		_task_id = -1;
		return -errno;
	}

	return 0;
}

GDPayload *GDPayload::instantiate(int argc, char *argv[])
{
	bool debug_flag = false;
	bool error_flag = false;
	int ch;
	int myoptind = 1;
	const char *myoptarg = nullptr;
	const char *device = nullptr;

	// parse CLI arguments
	while ((ch = px4_getopt(argc, argv, "d:v", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'd':
			device = myoptarg;
			break;

		case 'v':
			debug_flag = true;
			break;

		case '?':
			error_flag = true;
			break;

		default:
			PX4_WARN("unrecognized flag");
			error_flag = true;
			break;
		}
	}

	if (error_flag) {
		return nullptr;
	}

	GDPayload *instance = new GDPayload(device, debug_flag);

	if (instance == nullptr) {
		PX4_ERR("alloc failed");
	}

	return instance;
}

GDPayload::GDPayload(char const *const device, bool debug_flag):
ModuleParams(nullptr),
_pub_battery(nullptr)
{
	if(device)
	{
		memcpy(_device, device,sizeof(_device));
	} else{
		memcpy(_device, PX4IO_DEV,sizeof(PX4IO_DEV));
	}

	_debug_flag = debug_flag;
	memset(&_battery_status,0,sizeof(_battery_status));
	memset(&_buf_adc,0,sizeof(_buf_adc));
	_used_mAh = 0.0f;
	_voltage_v = 0.0f;
	_current_a = 0.0f;
}

bool GDPayload::init()
{
	bool result = true;

	DriverFramework::DevMgr::getHandle(ADC0_DEVICE_PATH, _h_adc);

	_parameters_handles.battery_v_div = param_find("BAT_V_DIV");
	_parameters_handles.battery_a_per_v = param_find("BAT_A_PER_V");

	// initialize parameters
	_parameter_update_sub = orb_subscribe(ORB_ID(parameter_update));
	parameters_update(_parameter_update_sub, true);

	/* needed to read arming status */
	_vcontrol_mode_sub = orb_subscribe(ORB_ID(vehicle_control_mode));
	return result;
}

void GDPayload::parameters_update(int parameter_update_sub, bool force)
{
	bool updated;
	struct parameter_update_s param_upd;

	orb_check(parameter_update_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(parameter_update), parameter_update_sub, &param_upd);
	}

	if (force || updated) {

		const char *paramerr = "FAIL PARM LOAD";
		if(_debug_flag)
		{
			PX4_DEBUG("param upd");
		}
		if (param_get(_parameters_handles.battery_v_div, &(_parameters.battery_v_div)) != OK) {
			PX4_WARN("%s", paramerr);
			_parameters.battery_v_div = 0.0f;

		} else if (_parameters.battery_v_div <= 0.0f) {
			/* apply scaling according to defaults if set to default */

			_parameters.battery_v_div = 15.468f; /* 68k and 4k7 ohm voltage divider */
			param_set_no_notification(_parameters_handles.battery_v_div, &_parameters.battery_v_div);
		}

		if (param_get(_parameters_handles.battery_a_per_v, &(_parameters.battery_a_per_v)) != OK) {
			PX4_WARN("%s", paramerr);
			_parameters.battery_a_per_v = 0.0f;

		} else if (_parameters.battery_a_per_v <= 0.0f) {
			/* apply scaling according to defaults if set to default */

			_parameters.battery_a_per_v = 1.0f; /* 1V=1A */
			param_set_no_notification(_parameters_handles.battery_a_per_v, &_parameters.battery_a_per_v);
		}
	}
}

/* read arming state */
void GDPayload::vehicle_control_mode_poll()
{
	struct vehicle_control_mode_s vcontrol_mode;
	bool vcontrol_mode_updated;
	orb_check(_vcontrol_mode_sub, &vcontrol_mode_updated);
	if (vcontrol_mode_updated) {
		orb_copy(ORB_ID(vehicle_control_mode), _vcontrol_mode_sub, &vcontrol_mode);
		_armed = vcontrol_mode.flag_armed;
	}
}

/*
 * after timeout or when exit the module signal that we lost connection to the battery sensor
 */
void GDPayload::updateBatteryDisconnect()
{
	_battery_status.warning = battery_status_s::BATTERY_WARNING_FAILED;
	int instance;
	_battery_status.temperature = -1;
	_battery_status.timestamp = hrt_absolute_time();
	orb_publish_auto(ORB_ID(battery_status), &_pub_battery, &_battery_status, &instance, ORB_PRIO_DEFAULT);
	if(_debug_flag)
	{
		PX4_INFO("disconnect");
	}

}

bool  GDPayload::readPayloadAdc()
{
	if(!_h_adc.isValid())
	{
		DriverFramework::DevMgr::getHandle(ADC0_DEVICE_PATH, _h_adc);
		if(!_h_adc.isValid())
		{
			return false;
		}
	}

		/* make space for a maximum of twelve channels (to ensure reading all channels at once) */
		px4_adc_msg_t buf_adc[PX4_MAX_ADC_CHANNELS];
		/* read all channels available */
		int ret = _h_adc.read(&buf_adc, sizeof(buf_adc));


		/* Based on the valid_chan, used to indicate the selected the lowest index
		 * (highest priority) supply that is the source for the VDD_5V_IN
		 * When < 0 none selected
		 */
		if (ret >= (int)sizeof(buf_adc[0])) {
			/* Read adc channels we got */
			for (unsigned i = 0; i < ret / sizeof(buf_adc[0]); i++)
			{
				_buf_adc[i] = buf_adc[i];
				/* look for specific channels and process the raw voltage to measurement data */
				if (buf_adc[i].am_channel == 13) /* PC3=adc0 channel 13 */
				{
					/* Voltage in volts */
					_voltage_v = ((float)buf_adc[i].am_data * (3.3f / 4096)) * _parameters.battery_v_div;

				} else if (buf_adc[i].am_channel == 14) /* PC4=adc0 channel 14 */
				{
					_current_a = ((float)buf_adc[i].am_data * (3.3f / 4096)) * _parameters.battery_a_per_v;
				}
			}
			return true;
		}
	return false;
}



void GDPayload::run()
{
	if(!init())
		return;
	hrt_abstime second_timer = hrt_absolute_time();
	int n = 0;
	if(_debug_flag)
	{
		PX4_INFO("Start");
	}
	while (!should_exit()) {
		parameters_update(_parameter_update_sub);
		if(readPayloadAdc())
		{
				n++;
				vehicle_control_mode_poll();

				bool connected = _voltage_v > 5.0f;

				if(_battery_status.timestamp>0)
				{
					float dt =  (hrt_absolute_time() -_battery_status.timestamp)*1.0e-6f;
					_used_mAh += _current_a*dt*1000.0f/3600.0f; /* convert to mAh */
				}
				_battery_status.timestamp = hrt_absolute_time();
				_battery_status.connected = connected;
				_battery_status.temperature = INT16_MAX;
				_battery_status.voltage_v = _voltage_v;
				_battery_status.cell_count = 8;
				_battery_status.current_a = _current_a;
				_battery_status.voltage_filtered_v = _battery_status.voltage_filtered_v*0.8f + _voltage_v*0.2f; /* override filtered value */
				_battery_status.current_filtered_a = _battery_status.current_filtered_a*0.8f + _current_a*0.2f;
				_battery_status.discharged_mah = _used_mAh;
				if(orb_publish_auto(ORB_ID(battery_status), &_pub_battery, &_battery_status, &_instance, ORB_PRIO_DEFAULT))
				{
					if(_debug_flag)
					{
						PX4_INFO("pup failed");
					}
				}
				_timeout = 0;
		}
		usleep(100000);
		if(hrt_elapsed_time(&second_timer)>=10e6) /* every 10 seconds*/
		{
			_rate = n/10; /* update rate in Hz */
			second_timer += 10e6;
			if(n==0 && _timeout==0)
			{
				updateBatteryDisconnect(); /* lost connection to unisens */
				_timeout = 1;
			}
			n=0;
		}
	}
	updateBatteryDisconnect();
	orb_unsubscribe(_vcontrol_mode_sub);
	orb_unsubscribe(_parameter_update_sub);
	orb_unadvertise(_pub_battery);
}

int gd_payload_main(int argc, char *argv[])
{
	return GDPayload::main(argc, argv);
}
