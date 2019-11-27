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

#ifdef __PX4_NUTTX
#include <nuttx/arch.h>
#include <nuttx/irq.h>
#endif
#include <arch/board/board.h>

#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_command_ack.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/trip2_sys_report.h>
#include <uORB/topics/trip2_los_report.h>
#include <uORB/topics/trip2_gnd_report.h>
#include <uORB/topics/actuator_controls.h>

#include <drivers/drv_hrt.h>
#include <drivers/drv_pwm_output.h>
#include <drivers/drv_adc.h>
#include "gd_payload.h"


#ifndef __PX4_NUTTX
int g_gd_payload_on = 1;
#endif
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
	PRINT_MODULE_USAGE_COMMAND("batt");
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
#ifdef __PX4_NUTTX
	for (unsigned i = 0; i < PX4_MAX_ADC_CHANNELS; i++)
	{
		PX4_INFO("ADC:%i, %i=%i",i,_buf_adc[i].am_channel, _buf_adc[i].am_data);
	}
#endif
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
	if (!strcmp(verb, "batt")) {
		printBatteryReport();
		return 0;
	}

	return print_usage("unknown command");
}

void GDPayload::writePayloadPower(bool on)
{
#ifdef __PX4_NUTTX
	int fd = open(PX4IO_DEV, O_RDWR);

	if (fd < 0) {
		PX4_ERR("open fail");
		return;
	}

	if(on)
	{
		ioctl(fd, DSM_BIND_POWER_UP, 0);
	} else
	{
		ioctl(fd, DSM_BIND_START,0);
	}
	close(fd);
#else
	PX4_INFO("power %i", (int)on);
	if(on) g_gd_payload_on=0; else g_gd_payload_on=1;
#endif
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

void GDPayload::printBatteryReport()
{
	int num_inst = orb_group_count(ORB_ID(battery_status));
	char batt_report[256];
	const int stdin_fileno = 0;

	struct pollfd fds;
	fds.fd = stdin_fileno;
	fds.events = POLLIN;
	bool quit = false;

	while (!quit) {
		size_t len = 0;
		batt_report[0]=0;
		for(int i=0;i<num_inst;i++)
		{
			int sub_battery = orb_subscribe_multi(ORB_ID(battery_status),i);
			if(sub_battery!=-1)
			{
				struct battery_status_s bat;
				orb_copy(ORB_ID(battery_status), sub_battery, &bat);
				len += sprintf(&batt_report[len]," %d; %5.2fV; %5.2fA; %6.1fmAh;", bat.serial_number, (double)bat.voltage_v, (double)bat.current_a, (double)bat.discharged_mah);
				//print_message(bat);
				orb_unsubscribe(sub_battery);
			} else
			{
				PX4_ERR("failed to subscribe to batt topic");
			}
		}
		PX4_INFO("%s", batt_report);
		char c;
		int ret = ::poll(&fds, 1, 0); //just want to check if there is new data available
		if (ret > 0) {
			ret = ::read(stdin_fileno, &c, 1);
			if (ret) {
				quit = true;
				break;
			}
		}
		usleep(200000);
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

		default:
			PX4_WARN("unrecognized flag");
			break;
		}
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

#ifdef __PX4_NUTTX
	if(device)
	{
		memcpy(_device, device,sizeof(_device));
	} else{
		memcpy(_device, PX4IO_DEV,sizeof(PX4IO_DEV));
	}
	memset(&_buf_adc,0,sizeof(_buf_adc));
#endif
	_debug_flag = debug_flag;
	memset(&_battery_status,0,sizeof(_battery_status));
	_used_mAh = 0.0f;
	_voltage_v = 0.0f;
	_current_a = 0.0f;
}

bool GDPayload::init()
{
	bool result = true;
#ifdef __PX4_NUTTX
	DriverFramework::DevMgr::getHandle(ADC0_DEVICE_PATH, _h_adc);
#endif
	_parameters_handles.battery_v_div = param_find("BAT_V_DIV");
	_parameters_handles.battery_a_per_v = param_find("BAT_A_PER_V");

	// initialize parameters
	_parameter_update_sub = orb_subscribe(ORB_ID(parameter_update));
	parameters_update(_parameter_update_sub, true);

	/* needed to read arming status */
	_sub_vcontrol_mode = orb_subscribe(ORB_ID(vehicle_control_mode));
	_sub_vehicle_cmd = orb_subscribe(ORB_ID(vehicle_command));
	_sub_vehicle_status = orb_subscribe(ORB_ID(vehicle_status));
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

void GDPayload::vehicleCommandAck(const vehicle_command_s *cmd)
{
	vehicle_command_ack_s vehicle_command_ack = {
		.timestamp = hrt_absolute_time(),
		.result_param2 = 0,
		.command = cmd->command,
		.result = vehicle_command_s::VEHICLE_CMD_RESULT_ACCEPTED,
		.from_external = false,
		.result_param1 = 0,
		.target_system = cmd->source_system,
		.target_component = cmd->source_component
	};

	if (_vehicle_command_ack_pub == nullptr) {
		_vehicle_command_ack_pub = orb_advertise_queue(ORB_ID(vehicle_command_ack), &vehicle_command_ack,
					   vehicle_command_ack_s::ORB_QUEUE_LENGTH);

	} else {
		orb_publish(ORB_ID(vehicle_command_ack), _vehicle_command_ack_pub, &vehicle_command_ack);
	}

}

bool GDPayload::cmdTripCordinate(double lat, double lon, float alt){
	if(_debug_flag)	{
		PX4_INFO("trip point to %f %f %f", lat, lon, (double)alt);
	}
	return true;
}

bool GDPayload::cmdTripRecord(bool on){
	if(_debug_flag)	{
		PX4_INFO("trip record %d", on);
	}
	return true;
}

bool GDPayload::cmdTripSnapshot(){
	if(_debug_flag)	{
		PX4_INFO("trip snapshot");
	}
	return true;
}

bool GDPayload::cmdTripMode(int mode){
	if(_debug_flag)	{
		PX4_INFO("trip mode %d", mode);
	}
	return true;
}


void GDPayload::vehicleCommand(const vehicle_command_s *vcmd)
{
	if(_debug_flag)
	{
		PX4_INFO("cmd=%d", vcmd->command);
	}
	switch(vcmd->command)
	{
	case vehicle_command_s::VEHICLE_CMD_CUSTOM_0:
		writePayloadPower(vcmd->param1>=0.5f);
		vehicleCommandAck(vcmd);
		break;
	case vehicle_command_s::VEHICLE_CMD_DO_SET_ROI:
	case vehicle_command_s::VEHICLE_CMD_DO_SET_ROI_LOCATION:
		if(cmdTripCordinate(vcmd->param5,vcmd->param6, vcmd->param7))
		{
			vehicleCommandAck(vcmd);
		}
		break;
	case vehicle_command_s::VEHICLE_CMD_SET_CAMERA_MODE:
	case vehicle_command_s::VEHICLE_CMD_DO_DIGICAM_CONTROL:
		if((int)vcmd->param1==0) //Set System Mode
		{
			cmdTripMode(vcmd->param2);
		} else if((int)vcmd->param1==1) //Take Snap Shot
		{
			cmdTripSnapshot();
		}  else if((int)vcmd->param1==2) // set rec state
		{
			cmdTripRecord(vcmd->param2>0.5f);
		}
		break;
	case vehicle_command_s::VEHICLE_CMD_DO_TRIGGER_CONTROL:
		cmdTripSnapshot();
		break;

	default:
		//print_message(&vcmd);
		break;
	}
}

/* read arming state */
void GDPayload::vehicle_control_mode_poll()
{
	if(_sub_vcontrol_mode!=-1)
	{
		bool vcontrol_mode_updated;
		orb_check(_sub_vcontrol_mode, &vcontrol_mode_updated);
		if (vcontrol_mode_updated) {
			struct vehicle_control_mode_s vcontrol_mode;
			orb_copy(ORB_ID(vehicle_control_mode), _sub_vcontrol_mode, &vcontrol_mode);
			_armed = vcontrol_mode.flag_armed;
		}
	}
	if(_sub_vehicle_cmd!=-1)
	{
		bool updated;
		orb_check(_sub_vehicle_cmd, &updated);
		if (updated)
		{
			struct vehicle_command_s vcmd;
			orb_copy(ORB_ID(vehicle_command), _sub_vehicle_cmd, &vcmd);
			vehicleCommand(&vcmd);
			_cmdold = vcmd.command;
		}
	}
	if(_sub_vehicle_status!=-1)
	{
		bool updated;
		orb_check(_sub_vehicle_status, &updated);
		if (updated)
		{
			struct vehicle_status_s vstatus;
			orb_copy(ORB_ID(vehicle_status), _sub_vehicle_status, &vstatus);
/*			if(_vstatus.in_transition_mode != vstatus.in_transition_mode && vstatus.in_transition_mode)
			{
				PX4_INFO("in transition start");
			}
			if(_vstatus.in_transition_mode != vstatus.in_transition_mode && !vstatus.in_transition_mode)
			{
				PX4_INFO("in transition end");
			}
			if(_vstatus.in_transition_to_fw != vstatus.in_transition_to_fw && vstatus.in_transition_to_fw)
			{
				PX4_INFO("transition to fw start");
			}
			if(_vstatus.in_transition_to_fw != vstatus.in_transition_to_fw && !vstatus.in_transition_to_fw)
			{
				PX4_INFO("transition to fw finished");
			}
			if(_vstatus.is_rotary_wing != vstatus.is_rotary_wing && vstatus.is_rotary_wing)
			{
				PX4_INFO("transition copter");
			}
			if(_vstatus.is_rotary_wing != vstatus.is_rotary_wing && !vstatus.is_rotary_wing)
			{
				PX4_INFO("transition fw");
			}*/
			_vstatus = vstatus;
		}
	}
}

/*
 * after timeout or when exit the module signal that we lost connection to the battery sensor
 */
void GDPayload::updateBatteryDisconnect()
{
	_battery_status.warning = battery_status_s::BATTERY_WARNING_FAILED;
	_battery_status.temperature = -1;
	_battery_status.timestamp = hrt_absolute_time();
	orb_publish_auto(ORB_ID(battery_status), &_pub_battery, &_battery_status, &_instance, ORB_PRIO_LOW);
	if(_debug_flag)
	{
		PX4_INFO("disconnect");
	}

}

bool  GDPayload::readPayloadAdc()
{
#ifdef __PX4_NUTTX
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
#else
	_voltage_v = _battery_sim.cell_count()*_battery_sim.full_cell_voltage() + rand()*0.1f/RAND_MAX;
	_current_a = (0.83f + rand()*0.01f/RAND_MAX)*g_gd_payload_on;
	vehicle_control_mode_poll();
	if(_vstatus.hil_state == _vstatus.HIL_STATE_ON)
	{
		float sim_current_a=_current_a, sim_voltage_v= _voltage_v;
		/* needed for the Battery class */
		if(_sub_actuator_ctrl_0<0){
			_sub_actuator_ctrl_0 = orb_subscribe(ORB_ID(actuator_controls_0));
		}
		actuator_controls_s ctrl;
		orb_copy(ORB_ID(actuator_controls_0), _sub_actuator_ctrl_0, &ctrl);

		if (_vstatus.arming_state == _vstatus.ARMING_STATE_ARMED){
			if(_vstatus.is_rotary_wing){
				sim_current_a += 5.0f + 180.0f*ctrl.control[actuator_controls_s::INDEX_THROTTLE]*ctrl.control[actuator_controls_s::INDEX_THROTTLE];
			} else{
				sim_current_a += 1.0f + 75.0f*ctrl.control[actuator_controls_s::INDEX_THROTTLE]*ctrl.control[actuator_controls_s::INDEX_THROTTLE]
							     + rand()*10.0f/RAND_MAX;
			}
			if(_batt_sim.connected)
			{
				sim_voltage_v -= _battery_sim.cell_count()*1.3f*(1.0f-_batt_sim.remaining);
			}
			sim_voltage_v -= sim_current_a*0.007f;
		} else {
			_battery_sim.rechargeBattery();
		}
		_battery_sim.updateBatteryStatus(hrt_absolute_time(),
				sim_voltage_v, sim_current_a,
				true, true, 1, ctrl.control[actuator_controls_s::INDEX_THROTTLE],
				_vstatus.arming_state == _vstatus.ARMING_STATE_ARMED,
				&_batt_sim);
		orb_publish_auto(ORB_ID(battery_status), &_pub_battery_sim, &_batt_sim, &_instance_sim, ORB_PRIO_DEFAULT);
		return true;
	}
	return false;
#endif

}



void GDPayload::run()
{
	if(!init())
		return;
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
				uint64_t now = hrt_absolute_time();
				vehicle_control_mode_poll();

				bool connected = _voltage_v > 5.0f;

				if(_battery_status.timestamp>0)
				{
					float dt =  (now -_battery_status.timestamp)*1.0e-6f;
					_used_mAh += _current_a*dt*1000.0f/3600.0f; /* convert to mAh */
				}
				_battery_status.timestamp = now;
				_battery_status.connected = connected;
				_battery_status.temperature = INT16_MAX;
				_battery_status.voltage_v = _voltage_v;
				_battery_status.cell_count = 8;
				_battery_status.current_a = _current_a;
				_battery_status.voltage_filtered_v = _battery_status.voltage_filtered_v*0.8f + _voltage_v*0.2f; /* override filtered value */
				_battery_status.current_filtered_a = _battery_status.current_filtered_a*0.8f + _current_a*0.2f;
				_battery_status.discharged_mah = _used_mAh;
				if(orb_publish_auto(ORB_ID(battery_status), &_pub_battery, &_battery_status, &_instance, ORB_PRIO_LOW))
				{
					if(_debug_flag)
					{
						PX4_INFO("pup failed %d", _instance);
					}
				}
		}
		usleep(100000);
	}
	updateBatteryDisconnect();
	orb_unsubscribe(_sub_vcontrol_mode);
	orb_unsubscribe(_parameter_update_sub);
	orb_unsubscribe(_sub_vehicle_cmd);
	orb_unsubscribe(_sub_vehicle_status);

	orb_unadvertise(_pub_battery);

#ifndef __PX4_NUTTX
	/* needed for the Battery class */
	orb_unsubscribe(_sub_actuator_ctrl_0);
	orb_unadvertise(_pub_battery_sim);
#endif
}

int gd_payload_main(int argc, char *argv[])
{
	return GDPayload::main(argc, argv);
}
