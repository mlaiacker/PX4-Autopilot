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
 * @file gd_payload.cpp
 *
 * germandrones specific functions:
 * eading of payload voltage and current,
 * payload on off control
 * battery switchover for hybrid battery PDB
 * trip2 status debug
 * battery simulator for SITL
 *
 * @author Maximilian Laiacker <post@mlaiacker.de>
 */
#include <px4_platform_common/module.h>
#include <px4_platform_common/getopt.h>
#include <px4_log.h>
#include <math.h>

#include <sys/types.h>
#include <sys/ioctl.h>

#include <termios.h>
#include <poll.h>

#ifdef __PX4_NUTTX
#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <arch/board/board.h>
#else
#include <signal.h>
#endif
// for mavling warning messages
#include <systemlib/mavlink_log.h>

#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vtol_vehicle_status.h>
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

#define SW_LIPO	(0xf0|1)
#define SW_LION	(0xf0|2)
#define SW_BOTH	(0xf0|3)

#define WARN_INT_S		(20) // min time ins seconds between two warning messages with the same content
#define PDB_TEMP_NAME	("tempPDB")
#define PDB_TEMP_WARN	(75.0f) // if we have a PDB temperature sensor we will send a warning message

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
	PRINT_MODULE_USAGE_COMMAND("switch");
	PRINT_MODULE_USAGE_COMMAND("LIPO");
	PRINT_MODULE_USAGE_COMMAND("LION");
	PRINT_MODULE_USAGE_COMMAND("BOTH");
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
	adc_report_s adc_report;

	_adc_report_sub.copy(&adc_report);
	for (unsigned i = 0; i < sizeof(adc_report.raw_data)/sizeof(adc_report.raw_data[0]); i++)
	{
		PX4_INFO("ADC:%02i, %02i=%fV",i, adc_report.channel_id[i], (double)(adc_report.raw_data[i]*adc_report.v_ref/adc_report.resolution));
	}
#else
	PX4_INFO("PDB temp: %.1f", (double)_sim_temp_pdb.value);
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

	if (!strcmp(verb, "switch")) {
		PX4_INFO("bat active=%i",switchStatus());
		return 0;
	}

	if (!strcmp(verb, "LIPO")) {
		switchSet(SW_LIPO);
		return 0;
	}

	if (!strcmp(verb, "LION")) {
		switchSet(SW_LION);
		return 0;
	}

	if (!strcmp(verb, "BOTH")) {
		switchSet(SW_BOTH);
		return 0;
	}


	return print_usage("unknown command");
}

void GDPayload::writePayloadPower(bool on)
{
#ifdef __PX4_NUTTX
	int fd = open(PX4IO_DEV, O_RDWR);

	if (fd < 0) {
		PX4_ERR("open io failed");
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

void GDPayload::batGetAll(battery_status_s* lion, battery_status_s* lipo)
{
	if(lion!= NULL && lipo != NULL)
	{
		int num_inst = orb_group_count(ORB_ID(battery_status));
		for(int i=0;i<num_inst;i++)
		{
				int sub_battery = orb_subscribe_multi(ORB_ID(battery_status),i);
				if(sub_battery!=-1)
				{
					struct battery_status_s bat;
					orb_copy(ORB_ID(battery_status), sub_battery, &bat);
					if(bat.serial_number==64){
						*lion = bat;
					}
					if(bat.serial_number==69){
						*lipo = bat;
					}
					orb_unsubscribe(sub_battery);
				} else
				{
					PX4_ERR("failed to subscribe to batt topic");
				}
		}
	}
}

int GDPayload::switchStatus()
{
	int port = -1;
#if defined(__PX4_NUTTX) && defined(IOX_GET_MASK)
	int fd= open("/dev/pca9536", O_RDWR);
	if(fd>0){
		port = px4_ioctl(fd, IOX_GET_MASK, 0);
		close(fd);
	}
#endif
	return port;
}

int GDPayload::switchSet(int val)
{
	int port = -1;
#if defined(__PX4_NUTTX) && defined(IOX_SET_VALUE)
	int fd= open("/dev/pca9536", O_RDWR);
	if(fd>0){
		port = px4_ioctl(fd, IOX_SET_VALUE, val);
		close(fd);
	}
#endif
	return port;
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
#ifndef __PX4_NUTTX
_battery_sim(1, this, 100000),
#endif
_pub_battery(nullptr)
{
	_debug_flag = debug_flag;
	memset(&_battery_status, 0, sizeof(_battery_status));
	memset(&_vstatus, 0, sizeof(_vstatus));
	_battery_status.remaining = 1.0f; // must be 1 because mavlink will select the battery with the lowest remainig. since we are not realy a battery we have to set this to 100%
	_used_mAh = 0.0f;
	_voltage_v = 0.0f;
	_current_a = 0.0f;
	param_find("MNT_TRIP_MAVLINK");
#ifndef __PX4_NUTTX
	_battery_sim.rechargeBattery();
	_sim_temp_pdb.value = 20;
	snprintf(_sim_temp_pdb.key, sizeof(_sim_temp_pdb.key), "%s", PDB_TEMP_NAME);
#endif
}

bool GDPayload::init()
{
	bool result = true;
	_parameters_handles.battery_v_div = param_find("BAT_V_DIV");
	if(_parameters_handles.battery_v_div==PARAM_INVALID) {
		_parameters_handles.battery_v_div = param_find("BAT1_V_DIV");
	}
	_parameters_handles.battery_a_per_v = param_find("BAT_A_PER_V");
	if(_parameters_handles.battery_a_per_v == PARAM_INVALID) {
		_parameters_handles.battery_a_per_v = param_find("BAT1_A_PER_V");
	}

	_parameters_handles.payload_current_warn_a = param_find("MNT_CURR_WARN");

	// initialize parameters
	_parameter_update_sub = orb_subscribe(ORB_ID(parameter_update));
	parameters_update(_parameter_update_sub, true);

	/* needed to read arming status */
	_sub_vcontrol_mode = orb_subscribe(ORB_ID(vehicle_control_mode));
	_sub_vehicle_cmd = orb_subscribe(ORB_ID(vehicle_command));
	_sub_vehicle_status = orb_subscribe(ORB_ID(vehicle_status));
	_sub_vtol_status = orb_subscribe(ORB_ID(vtol_vehicle_status));
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

		if (param_get(_parameters_handles.payload_current_warn_a, &(_parameters.payload_current_warn_a)) != OK) {
			PX4_WARN("%s", paramerr);
			_parameters.payload_current_warn_a = 3.0f;
		} else if (_parameters.payload_current_warn_a < 0.0f) {
			/* apply scaling according to defaults if set to default */
			_parameters.payload_current_warn_a = 3.0f; /* 3A */
			param_set_no_notification(_parameters_handles.payload_current_warn_a, &_parameters.payload_current_warn_a);
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

void GDPayload::vehicleCommand(const vehicle_command_s *vcmd)
{
	if(_debug_flag)
	{
		PX4_INFO("cmd=%d (%f %f %f %f %f %f %f)", vcmd->command,
				(double)vcmd->param1, (double)vcmd->param2, (double)vcmd->param3, (double)vcmd->param4, (double)vcmd->param5, (double)vcmd->param6, (double)vcmd->param7);
	}
	switch(vcmd->command)
	{
	case vehicle_command_s::VEHICLE_CMD_CUSTOM_0:
		writePayloadPower(vcmd->param1>=0.5f);
		vehicleCommandAck(vcmd);
		break;
	default:
		//print_message(&vcmd);
		break;
	}
}

/* read arming state and other subs */
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
		int switch_status = switchStatus();
		if(switch_status>=0)
		{
			struct vtol_vehicle_status_s vtol_status;
			orb_copy(ORB_ID(vtol_vehicle_status), _sub_vtol_status, &vtol_status);
			struct battery_status_s lion={0}, lipo={0};
			batGetAll(&lion, &lipo);
			if(!_vstatus.in_transition_mode && !_vstatus.in_transition_to_fw)
			{
				if(!vtol_status.vtol_in_rw_mode )/* we are fixed wing */
				{
					if((lion.voltage_filtered_v > lipo.voltage_filtered_v || fabsf(lion.current_filtered_a) > 3.0f)
							&& lion.voltage_filtered_v > lion.cell_count*3.0f
							)
					{
						switchSet(SW_LION);
						PX4_INFO("LION Activated");
					}
				} else /* we are copter */
				{
					if((lipo.voltage_filtered_v > lion.voltage_filtered_v || fabsf(lipo.current_filtered_a) > 3.0f)
							&& lipo.voltage_filtered_v > lipo.cell_count*3.0f
							){
						switchSet(SW_LIPO);
						PX4_INFO("LIPO Activated");
					}
				}
			}
			if (updated)
			{
				struct vehicle_status_s vstatus;
				orb_copy(ORB_ID(vehicle_status), _sub_vehicle_status, &vstatus);
				if(_debug_flag) {
					PX4_INFO("LION(%.1fV,%fA) LIPO(%.1fV,%fA) 0x%x", (double)lion.voltage_filtered_v, (double)lion.current_filtered_a, (double)lipo.voltage_filtered_v, (double)lipo.current_filtered_a, switch_status&0x03);
				}
				if(_vstatus.in_transition_mode != vstatus.in_transition_mode && vstatus.in_transition_mode)
				{
					switchSet(SW_BOTH);
				}
/*				if(_vstatus.in_transition_mode != vstatus.in_transition_mode && !vstatus.in_transition_mode)
				{
					PX4_INFO("in transition end");
				}*/
				if(_vstatus.in_transition_to_fw != vstatus.in_transition_to_fw && vstatus.in_transition_to_fw)
				{
					switchSet(SW_BOTH);
				}
/*				if(_vstatus.in_transition_to_fw != vstatus.in_transition_to_fw && !vstatus.in_transition_to_fw)
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
		} else {
			orb_copy(ORB_ID(vehicle_status), _sub_vehicle_status, &_vstatus);
		}
	}
	struct debug_key_value_s debug_key;
	if(_sub_debug_key.copy(&debug_key)) {
		if(strlen(debug_key.key)>0){
			// check if match
			if(strcasecmp(debug_key.key, PDB_TEMP_NAME)==0) {
				// we got the value we waiting for
				if(debug_key.value>PDB_TEMP_WARN && (hrt_elapsed_time(&_temp_last_warn_time))>=(WARN_INT_S* 1000000ULL)){
					mavlink_log_critical(&_pub_mavlink_log, "PDB Temp. too high %.1fC", (double)debug_key.value);
					_temp_last_warn_time = debug_key.timestamp;
				}
				// report only once if dropped
				if(debug_key.value<(PDB_TEMP_WARN*0.6f) && _temp_last_report_c>(PDB_TEMP_WARN*0.6f) && _temp_last_warn_time!=0){
					mavlink_log_critical(&_pub_mavlink_log, "PDB Temp. normalized %.1fC", (double)debug_key.value);
					_temp_last_warn_time = 0; // only print if we had the over temperature
				}
				_temp_last_report_c = debug_key.value;
			}
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

	if(_adc_report_sub.update(&_adc_report)) {

			/* Read adc channels we got */
			for (unsigned i = 0; i < sizeof(_adc_report.raw_data)/sizeof(_adc_report.raw_data[0]); i++)
			{
				/* look for specific channels and process the raw voltage to measurement data */
				if (_adc_report.channel_id[i] == 13) /* PC3=adc0 channel 13 */
				{
					/* Voltage in volts */
					_voltage_v = ((float)_adc_report.raw_data[i]*_adc_report.v_ref/_adc_report.resolution) * _parameters.battery_v_div;

				} else if (_adc_report.channel_id[i] == 14) /* PC4=adc0 channel 14 */
				{
					_current_a = ((float)_adc_report.raw_data[i]*_adc_report.v_ref/_adc_report.resolution) * _parameters.battery_a_per_v;
				}
			}
			return true;
	}
	return false;
#else
	_voltage_v = _battery_sim.cell_count()*_battery_sim.full_cell_voltage() + rand()*0.1f/RAND_MAX;
	_current_a = (0.83f + rand()*0.2f/RAND_MAX)*g_gd_payload_on;
	vehicle_control_mode_poll();
	if(_vstatus.hil_state == _vstatus.HIL_STATE_ON)
	{
		vehicle_global_position_s gpos;
		_sub_global_pos.copy(&gpos);
		float sim_current_a=_current_a, sim_voltage_v= _voltage_v;
		/* needed for the Battery class */
		if(_sub_actuator_ctrl_0<0){
			_sub_actuator_ctrl_0 = orb_subscribe(ORB_ID(actuator_controls_0));
		}
		actuator_controls_s ctrl;
		orb_copy(ORB_ID(actuator_controls_0), _sub_actuator_ctrl_0, &ctrl);

		struct vtol_vehicle_status_s vtol_status;
		orb_copy(ORB_ID(vtol_vehicle_status), _sub_vtol_status, &vtol_status);

		if (_vstatus.arming_state == _vstatus.ARMING_STATE_ARMED){
			if(vtol_status.vtol_in_rw_mode){
				sim_current_a += 1.0f + 200.0f*ctrl.control[actuator_controls_s::INDEX_THROTTLE]*ctrl.control[actuator_controls_s::INDEX_THROTTLE]
								+ rand()*10.0f/RAND_MAX;
			} else{
				sim_current_a += 1.0f + 40.0f*ctrl.control[actuator_controls_s::INDEX_THROTTLE]*ctrl.control[actuator_controls_s::INDEX_THROTTLE]
							     + rand()*1.0f/RAND_MAX;
			}
			sim_voltage_v -= _battery_sim.cell_count()*1.3f*(1.0f-_battery_sim.getRemaining());
			sim_voltage_v -= sim_current_a*0.007f;
			_sim_was_armed = true;
		} else {
			if(!PX4_ISFINITE(gpos.alt))
			{
				PX4_ERR("alt invalid, exiting...");
				sleep(1);
				raise(SIGTERM);
				sleep(1);
				system_exit(0);
			}
			if(_sim_was_armed) {
				_battery_sim.rechargeBattery();
				_sim_was_armed = false;
			}
		}
		//_battery_sim._battery_status.temperature = _sim_temp_pdb.value;
		_battery_sim.updateBatteryStatus(hrt_absolute_time(),
				sim_voltage_v, sim_current_a,
				true, battery_status_s::BATTERY_SOURCE_EXTERNAL, 1, ctrl.control[actuator_controls_s::INDEX_THROTTLE]);
		float dt = 0.1f;
		// simulate pdb temerature
		_sim_temp_pdb.value = _sim_temp_pdb.value + sim_current_a*dt*0.005f*3.65f + (20.0f- _sim_temp_pdb.value )*0.0019f;
		_sim_temp_pdb.timestamp = hrt_absolute_time();
		_sim_pub_pdb_temp.publish(_sim_temp_pdb);

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
		vehicle_control_mode_poll();
		if(readPayloadAdc())
		{
				n++;
				uint64_t now = hrt_absolute_time();

				bool connected = _voltage_v > 5.0f;

				if(_battery_status.timestamp>0)
				{
					float dt =  (now -_battery_status.timestamp)*1.0e-6f;
					_used_mAh += _current_a*dt*1000.0f/3600.0f; /* convert to mAh */
				}
				_battery_status.timestamp = now;
				_battery_status.connected = connected;
				_battery_status.temperature = _temp_last_report_c;
				_battery_status.voltage_v = _voltage_v;
				_battery_status.cell_count = 8;
				_battery_status.current_a = _current_a;
				_battery_status.voltage_filtered_v = _battery_status.voltage_filtered_v*0.9f + _voltage_v*0.1f; /* override filtered value */
				_battery_status.current_filtered_a = _battery_status.current_filtered_a*0.9f + _current_a*0.1f;
				_battery_status.discharged_mah = _used_mAh;
				_battery_status.priority = (uint8_t)switchStatus()&0x03;
				_battery_status.serial_number = 13015; // songbird ID
				_battery_status.id = 2; // second battery
				for(int i = 0; i< _battery_status.cell_count; i++)
				{
					_battery_status.voltage_cell_v[i] = _battery_status.voltage_filtered_v/_battery_status.cell_count;
				}
				if(orb_publish_auto(ORB_ID(battery_status), &_pub_battery, &_battery_status, &_instance, ORB_PRIO_LOW))
				{
					if(_debug_flag)
					{
						PX4_INFO("pup failed %d", _instance);
					}
				}
				if(_battery_status.current_filtered_a > _parameters.payload_current_warn_a && _parameters.payload_current_warn_a>0.0f
						&& (hrt_elapsed_time(&_payload_last_warn_time))>=(WARN_INT_S* 1000000ULL) ){
					mavlink_log_critical(&_pub_mavlink_log, "Payload current: %.1fA", (double)_battery_status.current_filtered_a);
					_payload_last_warn_time = hrt_absolute_time();
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
#endif
}

int gd_payload_main(int argc, char *argv[])
{
	return GDPayload::main(argc, argv);
}
