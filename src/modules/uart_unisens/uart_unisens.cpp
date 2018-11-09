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

#include <px4_getopt.h>
#include <px4_log.h>
#include <px4_posix.h>

#include <sys/types.h>
#include <sys/ioctl.h>

#include <termios.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <arch/board/board.h>

#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/vehicle_control_mode.h>

#include <drivers/drv_hrt.h>
#include "uart_unisens.h"

#define UNISENS_MAX_MESSAGE_LEN 140
#define UNISENS_CHECKSUM_LEN 2
#define AP_BATTMONITOR_SERIAL_UNILOG_TIMEOUT_MICROS 4000000    // sensor becomes unhealthy if no successful readings for 2 seconds


int UartUnisens::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
	driver for unisens
### Examples
CLI usage example:
$ uart_unisens start -d <uart device> -v
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("uart_unisens", "modules");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAM_FLAG('v', "debug flag", true);
	PRINT_MODULE_USAGE_PARAM_STRING('d',"/dev/ttyS0", "", "debug flag", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

int UartUnisens::print_status()
{
	// print additional runtime information about the state of the module
	PX4_INFO("rate: %i", _rate);
	PX4_INFO("voltage: %.2fV", (double)_voltage_v);
	PX4_INFO("current: %.3fA", (double)_current_a);
	PX4_INFO("bat used: %.1fmAh", (double)_used_mAh);
	PX4_INFO("temp: %.1fdegC", (double)_temp_c);
	PX4_INFO("baro: %.1fhPa", (double)_baro_hPa);
	PX4_INFO("volt RC: %.1fV", (double)_voltage_rc_v);
	return 0;
}

int UartUnisens::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}


int UartUnisens::task_spawn(int argc, char *argv[])
{
	_task_id = px4_task_spawn_cmd("uart_unisens",
				      SCHED_DEFAULT,
				      SCHED_PRIORITY_DEFAULT,
				      1024,
				      (px4_main_t)&run_trampoline,
				      (char *const *)argv);

	if (_task_id < 0) {
		_task_id = -1;
		return -errno;
	}

	return 0;
}

UartUnisens *UartUnisens::instantiate(int argc, char *argv[])
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

	UartUnisens *instance = new UartUnisens(device, debug_flag);

	if (instance == nullptr) {
		PX4_ERR("alloc failed");
	}

	return instance;
}

UartUnisens::UartUnisens(char const *const device, bool debug_flag):
ModuleParams(nullptr),
_battery()
{
	if(device)
	{
		memcpy(_device, device,sizeof(_device));
	} else{
		memcpy(_device, UART_UNISENS_UART,sizeof(UART_UNISENS_UART));
	}

	_debug_flag = debug_flag;
	memset(&_battery_status,0,sizeof(_battery_status));
}

bool UartUnisens::init()
{
	bool result = true;
	// open uart
	_uart_fd = open(_device, O_RDWR | O_NOCTTY | O_SYNC);
	int termios_state = -1;

	if (_uart_fd < 0) {
		PX4_ERR("failed to open uart device %s", _device);
		return false;
	}

	// set baud rate
	int speed = B115200;
	struct termios uart_config;
	tcgetattr(_uart_fd, &uart_config);
	//
	// Input flags - Turn off input processing
	//
	// convert break to null byte, no CR to NL translation,
	// no NL to CR translation, don't mark parity errors or breaks
	// no input parity check, don't strip high bit off,
	// no XON/XOFF software flow control
	//
	uart_config.c_iflag &= ~(IGNBRK | BRKINT | ICRNL |
				 INLCR | PARMRK | INPCK | ISTRIP | IXON);
	//
	// Output flags - Turn off output processing
	//
	// no CR to NL translation, no NL to CR-NL translation,
	// no NL to CR translation, no column 0 CR suppression,
	// no Ctrl-D suppression, no fill characters, no case mapping,
	// no local output processing
	//
	// config.c_oflag &= ~(OCRNL | ONLCR | ONLRET |
	//                     ONOCR | ONOEOT| OFILL | OLCUC | OPOST);
	uart_config.c_oflag = 0;

	//
	// No line processing
	//
	// echo off, echo newline off, canonical mode off,
	// extended input processing off, signal chars off
	//
	uart_config.c_lflag &= ~(ECHO | ECHONL | ICANON | IEXTEN | ISIG);

	/* no parity, one stop bit, disable flow control */
	uart_config.c_cflag &= ~(CSTOPB | PARENB | CRTSCTS);

//	uart_config.c_cc[VMIN] = 0;
//	uart_config.c_cc[VTIME] = 10;

	// set baud rate
	if (cfsetispeed(&uart_config, speed) < 0 || cfsetospeed(&uart_config, speed) < 0) {
		PX4_ERR("failed to set baudrate for %s: %d\n", _device, termios_state);
		close(_uart_fd);
		return false;
	}

	if ((termios_state = tcsetattr(_uart_fd, TCSANOW, &uart_config)) < 0) {
		PX4_ERR("tcsetattr failed for %s\n", _device);
		close(_uart_fd);
		return false;
	}
	//_pub_battery = orb_advertise(ORB_ID(battery_status), &_battery_status);
	_actuator_ctrl_0_sub = orb_subscribe(ORB_ID(actuator_controls_0));
	_vcontrol_mode_sub = orb_subscribe(ORB_ID(vehicle_control_mode));
	return result;
}

void UartUnisens::vehicle_control_mode_poll()
{
	struct vehicle_control_mode_s vcontrol_mode;
	bool vcontrol_mode_updated;

	orb_check(_vcontrol_mode_sub, &vcontrol_mode_updated);

	if (vcontrol_mode_updated) {

		orb_copy(ORB_ID(vehicle_control_mode), _vcontrol_mode_sub, &vcontrol_mode);
		_armed = vcontrol_mode.flag_armed;
	}
}

bool UartUnisens::readPoll(uint32_t tout)
{
    struct pollfd uartPoll;
    uartPoll.fd = _uart_fd;
    uartPoll.events = POLLIN;
    int pollrc = poll(&uartPoll, 1, tout);
    if (pollrc < 1) return false;
    return true;
}

int UartUnisens::readStatusInit()
{
    // read version with poll
	if(!readPoll(100)) return 0;
	uint8_t rxmsg[1];
	size_t msgsize;
	ssize_t nbytes=0;
	msgsize = sizeof(rxmsg);
    nbytes = ::read(_uart_fd, rxmsg, msgsize);
	if(nbytes > (ssize_t)msgsize)
	{
		PX4_ERR("read(%i) returned %i",
			   (int)msgsize, (int)nbytes);
	}
	if(nbytes>0)
	{
		if(rxmsg[0]=='2')
		{
			if(_debug_flag)
			{
				PX4_INFO("init ack");
			}
			return 1;
		} else {
			if(_debug_flag)
			{
				PX4_INFO("init nack 0x%x", rxmsg[0]);
			}
		}
	}
    return 0;
}

int UartUnisens::readStatusData()
{
	if(!readPoll()) return 0;
	uint8_t rxmsg[257];
	ssize_t unisens_length=0;
	ssize_t nbytes=0;
	size_t msgsize;
    // read version with poll
    do{
    	nbytes=0;
    	msgsize = sizeof(rxmsg)-unisens_length-1;
        nbytes = ::read(_uart_fd, &rxmsg[unisens_length], msgsize);
		if(nbytes > (ssize_t)msgsize)
		{
			PX4_ERR("read(%i) returned %i",
				   (int)msgsize, (int)nbytes);
			break;
		}
		if(nbytes>0){
			unisens_length+=nbytes;
//	    	usleep(1000);
			nbytes = readPoll(100);
		}
    } while(nbytes>0 && msgsize>0);
	if(unisens_length>0)
	{
		rxmsg[unisens_length]=0;
		if(rxmsg[0]=='$'){
			if(_debug_flag)
			{
				PX4_INFO("data %i", unisens_length);
			}
			char *tok_ptr;
			char *token;
			int token_index=0;
			token = strtok_r((char*)rxmsg,",*",&tok_ptr);
			while(token != NULL){
				token_index++;
/*				if(_debug_flag)
				{
					PX4_INFO("%i : %s",token_index, token);
					usleep(100000);
				}*/
				if(token_index==5)
				{
					_voltage_v = atof(token);

				} else if(token_index==6)
				{
					_current_a = atof(token);
				}else if(token_index==11)
				{
					_voltage_rc_v = atof(token);
				}else if(token_index==12)
				{
					_battery_status.discharged_mah = _used_mAh= atof(token);
				} else if(token_index==14)
				{
					_baro_hPa = atof(token);
				} else  if(token_index==15)
				{
					_battery_status.temperature = _temp_c = atof(token);
				} else  if(token_index==21)
				{
					uint8_t check = calcCheckSum(&rxmsg[1], (tok_ptr - (char*)rxmsg)-1);
					char * pEnd;
					uint8_t check_rx = strtol(token, &pEnd, 16);
					if(check==check_rx)
					{
						return 1;
					} else
					{
						if(_debug_flag)
						{
							PX4_INFO("checksum : 0x%x != 0x%x",check, check_rx);
						}
					}
				}
				token = strtok_r(NULL,",*",&tok_ptr);
			}
			return 1;
		}
	}
    return 0;
}
// uart_unisens start -v -d /dev/ttyS2

int8_t UartUnisens::calcCheckSum(uint8_t *data, ssize_t length)
{
	int8_t iXor = 0;
	//calculate checksum by xor'ing over from $ to *
	for (ssize_t i = 0; i < length; i++)
	{
		iXor = iXor ^ data[i];
	}
	return iXor;
}

void UartUnisens::writeInit()
{
	char cmd[] = "g\r\n";
	tcflush(_uart_fd, TCIFLUSH);
	::write(_uart_fd, cmd, 3);
/*	if(_debug_flag)
	{
		PX4_INFO("init com");
	}*/
}

void UartUnisens::writeRequest()
{
	char cmd[] = "v\r\n";
	::write(_uart_fd, cmd,3);
/*
	if(_debug_flag)
	{
		PX4_INFO("req");
	}*/
}

void UartUnisens::run()
{
	if(!init())
		return;

	hrt_abstime now = hrt_absolute_time();
	hrt_abstime second_timer = now + 1e6;
	//hrt_abstime timer_request = now + 1e6/50; // 50hz
	int n = 0;
	//readStatusInit();
	while (!should_exit()) {
		writeInit();
		if(readStatusInit()>0)
		{
			writeRequest();
			if(readStatusData())
			{
				n++;
				vehicle_control_mode_poll();
				actuator_controls_s ctrl;
				orb_copy(ORB_ID(actuator_controls_0), _actuator_ctrl_0_sub, &ctrl);

				bool connected = _voltage_v > 5.0f;
				_battery.updateBatteryStatus(hrt_absolute_time(), _voltage_v, _current_a,
								connected, true, 0,
								ctrl.control[actuator_controls_s::INDEX_THROTTLE],
								_armed, &_battery_status);
				int instance;
				_battery_status.temperature = _temp_c;
				orb_publish_auto(ORB_ID(battery_status), &_pub_battery, &_battery_status, &instance, ORB_PRIO_DEFAULT);
			}
		}
		now = hrt_absolute_time();
		if(second_timer <= now)
		{
			_rate = n;
			n=0;
			second_timer += 1e6;
			if(_debug_flag)
			{
				print_status();
			}
		}
		usleep(100000);
	}
	orb_unadvertise(_pub_battery);
	orb_unsubscribe(_actuator_ctrl_0_sub);
	close(_uart_fd);
	PX4_INFO("Exit");
}

int uart_unisens_main(int argc, char *argv[])
{
	return UartUnisens::main(argc, argv);
}
