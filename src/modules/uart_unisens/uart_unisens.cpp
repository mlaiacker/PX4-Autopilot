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


#include <drivers/drv_hrt.h>
#include "uart_unisens.h"

#define UNISENS_MAX_MESSAGE_LEN 140
#define UNISENS_CHECKSUM_LEN 2
#define AP_BATTMONITOR_SERIAL_UNILOG_TIMEOUT_MICROS 4000000    // sensor becomes unhealthy if no successful readings for 2 seconds

enum UNISENS_STATE {
	UNISENS_INITIALIZE_CONNECTION, //!< While we wait for the unisens to be ready
	UNISENS_WAITING_FOR_INITIALIZATION, //!< If we have sent a initialzition req to the UniSens and are waiting for a echo of '2'
	UNISENS_WAITING_FOR_ANSWER, //!< If we have sent a request to the UniSens and are waiting for a complete answer
	UNISENS_WAITING_FOR_CHECKSUM
};


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
	// TODO: print additional runtime information about the state of the module
	PX4_INFO("unisens %i", _rate);
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

UartUnisens::UartUnisens(char const *const device, bool debug_flag)
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
	_uart_fd = open(_device, O_RDWR | O_NOCTTY | O_NONBLOCK);
	int termios_state = -1;

	if (_uart_fd < 0) {
		PX4_ERR("failed to open uart device %s", _device);
		return false;
	}

	// set baud rate
	int speed = B115200;
	struct termios uart_config;
	tcgetattr(_uart_fd, &uart_config);

	// clear ONLCR flag (which appends a CR for every LF)
	uart_config.c_oflag &= ~ONLCR;
	// setup output flow control
	uart_config.c_cflag &= ~CRTSCTS;

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
	_pub_battery = orb_advertise(ORB_ID(battery_status), &_battery_status);
	return result;
}

void UartUnisens::readStatus()
{
	uint8_t rxmsg[32];
	size_t msgsize;
	ssize_t nbytes;
	msgsize = sizeof(rxmsg);
    do
    {
		nbytes = read(_uart_fd, &rxmsg, msgsize);
		if(nbytes > (ssize_t)msgsize)
		{
			PX4_ERR("read(%i) returned %i",
				   (int)msgsize, (int)nbytes);
			break;
		} else
		if(nbytes>0)
		{
			if(_debug_flag)
			{
				PX4_INFO("RX %i",nbytes);
			}

		}

    } while(nbytes>0);
}



void UartUnisens::run()
{
	if(!init())
		return;

	hrt_abstime now = hrt_absolute_time();
	hrt_abstime second_timer = now + 1e6;
	//hrt_abstime timer_request = now + 1e6/50; // 50hz

	while (!should_exit()) {

		readStatus();
		now = hrt_absolute_time();
		if(second_timer <= now)
		{
			second_timer += 1e6;
			if(_debug_flag)
			{
				print_status();
			}
		}
	}
	orb_unadvertise(_pub_battery);
	close(_uart_fd);
	PX4_INFO("Exit");
}

int uart_unisens_main(int argc, char *argv[])
{
	return UartUnisens::main(argc, argv);
}
