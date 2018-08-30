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

#include "can_vesc.h"

#include <px4_getopt.h>
#include <px4_log.h>
#include <px4_posix.h>

#include <sys/types.h>
#include <sys/ioctl.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <arch/board/board.h>

#include <nuttx/can/can.h>
#include <NuttX/apps/include/canutils/canlib.h>


#include <uORB/topics/parameter_update.h>
#include <uORB/topics/actuator_controls.h>

#include <drivers/drv_hrt.h>

#define	VESC_CAN_DEVPATH	"/dev/can0"

// CAN commands
typedef enum {
	CAN_PACKET_SET_DUTY = 0,
	CAN_PACKET_SET_CURRENT,
	CAN_PACKET_SET_CURRENT_BRAKE,
	CAN_PACKET_SET_RPM,
	CAN_PACKET_SET_POS,
	CAN_PACKET_FILL_RX_BUFFER,
	CAN_PACKET_FILL_RX_BUFFER_LONG,
	CAN_PACKET_PROCESS_RX_BUFFER,
	CAN_PACKET_PROCESS_SHORT_BUFFER,
	CAN_PACKET_STATUS,
	CAN_PACKET_SET_CURRENT_REL,
	CAN_PACKET_SET_CURRENT_BRAKE_REL,
	CAN_PACKET_SET_CURRENT_HANDBRAKE,
	CAN_PACKET_SET_CURRENT_HANDBRAKE_REL,
	CAN_PACKET_STATUS2,
} CAN_PACKET_ID;

#pragma pack(push,1)
typedef struct {
	int32_t duty_1e5; // 100000 = 100%
}vescCanSetDutyT;

typedef struct{
	int32_t rpm;
	int16_t current; // 10=1A
	int16_t duty; //1000=100%
}vescCanStatusT;
typedef struct{
	uint16_t voltage; // 100=1V
	int16_t temperature; //10=1C
	uint8_t	fault; // 0=none
	uint8_t	state;
}vescCanStatus2T;
#pragma pack(pop)

int CanVESC::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
	GD driver for Songbird can bus communication
### Examples
CLI usage example:
$ can_vesc start -d
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("can_vesc", "modules");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAM_FLAG('d', "debug flag", true);
	PRINT_MODULE_USAGE_PARAM_INT('1', 6, 1, 20, "can ts1", true);
	PRINT_MODULE_USAGE_PARAM_INT('2', 6, 1, 20, "can ts2", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

int CanVESC::print_status()
{
	// TODO: print additional runtime information about the state of the module
	for(int i =0 ;i<VESC_CAN_NUM; i++)
	{
		PX4_INFO("VESC%i: %.1fA %.1fV %.1f%% %iRPM %.1fC", i,
				(double)_esc_feedback.esc[i].esc_current,
				(double)_esc_feedback.esc[i].esc_voltage,
				(double)_esc_feedback.esc[i].esc_setpoint,
				_esc_feedback.esc[i].esc_rpm,
				(double)_esc_feedback.esc[i].esc_temperature
				);
	}
	PX4_INFO("freq:%i", _esc_update_freq);
	return 0;
}

int CanVESC::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}


int CanVESC::task_spawn(int argc, char *argv[])
{
	_task_id = px4_task_spawn_cmd("can_vesc",
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

CanVESC *CanVESC::instantiate(int argc, char *argv[])
{
	bool debug_flag = false;
	bool error_flag = false;

	int myoptind = 1;
	int ch;
	const char *myoptarg = nullptr;

	// parse CLI arguments
	while ((ch = px4_getopt(argc, argv, "d", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
/*		case '1':
			param_ts1 = (int)strtol(myoptarg, nullptr, 10);
			break;
*/

		case 'd':
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

	CanVESC *instance = new CanVESC(debug_flag);

	if (instance == nullptr) {
		PX4_ERR("alloc failed");
	}

	return instance;
}

CanVESC::CanVESC(bool debug_flag)
	: ModuleParams(nullptr)
{

	_debug_flag = debug_flag;
	memset(&_esc_feedback,0,sizeof(_esc_feedback));
}

bool CanVESC::init()
{
	PX4_INFO("init");
	bool result = true;
	struct canioc_bittiming_s bt;
	struct canioc_bittiming_s timings;
	int ret;

	_can_fd = open(VESC_CAN_DEVPATH, O_RDWR | O_NONBLOCK);
	if (_can_fd < 0)
	{
		  PX4_ERR("ERROR: open %s failed: %d",
				  VESC_CAN_DEVPATH, errno);
		  result = false;
		  return result;
	}
	  ret = ioctl(_can_fd, CANIOC_GET_BITTIMING, (unsigned long)&timings);
	  if (ret != OK)
	    {
	      canerr("CANIOC_GET_BITTIMING failed, errno=%d\n", errno);
	      return ret;
	    }

	  timings.bt_baud = 500000;
	  timings.bt_tseg1 = 7;
	  timings.bt_tseg2 = 2;
	  timings.bt_sjw = 1;

	  ret = ioctl(_can_fd, CANIOC_SET_BITTIMING, (unsigned long)&timings);
	  if (ret != OK)
	    {
	      canerr("CANIOC_SET_BITTIMING failed, errno=%d\n", errno);
	    }

	  /* Show bit timing information if provided by the driver.  Not all CAN
	   * drivers will support this IOCTL.
	   */
	ret = ioctl(_can_fd, CANIOC_GET_BITTIMING, (unsigned long)((uintptr_t)&bt));
	if (ret < 0)
	{
		PX4_WARN("Bit timing not available: %d", errno);
	}
	else
	{
		PX4_INFO("Bit timing:");
		PX4_INFO("   Baud: %lu", (unsigned long)bt.bt_baud);
		PX4_INFO("  TSEG1: %u", bt.bt_tseg1);
		PX4_INFO("  TSEG2: %u", bt.bt_tseg2);
		PX4_INFO("    SJW: %u", bt.bt_sjw);
	}

	_armed_sub = orb_subscribe(ORB_ID(actuator_armed));
	if(_armed_sub<0)
	{
		  PX4_ERR("Sub actuator_armed");
		  result = false;
	}

	_actuator_outputs_sub = orb_subscribe(ORB_ID(actuator_outputs));
	if(_actuator_outputs_sub<0)
	{
		  PX4_ERR("Sub _actuator_outputs");
		  result = false;
	}
	orb_set_interval(_actuator_outputs_sub, 1);
	_esc_feedback_pub = orb_advertise(ORB_ID(esc_status), &_esc_feedback);

	return result;
}

void CanVESC::readStatus()
{
	struct can_msg_s rxmsg;

	size_t msgsize;
	ssize_t nbytes;
    msgsize = sizeof(struct can_msg_s);
    do
    {
		nbytes = read(_can_fd, &rxmsg, msgsize);
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
				PX4_INFO("ID: 0x%04x DLC: %u", rxmsg.cm_hdr.ch_id, rxmsg.cm_hdr.ch_dlc);
			}
			int vesc_cmd = rxmsg.cm_hdr.ch_id >> 8;
			int vesc_id = rxmsg.cm_hdr.ch_id & 0xff;
			if(vesc_cmd == CAN_PACKET_STATUS && rxmsg.cm_hdr.ch_dlc==8)
			{
				if(vesc_id>=VESC_CAN_ID_START && vesc_id<VESC_CAN_ID_START+VESC_CAN_NUM)
				{
					vescCanStatusT* canData = (vescCanStatusT*)rxmsg.cm_data;
					int index = vesc_id-VESC_CAN_ID_START;
/*					_status[index].timestamp = hrt_absolute_time();
					_status[index].counter++;
					_status[index].rpm =
					_status[index].current_A = ((float)swap_int16(canData->current))*0.1f;
					_status[index].duty_percent = ((float)swap_int16(canData->duty))*0.1f;
					*/
					_esc_feedback.esc[index].timestamp = hrt_absolute_time();
					_esc_feedback.esc[index].esc_address = vesc_id;
					_esc_feedback.esc[index].esc_vendor = esc_status_s::ESC_VENDOR_GENERIC;
					_esc_feedback.esc[index].esc_rpm = swap_int32(canData->rpm);
					_esc_feedback.esc[index].esc_current = ((float)swap_int16(canData->current))*0.1f;
					_esc_feedback.esc[index].esc_setpoint = ((float)swap_int16(canData->duty))*0.1f;
					_esc_feedback.esc_connectiontype = esc_status_s::ESC_CONNECTION_TYPE_CAN;
					_esc_feedback.counter++;
					_esc_feedback.esc_count = VESC_CAN_NUM;
				}
			}else if(vesc_cmd == CAN_PACKET_STATUS2 && rxmsg.cm_hdr.ch_dlc==sizeof(vescCanStatus2T))
			{
				if(vesc_id>=VESC_CAN_ID_START && vesc_id<VESC_CAN_ID_START+VESC_CAN_NUM)
				{
					vescCanStatus2T* canData = (vescCanStatus2T*)rxmsg.cm_data;
					int index = vesc_id-VESC_CAN_ID_START;
					_esc_feedback.esc[index].timestamp = hrt_absolute_time();
					_esc_feedback.esc[index].esc_state = canData->state;
					_esc_feedback.esc[index].esc_errorcount = canData->fault;
					_esc_feedback.esc[index].esc_voltage = swap_uint16(canData->voltage)*0.01;
					_esc_feedback.esc[index].esc_temperature = swap_int16(canData->temperature)*0.1;
					_esc_feedback.esc_connectiontype = esc_status_s::ESC_CONNECTION_TYPE_CAN;
					_esc_feedback.counter++;
					_esc_feedback.esc_count = VESC_CAN_NUM;
				}
			} else if(_debug_flag)
			{
				PX4_INFO("cmd=0x%x id=%i",vesc_cmd, vesc_id);
			}

		}

    } while(nbytes>0);
}

/*
 * duty 0..1
 */
void CanVESC::writeDuty(uint8_t vescAddr, float duty)
{
	struct can_msg_s txmsg;
	size_t msgsize;
	ssize_t nbytes;
    txmsg.cm_hdr.ch_id     = (vescAddr & 0xff) | (CAN_PACKET_SET_DUTY>>8);
    txmsg.cm_hdr.ch_rtr    = false;
    txmsg.cm_hdr.ch_dlc    = sizeof(vescCanSetDutyT);
    txmsg.cm_hdr.ch_unused = 0;
    txmsg.cm_hdr.ch_extid  = true;

    vescCanSetDutyT* vescDuty = (vescCanSetDutyT*)txmsg.cm_data;

    if(duty<0) duty=0;
    if(duty>1) duty=1;

    vescDuty->duty_1e5 = swap_int32((int32_t)(duty * 1e5f));

    /* Send the TX message */
    msgsize = CAN_MSGLEN(txmsg.cm_hdr.ch_dlc);
    nbytes = write(_can_fd, &txmsg, msgsize);
    if (nbytes != (ssize_t)msgsize)
    {
/*        PX4_ERR("write(%ld) returned %ld",
               (long)msgsize, (long)nbytes);*/
    }

}


void CanVESC::run()
{
	if(!init())
		return;

	/* poll descriptor */
	pollfd fds[1];
	fds[0].fd = _actuator_outputs_sub;
	fds[0].events = POLLIN;

	hrt_abstime now = hrt_absolute_time();
	hrt_abstime second_timer = now + 1e6;
	hrt_abstime timer_esc_feedback = now + 1e6/50; // 50hz
	uint16_t esc_counter=0;

	while (!should_exit()) {

		readStatus();
		int ret = ::poll(fds, 1, 10);
		/* this would be bad... */
		if (ret < 0) {
			warnx("poll error %d", errno);
			continue;
		}
		/* if we have new control data from the ORB, handle it */
		if (fds[0].revents & POLLIN) {
			/* we're not nice to the lower-priority control groups and only check them
			   when the primary group updated (which is now). */
			actuator_outputs_s	actuators;	///< actuator outputs
			orb_copy(ORB_ID(actuator_outputs), _actuator_outputs_sub, &actuators);
			if(_is_armed)
			{
				_esc_update_count++;
				for(int i=0; i<VESC_CAN_NUM;i++)
				{
					writeDuty(i+VESC_CAN_ID_START,(actuators.output[i]-1000.0f)/1000.0f);
				}
			}
		}
		bool updated;
		orb_check(_armed_sub, &updated);
		if (updated) {
			orb_copy(ORB_ID(actuator_armed), _armed_sub, &_armed);

			if(_is_armed != _armed.armed && _debug_flag)
			{
				PX4_INFO("VESC arming %i", _armed.armed);
			}
			_is_armed = _armed.armed;
			if(!_is_armed)
			{
				for(int i=0; i<VESC_CAN_NUM;i++)
				{
					writeDuty(i+VESC_CAN_ID_START, 0);
				}
				_esc_update_count++;
			}
		}
		now = hrt_absolute_time();
		if(second_timer <= now)
		{
			second_timer += 1e6;
			_esc_update_freq = _esc_update_count;
			_esc_update_count = 0;
			if(_debug_flag)
			{
				print_status();
			}
		}
		if(timer_esc_feedback <= now && esc_counter != _esc_feedback.counter)
		{
			esc_counter = _esc_feedback.counter;
			_esc_feedback.timestamp = now;
			orb_publish(ORB_ID(esc_status), _esc_feedback_pub, &_esc_feedback);
			timer_esc_feedback += 1e6/50; // 50hz
		}
	}
	orb_unsubscribe(_armed_sub);
	orb_unsubscribe(_actuator_outputs_sub);

	orb_unadvertise(_esc_feedback_pub);
	close(_can_fd);
	PX4_INFO("Exit");
}

void CanVESC::parameters_update(int parameter_update_sub, bool force)
{
	bool updated;
	struct parameter_update_s param_upd;

	orb_check(parameter_update_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(parameter_update), parameter_update_sub, &param_upd);
	}

	if (force || updated) {
		updateParams();
	}
}

int can_vesc_main(int argc, char *argv[])
{
	return CanVESC::main(argc, argv);
}
