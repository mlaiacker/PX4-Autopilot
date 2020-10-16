/**
 * @file uart_phaseone.cpp
 *
 * generate NAVINFO1(0x36), NAVINFO2(0x37) and GPSTIME(0x3A) in phaseone binay format
 *
 * @author Maximilian Laiacker <post@mlaiacker.de>
 */

#include <px4_getopt.h>
#include <px4_log.h>
#include <px4_posix.h>
#include <px4_module.h>
#include <px4_module_params.h>

#include <sys/types.h>
#include <sys/ioctl.h>

#include <termios.h>

#include <arch/board/board.h>

#include <uORB/topics/camera_trigger.h>
#include <uORB/topics/camera_capture.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_command_ack.h>

#include <matrix/math.hpp>

#include <drivers/drv_hrt.h>
#include "uart_phaseone.h"

/* default uart */
#define UART_PHASEONE_UART "/dev/ttyS2"

extern "C" __EXPORT int uart_phaseone_main(int argc, char *argv[]);

class UartPhaseOne : public ModuleBase<UartPhaseOne>, public ModuleParams
{
public:
	UartPhaseOne(char const *const device, bool debug_flag);

	virtual ~UartPhaseOne() = default;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static UartPhaseOne *instantiate(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	/** @see ModuleBase::run() */
	void run() override;

	/** @see ModuleBase::print_status() */
	int print_status() override;
	static 	uint8_t ixChecksum(uint8_t len, void *ix_data);

private:
	char 	_device[32];
	int 	_uart_fd = -1;
	int 	_rate = 0;
	int		_rate_rx = 0;
	int		_timeout = 0;
	bool	_debug_flag = false;
	bool		_armed{false};
	int 	_sub_vehicle_cmd{-1};
	vehicle_command_s	_v_cmd;
	orb_advert_t		_vehicle_command_ack_pub;


	ix_ext_sys_status_t	_p1_sys_status;
	ix_local_storage_status_t _p1_storage;
	ix_shutter_speed_t	_p1_shutter;
	ix_iso_t			_p1_iso;
	ix_apeture_t		_p1_apeture;

	bool init();

	bool readPoll(uint32_t tout=5000);
	void updateData();
	void vehicleCmd(vehicle_command_s *vcmd);
	void vehicleCommandAck(const vehicle_command_s *cmd);

	int ixSendMessage(uint8_t ix_cmd, uint8_t ix_size, void *ix_data);
	static int ixParseData(uint8_t byte, ix_reply_t* reply);
	/**
	 * Check for parameter changes and update them if needed.
	 * @param parameter_update_sub uorb subscription to parameter_update
	 * @param force for a parameter update
	 */
	void parameters_update(int parameter_update_sub, bool force = false);
};

int UartPhaseOne::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
	driver for phaseone camera
### Examples
CLI usage example:
$ uart_phaseone start -d <uart device> -v
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("uart_phaseone", "modules");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAM_FLAG('v', "debug flag", true);
	PRINT_MODULE_USAGE_PARAM_STRING('d',UART_PHASEONE_UART, "", "debug flag", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

int UartPhaseOne::print_status()
{
	// print additional runtime information about the state of the module
	PX4_INFO("rate:%2i rx:%2i", _rate, _rate_rx);
	PX4_INFO("p1 status:%2i", _p1_sys_status.System_status);
	return 0;
}

int UartPhaseOne::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}


int UartPhaseOne::task_spawn(int argc, char *argv[])
{
	_task_id = px4_task_spawn_cmd("uart_phaseone",
				      SCHED_DEFAULT,
				      SCHED_PRIORITY_DEFAULT+12, /* reduced pritority */
				      1024,
				      (px4_main_t)&run_trampoline,
				      (char *const *)argv);

	if (_task_id < 0) {
		_task_id = -1;
		return -errno;
	}

	return 0;
}

UartPhaseOne *UartPhaseOne::instantiate(int argc, char *argv[])
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

	UartPhaseOne *instance = new UartPhaseOne(device, debug_flag);

	if (instance == nullptr) {
		PX4_ERR("alloc failed");
	}

	return instance;
}

UartPhaseOne::UartPhaseOne(char const *const device, bool debug_flag):
ModuleParams(nullptr)
{
	if(device)
	{
		memcpy(_device, device,sizeof(_device));
	} else{
		memcpy(_device, UART_PHASEONE_UART,sizeof(UART_PHASEONE_UART));
	}

	_debug_flag = debug_flag;
	memset(&_v_cmd, 0, sizeof(_v_cmd));
}

bool UartPhaseOne::init()
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
	/* subscribe to topics */
	_sub_vehicle_cmd = orb_subscribe(ORB_ID(vehicle_command));
	if(_sub_vehicle_cmd<0) {
		result = false;
	}
	return result;
}

/* wait tout ms for data available on the serial interface to read */
bool UartPhaseOne::readPoll(uint32_t tout)
{
    struct pollfd uartPoll[2];
    uartPoll[0].fd = _uart_fd;
    uartPoll[0].events = POLLIN;
    uartPoll[1].fd = _sub_vehicle_cmd;
    uartPoll[1].events = POLLIN;

    int pollrc = poll(&uartPoll[0], sizeof(uartPoll)/sizeof(pollfd), tout);
    if (pollrc < 1) return false; /* no data to read */
    return true; /* data available */
}

void UartPhaseOne::vehicleCommandAck(const vehicle_command_s *cmd)
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
void UartPhaseOne::vehicleCmd(vehicle_command_s *vcmd)
{
	/*	if(_debug_flag)
		{
			PX4_INFO("cmd=%d", vcmd->command);
			print_message(*vcmd);
		}*/
		switch(vcmd->command)
		{
		case vehicle_command_s::VEHICLE_CMD_CUSTOM_0:
			break;
		case vehicle_command_s::VEHICLE_CMD_DO_SET_ROI:
		case vehicle_command_s::VEHICLE_CMD_DO_SET_ROI_LOCATION:
			break;
		case vehicle_command_s::VEHICLE_CMD_SET_CAMERA_MODE: /* Set camera running mode. Use NAN for reserved values. |Reserved (Set to 0)| Camera mode (see CAMERA_MODE enum)| Reserved (all remaining params)|  */
		case vehicle_command_s::VEHICLE_CMD_DO_DIGICAM_CONTROL:

			break;
		case vehicle_command_s::VEHICLE_CMD_DO_TRIGGER_CONTROL:
			PX4_INFO("Trigger");
			print_message(*vcmd);
			break;
#ifdef MAV_CMD_REQUEST_CAMERA_SETTINGS
		case MAV_CMD::MAV_CMD_REQUEST_CAMERA_SETTINGS:
			PX4_INFO("MAV_CMD_REQUEST_CAMERA_SETTINGS");
			break;

		case MAV_CMD::MAV_CMD_DO_DIGICAM_CONFIGURE:
			/*
			 * MAV_CMD_DO_DIGICAM_CONFIGURE (202 )

			[Command] Configure digital camera. This is a fallback message for systems that have not yet implemented PARAM_EXT_XXX messages and camera definition files (see https://mavlink.io/en/services/camera_def.html ).
			Param (:Label)	Description	Values	Units
			1: Mode	Modes: P, TV, AV, M, Etc.	min:0 increment:1
			2: Shutter Speed	Shutter speed: Divisor number for one second.	min:0 increment:1
			3: Aperture	Aperture: F stop number.	min:0
			4: ISO	ISO number e.g. 80, 100, 200, Etc.	min:0 increment:1
			5: Exposure	Exposure type enumerator.
			6: Command Identity	Command Identity.
			7: Engine Cut-off	Main engine cut-off time before camera trigger. (0 means no cut-off)	min:0 increment:1	ds
			 */
			PX4_INFO("MAV_CMD_DO_DIGICAM_CONFIGURE");
			print_message(&vcmd);
			break;
#endif
		default:
			//print_message(&vcmd);
			break;
		}

}

/*
 * read subscriptions
 */
void UartPhaseOne::updateData()
{
	bool updated = false;
	orb_check(_sub_vehicle_cmd, &updated);
	if(updated) {
		orb_copy(ORB_ID(vehicle_command), _sub_vehicle_cmd, &_v_cmd);
	}
}

uint8_t UartPhaseOne::ixChecksum(uint8_t len, void *ix_data)
{
	uint8_t cs = 0;
	for(int i=0;i<len;i++)
	{
		cs^=((uint8_t*)ix_data)[i];
	}
	return cs;
}

int UartPhaseOne::ixSendMessage(uint8_t ix_cmd, uint8_t ix_size, void *ix_data)
{
	ix_request_header_t header;
	ssize_t written=0;
	header.header = IX_HEADER;
	header.msg_size = ix_size+2;
	header.version = 1;
	header.cmd_id = ix_cmd;

	written += write(_uart_fd, &header, sizeof(header));
	uint8_t cs = ixChecksum(2 ,&header.version);
	if(ix_size>0 && ix_size<IX_MAX_SIZE && ix_data!= nullptr){
		written += write(_uart_fd, ix_data, ix_size);
		cs ^= ixChecksum(ix_size ,ix_data);
	}
	written += write(_uart_fd, &cs, 1);

	return written!=(ssize_t)(sizeof(header) + ix_size+1);
}

int UartPhaseOne::ixParseData(uint8_t byte, ix_reply_t* reply)
{
	switch(reply->parse_state)
	{
	case 0:
		if(byte == IX_HEADER){
			reply->parse_state++;
			reply->msg_size = 0;
		}
		break;
	case 1:
		if(byte<=IX_MAX_SIZE)
		{
			reply->msg_size = byte;
			reply->parse_state++;
		} else {
			reply->parse_state=0; // size out of range
		}
		break;
	case 2:
		reply->version = byte;
		if(reply->version==1){
			reply->parse_state++;
		} else {
			reply->parse_state=0; // failed
		}
		break;
	case 3:
		reply->cmd_completion_code = byte;
		reply->parse_state++;
		break;
	case 4:
		reply->cmd_id = byte;
		if(reply->cmd_completion_code!= IX_CMD_COMPLETION_CODE_ID::NO_ERROR)
		{
			PX4_INFO("cmd(%i) error %i", reply->cmd_id, reply->cmd_completion_code);
			reply->parse_state=0;
			return reply->cmd_completion_code;
		}
		reply->parse_state++;
		break;
	default:
		if((reply->parse_state-5<IX_MAX_SIZE) && (reply->parse_state-5<reply->msg_size))
		{
			reply->cmd_data[reply->parse_state-5] = byte;
			reply->parse_state++;
		} else if(reply->parse_state-5 == reply->msg_size) { // checksum
			uint8_t cs = ixChecksum(reply->msg_size+3, &(reply->version));
			if (cs==byte){
				return reply->cmd_id;
			} else {
				PX4_INFO("cmd(%i) CS error %i != %i", reply->cmd_id, cs , byte);
			}
		}
		break;

	}
	return 0;
}

void UartPhaseOne::run()
{
	if(!init())
		return;
	hrt_abstime second_timer = hrt_absolute_time();
	ix_reply_t reply;
	int n = 0, n_rx=0;
	int request_update = 10;
	if(_debug_flag)
	{
		PX4_INFO("Start");
	}
	while (!should_exit()) {
		if(request_update==10) {
			ixSendMessage(IX_CMD_ID::GET_EXT_SYSTEM_STATUS,0, nullptr);
		}
		if(request_update==9) {
			ixSendMessage(IX_CMD_ID::GET_APERTURE,0, nullptr);
		}
		if(request_update==8) {
			uint8_t storage_type = IX_STORAGE_TYPE::LOCAL_STORAGE_XQD;
			ixSendMessage(IX_CMD_ID::GET_LOCAL_STORAGE_STATUS,1, &storage_type);
		}
		if(request_update>0) {
			request_update--;
		}
		if(readPoll(100000))
		{
			n++;
			updateData();
			uint8_t rxmsg[1];
			size_t msgsize;
			ssize_t nbytes=0;
			msgsize = sizeof(rxmsg);
			do{
				nbytes = ::read(_uart_fd, rxmsg, msgsize);
				if(nbytes > (ssize_t)msgsize) /* more read then asked for, should not happen */
				{
					PX4_ERR("read(%i) returned %i",
						   (int)msgsize, (int)nbytes);
				}
				if(nbytes>0) /* we got an answer */
				{
					int res = ixParseData(rxmsg[0], &reply);
					if(res>0) // command received
					{
						PX4_INFO("got reply for cmd %i", reply.cmd_id);
					}
					_timeout = 0;
					n_rx++;
				}
			}while(nbytes>0);
			usleep(10000);
		}
		if(hrt_elapsed_time(&second_timer)>=1e6) /* every second*/
		{
			_rate = n/1; /* update rate */
			_rate_rx = n_rx/1;
			n_rx = 0;
			second_timer += 1e6;
			request_update = 10;
			if(n==0 && _timeout==0)
			{
				/* lost connection */
				_timeout = 1;
			}
			n=0;
		}
	}
	orb_unsubscribe(_sub_vehicle_cmd);
	if(_vehicle_command_ack_pub) {
		orb_unadvertise(_vehicle_command_ack_pub);
	}
	close(_uart_fd);
}

int uart_phaseone_main(int argc, char *argv[])
{
	return UartPhaseOne::main(argc, argv);
}
