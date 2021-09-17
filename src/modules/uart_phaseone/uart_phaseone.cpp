/**
 * @file uart_phaseone.cpp
 *
 * generate NAVINFO1(0x36), NAVINFO2(0x37) and GPSTIME(0x3A) in phaseone binay format
 *
 * @author Maximilian Laiacker <post@mlaiacker.de>
 */

#include <px4_platform_common/module.h>
#include <px4_platform_common/getopt.h>
#include <px4_log.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/tasks.h>

#include <sys/types.h>
#include <sys/ioctl.h>
#include <fcntl.h>

#include <termios.h>
#include <poll.h>
#include <errno.h>

#ifdef __PX4_NUTTX
#include <nuttx/arch.h>
#include <arch/board/board.h>
#endif

#include <uORB/topics/camera_trigger.h>
#include <uORB/topics/camera_capture.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_command_ack.h>
#include <uORB/topics/debug_vect.h>
#include <uORB/topics/camera_trigger.h>

#include <matrix/math.hpp>

#include <drivers/drv_hrt.h>
#include "uart_phaseone.h"

#ifndef be16toh
#define be16toh(w)                       __builtin_bswap16((w))
#define be32toh(w)                       __builtin_bswap32((w))
#define be64toh(w)                       __builtin_bswap64((w))
#endif

/* default uart */
#define UART_PHASEONE_UART "/dev/ttyS2"

extern "C" __EXPORT int uart_phaseone_main(int argc, char *argv[]);

class UartPhaseOne : public ModuleBase<UartPhaseOne>
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
	bool	_debug_flag = false;
	int 	_sub_vehicle_cmd{-1};
	int 	_sub_trigger{-1};
	vehicle_command_s	_v_cmd;
	camera_trigger_s	_trigger_last;
	orb_advert_t		_vehicle_command_ack_pub = 0;
	orb_advert_t		_pub_debug_vect = 0;


	ix_ext_sys_status_t	_p1_sys_status;
	ix_local_storage_status_t _p1_storage;
	ix_shutter_speed_t	_p1_shutter;
	ix_iso_t			_p1_iso;
	ix_apeture_t		_p1_aperture;
	ix_exposure_t		_p1_exposure;
	ix_system_info_t	_p1_system_info;

	bool init();

	bool readPoll(uint32_t tout=5000);
	void updateData();
	void vehicleCmd(vehicle_command_s *vcmd);
	void vehicleCommandAck(const vehicle_command_s *cmd);
	void dataPublish(const debug_vect_s *vect);
	void formatCard();
	void capture();

	int ixSendMessage(uint8_t ix_cmd, uint8_t ix_size, void *ix_data);
	static int ixParseData(uint8_t byte, ix_reply_t* reply);
	void ixParseReply( ix_reply_t *reply);
};

int UartPhaseOne::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
	driver for phaseone ix Protocol
### Examples
CLI usage example:
$ uart_phaseone start -d <uart device> -v
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("uart_phaseone", "modules");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_COMMAND("capture");
	PRINT_MODULE_USAGE_COMMAND("format");
	PRINT_MODULE_USAGE_PARAM_FLAG('v', "debug flag", true);
	PRINT_MODULE_USAGE_PARAM_STRING('d',UART_PHASEONE_UART, "", "uart device", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

int UartPhaseOne::print_status()
{
	// print additional runtime information about the state of the module
	PX4_INFO("rate:%2i rx:%2i", _rate, _rate_rx);
	PX4_INFO("p1 status:%2i", _p1_sys_status.System_status);
	PX4_INFO("Camera:%s Lens:%s (%umm)", _p1_system_info.Camera_name, _p1_system_info.Lens_name, _p1_system_info.Lens_focal_length);
	PX4_INFO("storage state:%u space:%u Mb images left:%u mode:%u",
			_p1_storage.Local_storage_status, (uint32_t)(_p1_storage.Local_storage_capacity/1024/1024), _p1_storage.Local_storage_image_capacity, _p1_storage.Mass_storage_mode);
	return 0;
}

int UartPhaseOne::custom_command(int argc, char *argv[])
{

	const char *verb = argv[0];

	if (!is_running()){
		PX4_ERR("not running");
		return -1;
	}
	UartPhaseOne* inst = get_instance();
	if(inst==nullptr)
	{
		PX4_ERR("failed to get instance");
		return -1;
	}

	if (!strcmp(verb, "capture")) {
		inst->capture();
		return  0;
	}
	if (!strcmp(verb, "format")) {
		inst->formatCard();
		return  0;
	}
	return print_usage("unknown command");
}


int UartPhaseOne::task_spawn(int argc, char *argv[])
{
	_task_id = px4_task_spawn_cmd("uart_phaseone",
				      SCHED_DEFAULT,
				      SCHED_PRIORITY_DEFAULT, /* reduced pritority */
				      1424,
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

UartPhaseOne::UartPhaseOne(char const *const device, bool debug_flag)
{
	if(device)
	{
		memcpy(_device, device,sizeof(_device));
	} else{
		memcpy(_device, UART_PHASEONE_UART,sizeof(UART_PHASEONE_UART));
	}

	_debug_flag = debug_flag;
	memset(&_v_cmd, 0, sizeof(_v_cmd));
	memset(&_trigger_last,0 ,sizeof(_trigger_last));
	memset(&_p1_system_info, 0, sizeof(_p1_system_info));
	memset(&_p1_storage, 0, sizeof(_p1_storage));
	memset(&_p1_sys_status, 0, sizeof(_p1_sys_status));

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
//	int speed = B115200;
	int speed = B9600;
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
	_sub_trigger = orb_subscribe(ORB_ID(camera_trigger));
	if (_sub_trigger<0) {
		PX4_ERR("failed to sub to camera_trigger");
		result = false;
	}
	return result;
}

/* wait tout ms for data available on the serial interface to read */
bool UartPhaseOne::readPoll(uint32_t tout)
{
    struct pollfd uartPoll[3];
    uartPoll[0].fd = _uart_fd;
    uartPoll[0].events = POLLIN;
    uartPoll[1].fd = _sub_vehicle_cmd;
    uartPoll[1].events = POLLIN;
    uartPoll[2].fd = _sub_trigger;
    uartPoll[2].events = POLLIN;


    int pollrc = poll(&uartPoll[0], sizeof(uartPoll)/sizeof(uartPoll[0]), tout);
    if (pollrc < 1) return false; /* no data to read */
    return true; /* data available */
}

void UartPhaseOne::vehicleCommandAck(const vehicle_command_s *cmd)
{
	vehicle_command_ack_s vehicle_command_ack = {
		.timestamp = hrt_absolute_time(),
		.command = cmd->command,
		.result_param2 = 0,
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

void UartPhaseOne::dataPublish(const debug_vect_s *vect)
{
	if (_pub_debug_vect == nullptr) {
		_pub_debug_vect = orb_advertise(ORB_ID(debug_vect), vect);

	} else {
		orb_publish(ORB_ID(debug_vect), _pub_debug_vect, vect);
	}

}

void UartPhaseOne::formatCard(){
	{
		uint8_t cmd[2] = {IX_STORAGE_TYPE::LOCAL_STORAGE_XQD, IX_STORAGE_ACTION::DISABLE_MASS_STORAGE_MODE};
		ixSendMessage(IX_CMD_ID::SET_LOCAL_STORAGE_ACTION,2, cmd);
	}
	usleep(500000);
	{
		uint8_t cmd[2] = {IX_STORAGE_TYPE::LOCAL_STORAGE_XQD ,IX_STORAGE_ACTION::QUICK_FORMAT};
		ixSendMessage(IX_CMD_ID::SET_LOCAL_STORAGE_ACTION, 2, cmd);
	}
	if(_debug_flag){
		PX4_INFO("formatCard()");
	}
}

void UartPhaseOne::capture(){
	uint8_t cmd[1] = {IX_REPLY_MODE::ASYNC_REPLY_MODE};
	if(_debug_flag){
		PX4_INFO("Capture");
	}
	ixSendMessage(IX_CMD_ID::CAPTURE, 1, cmd);
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
		case vehicle_command_s::VEHICLE_CMD_CUSTOM_1:
			if(_debug_flag){
				PX4_INFO("VEHICLE_CMD_CUSTOM_1");
				print_message(*vcmd);
			}
			if(((int)vcmd->param2) == 87342)
			{
				vehicleCommandAck(vcmd);
				formatCard();
			}
			break;
		case vehicle_command_s::VEHICLE_CMD_DO_SET_ROI:
		case vehicle_command_s::VEHICLE_CMD_DO_SET_ROI_LOCATION:
			break;
		case vehicle_command_s::VEHICLE_CMD_SET_CAMERA_MODE: /* Set camera running mode. Use NAN for reserved values. |Reserved (Set to 0)| Camera mode (see CAMERA_MODE enum)| Reserved (all remaining params)|  */
			/* THIS INTERFACE IS DEPRECATED since 2018-01.
			 * Please use PARAM_EXT_XXX messages and the camera definition format described in https://mavlink.io/en/protocol/camera_def.html.
			 * |Session control e.g. show/hide lens
			 * | Zoom's absolute position
			 * | Zooming step value to offset zoom from the current position
			 * | Focus Locking, Unlocking or Re-locking
			 * | Shooting Command
			 * | Command Identity
			 * | Test shot identifier. If set to 1, image will only be captured, but not counted towards internal frame count.|  */
		case vehicle_command_s::VEHICLE_CMD_DO_DIGICAM_CONTROL:
			/*if(_debug_flag){
				PX4_INFO("DO_DIGICAM_CONTROL");
				print_message(*vcmd);
			}*/
			if(vcmd->param5>0){
				capture();
				vehicleCommandAck(vcmd);
			}
			break;
		case vehicle_command_s::VEHICLE_CMD_DO_TRIGGER_CONTROL:
			/*if(_debug_flag){
				PX4_INFO("Trigger control");
				print_message(*vcmd);
			}*/
			break;
		case vehicle_command_s::VEHICLE_CMD_CUSTOM_0:
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
		vehicleCmd(&_v_cmd);
	}
	camera_trigger_s trigger;
	orb_copy(ORB_ID(camera_trigger), _sub_trigger, &trigger);
	if(trigger.seq>_trigger_last.seq){
		capture();
		_trigger_last.seq = trigger.seq;
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

	//PX4_INFO("ixSend %i", ix_cmd);

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
		if((reply->parse_state-5<IX_MAX_SIZE) && (reply->parse_state-2 < reply->msg_size))
		{
			reply->cmd_data[reply->parse_state-5] = byte;
			reply->parse_state++;
		} else if(reply->parse_state-2 == reply->msg_size) { // checksum
			uint8_t cs = ixChecksum(reply->msg_size, &(reply->version));
			reply->parse_state = 0;
			if (cs==byte){
				return reply->cmd_id;
			} else {
				PX4_INFO("cmd(%i) CS error %i != %i", reply->cmd_id, cs , byte);
				return reply->cmd_id;
			}
		}
		break;

	}
	return 0;
}

void UartPhaseOne::ixParseReply( ix_reply_t *reply)
{
	switch(reply->cmd_id){
	case IX_CMD_ID::GET_SYSTEM_INFO:
		memcpy(&_p1_system_info, reply->cmd_data, sizeof(_p1_system_info));
		_p1_system_info.Camera_model_id = be32toh(_p1_system_info.Camera_model_id);
		_p1_system_info.Lens_model_id = be16toh(_p1_system_info.Lens_model_id);
		_p1_system_info.Lens_focal_length = be16toh(_p1_system_info.Lens_focal_length);

		_p1_system_info.Camera_name[15]=0; // to be sure
		_p1_system_info.Lens_name[31]=0;
		if(_debug_flag){
			PX4_INFO("Camera:%s id:%i Lens:%s (%imm)",
					_p1_system_info.Camera_name, _p1_system_info.Camera_model_id, _p1_system_info.Lens_name, _p1_system_info.Lens_focal_length);
		}
		debug_vect_s cam_lens;
		snprintf(cam_lens.name, sizeof(cam_lens.name), "PH1_CL_ID"); // phaseone camera and lend ID
		cam_lens.x = (float)_p1_system_info.Camera_model_id;
		cam_lens.y = (float)_p1_system_info.Lens_brand_id;
		cam_lens.z = (float)_p1_system_info.Lens_focal_length;
		cam_lens.timestamp = hrt_absolute_time();
		dataPublish(&cam_lens);
		break;
	case IX_CMD_ID::GET_EXT_SYSTEM_STATUS:
		memcpy(&_p1_sys_status, reply->cmd_data, sizeof(_p1_sys_status));
		_p1_sys_status.Remaining_captures = be32toh(_p1_sys_status.Remaining_captures);
		_p1_sys_status.Successful_captures_counter = be32toh(_p1_sys_status.Successful_captures_counter);
		_p1_sys_status.Missed_captures_counter = be32toh(_p1_sys_status.Missed_captures_counter);
		if(_debug_flag){
			PX4_INFO("status:%i remaining %i successfull %i",
					_p1_sys_status.System_status, _p1_sys_status.Remaining_captures, _p1_sys_status.Successful_captures_counter);
		}
		debug_vect_s sys_status;
		sprintf(sys_status.name,"PH1_CAP"); // phaseone Captures
		sys_status.x = _p1_sys_status.Remaining_captures;
		sys_status.y = _p1_sys_status.Successful_captures_counter;
		sys_status.z = _p1_sys_status.Missed_captures_counter;
		sys_status.timestamp = hrt_absolute_time();
		dataPublish(&sys_status);
		break;
	case IX_CMD_ID::GET_SYSTEM_STATUS:
		memcpy(&_p1_sys_status.System_status, reply->cmd_data, sizeof(_p1_sys_status.System_status));
		if(_debug_flag){
			PX4_INFO("status:%i", _p1_sys_status.System_status);
		}
		break;
	case IX_CMD_ID::GET_LOCAL_STORAGE_STATUS:
		memcpy(&_p1_storage, reply->cmd_data, sizeof(_p1_storage));
		_p1_storage.Local_storage_size = be64toh(_p1_storage.Local_storage_size);
		_p1_storage.Local_storage_capacity = be64toh(_p1_storage.Local_storage_capacity);
		_p1_storage.Local_storage_image_capacity = be32toh(_p1_storage.Local_storage_image_capacity);
		if(_debug_flag){
			PX4_INFO("storage: type %u  status %u capacity %u",
					_p1_storage.Local_storage_type, _p1_storage.Local_storage_status, (uint32_t)(_p1_storage.Local_storage_capacity/1024/1024));
		}
		debug_vect_s storage;
		snprintf(storage.name, sizeof(storage.name), "PH1_STORE"); // phaseone storage
		storage.x = (float)_p1_storage.Local_storage_image_capacity;
		storage.y = (float)(_p1_storage.Local_storage_capacity/1024/1024); // in Mega bytes
		storage.z = (float)_p1_storage.Local_storage_status;
		storage.timestamp = hrt_absolute_time();

		dataPublish(&storage);
		break;
	case IX_CMD_ID::GET_APERTURE:
		memcpy(&_p1_aperture, reply->cmd_data, sizeof(_p1_aperture));
		if(_debug_flag){
			double aperture  = 0;
			if(_p1_aperture.Apeture_value_denom!=0)
			{
				aperture = (float)_p1_aperture.Apeture_value_num/(float)_p1_aperture.Apeture_value_denom;
			}
			PX4_INFO("aperture:%f %i/%u", aperture, _p1_aperture.Apeture_value_num, _p1_aperture.Apeture_value_denom);
		}
		break;
	case IX_CMD_ID::GET_ISO:
		memcpy(&_p1_iso, reply->cmd_data, sizeof(_p1_iso));
		if(_debug_flag){
			double iso  = 0;
			if(_p1_iso.ISO_value_denom!=0)
			{
				iso = (float)_p1_iso.ISO_value_num/(float)_p1_iso.ISO_value_denom;
			}
			PX4_INFO("iso:%f %i/%i", iso, _p1_iso.ISO_value_num, _p1_iso.ISO_value_denom);
		}
		break;
	case IX_CMD_ID::GET_SHUTTER_SPEED:
		memcpy(&_p1_shutter, reply->cmd_data, sizeof(_p1_shutter));
		if(_debug_flag){
			double shutter  = 0;
			if(_p1_shutter.Shutter_speed_value_denom!=0)
			{
				shutter = (float)_p1_shutter.Shutter_speed_value_num/(float)_p1_shutter.Shutter_speed_value_denom;
			}
			PX4_INFO("shutter:%f %i/%u", shutter, _p1_shutter.Shutter_speed_value_num, _p1_shutter.Shutter_speed_value_denom);
		}
		debug_vect_s img_settings;
		memset(&img_settings, 0 , sizeof(img_settings));
		sprintf(img_settings.name,"PH1_ASI"); // phaseone apeture iso shutter
		if(_p1_aperture.Apeture_value_denom!=0) {
			img_settings.x = (float)_p1_aperture.Apeture_value_num/(float)_p1_aperture.Apeture_value_denom;
		}
		if (_p1_shutter.Shutter_speed_value_denom!=0){
			img_settings.y = (float)_p1_shutter.Shutter_speed_value_num/(float)_p1_shutter.Shutter_speed_value_denom;
		}
		if (_p1_iso.ISO_value_denom!=0){
			img_settings.z = (float)_p1_iso.ISO_value_num/(float)_p1_iso.ISO_value_denom;
		}
		img_settings.timestamp = hrt_absolute_time();
		dataPublish(&img_settings);
		break;
	case IX_CMD_ID::GET_EXPOSURE_COMPENSATION:
		memcpy(&_p1_exposure, reply->cmd_data, sizeof(_p1_exposure));
		if(_debug_flag){
			float exposure  = 0;
			if(_p1_exposure.Exposure_Compensation_value_denom!=0)
			{
				exposure = (float)_p1_exposure.Exposure_Compensation_value_num/(float)_p1_exposure.Exposure_Compensation_value_denom;
			}
			PX4_INFO("exposure:%f %i/%u", (double)exposure, _p1_exposure.Exposure_Compensation_value_num, _p1_exposure.Exposure_Compensation_value_denom);
		}
		break;
	default:
		if(_debug_flag){
			PX4_INFO("reply cmd id:%i code %i", reply->cmd_id, reply->cmd_completion_code);
		}

	}
}

void UartPhaseOne::run()
{
	if(!init())
		return;
	hrt_abstime second_timer = hrt_absolute_time();
	ix_reply_t reply;
	reply.parse_state = 0;
	int n = 0, n_rx=0;
	int request_update = 10;
	bool wait_for_reply = false;
	if(_debug_flag)
	{
		PX4_INFO("Start");
	}
	while (!should_exit()) {
		n++;
		if(readPoll(200))
		{
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
						ixParseReply(&reply);
						wait_for_reply = false;
					}
					n_rx++;
				}
			}while(nbytes>0);
		}
		if(hrt_elapsed_time(&second_timer)>=1e6) /* every second*/
		{
			_rate = n/1; /* update rate */
			_rate_rx = n_rx/1;
			n_rx = 0;
			second_timer += 1e6;
			n=0;
			if (wait_for_reply) {
				wait_for_reply = false; // timeout
			}
			if(!wait_for_reply)	{
				if(request_update==10) {
					ixSendMessage(IX_CMD_ID::GET_EXT_SYSTEM_STATUS, 0, nullptr);
					wait_for_reply = true;
				}
				if(request_update==9) {
					ixSendMessage(IX_CMD_ID::GET_APERTURE, 0, nullptr);
					wait_for_reply = true;
				}
				if(request_update==8) {
					ixSendMessage(IX_CMD_ID::GET_ISO, 0, nullptr);
					wait_for_reply = true;
				}
				if(request_update==7) {
					ixSendMessage(IX_CMD_ID::GET_SHUTTER_SPEED, 0, nullptr);
					wait_for_reply = true;
				}
				if(request_update==6) {
					uint8_t storage_type = IX_STORAGE_TYPE::LOCAL_STORAGE_XQD;
					ixSendMessage(IX_CMD_ID::GET_LOCAL_STORAGE_STATUS,1, &storage_type);
				}
				if(request_update==5) {
					ixSendMessage(IX_CMD_ID::GET_SYSTEM_INFO, 0, nullptr);
					wait_for_reply = true;
				}
				if(request_update==4) {
					ixSendMessage(IX_CMD_ID::GET_EXPOSURE_COMPENSATION, 0, nullptr);
					wait_for_reply = true;
				}
				if(request_update==3) {
					ixSendMessage(IX_CMD_ID::GET_SYSTEM_STATUS, 0, nullptr);
					wait_for_reply = true;
				}
				if(request_update==2) {
					uint8_t cmd[2] = {IX_STORAGE_TYPE::LOCAL_STORAGE_XQD, IX_STORAGE_ACTION::ENABLE_MASS_STORAGE_MODE};
					ixSendMessage(IX_CMD_ID::SET_LOCAL_STORAGE_ACTION,2, cmd);
					wait_for_reply = true;
				}
	/*			if(request_update==1) {
					uint8_t cmd[1] = {IX_REPLY_MODE::ASYNC_REPLY_MODE};
					ixSendMessage(IX_CMD_ID::CAPTURE,1, cmd);
					wait_for_reply = true;
				}*/
				if(request_update>0) {
					request_update--;
				} else {
					request_update = 10;
				}
			}
		}
	}
	orb_unsubscribe(_sub_vehicle_cmd);
	orb_unsubscribe(_sub_trigger);
	if(_vehicle_command_ack_pub) {
		orb_unadvertise(_vehicle_command_ack_pub);
	}
	if(_pub_debug_vect) {
		orb_unadvertise(_pub_debug_vect);
	}
	close(_uart_fd);
}

int uart_phaseone_main(int argc, char *argv[])
{
	return UartPhaseOne::main(argc, argv);
}
