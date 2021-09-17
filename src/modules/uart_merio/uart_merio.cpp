/**
 * @file uart_merio.cpp
 *
 * generate NAVINFO1(0x36), NAVINFO2(0x37) and GPSTIME(0x3A) in merio binay format
 *
 * @author Maximilian Laiacker <post@mlaiacker.de>
 */

#include <px4_platform_common/module.h>
#include <px4_platform_common/getopt.h>
#include <px4_log.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/tasks.h>
#include <systemlib/mavlink_log.h>

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
#include <ecl/geo/geo.h>

#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_gps_position.h>
#include <uORB/topics/vehicle_attitude.h>

#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_command_ack.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/trip2_sys_report.h>
#include <uORB/topics/trip2_los_report.h>
#include <uORB/topics/trip2_gnd_report.h>
#include <uORB/topics/debug_key_value.h>
#include <uORB/topics/mount_orientation.h>

#include <matrix/math.hpp>


#include <drivers/drv_hrt.h>

#include "linker.h"
/* default uart */
#define UART_MERIO_UART "/dev/ttyS2"

extern "C" __EXPORT int uart_merio_main(int argc, char *argv[]);

class UartMerio : public ModuleBase<UartMerio>
{
public:
	UartMerio(char const *const device, bool debug_flag);

	virtual ~UartMerio() = default;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static UartMerio *instantiate(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	/** @see ModuleBase::run() */
	void run() override;

	/** @see ModuleBase::print_status() */
	int print_status() override;

	void setUartBaud(int uartBaud = -1) {
		_uart_baud = uartBaud;
	};

private:
	Linker  _merioParser;
	char 	_device[32];
	int 	_uart_fd = -1;
	int 	_uart_baud = -1;
	int 	_rate = 0;
	int 	_rate_gps = 0;
	int 	_rate_atti = 0;
	int		_rate_rx = 0;
	int		_n_cmd=0;
	int		_rate_cmd=0;
	int		_timeout = 0;
	int 	_active = true; // if we control the gimbal or the image based tracking does it from extern
	bool	_debug_flag = false;
//	bool		_armed{false};
//	int		_sub_vcontrol_mode{-1};		/**< vehicle control mode subscription */
//	vehicle_control_mode_s _v_cmode{0};
//	bool 	_updated_vcontrol_mode=false;
	int 	_sub_v_attitude{-1};
	vehicle_attitude_s _v_attitude{0};
	bool	_updated_attitude = false;
	int 	_sub_v_gpos{-1};
	vehicle_global_position_s _v_gpos{0};
	bool	_updated_gpos = false;
	int 	_sub_v_gps{-1};
	vehicle_gps_position_s _v_gps{0};
	int 	_sub_vehicle_cmd{-1}; // for command long
	orb_advert_t	_pub_vehicle_command_ack{nullptr};
	orb_advert_t	_pub_temp_topic{nullptr};	///< for temperature debug topic
	orb_advert_t	_mavlink_log_pub{nullptr};	// mavlink log uORB handle
	orb_advert_t	_pub_trip_sys{nullptr};
	orb_advert_t	_pub_trip_gnd{nullptr};
	orb_advert_t	_pub_trip_los{nullptr};
	orb_advert_t	_pub_mount_orientation{nullptr};

	static const int _update_rate = 50;
	uint8_t _zoom_rate = 99;
	uint8_t _nav_mode = 4;
	/*
	 * Navigation modes :
 0 : OFF, no navigation filter active
 1 : FIXE, the gimbal is assumed immobile. Gimbal will compute roll and pitch bias
using its internal sensors. Heading value comes from the HEADING bloc (0x3C).
Yaw gyroscope bias is not estimated.
2 : INDOOR, GPS is not used for navigation, even if valid. Gimbal will compute roll
and pitch bias using its internal sensors. Heading value comes from the HEADING
bloc (0x3C). Yaw gyroscope bias is estimated, so the heading value must be
reliable.
3 : OUTDOOR, GPS is used for navigation if it is valid. Gyroscopes and
accelerometers biases are estimated.
4 : EXTNAV, External information is used to compute gimbal orientation. Gimbal
does not use the built-in inertial system.
	 */

	bool init();

	bool readPoll(uint32_t tout=5000);
	void updateData();
	void vehicle_control_mode_poll();
	void vehicleCommand(const vehicle_command_s *vcmd);
	void vehicleCommandAck(const vehicle_command_s *cmd)
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

		if (_pub_vehicle_command_ack == nullptr) {
			_pub_vehicle_command_ack = orb_advertise_queue(ORB_ID(vehicle_command_ack), &vehicle_command_ack,
						   vehicle_command_ack_s::ORB_QUEUE_LENGTH);

		} else {
			orb_publish(ORB_ID(vehicle_command_ack), _pub_vehicle_command_ack, &vehicle_command_ack);
		}

	}
	/* publish status in trip2 messages to be compatible with trip2 gimbal */
	void pubTrip();
	/* publish mount orientation for mavlink MOUNT_ORIENTATION message*/
	void pubMount();
};

int UartMerio::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
	driver for merio gimbal
### Examples
CLI usage example:
$ uart_merio start -d <uart device> -v -b baud
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("uart_merio", "modules");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAM_FLAG('v', "debug flag", true);
	PRINT_MODULE_USAGE_PARAM_STRING('d',UART_MERIO_UART, "", "debug flag", true);
	PRINT_MODULE_USAGE_PARAM_INT('b',115200, 300, 115200, "baudrate", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

int UartMerio::print_status()
{
	// print additional runtime information about the state of the module
	PX4_INFO("rate:%2i atti:%2i gps:%2i rx:%2i", _rate, _rate_atti, _rate_gps, _rate_rx);
	PX4_INFO("conn:%i crc errors %d ncmd:%i", _merioParser.getConnectStatus(), _merioParser._errors_crc, _n_cmd);
	PX4_INFO("Config Model:%i Day:%i(%i) Ir:%i %i",
			_merioParser._config.gimbal_model,
			_merioParser._config.day_model, _merioParser._config.day_format,
			_merioParser._config.ircam1_model,_merioParser._config.ircam2_model);
	PX4_INFO("TempDay=%.1f TempEle=%.1f",
			_merioParser._temperature.TemperatureDay*1e-2,
			_merioParser._temperature.TemperatureElectronics*1e-2);
	PX4_INFO("Pan=%.1fdeg Tilt=%.1fdeg Zoom=%i%%",
			_merioParser._status.pan_rad*1e-4*180/M_PI,
			_merioParser._status.tilt_rad*1e-4*180/M_PI,
			_merioParser._status.zoom_level);
	char state[16]="?";
	switch(_merioParser._status.status1&0x0f)
	{
	case Linker::MERIO_STATE::INIT: sprintf(state,"INIT"); break;
	case Linker::MERIO_STATE::INDEX : sprintf(state,"INDEX"); break;
	case Linker::MERIO_STATE::WAIT: sprintf(state,"WAIT"); break;
	case Linker::MERIO_STATE::ZEROS: sprintf(state,"ZEROS"); break;
	case Linker::MERIO_STATE::VELOCITY: sprintf(state,"VELOCITY"); break;
	case Linker::MERIO_STATE::POSITION: sprintf(state,"POSITION"); break;
	case Linker::MERIO_STATE::STOW: sprintf(state,"STOW"); break;
	case Linker::MERIO_STATE::GEOTRACK: sprintf(state,"GEOTRACK"); break;
	case Linker::MERIO_STATE::NUC: sprintf(state,"NUC"); break;
	case Linker::MERIO_STATE::FACTROY: sprintf(state,"FACTORY"); break;
	case Linker::MERIO_STATE::CONFIG: sprintf(state,"CONFIG"); break;
	case Linker::MERIO_STATE::MOTOR_PROBLEM: sprintf(state,"MOTOR PROBLEM"); break;
	}
	char mode[16]="?";
	switch(_merioParser._status.status1>>5)
	{
	case Linker::MERIO_MODE::VELOCITY : sprintf(mode,"VELOCITY"); break;
	case Linker::MERIO_MODE::POSITION : sprintf(mode,"POSITION"); break;
	case Linker::MERIO_MODE::STOW : sprintf(mode,"STOW"); break;
	case Linker::MERIO_MODE::GEO_TRACK : sprintf(mode,"GEOTRACK"); break;
	default: sprintf(mode,"%i",_merioParser._status.status1>>5); break;
	}
	PX4_INFO("S1 state:%s mode:%s", state, mode);
	PX4_INFO("S2 valid:%i EOGyCom:%i tiltCom:%i panCom:%i tiltIdx:%i panIdx:%i",
			_merioParser._status.status2 & 1,
			(_merioParser._status.status2 & 2)==2,
			(_merioParser._status.status2 & 4)==4,
			(_merioParser._status.status2 & 8)==8,
			(_merioParser._status.status2 & 16)==16,
			(_merioParser._status.status2 & 32)==32);
	PX4_INFO("S3 tiltMotErr:%i panMotErr:%i dayCom:%i IRCom:%i AF:%i ICR:%i VidSw:%i Laser:%i",
			_merioParser._status.status3 & 1,
			(_merioParser._status.status3 & 2)==2,
			(_merioParser._status.status3 & 4)==4,
			(_merioParser._status.status3 & 8)==8,
			(_merioParser._status.status3 & 16)==16,
			(_merioParser._status.status3 & 32)==32,
			(_merioParser._status.status3 & 64)==64,
			(_merioParser._status.status3 & 128)==128
			);
	PX4_INFO("S4 biasFilt:%i biasFiltSim:%i IRZoomLck:%i LRFlnk:%i DayWDR:%i DayStab:%i alimDay:%i alimIR:%i",
			_merioParser._status.status4 & 1,
			(_merioParser._status.status4 & 2)==2,
			(_merioParser._status.status4 & 4)==4,
			(_merioParser._status.status4 & 8)==8,
			(_merioParser._status.status4 & 16)==16,
			(_merioParser._status.status4 & 32)==32,
			(_merioParser._status.status4 & 64)==64,
			(_merioParser._status.status4 & 128)==128
			);
	PX4_INFO("S5 navMode:%i", _merioParser._status.navigation_mode);
	PX4_INFO("uav:%f,%f,%.1fm sats:%i status:%i",
			_merioParser._llh_uav.lat_rad*1e-9*180/M_PI,
			_merioParser._llh_uav.lon_rad*1e-9*180/M_PI,
			_merioParser._llh_uav.alt_m*1e-2,
			_merioParser._llh_uav.sats,
			_merioParser._llh_uav.status);
	PX4_INFO("euler: %.1f,%.1f,%.1f",
			_merioParser._euler_uav.roll_rad4*1e-4*180/M_PI,
			_merioParser._euler_uav.pitch_rad4*1e-4*180/M_PI,
			_merioParser._euler_uav.yaw_rad4*1e-4*180/M_PI);
	PX4_INFO("target:%f,%f,%.1fm",
			_merioParser._target.lat_rad*1e-9*180/M_PI,
			_merioParser._target.lon_rad*1e-9*180/M_PI,
			_merioParser._target.alt_m*1e-2);
	PX4_INFO("t. ref:%f,%f,%.1fm",
			_merioParser._target_ref.lat_rad*1e-9*180/M_PI,
			_merioParser._target_ref.lon_rad*1e-9*180/M_PI,
			_merioParser._target_ref.alt_m*1e-2);
	return 0;
}

int UartMerio::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}


int UartMerio::task_spawn(int argc, char *argv[])
{
	_task_id = px4_task_spawn_cmd("uart_merio",
				      SCHED_DEFAULT,
				      SCHED_PRIORITY_DEFAULT-28, /* reduced pritority */
				      1300,
				      (px4_main_t)&run_trampoline,
				      (char *const *)argv);

	if (_task_id < 0) {
		_task_id = -1;
		return -errno;
	}

	return 0;
}

UartMerio *UartMerio::instantiate(int argc, char *argv[])
{
	bool debug_flag = false;
	bool error_flag = false;
	int ch;
	int myoptind = 1;
	const char *myoptarg = nullptr;
	const char *device = nullptr;
	int baud = -1;

	// parse CLI arguments
	while ((ch = px4_getopt(argc, argv, "b:d:v", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'd':
			device = myoptarg;
			break;

		case 'b':
			baud = atoi(myoptarg);
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

	UartMerio *instance = new UartMerio(device, debug_flag);

	if (instance == nullptr) {
		PX4_ERR("alloc failed");
	} else	if(baud>0){
		instance->setUartBaud(baud);
	}

	return instance;
}

UartMerio::UartMerio(char const *const device, bool debug_flag):
_merioParser()
{
	if(device)
	{
		memcpy(_device, device,sizeof(_device));
	} else{
		memcpy(_device, UART_MERIO_UART,sizeof(UART_MERIO_UART));
	}

	_debug_flag = debug_flag;
	memset(&_v_attitude,0,sizeof(_v_attitude));
	memset(&_v_gpos, 0, sizeof(_v_gpos));
	memset(&_v_gps, 0, sizeof(_v_gps));
//	memset(&_v_cmode, 0, sizeof(_v_cmode));
}

bool UartMerio::init()
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
	if(_uart_baud>0) {
		speed = _uart_baud;
	}
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
		PX4_ERR("failed to set baudrate for %s: %d", _device, termios_state);
		close(_uart_fd);
		return false;
	}

	if ((termios_state = tcsetattr(_uart_fd, TCSANOW, &uart_config)) < 0) {
		PX4_ERR("tcsetattr failed for %s", _device);
		close(_uart_fd);
		return false;
	}
	_merioParser.connectWith(_uart_fd);
	/* subscribe to topics */
	_sub_v_attitude = orb_subscribe(ORB_ID(vehicle_attitude));
	//	_sub_vcontrol_mode = orb_subscribe(ORB_ID(vehicle_control_mode));
	_sub_v_gpos = orb_subscribe(ORB_ID(vehicle_global_position));
	_sub_v_gps = orb_subscribe(ORB_ID(vehicle_gps_position));
	_sub_vehicle_cmd = orb_subscribe(ORB_ID(vehicle_command));

	if(_sub_v_attitude<0 || _sub_v_gpos<0 || _sub_v_gps<0 || _sub_vehicle_cmd<0) {
		PX4_ERR("failed to subscribe");
		result = false;
	}
	orb_set_interval(_sub_v_gpos, 1e3/_update_rate);
	orb_set_interval(_sub_v_attitude, 1e3/_update_rate);
	orb_set_interval(_sub_v_gps, 1e3/_update_rate);
	return result;
}
/*
void UartMerio::roiControl(){
	vehicle_roi_s vehicle_roi;
	orb_copy(ORB_ID(vehicle_roi), _sub_vehicle_roi, &vehicle_roi);
}
*/
void UartMerio::vehicleCommand(const vehicle_command_s *vcmd)
{
	_n_cmd++;
	switch(vcmd->command)
	{
	case vehicle_command_s::VEHICLE_CMD_DO_MOUNT_CONTROL:
		switch ((int)vcmd->param7) {
		case vehicle_command_s::VEHICLE_MOUNT_MODE_RETRACT:
		/* FALLTHROUGH */
		case vehicle_command_s::VEHICLE_MOUNT_MODE_NEUTRAL:
			_merioParser.setPosition(0, 0);
			_merioParser.setMode(Linker::MERIO_MODE::POSITION);
			break;
		case vehicle_command_s::VEHICLE_MOUNT_MODE_MAVLINK_TARGETING:
			//MAVLink spec has pitch on channel 0
			//MAVLink spec has roll on channel 1
			// both specs have yaw on channel 2
			_merioParser.setPosition(vcmd->param3 * M_DEG_TO_RAD_F, vcmd->param1 * M_DEG_TO_RAD_F);
			_merioParser.setMode(Linker::MERIO_MODE::POSITION);
			break;
		case vehicle_command_s::VEHICLE_MOUNT_MODE_GPS_POINT:
			//control_data_set_lon_lat((double)vehicle_command.param2, (double)vehicle_command.param1, vehicle_command.param3);
			_merioParser.setGeoPoint(vcmd->param1, vcmd->param2, vcmd->param3);
			// set mode to geo tracking
			_merioParser.setMode(Linker::MERIO_MODE::GEO_TRACK);
		}
		vehicleCommandAck(vcmd);
		break;
	case vehicle_command_s::VEHICLE_CMD_DO_SET_ROI:
	case vehicle_command_s::VEHICLE_CMD_DO_SET_ROI_LOCATION:
		if(_debug_flag) {
			PX4_INFO("GTrack %f, %f, %f",(double)vcmd->param5, (double)vcmd->param6, (double)vcmd->param7);
		}
		/*
		 * vcmd->param5, lat
		 * vcmd->param6, lon
		 * vcmd->param7  alt
		 */
		_merioParser.setGeoPoint(vcmd->param5, vcmd->param6, vcmd->param7);
		// set mode to geo tracking
		_merioParser.setMode(Linker::MERIO_MODE::GEO_TRACK);
		vehicleCommandAck(vcmd);
		break;
	case vehicle_command_s::VEHICLE_CMD_SET_CAMERA_MODE:
	case vehicle_command_s::VEHICLE_CMD_DO_DIGICAM_CONTROL:
/*		if(_debug_flag) {
			PX4_INFO("cmd=DO_DIGICAM_CONTROL");
			print_message(*vcmd);
		}*/
		if((int)vcmd->param1==0) //Set System Mode
		{
			switch((int)vcmd->param2){
			case 0: // colibri stow
				if(_debug_flag) {
					PX4_INFO("STOW");
				}
				_merioParser.setPosition(0.0f, -3.0f*M_PI_F/180.0f);
				_merioParser.setMode(Linker::MERIO_MODE::POSITION);
				break;
			case 1: // colibri pilot
				if(_debug_flag) {
					PX4_INFO("PILOT");
				}
				_merioParser.setPosition(0, -30.0f*M_PI_F/180.0f);
				_merioParser.setMode(Linker::MERIO_MODE::POSITION);
				break;
			case 3: // colibri observe
				if(_debug_flag) {
					PX4_INFO("OBS");
				}
				_merioParser.setMode(Linker::MERIO_MODE::VELOCITY);
				_active = true;
				break;
			case 4: // colibri local position
				if(_debug_flag) {
					PX4_INFO("L.POS");
				}
				_merioParser.setPosition(vcmd->param4*M_PI_F/180.0f, vcmd->param3*M_PI_F/180.0f);
				_merioParser.setMode(Linker::MERIO_MODE::POSITION);
				break;
			default:
				if(_debug_flag) {
					PX4_INFO("unknown mode %d", (int)vcmd->param2);
				}
				_active = false;
				break;
			}
			vehicleCommandAck(vcmd);
		} else if((int)vcmd->param1==6) //set gimbal
		{
			if(_active) {
				_merioParser.setVelocity(-vcmd->param2*Linker::VEL_MAX_RAD_S,
					vcmd->param3*Linker::VEL_MAX_RAD_S, 0.2f);
			}
			int8_t zoom=0;
			if((int)vcmd->param4==1){ // zoom in
				if(_debug_flag) {
					PX4_INFO("ZOOM in %i%%", _merioParser._status.zoom_level);
				}
				zoom = _zoom_rate;
			} else if((int)vcmd->param4==2){ // zoom out
				if(_debug_flag) {
					PX4_INFO("ZOOM out %i%%", _merioParser._status.zoom_level);
				}
				zoom = -_zoom_rate;
			}
			_merioParser.setZoomVel(zoom);
			if(_active) {
				if(PX4_ISFINITE(vcmd->param5)){
					_merioParser.setGroundAltitude(vcmd->param5);
				}
			}
		}else if((int)vcmd->param1==11) //do nuc
		{
			if(_debug_flag) {
				PX4_INFO("NUC");
			}
			_merioParser.doNuc();
			vehicleCommandAck(vcmd);
		}else if((int)vcmd->param1==17) //gnd alt
		{
			if(PX4_ISFINITE(vcmd->param2)){
				_merioParser.setGroundAltitude(vcmd->param2);
			}
		}else if((int)vcmd->param1==26) //pilot view
		{
			if(_debug_flag) {
				PX4_INFO("PILOT %fdeg",(double)vcmd->param2);
			}
			_merioParser.setPosition(0, vcmd->param2*M_PI_F/180.0f);
			_merioParser.setMode(Linker::MERIO_MODE::POSITION);
			vehicleCommandAck(vcmd);
		} else {
			if(_debug_flag) {
				PX4_INFO("trip2 cmd %f", (double)vcmd->param1);
				print_message(*vcmd);
			}
		}

		break;
	case vehicle_command_s::VEHICLE_CMD_DO_TRIGGER_CONTROL:
		break;

	default:
		//print_message(&vcmd);
		break;
	}
}

/* read arming state */
/*
void UartMerio::vehicle_control_mode_poll()
{
	orb_check(_sub_vcontrol_mode, &_updated_vcontrol_mode);
	if (_updated_vcontrol_mode) {
		orb_copy(ORB_ID(vehicle_control_mode), _sub_vcontrol_mode, &_v_cmode);
		_armed = _v_cmode.flag_armed;
	}
}
*/
/* wait tout ms for data available on the serial interface to read */
bool UartMerio::readPoll(uint32_t tout)
{
    struct pollfd uartPoll[4];
    uartPoll[0].fd = _uart_fd;
    uartPoll[0].events = POLLIN;
    uartPoll[1].fd = _sub_v_gpos;
    uartPoll[1].events = POLLIN;
    uartPoll[2].fd = _sub_v_attitude;
    uartPoll[2].events = POLLIN;
    uartPoll[3].fd = _sub_vehicle_cmd;
    uartPoll[3].events = POLLIN;

    int pollrc = poll(&uartPoll[0], 4, tout);
    if (pollrc < 1) return false; /* no data to read */
    return true; /* data available */
}

/*
 * read subscriptions
 */
void UartMerio::updateData()
{
	//vehicle_control_mode_poll();
	orb_check(_sub_v_gpos, &_updated_gpos);
	if(_updated_gpos) {
		orb_copy(ORB_ID(vehicle_global_position), _sub_v_gpos, &_v_gpos);
		orb_copy(ORB_ID(vehicle_gps_position), _sub_v_gps, &_v_gps);
	}
	orb_check(_sub_v_attitude, &_updated_attitude);
	if(_updated_attitude) {
		orb_copy(ORB_ID(vehicle_attitude), _sub_v_attitude, &_v_attitude);
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
		}
	}

}

void UartMerio::pubTrip(){
	trip2_sys_report_s trip2_sys;
	trip2_gnd_report_s trip2_gnd;
//	trip2_los_report_s trip2_los;

	memset(&trip2_sys, 0, sizeof(trip2_sys));
	memset(&trip2_gnd, 0, sizeof(trip2_gnd));
//	memset(&trip2_los, 0, sizeof(trip2_los));

	trip2_sys.roll = _merioParser._status.pan_rad*1e-4f*180/M_PI_F;
	trip2_sys.pitch = _merioParser._status.tilt_rad*1e-4f*180/M_PI_F;
	trip2_sys.fov = _merioParser._status.fov_day_rad*1e-4f*180/M_PI_F;
	trip2_sys.cpu_temperature = _merioParser._temperature.TemperatureElectronics*1e-2f;
	trip2_sys.camera_temperature = _merioParser._temperature.TemperatureDay*1e-2f;
	trip2_sys.timestamp = hrt_absolute_time();

	trip2_sys.system_mode = 0;
	switch((_merioParser._status.status1) & 0x0f){
	case Linker::MERIO_STATE::VELOCITY:
		trip2_sys.system_mode = 4; // observation
		break;
	case Linker::MERIO_STATE::POSITION:
		trip2_sys.system_mode = 8; // local position
		break;
	case Linker::MERIO_STATE::GEOTRACK :
		trip2_sys.system_mode = 7; // PTC
		break;
	case Linker::MERIO_STATE::STOW :
		trip2_sys.system_mode = 0; // stow
		break;
	}
	if(_active) { // only send trip messages if we are active
		if (_pub_trip_sys == nullptr) {
			_pub_trip_sys = orb_advertise(ORB_ID(trip2_sys_report), &trip2_sys);
		} else {
			orb_publish(ORB_ID(trip2_sys_report), _pub_trip_sys, &trip2_sys);
		}

		if (_merioParser.getBlocReceivedClear(B_LLHTARGET) &&
				(_merioParser._target.alt_m!=0 ||
				_merioParser._target.lat_rad!=0 ||
				_merioParser._target.lon_rad!=0)){
			trip2_gnd.gndcrsalt = _merioParser._target.alt_m*1e-2f;
			trip2_gnd.gndcrslat = _merioParser._target.lat_rad*1e-9f*180/M_PI_F;
			trip2_gnd.gndcrslon = _merioParser._target.lon_rad*1e-9f*180/M_PI_F;
			float dist_xy=0, dist_z=0;
			get_distance_to_point_global_wgs84(_v_gpos.lat, _v_gpos.lon, _v_gpos.alt, trip2_gnd.gndcrslat, trip2_gnd.gndcrslon, trip2_gnd.gndcrsalt, &dist_xy, &dist_z);
			trip2_gnd.gndcrsslantrange = sqrt(dist_xy*dist_xy + dist_z*dist_z);
			trip2_gnd.timestamp = trip2_sys.timestamp;

			if (_pub_trip_gnd == nullptr) {
				_pub_trip_gnd = orb_advertise(ORB_ID(trip2_gnd_report), &trip2_gnd);
			} else {
				orb_publish(ORB_ID(trip2_gnd_report), _pub_trip_gnd, &trip2_gnd);
			}
		}
	}

}

void UartMerio::pubMount(){
	int instance;
	mount_orientation_s mount_orientation;
	mount_orientation.timestamp = hrt_absolute_time();
	mount_orientation.attitude_euler_angle[0]= 0;
	mount_orientation.attitude_euler_angle[1]= _merioParser._status.tilt_rad*1e-4f;
	mount_orientation.attitude_euler_angle[2]= _merioParser._status.pan_rad*1e-4f;

	orb_publish_auto(ORB_ID(mount_orientation), &_pub_mount_orientation, &mount_orientation, &instance);
}

void UartMerio::run()
{
	if(!init())
		return;
	hrt_abstime second_timer = hrt_absolute_time();
	int n = 0, n_atti=0, n_gps=0, n_rx=0;
	bool update_time = true;
	if(_debug_flag)
	{
		PX4_INFO("Start");
	}
	_merioParser.addBloc(GENPAYLOAD, B_REQCMD);
	_merioParser.addLEN();
	_merioParser.sendProtocol();
	while (!should_exit()) {
		if(readPoll(50000) || update_time)
		{
			n++;
			updateData();
			if(!_merioParser.getConnectStatus())
			{
				_merioParser.addBloc(GENPAYLOAD, B_REQCMD);
				_merioParser.addLEN();
				_merioParser.sendProtocol();
			}
			{
				bool send=false;
				if(_updated_gpos) {
					_merioParser.addBloc(GENPAYLOAD, B_NAVINFO1);
					_merioParser.fillInt32((int32_t)(_v_gpos.lat*1e9*M_PI/180.0)); // in rad*1e9
					_merioParser.fillInt32((int32_t)(_v_gpos.lon*1e9*M_PI/180.0)); // in rad*1e9
					_merioParser.fillInt32((int32_t)(_v_gpos.alt*1e2f));
					_merioParser.fillUInt8(_v_gps.satellites_used); // sats
					_merioParser.addLEN(); // should be 15
					send = true;
					_updated_gpos = false;
					n_gps++;
				}
				if(_updated_attitude){
					_merioParser.addBloc(GENPAYLOAD, B_NAVINFO2);
					matrix::Eulerf euler = matrix::Quatf(_v_attitude.q);
					_merioParser.fillInt16((int16_t)(euler.phi()*1.0e4f)); // ROLL in rad*1e4
					_merioParser.fillInt16((int16_t)(euler.theta()*1.0e4f)); // PITCH in rad*1e4
					_merioParser.fillInt16((int16_t)(euler.psi()*1.0e4f)); // YAW in rad*1e4
					_merioParser.addLEN(); // should be 8
					send = true;
					_updated_attitude = false;
					n_atti++;
				}
				if(update_time){
					time_t now;
					time(&now);
					struct tm* utc = gmtime(&now);
					_merioParser.addBloc(GENPAYLOAD, B_GPSTIME);
					_merioParser.fillInt16(utc->tm_year+1900);
					_merioParser.fillUInt8(utc->tm_mon+1);
					_merioParser.fillUInt8(utc->tm_mday);
					_merioParser.fillUInt8(utc->tm_hour);
					_merioParser.fillUInt8(utc->tm_min);
					_merioParser.fillUInt8(utc->tm_sec);
					_merioParser.fillUInt8(_v_gps.satellites_used);
					_merioParser.addLEN(); // should be 10
					send = true;
					update_time = false;

				}
				if (send) {
					_merioParser.sendProtocol();
				}
			}
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
					_merioParser.receive(rxmsg[0]);
					_timeout = 0;
					n_rx++;
				}
			}while(nbytes>0);
			// check if we got a status message
			if(_merioParser.getBlocReceivedClear(B_STATUS)){
				// parse last status message
				pubTrip();
				pubMount();
				// request a new status
				_merioParser.addBloc(GENPAYLOAD, B_REQCMD);
				_merioParser.fillUInt8(B_LLHTARGET);
				_merioParser.addLEN();
				_merioParser.sendProtocol();
			}
			// temperature publish every second
			if(_merioParser.getBlocReceivedClear(B_TEMPERATURE)) {
				struct debug_key_value_s new_report = {};
				snprintf(new_report.key, sizeof(new_report.key), "TempDay");
				new_report.value = _merioParser._temperature.TemperatureDay*1e-2f;
				new_report.timestamp = hrt_absolute_time();
				if (_pub_temp_topic != nullptr) {
						orb_publish(ORB_ID(debug_key_value), _pub_temp_topic, &new_report);
				} else {
					_pub_temp_topic = orb_advertise(ORB_ID(debug_key_value), &new_report);
				}
			}

#ifndef __PX4_NUTTX
			// some sleep on posix because poll timout dosn't seem to work
			usleep(20000);
#endif
		}
		if(hrt_elapsed_time(&second_timer)>=1e6) /* every second*/
		{
			update_time = true;
			_rate = n/1; /* update rate main loop*/
			_rate_atti = n_atti/1; // attitude message rate send to merio
			n_atti = 0;
			_rate_gps = n_gps/1; // gps message rate send to merio
			n_gps = 0;
			_rate_rx = n_rx/1; // bytes/s received from merio
			n_rx = 0;
			second_timer += 1e6;
			if(_rate_rx==0) {
				/* lost connection */
				if (_timeout<10)
				{
					_timeout += 1;
					if(_timeout==10){
						mavlink_log_emergency(&_mavlink_log_pub,"MERIO timeout");
					}
				}
			} else {
				if((_merioParser._status.status1 & 0x0f) == Linker::MERIO_STATE::MOTOR_PROBLEM){
					mavlink_log_emergency(&_mavlink_log_pub,"MERIO tiltMotErr:%i panMotErr:%i",
							_merioParser._status.status3 & 1,
							(_merioParser._status.status3 & 2)==2);
				}
			}
			n=0;
			_merioParser.addBloc(GENPAYLOAD, B_REQCMD);
			_merioParser.fillUInt8(B_TEMPERATURE);
			_merioParser.fillUInt8(B_LLHUAV);
			_merioParser.fillUInt8(B_EULERUAV);
			_merioParser.fillUInt8(B_LLHTARGET);
			_merioParser.fillUInt8(B_LLHTARGETREF);
			_merioParser.addLEN();
//			_merioParser.sendProtocol();

			if(_merioParser._status.navigation_mode != _nav_mode){
				_merioParser.addBloc(GENPAYLOAD, B_NAVCTRL);
				_merioParser.fillUInt8((_v_gps.fix_type>0)&0x01); // CMD: Bit 0 : on/off Bit 1 : simulation
				_merioParser.fillUInt8(_nav_mode);
				_merioParser.addLEN();
			}
			_merioParser.sendProtocol();
		}
	}
	orb_unsubscribe(_sub_v_attitude);
	orb_unsubscribe(_sub_v_gpos);
	orb_unsubscribe(_sub_v_gps);
	//orb_unsubscribe(_sub_vcontrol_mode);
	orb_unsubscribe(_sub_vehicle_cmd);
	if(_pub_temp_topic!=nullptr)
	{
		orb_unadvertise(_pub_temp_topic);
	}
	if(_pub_vehicle_command_ack!=nullptr)
	{
		orb_unadvertise(_pub_vehicle_command_ack);
	}

	close(_uart_fd);
}

int uart_merio_main(int argc, char *argv[])
{
	return UartMerio::main(argc, argv);
}
