/**
 * @file uart_merio.cpp
 *
 * generate NAVINFO1(0x36), NAVINFO2(0x37) and GPSTIME(0x3A) in merio binay format
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

#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_gps_position.h>
#include <uORB/topics/vehicle_attitude.h>
#include <matrix/math.hpp>

#include <drivers/drv_hrt.h>

#include "linker.h"
/* default uart */
#define UART_MERIO_UART "/dev/ttyS2"

extern "C" __EXPORT int uart_merio_main(int argc, char *argv[]);

class UartMerio : public ModuleBase<UartMerio>, public ModuleParams
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
private:
	Linker  _merioParser;
	char 	_device[32];
	int 	_uart_fd = -1;
	int 	_rate = 0;
	int 	_rate_gps = 0;
	int 	_rate_atti = 0;
	int		_rate_rx = 0;
	int		_timeout = 0;
	bool	_debug_flag = false;
	bool		_armed{false};
	int		_sub_vcontrol_mode{-1};		/**< vehicle control mode subscription */
	vehicle_control_mode_s _v_cmode;
	bool 	_updated_vcontrol_mode=false;
	int 	_sub_v_attitude{-1};
	vehicle_attitude_s _v_attitude;
	bool	_updated_attitude = false;
	int 	_sub_v_gpos{-1};
	vehicle_global_position_s _v_gpos;
	bool	_updated_gpos = false;
	int 	_sub_v_gps{-1};
	vehicle_gps_position_s _v_gps;

	bool init();

	bool readPoll(uint32_t tout=5000);
	void updateData();
	void vehicle_control_mode_poll();
	/**
	 * Check for parameter changes and update them if needed.
	 * @param parameter_update_sub uorb subscription to parameter_update
	 * @param force for a parameter update
	 */
	void parameters_update(int parameter_update_sub, bool force = false);
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
$ uart_merio start -d <uart device> -v
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("uart_merio", "modules");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAM_FLAG('v', "debug flag", true);
	PRINT_MODULE_USAGE_PARAM_STRING('d',UART_MERIO_UART, "", "debug flag", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

int UartMerio::print_status()
{
	// print additional runtime information about the state of the module
	PX4_INFO("rate:%2i atti:%2i gps:%2i rx:%2i", _rate, _rate_atti, _rate_gps, _rate_rx);
	PX4_INFO("conn:%2i", _merioParser.getConnectStatus());
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
				      SCHED_PRIORITY_DEFAULT+10, /* reduced pritority */
				      1024,
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

	UartMerio *instance = new UartMerio(device, debug_flag);

	if (instance == nullptr) {
		PX4_ERR("alloc failed");
	}

	return instance;
}

UartMerio::UartMerio(char const *const device, bool debug_flag):
ModuleParams(nullptr),
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
	memset(&_v_cmode, 0, sizeof(_v_cmode));
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
	int speed = B57600;
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
	_merioParser.connectWith(_uart_fd);
	/* subscribe to topics */
	_sub_v_attitude = orb_subscribe(ORB_ID(vehicle_attitude));
	_sub_vcontrol_mode = orb_subscribe(ORB_ID(vehicle_control_mode));
	_sub_v_gpos = orb_subscribe(ORB_ID(vehicle_global_position));
	_sub_v_gps = orb_subscribe(ORB_ID(vehicle_gps_position));
	if(_sub_v_attitude<0 || _sub_v_gpos<0 || _sub_v_gps<0) {
		result = false;
	}
	return result;
}

/* read arming state */
void UartMerio::vehicle_control_mode_poll()
{
	orb_check(_sub_vcontrol_mode, &_updated_vcontrol_mode);
	if (_updated_vcontrol_mode) {
		orb_copy(ORB_ID(vehicle_control_mode), _sub_vcontrol_mode, &_v_cmode);
		_armed = _v_cmode.flag_armed;
	}
}

/* wait tout ms for data available on the serial interface to read */
bool UartMerio::readPoll(uint32_t tout)
{
    struct pollfd uartPoll[3];
    uartPoll[0].fd = _uart_fd;
    uartPoll[0].events = POLLIN;
    uartPoll[1].fd = _sub_v_gpos;
    uartPoll[1].events = POLLIN;
    uartPoll[2].fd = _sub_v_attitude;
    uartPoll[2].events = POLLIN;

    int pollrc = poll(&uartPoll[0], 3, tout);
    if (pollrc < 1) return false; /* no data to read */
    return true; /* data available */
}

/*
 * read subscriptions
 */
void UartMerio::updateData()
{
	vehicle_control_mode_poll();
	orb_check(_sub_v_gpos, &_updated_gpos);
	if(_updated_gpos) {
		orb_copy(ORB_ID(vehicle_global_position), _sub_v_gpos, &_v_gpos);
	}
	orb_copy(ORB_ID(vehicle_gps_position), _sub_v_gps, &_v_gps);
	orb_check(_sub_v_attitude, &_updated_attitude);
	if(_updated_attitude) {
		orb_copy(ORB_ID(vehicle_attitude), _sub_v_attitude, &_v_attitude);
	}
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
	_merioParser.addBloc(GENPAYLOAD, B_POSITION);
	_merioParser.sendProtocol();
	while (!should_exit()) {
		if(readPoll(100000))
		{
			n++;
			updateData();
			if(!_merioParser.getConnectStatus())
			{
				_merioParser.addBloc(GENPAYLOAD, B_REQCMD);
				//_merioParser.addBloc(GENPAYLOAD, B_ICR);
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
					_updated_gpos = false;
					send = true;
					n_gps++;
				}
				if(_updated_attitude){
					_merioParser.addBloc(GENPAYLOAD, B_NAVINFO2);
					matrix::Eulerf euler = matrix::Quatf(_v_attitude.q);
					_merioParser.fillInt16((int16_t)(euler.phi()*1.0e4f)); // ROLL in rad*1e4
					_merioParser.fillInt16((int16_t)(euler.theta()*1.0e4f)); // PITCH in rad*1e4
					_merioParser.fillInt16((int16_t)(euler.psi()*1.0e4f)); // YAW in rad*1e4
					_merioParser.addLEN(); // should be 8
					_updated_attitude = false;

					send = true;
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
					update_time = false;
					send = true;
				}
				if(send) {
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
			usleep(20000);
		}
		if(hrt_elapsed_time(&second_timer)>=1e6) /* every 10 seconds*/
		{
			update_time = true;
			_rate = n/1; /* update rate */
			_rate_atti = n_atti/1;
			n_atti = 0;
			_rate_gps = n_gps/1;
			n_gps = 0;
			_rate_rx = n_rx/1;
			n_rx = 0;
			second_timer += 1e6;
			if(n==0 && _timeout==0)
			{
				/* lost connection */
				_timeout = 1;
			}
			n=0;
		}
	}
	orb_unsubscribe(_sub_v_attitude);
	orb_unsubscribe(_sub_v_gpos);
	orb_unsubscribe(_sub_v_gps);
	orb_unsubscribe(_sub_vcontrol_mode);
	close(_uart_fd);
}

int uart_merio_main(int argc, char *argv[])
{
	return UartMerio::main(argc, argv);
}
