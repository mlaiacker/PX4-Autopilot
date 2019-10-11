/****************************************************************************
 *
 *   Copyright (c) 2012-2015 PX4 Development Team. All rights reserved.
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
 * @file pwr_isl28022xx.cpp
 *
 * Driver for a voltage current monitor connected via SMBus (I2C).
 * Designed for pac1710/pac1720
 *
 * @author Randy Mackay <rmackay9@yahoo.com>
 * @author Alex Klimaj <alexklimaj@gmail.com>
 * @author Maximilian Laiacker <m.laiacker@germandrones.com>
 */

#include <float.h>
#include <stdio.h>
#include <string.h>
#include <ecl/geo/geo.h>

#include <drivers/device/i2c.h>
#include <drivers/device/ringbuffer.h>
#include <drivers/drv_hrt.h>
#include <px4_config.h>
#include <px4_workqueue.h>
#include <px4_getopt.h>
#include <perf/perf_counter.h>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/uORB.h>
#include <battery/battery.h>
#include <mathlib/mathlib.h>

#define PWR_ISL28022_ADDR_MIN             0x40	///< lowest possible address
#define PWR_ISL28022_ADDR_MAX             0x4F	///< highest possible address

#define PWR_ISL28022_MEASUREMENT_INTERVAL_US	(100000)	///< time in microseconds, measure at 10Hz
#define PWR_ISL28022_TIMEOUT_US			(10000000)	///< timeout looking for battery 10seconds after startup
#define PWR_ISL28022_SENS_RANGE			(320)   // mV
#define PWR_ISL28022_SENS_R				(0.1f) // mOhm

#ifndef CONFIG_SCHED_WORKQUEUE
# error This requires CONFIG_SCHED_WORKQUEUE.
#endif

#define PWR_ISL28022_ADDR			0x40 //default 0x80 in 8 bit
#define PWR_ISL28022_I2C_BUS		1

#define PWR_ISL28022_REG_CONFIG		0x00
#define PWR_ISL28022_CONFIG_AVG04	(0xAA<<3)
#define PWR_ISL28022_CONFIG_AVG08	(0xBB<<3)
#define PWR_ISL28022_CONFIG_AVG16	(0xCC<<3)
#define PWR_ISL28022_CONFIG_MODE_SB_CONT	(0x7) //Shunt and bus, continuous
#define PWR_ISL28022_CONFIG_BUS60V	(0x3<<13) //60v bus range

#define PWR_ISL28022_REG_SHUNT		0x01
#define PWR_ISL28022_REG_BUS		0x02


class PWR_ISL28022 : public device::I2C
{
public:
	PWR_ISL28022(int bus = PX4_I2C_BUS_EXPANSION, uint16_t pwr_isl28022_addr = PWR_ISL28022_ADDR,
			float sens_resistor=0, uint16_t sens_range=0);
	virtual ~PWR_ISL28022();

	/**
	 * Initialize device
	 *
	 * Calls probe() to check for device on bus.
	 *
	 * @return 0 on success, error code on failure
	 */
	virtual int		init();

	/**
	 * Test device
	 *
	 * @return 0 on success, error code on failure
	 */
	virtual int		test();

	/**
	 * Search all possible slave addresses for a smart battery
	 */
	int			search();

	int			dumpreg();

	bool		identify();

protected:
	/**
	 * Check if the device can be contacted
	 */
	virtual int		probe();

private:

	/**
	 * Start periodic reads from the battery
	 */
	void			start();

	/**
	 * Stop periodic reads from the battery
	 */
	void			stop();

	/**
	 * static function that is called by worker queue
	 */
	static void		cycle_trampoline(void *arg);

	/**
	 * perform a read from the battery
	 */
	void			cycle();

	bool			try_read_data(battery_status_s &new_report, uint64_t now);
	/**
	 * Read a word from specified register
	 */
	int			read_reg(uint8_t reg, uint16_t &val);

	/**
	 * Write a word to specified register
	 */
	int			write_reg(uint8_t reg, uint16_t val);


	void 			vehicle_control_mode_poll();

	// internal variables
	bool			_enabled;	///< true if we have successfully connected to battery
	work_s			_work{};		///< work queue for scheduling reads

	battery_status_s _last_report;	///< last published report, used for test()
	float			_discharged_mah{0};
	float			_current_a_filtered;
	float			_voltage_v;
	float			_voltage_v_filtered;
	float			_current_a;

	orb_advert_t	_batt_topic;	///< uORB battery topic
	orb_id_t		_batt_orb_id;	///< uORB battery topic ID
	int				_instance;

	int				_actuator_ctrl_0_sub{-1};		/**< attitude controls sub */
	int				_vcontrol_mode_sub{-1};		/**< vehicle control mode subscription */
	bool			_armed{false};

	uint64_t		_start_time;	///< system time we first attempt to communicate with battery
	uint16_t		_sens_full_scale; ///< current sense full range voltage mV
	uint16_t		_sens_sample_reg; ///< sample scale register value
	float			_sens_resistor;	///< current sense resistor value in mOhm
	float 			_sens_resolution;
	int 			_startupDelay; ///< prevent publish voltage before filter converged
	float 			_startRemaining; //< remain estimate based on voltage at beginning
	Battery				_battery;			/**< Helper lib to publish battery_status topic. */
};

#define	INST_MAX 2
namespace
{
PWR_ISL28022* g_pwr_isl28022[INST_MAX];	///< device handle. For now, we only support INST_MAX devices
}

void pwr_isl28022_usage();

extern "C" __EXPORT int pwr_isl28022_main(int argc, char *argv[]);


PWR_ISL28022::PWR_ISL28022(int bus, uint16_t pwr_isl28022_addr, float sens_resistor, uint16_t sens_range) :
	I2C("pwr_isl28022", nullptr, bus, pwr_isl28022_addr, 400000),
	_enabled(false),
	_last_report{},
	_batt_topic(nullptr),
	_batt_orb_id(nullptr),
	_start_time(0),
	_sens_full_scale(PWR_ISL28022_SENS_RANGE),
	_sens_sample_reg(0),
	_sens_resistor(PWR_ISL28022_SENS_R),
	_startRemaining(0),
	_battery()
{
	// capture startup time
	_start_time = hrt_absolute_time();
	_startupDelay = 100;

	if(sens_resistor>0){
		_sens_resistor=sens_resistor;
	}
	// 0x5x = sign + 11 bits resolution
	if(sens_range==40)
	{
		_sens_full_scale = 40;
		_sens_sample_reg = 0x0<11; // 0101 0000
		_sens_resolution = 4096.0f;
	}
	if(sens_range==80)
	{
		_sens_full_scale = 80;
		_sens_sample_reg = 0x1<11;
		_sens_resolution = 8192.0f;
	}
	if(sens_range==160)
	{
		_sens_full_scale = 160;
		_sens_sample_reg = 0x2<<11;
		_sens_resolution = 16384.0f;
	}
	if(sens_range==320)
	{
		_sens_full_scale = 320;
		_sens_sample_reg = 0x3<<11;
		_sens_resolution = 32768.0f;
	}
	_current_a_filtered = 0.0f;
	_voltage_v_filtered = 0.0f;
	_current_a = 0.0f;
	_discharged_mah = 0.0f;
}

PWR_ISL28022::~PWR_ISL28022()
{
	// make sure we are truly inactive
	stop();
	orb_unsubscribe(_actuator_ctrl_0_sub);
	orb_unsubscribe(_vcontrol_mode_sub);
	if(_batt_topic!=nullptr)
	{
		orb_unadvertise(_batt_topic);
	}
}

int
PWR_ISL28022::init()
{
	int ret = ENOTTY;

	// attempt to initialise I2C bus
	ret = I2C::init();

	if (ret != OK) {
		PX4_ERR("failed to init I2C");
		return ret;

	} else {
		//Find the IC on the bus
		identify();
		// start work queue
		start();
	}

	// init orb id
	_batt_orb_id = ORB_ID(battery_status);

	/* needed for the Battery class */
	_actuator_ctrl_0_sub = orb_subscribe(ORB_ID(actuator_controls_0));
	/* needed to read arming status */
	_vcontrol_mode_sub = orb_subscribe(ORB_ID(vehicle_control_mode));
	return ret;
}

int
PWR_ISL28022::test()
{
	uint64_t start_time = hrt_absolute_time();

	// loop for 3 seconds
	while ((hrt_absolute_time() - start_time) < 3000000) {
		print_message(_last_report);
		// sleep for 0.2 seconds
		usleep(200000);
	}

	return OK;
}

bool
PWR_ISL28022::identify()
{
	uint16_t reg=0;
	if (read_reg(PWR_ISL28022_REG_CONFIG, reg) == OK) {
		if(reg>0){
			PX4_INFO("config=0x%04x", reg);
			if (read_reg(PWR_ISL28022_REG_BUS, reg) == OK) {
				PX4_INFO("bus=0x%04x", reg);
			}
			if (read_reg(PWR_ISL28022_REG_SHUNT, reg) == OK) {
				PX4_INFO("shunt=0x%04x", reg);
			}
			PX4_INFO("ISL28022 found at 0x%x", get_device_address());
			return true;
		} else
		{
			PX4_INFO("dev found at 0x%x reg0=0x%x", get_device_address(), reg);
		}
	}
	return false;
}

int
PWR_ISL28022::search()
{
	bool found_slave = false;
	int16_t orig_addr = get_device_address();

	// search through all valid SMBus addresses
	for (uint8_t i = PWR_ISL28022_ADDR_MIN; i < PWR_ISL28022_ADDR_MAX; i++) {
		set_device_address(i);
		if(identify())
		{
			found_slave = true;
			break;
		}
		usleep(1);

	}

	if (found_slave == false) {
		// restore original i2c address
		set_device_address(orig_addr);
	}

	// display completion message
	if (found_slave) {
		PX4_INFO("current monitor connected");

	} else {
		PX4_ERR("No current monitor found.");
	}

	return OK;
}

int
PWR_ISL28022::probe()
{
	// always return OK to ensure device starts
	return OK;
}

void
PWR_ISL28022::start()
{
	_startupDelay = 100;
	// schedule a cycle to start things
	work_queue(HPWORK, &_work, (worker_t)&PWR_ISL28022::cycle_trampoline, this, 1);
}

void
PWR_ISL28022::stop()
{
	work_cancel(HPWORK, &_work);
}

void
PWR_ISL28022::cycle_trampoline(void *arg)
{
	PWR_ISL28022 *dev = (PWR_ISL28022 *)arg;

	dev->cycle();
}
bool
PWR_ISL28022::try_read_data(battery_status_s &new_report, uint64_t now){
	// temporary variable for storing SMBUS reads
	uint16_t shunt=0;
	uint16_t bus=0;


	if (read_reg(PWR_ISL28022_REG_BUS, bus) == OK) {
		// read data from sensor
		memset(&new_report,0,sizeof(new_report));
		new_report.timestamp = now;

		// convert millivolts to volts
		_voltage_v = ((float)(bus>>2)*0.004f);//60.0f) / 16384.0f;
		_voltage_v_filtered = _voltage_v*0.05f + _voltage_v_filtered*0.95f; /* voltage filter */
		// read current
		if (read_reg(PWR_ISL28022_REG_SHUNT, shunt) == OK){
			int16_t current = 0;
			if(shunt&0x8000) // sign bit
			{ // negative
				current = 0x8000 | shunt;
			} else
			{
				current = shunt;
			}
			_current_a = ((float)_sens_full_scale/_sens_resistor)*((float)current)/_sens_resolution;
			_current_a_filtered = _current_a*0.05f + _current_a_filtered*0.95f; /* current filter */
		}

		// calculate total discharged amount
		_discharged_mah = _discharged_mah + _current_a*1000.0f/3600.0f*PWR_ISL28022_MEASUREMENT_INTERVAL_US/1000000.0f;
		vehicle_control_mode_poll();
		actuator_controls_s ctrl;
		orb_copy(ORB_ID(actuator_controls_0), _actuator_ctrl_0_sub, &ctrl);

		_battery.updateBatteryStatus(now, _voltage_v, _current_a,
				_voltage_v>2.0f, true , 0,
				ctrl.control[actuator_controls_s::INDEX_THROTTLE],
				_armed, &new_report);
		new_report.voltage_filtered_v = _voltage_v_filtered;
		new_report.discharged_mah = _discharged_mah;
		if((hrt_absolute_time()-_start_time)>=1000000)
		{
			new_report.average_current_a = _discharged_mah*3600.0f/((hrt_absolute_time()-_start_time)*1.0e-6f*1000.0f);
		}
		new_report.current_filtered_a = _current_a_filtered;
		if(_startRemaining>=0.0f && _startRemaining<=1.0f)
		{
			// subtract start remaining from remaining based on used mAh to deal with a startup with not fully charged battery
			new_report.remaining = math::max(new_report.remaining-(1.0f-_startRemaining), 0.0f);
		}
		new_report.serial_number = get_device_address();
		return true;
	}
	return false;
}

void
PWR_ISL28022::cycle()
{
	// get current time
	uint64_t now = hrt_absolute_time();

	battery_status_s new_report = {};
	if(try_read_data(new_report, now)){
		if (_batt_topic != nullptr) {
				if(_startupDelay<=0) {
					// publish to orb
					orb_publish(_batt_orb_id, _batt_topic, &new_report);
				} else
				{
					_startupDelay--;
					_startRemaining = _battery.getRemainingVoltage();
					if(_startupDelay==0)
					{
						PX4_INFO("remaining at start=%i", (int)_startRemaining*100);
					}
				}
		} else {
			uint16_t config = _sens_sample_reg|
					PWR_ISL28022_CONFIG_AVG16|
					PWR_ISL28022_CONFIG_MODE_SB_CONT |
					PWR_ISL28022_CONFIG_BUS60V;
			PX4_INFO("config reg 0x%04x",config);
			write_reg(PWR_ISL28022_REG_CONFIG, config);
			new_report.connected=0;
			_batt_topic = orb_advertise_multi(_batt_orb_id, &new_report, &_instance, ORB_PRIO_DEFAULT);
			if (_batt_topic == nullptr) {
				PX4_ERR("ADVERT FAIL");
			} else {
				PX4_INFO("instance %i", _instance);
			}
		}
		// copy report for test()
		_last_report = new_report;
		// record we are working
		_enabled = true;
	} else { // failed to read data
		if (_last_report.connected){
			_last_report.connected=0;
			if (_batt_topic != nullptr) {
				orb_publish(_batt_orb_id, _batt_topic, &_last_report); // report lost connection to battery
			}
		}
	}
	// schedule a fresh cycle call when the measurement is done
	work_queue(HPWORK, &_work, (worker_t)&PWR_ISL28022::cycle_trampoline, this,
		   USEC2TICK(PWR_ISL28022_MEASUREMENT_INTERVAL_US));
}


int
PWR_ISL28022::read_reg(uint8_t reg, uint16_t &val)
{
	uint8_t buff[2];
	// read from register
	int ret = transfer(&reg, 1, buff, 2);

	val = (((uint16_t)buff[0])<<8)|(uint16_t)buff[1];
	// return success or failure
	return ret;
}

int
PWR_ISL28022::write_reg(uint8_t reg, uint16_t val)
{
	uint8_t buff[3];  // reg + 1 bytes of data

	buff[0] = reg;
	buff[1] = (uint8_t)(val>>8);
	buff[2] = (uint8_t)(val&0xff);

	// write bytes to register
	int ret = transfer(buff, 3, nullptr, 0);
	if (ret != OK) {
		PX4_ERR("Reg 0x%x write error", reg);
	}
	// return success or failure
	return ret;
}

/* read arming state */
void PWR_ISL28022::vehicle_control_mode_poll()
{
	struct vehicle_control_mode_s vcontrol_mode;
	bool vcontrol_mode_updated;
	orb_check(_vcontrol_mode_sub, &vcontrol_mode_updated);
	if (vcontrol_mode_updated) {
		orb_copy(ORB_ID(vehicle_control_mode), _vcontrol_mode_sub, &vcontrol_mode);
		_armed = vcontrol_mode.flag_armed;
	}
}

///////////////////////// shell functions ///////////////////////

void
pwr_isl28022_usage()
{
	PX4_INFO("missing command: try 'start', 'test', 'stop', 'search'");
	PX4_INFO("options:");
	PX4_INFO("    -i <instance (0..1)>");
	PX4_INFO("    -b i2cbus (%d)", PWR_ISL28022_I2C_BUS);
	PX4_INFO("    -a addr (0x%x)", PWR_ISL28022_ADDR);
	PX4_INFO("    -r sense resistor (%.2fmOhm)",(double)PWR_ISL28022_SENS_R);
	PX4_INFO("    -s full range sense voltage (%imV) 40,80,160,320",PWR_ISL28022_SENS_RANGE);
}



int
pwr_isl28022_main(int argc, char *argv[])
{
	unsigned int instance = 0;
	int i2cdevice = PWR_ISL28022_I2C_BUS;
	int pwr_isl28022adr = PWR_ISL28022_ADDR; // 7bit address
	float	resistor = 0;
	int range = 0;

	int myoptind = 1;
	const char *myoptarg = nullptr;

	for(instance=0;g_pwr_isl28022[instance]!=nullptr && instance<INST_MAX;instance++);

	int ch;
	while ((ch = px4_getopt(argc, argv, "a:b:r:s:i:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'a':
			pwr_isl28022adr = strtol(myoptarg, nullptr, 0);
			break;

		case 'b':
			i2cdevice = strtol(myoptarg, nullptr, 0);
			break;

		case 's':
			range = strtol(myoptarg, nullptr, 0);
			PX4_INFO("range +-%imV", range);
			break;

		case 'r':
			resistor = atof(myoptarg);
			PX4_INFO("resistor %.2fmOhm", (double)resistor);
			break;

		case 'i':
			instance = strtol(myoptarg, nullptr, 0);
			break;

		default:
			pwr_isl28022_usage();
			return 0;
		}
	}
	if(instance>=INST_MAX)
	{
		instance = INST_MAX-1;
	}

	if (myoptind >= argc) {
		pwr_isl28022_usage();
		return 1;
	}

	const char *verb = argv[myoptind];

	if (!strcmp(verb, "start")) {
		PWR_ISL28022 *drv = g_pwr_isl28022[instance];
		if (drv != nullptr) {
			PX4_ERR("already started");
		} else {
			// create new global object
			drv = new PWR_ISL28022(i2cdevice, pwr_isl28022adr, resistor, range);

			if (drv == nullptr) {
				PX4_ERR("new failed");
				return 1;
			}

			if (OK != drv->init()) {
				delete drv;
				drv = nullptr;
				PX4_ERR("init failed");
				return 1;
			}
			PX4_INFO("%d started",instance);
			g_pwr_isl28022[instance] = drv;
		}

		return 0;
	}

	// need the driver past this point
	if (g_pwr_isl28022[instance] == nullptr) {
		PX4_INFO("not started");
		pwr_isl28022_usage();
		return 1;
	}

	if (!strcmp(verb, "test")) {
		g_pwr_isl28022[instance]->identify();
		g_pwr_isl28022[instance]->test();
		return 0;
	}

	if (!strcmp(verb, "stop")) {
		delete g_pwr_isl28022[instance];
		g_pwr_isl28022[instance] = nullptr;
		return 0;
	}

	if (!strcmp(verb, "search")) {
		if(g_pwr_isl28022[instance]->search()==OK)
		{
			//g_pwr_isl28022->dumpreg();
			return 0;
		}
		return 1;
	}


	pwr_isl28022_usage();
	return 0;
}
