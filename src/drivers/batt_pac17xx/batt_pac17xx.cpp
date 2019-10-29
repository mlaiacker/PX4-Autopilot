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
 * @file batt_pac17xx.cpp
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

#define BATT_PAC17_ADDR_MIN             0x00	///< lowest possible address
#define BATT_PAC17_ADDR_MAX             0x7F	///< highest possible address

#define BATT_PAC17_MEASUREMENT_INTERVAL_US	(50000)	///< time in microseconds, measure at xHz
#define BATT_PAC17_TIMEOUT_US			(5000000)	///< timeout looking for battery xseconds after startup
#define BATT_PAC17_SENS_RANGE			(80)   // mV
#define BATT_PAC17_SENS_R				(0.1f) // mOhm

#ifndef CONFIG_SCHED_WORKQUEUE
# error This requires CONFIG_SCHED_WORKQUEUE.
#endif

#define BATT_PAC17_ADDR			0x4c //default 0x98 in 8 bit
#define BATT_PAC17_I2C_BUS		1


#define BATT_PAC17_REG_CONFIG			0x00
#define BATT_PAC17_REG_CONVERSION_RATE	0x01
#define BATT_PAC17_REG_ONE_SHOT			0x02
#define BATT_PAC17_REG_CHANNEL_MASK		0x03
#define BATT_PAC17_REG_VOLT_SAMPLE_CONF	0x0A
#define BATT_PAC17_REG_SENS1_SAMPLE_CONF	0x0B
#define BATT_PAC17_REG_SENS2_SAMPLE_CONF	0x0C

#define BATT_PAC17_REG_SENS_CH1_H	0x0D
#define BATT_PAC17_REG_SENS_CH1_L	0x0E
#define BATT_PAC17_REG_SENS_CH2_H	0x0F
#define BATT_PAC17_REG_SENS_CH2_L	0x10

#define BATT_PAC17_REG_VOLT_CH1_H	0x11
#define BATT_PAC17_REG_VOLT_CH1_L	0x12
#define BATT_PAC17_REG_VOLT_CH2_H	0x13
#define BATT_PAC17_REG_VOLT_CH2_L	0x14

#define BATT_PAC17_REG_PID		0xfd // should be 0x57 or 0x58
#define BATT_PAC17_REG_MID		0xfe // should be 0x5d
#define BATT_PAC17_REG_REVISION	0xff // should be 0x81

class BATT_PAC17 : public device::I2C
{
public:
	BATT_PAC17(int bus = PX4_I2C_BUS_EXPANSION, uint16_t batt_pac17_addr = BATT_PAC17_ADDR,
			float sens_resistor=0, uint16_t sens_range=0);
	virtual ~BATT_PAC17();

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
	int			read_reg(uint8_t reg, uint8_t &val);

	/**
	 * Write a word to specified register
	 */
	int			write_reg(uint8_t reg, uint8_t val);

	/**
	 * Convert from 2's compliment to decimal
	 * @return the absolute value of the input in decimal
	 */
	uint16_t	convert_twos_comp(uint16_t val);

	/**
	 * Read block from bus
	 * @return returns number of characters read if successful, zero if unsuccessful
	 */
	uint8_t			read_block(uint8_t reg, uint8_t *data, uint8_t max_len, bool append_zero);

	/**
	 * Write block to the bus
	 * @return the number of characters sent if successful, zero if unsuccessful
	 */
	uint8_t			write_block(uint8_t reg, uint8_t *data, uint8_t len);


	void 			vehicle_control_mode_poll();

	// internal variables
	bool			_enabled;	///< true if we have successfully connected to battery
	work_s			_work{};		///< work queue for scheduling reads

	battery_status_s _last_report;	///< last published report, used for test()
	float			_discharged_mah;
	float			_current_a_filtered;
	float			_voltage_v;
	float			_voltage_v_filtered;
	float			_current_a;
	float			_voltage2; 	///< for pac1720
	float			_current2; 	///< for pac1720

	orb_advert_t	_batt_topic;	///< uORB battery topic
	orb_id_t		_batt_orb_id;	///< uORB battery topic ID

	int				_actuator_ctrl_0_sub{-1};		/**< attitude controls sub */
	int				_vcontrol_mode_sub{-1};		/**< vehicle control mode subscription */
	bool			_armed{false};

	uint64_t		_start_time;	///< system time we first attempt to communicate with battery
	uint16_t		_sens_full_scale; ///< current sense full range voltage 10,20,40,80 mV
	uint8_t			_sens_sample_reg; ///< sample scale register value
	float			_sens_resistor;	///< current sense resistor value in mOhm
	int 			_startupDelay; ///< prevent publish voltage before filter converged
	float 			_startRemaining; //< remain estimate based on voltage at beginning
	enum SMBUS_BATT_DEV_E
	{
		NONE=0,
		BQ40Z50,
		PAC1710,
		PAC1720,
	};
	SMBUS_BATT_DEV_E			_dev_id;	// 0 = BQ40Z50, 1 = PAC1710, 2=PAC1720
	Battery				_battery;			/**< Helper lib to publish battery_status topic. */
};

namespace
{
BATT_PAC17 *g_batt_pac17;	///< device handle. For now, we only support one BATT_PAC17 device
}

void batt_pac17_usage();

extern "C" __EXPORT int batt_pac17xx_main(int argc, char *argv[]);


BATT_PAC17::BATT_PAC17(int bus, uint16_t batt_pac17_addr, float sens_resistor, uint16_t sens_range) :
	I2C("batt_pac17", "/dev/batt_pac170", bus, batt_pac17_addr, 400000),
	_enabled(false),
	_last_report{},
	_batt_topic(nullptr),
	_batt_orb_id(nullptr),
	_start_time(0),
	_sens_full_scale(BATT_PAC17_SENS_RANGE),
	_sens_sample_reg(0x53),
	_sens_resistor(BATT_PAC17_SENS_R),
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
	if(sens_range==10)
	{
		_sens_full_scale = 10;
		_sens_sample_reg = 0x50; // 0101 0000
	}
	if(sens_range==20)
	{
		_sens_full_scale = 20;
		_sens_sample_reg = 0x51; // 0101 0001
	}
	if(sens_range==40)
	{
		_sens_full_scale = 40;
		_sens_sample_reg = 0x52;
	}
	if(sens_range==80)
	{
		_sens_full_scale = 80;
		_sens_sample_reg = 0x53;
	}
	_current_a_filtered = 0.0f;
	_voltage_v_filtered = 0.0f;
	_current_a = 0.0f;
	_dev_id = SMBUS_BATT_DEV_E::NONE;
}

BATT_PAC17::~BATT_PAC17()
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
BATT_PAC17::init()
{
	int ret = ENOTTY;

	// attempt to initialise I2C bus
	ret = I2C::init();

	if (ret != OK) {
		PX4_ERR("failed to init I2C");
		return ret;

	} else {
		//Find the battery on the bus
		//search();
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
BATT_PAC17::test()
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

int
BATT_PAC17::dumpreg()
{

	for(uint8_t addr = 0;addr<0xff;addr++)
	{
		uint8_t reg=0;
		if (read_reg(addr, reg) == OK) {
			PX4_INFO("%i=0x%x", addr, reg);
		}
		usleep(1);
	}
	return 0;
}

bool
BATT_PAC17::identify()
{
	uint8_t reg=0;
	if (read_reg(BATT_PAC17_REG_PID, reg) == OK) {
		if(reg>0){
			uint8_t manufacturer=0;
			if (read_reg(BATT_PAC17_REG_MID, manufacturer) == OK) {
				if(manufacturer==0x5d) //microchip?
				{
					if(reg==0x57){
						_dev_id = SMBUS_BATT_DEV_E::PAC1720;
						PX4_INFO("PAC1720 found at 0x%x", get_device_address());
						return true;
					}
					if(reg==0x58){
						_dev_id = SMBUS_BATT_DEV_E::PAC1710;
						PX4_INFO("PAC1710 found at 0x%x", get_device_address());
						return true;
					}
				}
			}
		} else
		{
			PX4_INFO("dev found at 0x%x PID=0x%x", get_device_address(), reg);
			dumpreg();
		}
	}
	return false;
}

int
BATT_PAC17::search()
{
	bool found_slave = false;
	int16_t orig_addr = get_device_address();

	// search through all valid SMBus addresses
	for (uint8_t i = BATT_PAC17_ADDR_MIN; i < BATT_PAC17_ADDR_MAX; i++) {
		set_device_address(i);
		if(identify())
		{
			found_slave = true;
			//break;
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
BATT_PAC17::probe()
{
	// always return OK to ensure device starts
	return OK;
}

void
BATT_PAC17::start()
{
	_startupDelay = 100;
	// schedule a cycle to start things
	work_queue(HPWORK, &_work, (worker_t)&BATT_PAC17::cycle_trampoline, this, 1);
}

void
BATT_PAC17::stop()
{
	work_cancel(HPWORK, &_work);
}

void
BATT_PAC17::cycle_trampoline(void *arg)
{
	BATT_PAC17 *dev = (BATT_PAC17 *)arg;

	dev->cycle();
}
bool
BATT_PAC17::try_read_data(battery_status_s &new_report, uint64_t now){
	// temporary variable for storing SMBUS reads
	uint8_t regval_H,regval_L;
	int result = -1;


//#define BATT_PAC17_CONVERSION_RATE 0x
//	write_reg(BATT_PAC17_REG_CONVERSION_RATE, BATT_PAC17_CONVERSION_RATE);
	write_reg(BATT_PAC17_REG_VOLT_SAMPLE_CONF, 0x88);
	write_reg(BATT_PAC17_REG_SENS1_SAMPLE_CONF, _sens_sample_reg); // first channel range (10-80mV)
	if(_dev_id == SMBUS_BATT_DEV_E::PAC1720) write_reg(BATT_PAC17_REG_SENS2_SAMPLE_CONF, _sens_sample_reg); // second channel range

	if (read_reg(BATT_PAC17_REG_VOLT_CH1_H, regval_H) == OK) {
		// read data from sensor
		memset(&new_report,0,sizeof(new_report));
		new_report.timestamp = now;

		result = read_reg(BATT_PAC17_REG_VOLT_CH1_L, regval_L);

		uint16_t voltage = (((uint16_t)regval_H)<<3) | ((uint16_t)regval_L>>5);
		// convert millivolts to volts
		_voltage_v = ((float)voltage*19.53125f) / 1000.0f;
		_voltage_v_filtered = _voltage_v*0.06f + _voltage_v_filtered*0.94f; /* voltage filter */
		// read current
		if ((read_reg(BATT_PAC17_REG_SENS_CH1_H, regval_H) == OK) &&
			(read_reg(BATT_PAC17_REG_SENS_CH1_L, regval_L) == OK) ){
			int16_t current = 0;
			if(regval_H&0x80) // sign bit
			{ // negative
				current = 0xf000 | (((int16_t)regval_H)<<4) | ((int16_t)regval_L>>4);
			} else
			{
				current = (((int16_t)regval_H)<<4) | ((int16_t)regval_L>>4);
			}
			_current_a = ((float)_sens_full_scale/_sens_resistor)*((float)current)/2047.0f;
			_current_a_filtered = _current_a*0.06f + _current_a_filtered*0.94f; /* current filter */
		}

		// calculate total discharged amount
		_discharged_mah = _discharged_mah + _current_a*1000.0f/3600.0f*BATT_PAC17_MEASUREMENT_INTERVAL_US/1000000.0f;
		vehicle_control_mode_poll();
		actuator_controls_s ctrl;
		orb_copy(ORB_ID(actuator_controls_0), _actuator_ctrl_0_sub, &ctrl);

		_battery.updateBatteryStatus(now, _voltage_v, _current_a,
				_voltage_v>2.0f, true , 0,
				ctrl.control[actuator_controls_s::INDEX_THROTTLE],
				_armed, &new_report);
		new_report.voltage_filtered_v = _voltage_v_filtered;
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

		if(_dev_id == SMBUS_BATT_DEV_E::PAC1720)
		{
			result  = read_reg(BATT_PAC17_REG_VOLT_CH2_H, regval_H) == OK;
			result &= read_reg(BATT_PAC17_REG_VOLT_CH2_L, regval_L) == OK;
			if(result)
			{
				uint16_t voltage2 = (((uint16_t)regval_H)<<3) + (regval_L>>5);
				// convert millivolts to volts
				_voltage2 = ((float)voltage2*19.53125f) / 1000.0f;
				new_report.temperature = _voltage2; // hack to publish second channel
			}
		}

		return true;
	}
	return false;
}

void
BATT_PAC17::cycle()
{
	// get current time
	uint64_t now = hrt_absolute_time();

	if(_dev_id==SMBUS_BATT_DEV_E::NONE)
	{
		identify();
	}

	if(_dev_id!=SMBUS_BATT_DEV_E::NONE)
	{
		battery_status_s new_report = {};
		if(try_read_data(new_report, now)){
			// publish to orb
			if (_batt_topic != nullptr) {
				if(_startupDelay<=0)
				{
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
				_batt_topic = orb_advertise(_batt_orb_id, &new_report);
				if (_batt_topic == nullptr) {
					PX4_ERR("ADVERT FAIL");
				}
			}

			// copy report for test()
			_last_report = new_report;
			// record we are working
			_enabled = true;
		} else {
			if (_last_report.connected)
			{
				_last_report.connected=0;
				if (_batt_topic != nullptr) {
					orb_publish(_batt_orb_id, _batt_topic, &_last_report); // report lost connection to battery
				}
			}
		}
	}

	// schedule a fresh cycle call when the measurement is done
	work_queue(HPWORK, &_work, (worker_t)&BATT_PAC17::cycle_trampoline, this,
		   USEC2TICK(BATT_PAC17_MEASUREMENT_INTERVAL_US));
}


int
BATT_PAC17::read_reg(uint8_t reg, uint8_t &val)
{
	uint8_t buff[2];	// 2 bytes of data

	// read from register
	int ret = transfer(&reg, 1, buff, 2);

	val = buff[0];
	// return success or failure
	return ret;
}

int
BATT_PAC17::write_reg(uint8_t reg, uint8_t val)
{
	uint8_t buff[2];  // reg + 1 bytes of data

	buff[0] = reg;
	buff[1] = (uint8_t)val;

	// write bytes to register
	int ret = transfer(buff, 2, nullptr, 0);
	if (ret != OK) {
		PX4_ERR("Reg 0x%x write error", reg);
	}
	// return success or failure
	return ret;
}

uint16_t
BATT_PAC17::convert_twos_comp(uint16_t val)
{
	if ((val & 0x8000) == 0x8000) {
		uint16_t tmp;
		tmp = ~val;
		tmp = tmp + 1;
		return tmp;
	}

	return val;
}
/*
uint8_t
BATT_PAC17::read_block(uint8_t reg, uint8_t *data, uint8_t max_len, bool append_zero)
{
	uint8_t buff[max_len + 2];  // buffer to hold results

	// read bytes including PEC
	int ret = transfer(&reg, 1, buff, max_len + 2);

	// return zero on failure
	if (ret != OK) {
		return 0;
	}

	// get length
	uint8_t bufflen = buff[0];

	// sanity check length returned by smbus
	if (bufflen == 0 || bufflen > max_len) {
		return 0;
	}

	// check PEC
	uint8_t pec = get_PEC(reg, true, buff, bufflen + 1);

	if (pec != buff[bufflen + 1]) {
		return 0;
	}

	// copy data
	memcpy(data, &buff[1], bufflen);

	// optionally add zero to end
	if (append_zero) {
		data[bufflen] = '\0';
	}

	// return success
	return bufflen;
}

uint8_t
BATT_PAC17::write_block(uint8_t reg, uint8_t *data, uint8_t len)
{
	uint8_t buff[len + 3];  // buffer to hold results

	usleep(1);

	buff[0] = reg;
	buff[1] = len;
	memcpy(&buff[2], data, len);
	buff[len + 2] = get_PEC(reg, false, &buff[1],  len + 1); // Append PEC

	// send bytes
	int ret = transfer(buff, len + 3, nullptr, 0);

	// return zero on failure
	if (ret != OK) {
		PX4_DEBUG("Block write error");
		return 0;
	}

	// return success
	return len;
}
*/

/* read arming state */
void BATT_PAC17::vehicle_control_mode_poll()
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
batt_pac17_usage()
{
	PX4_INFO("missing command: try 'start', 'test', 'stop', 'search'");
	PX4_INFO("options:");
	PX4_INFO("    -b i2cbus (%d)", BATT_PAC17_I2C_BUS);
	PX4_INFO("    -a addr (0x%x)", BATT_PAC17_ADDR);
	PX4_INFO("    -r sense resistor (%.2fmOhm)",(double)BATT_PAC17_SENS_R);
	PX4_INFO("    -s full range sense voltage (%imV) 10,20,40,80",BATT_PAC17_SENS_RANGE);
}



int
batt_pac17xx_main(int argc, char *argv[])
{
	int i2cdevice = BATT_PAC17_I2C_BUS;
	int batt_pac17adr = BATT_PAC17_ADDR; // 7bit address
	float	resistor = 0;
	int range = 0;

	int myoptind = 1;
	const char *myoptarg = nullptr;

	int ch;
	while ((ch = px4_getopt(argc, argv, "a:b:r:s:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'a':
			batt_pac17adr = strtol(myoptarg, nullptr, 0);
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

		default:
			batt_pac17_usage();
			return 0;
		}
	}

	if (myoptind >= argc) {
		batt_pac17_usage();
		return 1;
	}

	const char *verb = argv[myoptind];

	if (!strcmp(verb, "start")) {
		if (g_batt_pac17 != nullptr) {
			PX4_ERR("already started");
			return 1;

		} else {
			// create new global object
			g_batt_pac17 = new BATT_PAC17(i2cdevice, batt_pac17adr, resistor, range);

			if (g_batt_pac17 == nullptr) {
				PX4_ERR("new failed");
				return 1;
			}

			if (OK != g_batt_pac17->init()) {
				delete g_batt_pac17;
				g_batt_pac17 = nullptr;
				PX4_ERR("init failed");
				return 1;
			}
		}

		return 0;
	}

	// need the driver past this point
	if (g_batt_pac17 == nullptr) {
		PX4_INFO("not started");
		batt_pac17_usage();
		return 1;
	}

	if (!strcmp(verb, "test")) {
		g_batt_pac17->test();
		return 0;
	}

	if (!strcmp(verb, "stop")) {
		delete g_batt_pac17;
		g_batt_pac17 = nullptr;
		return 0;
	}

	if (!strcmp(verb, "search")) {
		if(g_batt_pac17->search()==OK)
		{
			//g_batt_pac17->dumpreg();
			return 0;
		}
		return 1;
	}


	batt_pac17_usage();
	return 0;
}
