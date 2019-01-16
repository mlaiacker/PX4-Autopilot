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
#include <uORB/uORB.h>

#define BATT_PAC17_ADDR_MIN             0x00	///< lowest possible address
#define BATT_PAC17_ADDR_MAX             0xFF	///< highest possible address

#define BATT_PAC17_MEASUREMENT_INTERVAL_US	(1000000 / 10)	///< time in microseconds, measure at 10Hz
#define BATT_PAC17_TIMEOUT_US			(10000000)	///< timeout looking for battery 10seconds after startup
#define BATT_PAC17_SENS_RANGE			(80)   // mV
#define BATT_PAC17_SENS_R				(0.1f) // mOhm

#ifndef CONFIG_SCHED_WORKQUEUE
# error This requires CONFIG_SCHED_WORKQUEUE.
#endif

#define BATT_PAC17_ADDR			0x4c //default 0x98 in 8 bit
#define BATT_PAC17_I2C_BUS		2


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



	// internal variables
	bool			_enabled;	///< true if we have successfully connected to battery
	work_s			_work{};		///< work queue for scheduling reads

	battery_status_s _last_report;	///< last published report, used for test()

	orb_advert_t	_batt_topic;	///< uORB battery topic
	orb_id_t		_batt_orb_id;	///< uORB battery topic ID

	uint64_t		_start_time;	///< system time we first attempt to communicate with battery
	uint16_t		_batt_capacity;	///< battery's design capacity in mAh (0 means unknown)
	uint16_t		_batt_startup_capacity;	///< battery's remaining capacity on startup
	uint16_t		_sens_full_scale; ///< current sense full range voltage 10,20,40,80 mV
	uint8_t			_sens_sample_reg; ///< sample scale reidter value
	float			_sens_resistor;	///< current sense resistor value in mOhm
	float 			_crit_thr;	///< Critical battery threshold param
	float 			_low_thr;	///< Low battery threshold param
	float 			_emergency_thr;		///< Emergency battery threshold param
	enum SMBUS_BATT_DEV_E
	{
		NONE=0,
		BQ40Z50,
		PAC1710,
		PAC1720,
	};
	SMBUS_BATT_DEV_E			dev_id;	// 0 = BQ40Z50, 1 = PAC1710, 2=PAC1720
};

namespace
{
BATT_PAC17 *g_batt_pac17;	///< device handle. For now, we only support one BATT_PAC17 device
}

void batt_pac17_usage();

extern "C" __EXPORT int batt_pac17xx_main(int argc, char *argv[]);


BATT_PAC17::BATT_PAC17(int bus, uint16_t batt_pac17_addr, float sens_resistor, uint16_t sens_range) :
	I2C("batt_pac17", "/dev/batt_pac170", bus, batt_pac17_addr, 100000),
	_enabled(false),
	_last_report{},
	_batt_topic(nullptr),
	_batt_orb_id(nullptr),
	_start_time(0),
	_batt_capacity(0),
	_batt_startup_capacity(0),
	_sens_full_scale(BATT_PAC17_SENS_RANGE),
	_sens_sample_reg(0x53),
	_sens_resistor(BATT_PAC17_SENS_R),
	_crit_thr(0.0f),
	_low_thr(0.0f),
	_emergency_thr(0.0f)
{
	// capture startup time
	_start_time = hrt_absolute_time();

	if(sens_resistor>0){
		_sens_resistor=sens_resistor;
	}
	if(sens_range==10)
	{
		_sens_full_scale = 10;
		_sens_sample_reg = 0x50;
	}
	if(sens_range==20)
	{
		_sens_full_scale = 20;
		_sens_sample_reg = 0x51;
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
}

BATT_PAC17::~BATT_PAC17()
{
	// make sure we are truly inactive
	stop();
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
		search();

		// start work queue
		start();
	}

	// init orb id
	_batt_orb_id = ORB_ID(battery_status);

	return ret;
}

int
BATT_PAC17::test()
{
	int sub = orb_subscribe(ORB_ID(battery_status));
	bool updated = false;
	struct battery_status_s status;
	uint64_t start_time = hrt_absolute_time();

	// loop for 3 seconds
	while ((hrt_absolute_time() - start_time) < 3000000) {

		// display new info that has arrived from the orb
		orb_check(sub, &updated);

		if (updated) {
			if (orb_copy(ORB_ID(battery_status), sub, &status) == OK) {
				print_message(status);
			}
		}

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

int
BATT_PAC17::search()
{
	bool found_slave = false;
	int16_t orig_addr = get_device_address();
	uint8_t reg;

	// search through all valid SMBus addresses
	for (uint8_t i = BATT_PAC17_ADDR_MIN; i < BATT_PAC17_ADDR_MAX; i++) {
		set_device_address(i);

		reg = 0;
		if (read_reg(BATT_PAC17_REG_PID, reg) == OK) {
			if(reg>0){
				uint8_t manufacturer;
				if (read_reg(BATT_PAC17_REG_MID, manufacturer) == OK) {
					if(manufacturer==0x5d) //microchip?
					{
						if(reg==0x57){
							dev_id = SMBUS_BATT_DEV_E::PAC1720;
							found_slave = true;
							PX4_INFO("PAC1720 found at 0x%x", get_device_address());
							break;
						}
						if(reg==0x58){
							dev_id = SMBUS_BATT_DEV_E::PAC1710;
							found_slave = true;
							PX4_INFO("PAC1710 found at 0x%x", get_device_address());
							break;
						}
					}
				} else
				{
					PX4_INFO("dev found at 0x%x PID=0x%x", get_device_address(), reg);
				}
			}
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
		PX4_INFO("No current monitor found.");
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

void
BATT_PAC17::cycle()
{
	// get current time
	uint64_t now = hrt_absolute_time();

	// exit without rescheduling if we have failed to find a battery after 10 seconds
	if (!_enabled && (now - _start_time > BATT_PAC17_TIMEOUT_US)) {
		PX4_INFO("did not find current monitor");
		return;
	}


	// temporary variable for storing SMBUS reads
	uint8_t regval_H,regval_L;
	int result = -1;


	// read battery threshold params on startup
	if (_crit_thr < 0.01f) {
		param_get(param_find("BAT_CRIT_THR"), &_crit_thr);
	}

	if (_low_thr < 0.01f) {
		param_get(param_find("BAT_LOW_THR"), &_low_thr);
	}

	if (_emergency_thr < 0.01f) {
		param_get(param_find("BAT_EMERGEN_THR"), &_emergency_thr);
	}

	// read data from sensor
	battery_status_s new_report = {};
	memset(&new_report,0,sizeof(new_report));

	// set time of reading
	new_report.timestamp = now;
//#define BATT_PAC17_CONVERSION_RATE 0x
//	write_reg(BATT_PAC17_REG_CONVERSION_RATE, BATT_PAC17_CONVERSION_RATE);
	write_reg(BATT_PAC17_REG_VOLT_SAMPLE_CONF, 0x33); // 8 averages
	write_reg(BATT_PAC17_REG_SENS1_SAMPLE_CONF, _sens_sample_reg);
	write_reg(BATT_PAC17_REG_SENS2_SAMPLE_CONF, _sens_sample_reg);

	if (read_reg(BATT_PAC17_REG_VOLT_CH1_H, regval_H) == OK) {

		result = read_reg(BATT_PAC17_REG_VOLT_CH1_L, regval_L);
		new_report.connected =(result== OK);

		uint16_t voltage = (((uint16_t)regval_H)<<3) + (regval_L>>5);

		// convert millivolts to volts
		new_report.voltage_v = ((float)voltage*19.53125f) / 1000.0f;
		new_report.voltage_filtered_v = new_report.voltage_v*0.1f + _last_report.voltage_filtered_v*0.9f;

		// read current
		if ((read_reg(BATT_PAC17_REG_SENS_CH1_H, regval_H) == OK) &&
			(read_reg(BATT_PAC17_REG_SENS_CH1_L, regval_L) == OK) ){
			int16_t current = (((uint16_t)regval_H)<<4) + (regval_L>>4);
			new_report.current_a = ((float)_sens_full_scale/_sens_resistor)*(float)current/2047.0f;
			new_report.current_filtered_a = new_report.current_a*0.1f + _last_report.current_filtered_a*0.9f;
		}

		new_report.average_current_a = new_report.current_filtered_a*0.1f+ _last_report.average_current_a*0.9f;
		new_report.run_time_to_empty = 0;
		new_report.average_time_to_empty = 0;
		_batt_capacity = (uint16_t)0;
		new_report.remaining = 0;

		// calculate total discharged amount
		new_report.discharged_mah = _last_report.discharged_mah + new_report.current_a*BATT_PAC17_MEASUREMENT_INTERVAL_US/1000000.0f;

		result  = read_reg(BATT_PAC17_REG_VOLT_CH2_H, regval_H) == OK;
		result &= read_reg(BATT_PAC17_REG_VOLT_CH2_L, regval_L) == OK;
		if(result)
		{
			uint16_t voltage2 = (((uint16_t)regval_H)<<3) + (regval_L>>5);
			// convert millivolts to volts
			new_report.temperature = ((float)voltage2*19.53125f) / 1000.0f;

			if(voltage2>10 && voltage>10)
			{
				new_report.warning = battery_status_s::BATTERY_WARNING_NONE;
				if((float)fabs((float)voltage - (float)voltage2) > ((voltage+voltage2)*0.5f)/5.0f )
				{
					new_report.warning = battery_status_s::BATTERY_WARNING_FAILED; // difference too high
				}
			}
		}

/*
		//Check if remaining % is out of range
		if ((new_report.remaining > 1.00f) || (new_report.remaining <= 0.00f)) {
			new_report.warning = battery_status_s::BATTERY_WARNING_EMERGENCY;
		}

		//Check if discharged amount is greater than the starting capacity
		else if (new_report.discharged_mah > (float)_batt_startup_capacity) {
			new_report.warning = battery_status_s::BATTERY_WARNING_EMERGENCY;
		}

		// propagate warning state
		else {
			if (new_report.remaining > _low_thr) {
				new_report.warning = battery_status_s::BATTERY_WARNING_NONE;

			} else if (new_report.remaining > _crit_thr) {
				new_report.warning = battery_status_s::BATTERY_WARNING_LOW;

			} else if (new_report.remaining > _emergency_thr) {
				new_report.warning = battery_status_s::BATTERY_WARNING_CRITICAL;

			} else {
				new_report.warning = battery_status_s::BATTERY_WARNING_EMERGENCY;
			}
		}
*/
		new_report.capacity = _batt_capacity;
		new_report.cycle_count = 0;
		new_report.serial_number = 0;

		// publish to orb
		if (_batt_topic != nullptr) {
			orb_publish(_batt_orb_id, _batt_topic, &new_report);

		} else {
			_batt_topic = orb_advertise(_batt_orb_id, &new_report);

			if (_batt_topic == nullptr) {
				PX4_ERR("ADVERT FAIL");
				return;
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
		PX4_DEBUG("Register 0x%x write error", reg);
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
			PX4_INFO("resistor %fmOhm", (double)resistor);
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
		g_batt_pac17->search();
		g_batt_pac17->dumpreg();
		return 0;
	}


	batt_pac17_usage();
	return 0;
}
