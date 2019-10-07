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
 * @file temp_lm75.cpp
 *
 * Driver for a temperature sensor (I2C).
 *
 * @author Randy Mackay <rmackay9@yahoo.com>
 * @author Alex Klimaj <alexklimaj@gmail.com>
 * @author Maximilian Laiacker <m.laiacker@germandrones.com>
 */

#include <float.h>
#include <stdio.h>
#include <string.h>

#include <drivers/device/i2c.h>
#include <drivers/device/ringbuffer.h>
#include <drivers/drv_hrt.h>
#include <px4_config.h>
#include <px4_workqueue.h>
#include <px4_getopt.h>
#include <perf/perf_counter.h>
#include <uORB/topics/debug_key_value.h>
#include <uORB/uORB.h>
#include <mathlib/mathlib.h>

#define TEMP_LM75_ADDR_MIN             0x00	///< lowest possible address
#define TEMP_LM75_ADDR_MAX             0x7F	///< highest possible address

#define TEMP_LM75_MEASUREMENT_INTERVAL_US	(200000)	///< time in microseconds,
#define TEMP_LM75_TIMEOUT_US			(10000000)	///< timeout

#ifndef CONFIG_SCHED_WORKQUEUE
# error This requires CONFIG_SCHED_WORKQUEUE.
#endif

#define TEMP_LM75_ADDR			0x4f //default
#define TEMP_LM75_I2C_BUS		1


#define TEMP_LM75_REG_CONFIG			0x01
#define TEMP_LM75_REG_TEMPERATURE		0x00


class TEMP_LM75 : public device::I2C
{
public:
	TEMP_LM75(int bus = PX4_I2C_BUS_EXPANSION, uint16_t temp_lm75_addr = TEMP_LM75_ADDR, const char* name=NULL);
	virtual ~TEMP_LM75();

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

	bool			try_read_data(debug_key_value_s &new_report, uint64_t now);
	/**
	 * Read a word from specified register
	 */
	int			read_reg(uint8_t reg, uint16_t &val);
	int			read_reg8(uint8_t reg, uint8_t &val);
	int			read(uint16_t &val);

	/**
	 * Write a word to specified register
	 */
	int			write_reg(uint8_t reg, uint16_t val);

	/**
	 * Convert from 2's compliment to decimal
	 * @return the absolute value of the input in decimal
	 */
	uint16_t	convert_twos_comp(uint16_t val);

	// internal variables
	bool			_enabled;	///< true if we have successfully connected to battery
	work_s			_work{};		///< work queue for scheduling reads

	float			_temperature_C;
	struct debug_key_value_s _last_report;
	char			_name[10];

	orb_advert_t	_temp_topic;	///< uORB
	orb_id_t		_temp_orb_id;	///< uORB
	uint64_t		_start_time;	///< system time we first attempt to communicate with battery

	int 			_startupDelay; ///< prevent publish voltage before filter converged
};

namespace
{
TEMP_LM75 *g_temp_lm75;	///< device handle. For now, we only support one TEMP_LM75 device
}

void temp_lm75_usage();

extern "C" __EXPORT int temp_lm75_main(int argc, char *argv[]);


TEMP_LM75::TEMP_LM75(int bus, uint16_t temp_lm75_addr, const char* name) :
	I2C("temp_lm75", "/dev/temp_lm750", bus, temp_lm75_addr, 400000),
	_enabled(false),
	_last_report{},
	_temp_topic(nullptr),
	_temp_orb_id(nullptr),
	_start_time(0)
{
	// capture startup time
	_start_time = hrt_absolute_time();
	_startupDelay = 100;
	memset(_name,0,sizeof(_name));
	if(name)
	{
		size_t len = strlen(name);
		if(len>sizeof(_name))
		{
			len = sizeof(_name);
		}
		memcpy(_name, name,len);
		_name[sizeof(_name)-1] = 0;
	} else
	{
		snprintf(_name,sizeof(_name),"temp0x%02x",temp_lm75_addr);
	}
}

TEMP_LM75::~TEMP_LM75()
{
	// make sure we are truly inactive
	stop();
	if(_temp_topic!=nullptr)
	{
		orb_unadvertise(_temp_topic);
	}
}

int
TEMP_LM75::init()
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
	_temp_orb_id = ORB_ID(debug_key_value);
	return ret;
}

int
TEMP_LM75::test()
{
	int sub = orb_subscribe(ORB_ID(debug_key_value));
	bool updated = false;
	struct debug_key_value_s status;
	uint64_t start_time = hrt_absolute_time();

	// loop for 3 seconds
	while ((hrt_absolute_time() - start_time) < 3000000) {

		// display new info that has arrived from the orb
		orb_check(sub, &updated);

		if (updated) {
			if (orb_copy(ORB_ID(debug_key_value), sub, &status) == OK) {
				print_message(status);
			}
		}

		// sleep for 0.2 seconds
		usleep(200000);
	}

	return OK;
}

int
TEMP_LM75::dumpreg()
{

	for(uint8_t addr = 0;addr<0x03;addr++)
	{
		uint16_t reg=0;
		if (read_reg(addr, reg) == OK) {
			PX4_INFO("%i=0x%x", addr, reg);
		}
		usleep(1);
	}
	return 0;
}

bool
TEMP_LM75::identify()
{
	uint16_t reg=0;
	bool result=false;
	if (read_reg(TEMP_LM75_REG_TEMPERATURE, reg) == OK) {
		if(reg!=0){
			PX4_INFO("LM75 temp: %i 0x%x", reg, reg);
			uint16_t reg2=0;
			if (read(reg2) == OK) {
				result = (reg==reg2);
				PX4_INFO("LM75 temp: %i 0x%x", reg2, reg2);
			}
			uint8_t reg_config=0;
			if (read_reg8(TEMP_LM75_REG_CONFIG, reg_config) == OK) {
				if((reg_config&0xe0)!=0) // config[7:5] should be 0
				{
					result = false;
				}
				PX4_INFO("LM75 config: 0x%x", reg_config);
			}
			if (read_reg(2, reg) == OK) {
				PX4_INFO("LM75 Thyst: %i 0x%x", reg, reg);
			}
			if (read_reg(3, reg) == OK) {
				PX4_INFO("LM75 Tos: %i 0x%x", reg, reg);
			}
			if(result)
			{
				PX4_INFO("LM75 found at 0x%x", get_device_address());
			}
			return result;
		} else
		{
			PX4_INFO("unknown at 0x%x", get_device_address());
		}
	}
	return result;
}

int
TEMP_LM75::search()
{
	bool found_slave = false;
	int16_t orig_addr = get_device_address();

	// search through all valid SMBus addresses
	for (uint8_t i = TEMP_LM75_ADDR_MIN; i < TEMP_LM75_ADDR_MAX; i++) {
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
		PX4_INFO("temp sensor connected");

	} else {
		PX4_ERR("No sensor found.");
	}

	return OK;
}

int
TEMP_LM75::probe()
{
	// always return OK to ensure device starts
	return OK;
}

void
TEMP_LM75::start()
{
	_startupDelay = 100;
	// schedule a cycle to start things
	work_queue(HPWORK, &_work, (worker_t)&TEMP_LM75::cycle_trampoline, this, 1);
}

void
TEMP_LM75::stop()
{
	work_cancel(HPWORK, &_work);
}

void
TEMP_LM75::cycle_trampoline(void *arg)
{
	TEMP_LM75 *dev = (TEMP_LM75 *)arg;

	dev->cycle();
}
bool
TEMP_LM75::try_read_data(debug_key_value_s &new_report, uint64_t now){
	uint16_t reg;
	if (read_reg(TEMP_LM75_REG_TEMPERATURE, reg) == OK) {
		int16_t temp_125c;
		if(0x8000&reg){// sign bit, negative
			temp_125c = 0xf800 | (reg>>5);
		} else
		{
			temp_125c = (reg>>5);
		}
		_temperature_C = temp_125c*0.125;
		new_report.timestamp = now;
		new_report.timestamp_ms = (uint32_t)(now/1000);
		new_report.value = _temperature_C;
		memcpy(new_report.key, _name, sizeof(new_report.key));
		return true;
	}
	return false;
}

void
TEMP_LM75::cycle()
{
	// get current time
	uint64_t now = hrt_absolute_time();

	if(1)
	{
		struct debug_key_value_s new_report = {};
		if(try_read_data(new_report, now)){
			// publish to orb
			if(_startupDelay<=0)
			{
				if (_temp_topic != nullptr) {
						orb_publish(_temp_orb_id, _temp_topic, &new_report);
				} else {
					_temp_topic = orb_advertise(_temp_orb_id, &new_report);
					if (_temp_topic == nullptr) {
						PX4_ERR("ADVERT FAIL");
					}
				}
			} else
			{
				_startupDelay--;
			}

			// copy report for test()
			_last_report = new_report;
			// record we are working
			_enabled = true;
		}
	}

	// schedule a fresh cycle call when the measurement is done
	work_queue(HPWORK, &_work, (worker_t)&TEMP_LM75::cycle_trampoline, this,
		   USEC2TICK(TEMP_LM75_MEASUREMENT_INTERVAL_US));
}


int
TEMP_LM75::read_reg(uint8_t reg, uint16_t &val)
{
	uint8_t buff[2];
	// read from register
	int ret = transfer(&reg, 1, buff, 2);

	val = (((uint16_t)buff[0])<<8)|(uint16_t)buff[1];

	// return success or failure
	return ret;
}

int
TEMP_LM75::read(uint16_t &val)
{
	uint8_t buff[2];
	uint8_t reg=0;
	// read from register
	int ret = transfer(&reg, 0, buff, 2);

	val = (((uint16_t)buff[0])<<8)|(uint16_t)buff[1];

	// return success or failure
	return ret;
}

int
TEMP_LM75::read_reg8(uint8_t reg, uint8_t &val)
{
	// read from register
	int ret = transfer(&reg, 1, (uint8_t*)&val, 1);

	// return success or failure
	return ret;
}

int
TEMP_LM75::write_reg(uint8_t reg, uint16_t val)
{
	uint8_t buff[3];  // reg + 1 bytes of data

	buff[0] = reg;
	buff[1] = (uint8_t)val>>8;
	buff[0] = (uint8_t)val&0xff;

	// write bytes to register
	int ret = transfer(buff, 3, nullptr, 0);
	if (ret != OK) {
		PX4_ERR("Reg 0x%x write error", reg);
	}
	// return success or failure
	return ret;
}

uint16_t
TEMP_LM75::convert_twos_comp(uint16_t val)
{
	if ((val & 0x8000) == 0x8000) {
		uint16_t tmp;
		tmp = ~val;
		tmp = tmp + 1;
		return tmp;
	}

	return val;
}

///////////////////////// shell functions ///////////////////////

void
temp_lm75_usage()
{
	PX4_INFO("missing command: try 'start', 'test', 'stop', 'search', 'ident'");
	PX4_INFO("options:");
	PX4_INFO("    -b i2cbus (%d)", TEMP_LM75_I2C_BUS);
	PX4_INFO("    -a addr (0x%x)", TEMP_LM75_ADDR);
	PX4_INFO("    -n \"name\"");
}



int
temp_lm75_main(int argc, char *argv[])
{
	int i2cdevice = TEMP_LM75_I2C_BUS;
	int temp_lm75adr = TEMP_LM75_ADDR; // 7bit address

	int myoptind = 1;
	const char *myoptarg = nullptr;
	const char* name = NULL;

	int ch;
	while ((ch = px4_getopt(argc, argv, "a:b:n:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'a':
			temp_lm75adr = strtol(myoptarg, nullptr, 0);
			break;

		case 'b':
			i2cdevice = strtol(myoptarg, nullptr, 0);
			break;

		case 'n':
			name = myoptarg;
			break;

		default:
			temp_lm75_usage();
			return 0;
		}
	}

	if (myoptind >= argc) {
		temp_lm75_usage();
		return 1;
	}

	const char *verb = argv[myoptind];

	if (!strcmp(verb, "start")) {
		if (g_temp_lm75 != nullptr) {
			PX4_ERR("already started");
			return 1;

		} else {
			// create new global object
			g_temp_lm75 = new TEMP_LM75(i2cdevice, temp_lm75adr, name);

			if (g_temp_lm75 == nullptr) {
				PX4_ERR("new failed");
				return 1;
			}

			if (OK != g_temp_lm75->init()) {
				delete g_temp_lm75;
				g_temp_lm75 = nullptr;
				PX4_ERR("init failed");
				return 1;
			}
		}

		return 0;
	}

	// need the driver past this point
	if (g_temp_lm75 == nullptr) {
		PX4_INFO("not started");
		temp_lm75_usage();
		return 1;
	}

	if (!strcmp(verb, "test")) {
		g_temp_lm75->test();
		return 0;
	}

	if (!strcmp(verb, "ident")) {
		g_temp_lm75->identify();
		return 0;
	}

	if (!strcmp(verb, "stop")) {
		delete g_temp_lm75;
		g_temp_lm75 = nullptr;
		return 0;
	}

	if (!strcmp(verb, "search")) {
		if(g_temp_lm75->search()==OK)
		{
			//g_temp_lm75->dumpreg();
			return 0;
		}
		return 1;
	}


	temp_lm75_usage();
	return 0;
}
