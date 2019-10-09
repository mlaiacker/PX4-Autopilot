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
 * @file pca9536xx.cpp
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


#define PCA9536_MEASUREMENT_INTERVAL_US	(100000)	///< time in microseconds, measure at 10Hz
#define PCA9536_TIMEOUT_US			(10000000)	///< timeout looking for battery 10seconds after startup

#ifndef CONFIG_SCHED_WORKQUEUE
# error This requires CONFIG_SCHED_WORKQUEUE.
#endif

#define PCA9536_ADDR			0x41 //default 0x80 in 8 bit
#define PCA9536_I2C_BUS		1

#define PCA9536_REG_INPUT		0x0
#define PCA9536_REG_OUTPUT		0x1
#define PCA9536_REG_INVERSION	0x2
#define PCA9536_REG_DDR			0x3 // gpio direction register 1=input 0=output


class PCA9536 : public device::I2C
{
public:
	PCA9536(int bus = PX4_I2C_BUS_EXPANSION);
	virtual ~PCA9536();

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
	bool		identify();

	bool		setPort(uint8_t value);
	bool		setDirection(uint8_t value);
	bool		getPort(uint8_t &value);

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


	void 			vehicle_control_mode_poll();

	// internal variables
	bool			_enabled;	///< true if we have successfully connected to device
	work_s			_work{};		///< work queue for scheduling reads

	int				_vcontrol_mode_sub{-1};		/**< vehicle control mode subscription */
	bool			_armed{false};

	uint64_t		_start_time;	///< system time we first attempt to communicate with device
};

namespace
{
PCA9536 *g_pca9536;	///< device handle. For now, we only support one PCA9536 device
}

void pca9536_usage();

extern "C" __EXPORT int pca9536_main(int argc, char *argv[]);


PCA9536::PCA9536(int bus) :
	I2C("pca9536", nullptr, bus, PCA9536_ADDR, 400000),
	_enabled(false),
	_start_time(0)
{
	// capture startup time
	_start_time = hrt_absolute_time();
}

PCA9536::~PCA9536()
{
	// make sure we are truly inactive
	stop();
	orb_unsubscribe(_vcontrol_mode_sub);
}

int
PCA9536::init()
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

	/* needed to read arming status */
	_vcontrol_mode_sub = orb_subscribe(ORB_ID(vehicle_control_mode));
	return ret;
}

int
PCA9536::test()
{
	uint64_t start_time = hrt_absolute_time();

	PX4_INFO("GPIO 3 2 1 0");
	// loop for 3 seconds
	while ((hrt_absolute_time() - start_time) < 3000000) {
		uint8_t port;
		if(getPort(port))
		{
			PX4_INFO("GPIO %d %d %d %d",(port>>3)&1,(port>>2)&1,(port>>1)&1,port&1);
		}
		// sleep for 0.2 seconds
		usleep(200000);
	}

	return OK;
}

bool
PCA9536::identify()
{
	uint8_t reg=0;
	if (read_reg(PCA9536_REG_INPUT, reg) == OK) {
		if(reg>0){
			PX4_INFO("input=0x%02x", reg);
			if (read_reg(PCA9536_REG_OUTPUT, reg) == OK) {
				PX4_INFO("output=0x%02x", reg);
			}
			if (read_reg(PCA9536_REG_INVERSION, reg) == OK) {
				PX4_INFO("inversion=0x%02x", reg);
			}
			if (read_reg(PCA9536_REG_DDR, reg) == OK) {
				PX4_INFO("ddr=0x%02x", reg);
			}
			PX4_INFO("PCA9536 found at 0x%x", get_device_address());
			return true;
		} else {
			PX4_INFO("dev found at 0x%x reg0=0x%x", get_device_address(), reg);
		}
	}
	return false;
}

int
PCA9536::probe()
{
	// always return OK to ensure device starts
	return OK;
}

void
PCA9536::start()
{
	// schedule a cycle to start things
	work_queue(HPWORK, &_work, (worker_t)&PCA9536::cycle_trampoline, this, 1);
}

void
PCA9536::stop()
{
	work_cancel(HPWORK, &_work);
}

void
PCA9536::cycle_trampoline(void *arg)
{
	PCA9536 *dev = (PCA9536 *)arg;

	dev->cycle();
}

void
PCA9536::cycle()
{
	// get current time
	//uint64_t now = hrt_absolute_time();

	// schedule a fresh cycle call when the measurement is done
	work_queue(HPWORK, &_work, (worker_t)&PCA9536::cycle_trampoline, this,
		   USEC2TICK(PCA9536_MEASUREMENT_INTERVAL_US));
}

bool PCA9536::setDirection(uint8_t value)
{
	return write_reg(PCA9536_REG_DDR, value)==OK;
}

bool PCA9536::setPort(uint8_t value)
{
	return write_reg(PCA9536_REG_OUTPUT, value)==OK;
}

bool PCA9536::getPort(uint8_t &value)
{
	return read_reg(PCA9536_REG_INPUT, value)==OK;
}

int
PCA9536::read_reg(uint8_t reg, uint8_t &val)
{
	uint8_t buff[1];
	// read from register
	int ret = transfer(&reg, 1, buff, 1);

	val = buff[0];
	// return success or failure
	return ret;
}

int
PCA9536::write_reg(uint8_t reg, uint8_t val)
{
	uint8_t buff[3];  // reg + 1 bytes of data

	buff[0] = reg;
	buff[1] = (uint8_t)val&0xff;

	// write bytes to register
	int ret = transfer(buff, 2, nullptr, 0);
	if (ret != OK) {
		PX4_ERR("Reg 0x%x write error", reg);
	}
	// return success or failure
	return ret;
}


///////////////////////// shell functions ///////////////////////

void
pca9536_usage()
{
	PX4_INFO("missing command: try 'start', 'test', 'stop', 'write'");
	PX4_INFO("options:");
	PX4_INFO("    -b <i2cbus (%d)>", PCA9536_I2C_BUS);
	PX4_INFO("    -d <direction mask>");
	PX4_INFO("    -v <value>");
}



int
pca9536_main(int argc, char *argv[])
{
	int i2cdevice = PCA9536_I2C_BUS;
	uint8_t	ddr=0;
	uint8_t val=0;

	int myoptind = 1;
	const char *myoptarg = nullptr;

	int ch;
	while ((ch = px4_getopt(argc, argv, "b:d:v:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'd':
			ddr = strtol(myoptarg, nullptr, 0);
			ddr |= 0xf0;
			break;

		case 'b':
			i2cdevice = strtol(myoptarg, nullptr, 0);
			break;

		case 'v':
			val = strtol(myoptarg, nullptr, 0);
			val |= 0xf0;
			break;

		default:
			pca9536_usage();
			return 0;
		}
	}

	if (myoptind >= argc) {
		pca9536_usage();
		return 1;
	}

	const char *verb = argv[myoptind];

	if (!strcmp(verb, "start")) {
		if (g_pca9536 != nullptr) {
			PX4_ERR("already started");
			return 1;
		} else {
			// create new global object
			g_pca9536 = new PCA9536(i2cdevice);

			if (g_pca9536 == nullptr) {
				PX4_ERR("new failed");
				return 1;
			}

			if (OK != g_pca9536->init()) {
				delete g_pca9536;
				g_pca9536 = nullptr;
				PX4_ERR("init failed");
				return 1;
			}
		}

		return 0;
	}

	// need the driver past this point
	if (g_pca9536 == nullptr) {
		PX4_INFO("not started");
		pca9536_usage();
		return 1;
	}

	if (!strcmp(verb, "test")) {
		g_pca9536->test();
		return 0;
	}

	if (!strcmp(verb, "stop")) {
		delete g_pca9536;
		g_pca9536 = nullptr;
		return 0;
	}

	if (!strcmp(verb, "write")) {
		if((ddr!=0))
		{
			if(!g_pca9536->setDirection(ddr))
			{
				PX4_INFO("write direction failed");
				return 1;
			} else {
				PX4_INFO("write direction 0x%x",ddr);
			}
		}
		if(val!=0){
			if(!g_pca9536->setPort(val))
			{
				PX4_INFO("write output failed");
				return 2;
			} else {
				PX4_INFO("write output 0x%x",val);
			}
		}
		g_pca9536->identify();
		return 0;
	}


	pca9536_usage();
	return 0;
}
