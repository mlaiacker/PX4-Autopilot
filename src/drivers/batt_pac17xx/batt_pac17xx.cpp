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
 * @author Maximilian Laiacker <m.laiacker@germandrones.com>
 */

#include <float.h>
#include <stdio.h>
#include <string.h>
#include <ecl/geo/geo.h>

#include <drivers/device/Device.hpp>
#include <mathlib/mathlib.h>
#include <perf/perf_counter.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/i2c_spi_buses.h>

#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/uORB.h>
#include <battery/battery.h>

#include <board_config.h>

#define BATT_PAC17_ADDR_MIN             0x00	///< lowest possible address
#define BATT_PAC17_ADDR_MAX             0x7F	///< highest possible address

#define BATT_PAC17_MEASUREMENT_INTERVAL_US	(50000)	///< time in microseconds, measure at xHz
#define BATT_PAC17_TIMEOUT_US			(5000000)	///< timeout looking for battery xseconds after startup
#define BATT_PAC17_SENS_RANGE			(80)   // mV
#define BATT_PAC17_SENS_R				(0.1f) // mOhm

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

class BATT_PAC17 : public ModuleParams, public I2CSPIDriver<BATT_PAC17>
{
public:
	BATT_PAC17(device::I2C *Device, I2CSPIBusOption bus_option, int bus, uint8_t batt_pac17_addr, float sens_resistor, uint16_t sens_range);
	~BATT_PAC17();

	static I2CSPIDriverBase *instantiate(const BusCLIArguments &cli, const BusInstanceIterator &iterator,
					     int runtime_instance);
	static void print_usage();

	void RunImpl();

	void custom_method(const BusCLIArguments &cli) override;

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

	void 		suspend();

	void 		resume();

protected:
	/**
	 * Check if the device can be contacted
	 */
	virtual int		probe();

private:
	/**
	 * static function that is called by worker queue
	 */
	static void		cycle_trampoline(void *arg);

	/**
	 * perform a read from the battery
	 */
	void			cycle();

	bool			try_read_data(uint64_t now);
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

	void 			vehicle_control_mode_poll();

	// internal variables
	device::I2C*	_interface;
	bool			_enabled;	///< true if we have successfully connected to battery

	float			_discharged_mah_armed; ///< value when we last armed to calc avg current
	float			_current_a_filtered;
	float			_voltage_v;
	float			_voltage_v_filtered;
	float			_current_a;
	float			_voltage2; 	///< for pac1720
	float			_current2; 	///< for pac1720

	int				_sub_status{-1};		/**< vehicle status*/
	bool			_armed;

	uint64_t		_time_arm;	///< system time we last armed
	uint16_t		_sens_full_scale; ///< current sense full range voltage 10,20,40,80 mV
	uint8_t			_sens_sample_reg; ///< sample scale register value
	float			_sens_resistor;	///< current sense resistor value in mOhm
	int 			_startupDelay; ///< prevent publish voltage before filter converged
	float 			_startRemaining; //< remain estimate based on voltage at beginning
	enum PAC17_DEV_E {
		NONE = 0,
		PAC1710,
		PAC1720,
	};
	PAC17_DEV_E			_dev_id;	// PAC1710, PAC1720
	Battery				_battery;	/**< Helper lib to publish battery_status topic. */
};

namespace
{
BATT_PAC17 *g_batt_pac17;	///< device handle. For now, we only support one BATT_PAC17 device
}

extern "C" __EXPORT int batt_pac17xx_main(int argc, char *argv[]);


BATT_PAC17::BATT_PAC17(I2CSPIBusOption bus_option, const int bus, SMBus *interface) :
	ModuleParams(nullptr),
	I2CSPIDriver(MODULE_NAME, px4::device_bus_to_wq(interface->get_device_id()), bus_option, bus),
	_interface(interface),
	_enabled(false),
	_sens_full_scale(BATT_PAC17_SENS_RANGE),
	_sens_sample_reg(0x53),
	_sens_resistor(BATT_PAC17_SENS_R),
	_startRemaining(0),
	_battery(1, this, BATT_PAC17_MEASUREMENT_INTERVAL_US)
{
	// capture startup time
	_startupDelay = 10;
	float sens_resistor = BATT_PAC17_SENS_R;
	int sens_range = BATT_PAC17_SENS_RANGE;
	if (sens_resistor > 0) {
		_sens_resistor = sens_resistor;
	}

	// 0x5x = sign + 11 bits resolution
	if (sens_range == 10) {
		_sens_full_scale = 10;
		_sens_sample_reg = 0x50; // 0101 0000
	}

	if (sens_range == 20) {
		_sens_full_scale = 20;
		_sens_sample_reg = 0x51; // 0101 0001
	}

	if (sens_range == 40) {
		_sens_full_scale = 40;
		_sens_sample_reg = 0x52;
	}

	if (sens_range == 80) {
		_sens_full_scale = 80;
		_sens_sample_reg = 0x53;
	}

	_current_a_filtered = 0.0f;
	_voltage_v_filtered = 0.0f;
	_current_a = 0.0f;
	_armed = false;
	_dev_id = PAC17_DEV_E::NONE;
	_interface->init();
}

BATT_PAC17::~BATT_PAC17()
{
	if (_interface != nullptr) {
		delete _interface;
	}
	orb_unsubscribe(_sub_status);
}

int
BATT_PAC17::init()
{
	int ret = 0;

	// attempt to initialise I2C bus
	_interface->init();
		//Find the ic on the bus
		identify();

		/* needed to read arming status */
		_sub_status = orb_subscribe(ORB_ID(vehicle_control_mode));

		if (_sub_status < 0) {
			PX4_ERR("status sub failed");
		}
		_time_arm = hrt_absolute_time();
		_discharged_mah_armed = 0;

	return ret;
}


void BATT_PAC17::suspend()
{
	ScheduleClear();
}

void BATT_PAC17::resume()
{
	ScheduleOnInterval(BATT_PAC17_MEASUREMENT_INTERVAL_US);
}

int
BATT_PAC17::test()
{
//	uint64_t start_time = hrt_absolute_time();
	PX4_INFO("armed=%i", _armed);
	PX4_INFO("armed_t=%f", (double)_time_arm * 1e-6);
	PX4_INFO("armed_mAh=%f", (double)_discharged_mah_armed);
	//print_message(_last_report);
	/*	// loop for 3 seconds
		while ((hrt_absolute_time() - start_time) < 3000000) {
			print_message(_last_report);
			// sleep for 0.2 seconds
			usleep(200000);
		}*/
	return OK;
}

int
BATT_PAC17::dumpreg()
{
	for (uint8_t addr = 0; addr < 0xff; addr++) {
		uint8_t reg = 0;

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
	uint8_t reg = 0;

	if (read_reg(BATT_PAC17_REG_PID, reg) == OK) {
		if (reg > 0) {
			uint8_t manufacturer = 0;

			if (read_reg(BATT_PAC17_REG_MID, manufacturer) == OK) {
				if (manufacturer == 0x5d) { //microchip?
					if (reg == 0x57) {
						_dev_id = PAC17_DEV_E::PAC1720;
						PX4_INFO("PAC1720 found at 0x%x", _interface->get_device_address());
						return true;
					}

					if (reg == 0x58) {
						_dev_id = PAC17_DEV_E::PAC1710;
						PX4_INFO("PAC1710 found at 0x%x", _interface->get_device_address());
						return true;
					}
				}
			}

		} else {
			PX4_INFO("dev found at 0x%x PID=0x%x", _interface->get_device_address(), reg);
			//dumpreg();
		}
	}

	return false;
}

int
BATT_PAC17::search()
{
	bool found_slave = false;
	int16_t orig_addr = _interface->get_device_address();

	// search through all valid SMBus addresses
	for (uint8_t i = BATT_PAC17_ADDR_MIN; i < BATT_PAC17_ADDR_MAX; i++) {
		_interface->set_device_address(i);

		if (identify()) {
			found_slave = true;
			//break;
		}

		usleep(1);

	}

	if (found_slave == false) {
		// restore original i2c address
		_interface->set_device_address(orig_addr);
	}

	// display completion message
	if (found_slave) {
		PX4_INFO("current monitor connected");

	} else {
		PX4_ERR("No current monitor found.");
	}

	return OK;
}


bool
BATT_PAC17::try_read_data(uint64_t now)
{
	// temporary variable for storing SMBUS reads
	uint8_t regval_H, regval_L;
	int result = -1;

	if (read_reg(BATT_PAC17_REG_VOLT_CH1_H, regval_H) == OK) {
		// read data from sensor
		result = read_reg(BATT_PAC17_REG_VOLT_CH1_L, regval_L);

		uint16_t voltage = (((uint16_t)regval_H) << 3) | ((uint16_t)regval_L >> 5);
		// convert millivolts to volts
		_voltage_v = ((float)voltage * 19.53125f) / 1000.0f;
		_voltage_v_filtered = _voltage_v * 0.06f + _voltage_v_filtered * 0.94f; /* voltage filter */

		// read current
		if ((read_reg(BATT_PAC17_REG_SENS_CH1_H, regval_H) == OK) &&
		    (read_reg(BATT_PAC17_REG_SENS_CH1_L, regval_L) == OK)) {
			int16_t current = 0;

			if (regval_H & 0x80) { // sign bit
				// negative
				current = 0xf000 | (((int16_t)regval_H) << 4) | ((int16_t)regval_L >> 4);

			} else {
				current = (((int16_t)regval_H) << 4) | ((int16_t)regval_L >> 4);
			}

			_current_a = ((float)_sens_full_scale / _sens_resistor) * ((float)current) / 2047.0f;
			_current_a_filtered = _current_a * 0.06f + _current_a_filtered * 0.94f; /* current filter */
		}

		_battery.updateBatteryStatus(now, _voltage_v, _current_a,
					     _voltage_v > 2.0f, battery_status_s::BATTERY_SOURCE_EXTERNAL, 0,	0.0f);

		if (_dev_id == PAC17_DEV_E::PAC1720) {
			result  = read_reg(BATT_PAC17_REG_VOLT_CH2_H, regval_H) == OK;
			result &= read_reg(BATT_PAC17_REG_VOLT_CH2_L, regval_L) == OK;

			if (result) {
				uint16_t voltage2 = (((uint16_t)regval_H) << 3) + (regval_L >> 5);
				// convert millivolts to volts
				_voltage2 = ((float)voltage2 * 19.53125f) / 1000.0f;
				//new_report.temperature = _voltage2; // hack to publish second channel
			}
		}

		return true;
	}

	return false;
}

void
BATT_PAC17::RunImpl()
{
	// get current time
	uint64_t now = hrt_absolute_time();
	vehicle_control_mode_poll();

	if (_dev_id == PAC17_DEV_E::NONE) {
		identify();
	}

	if (_dev_id != PAC17_DEV_E::NONE) {
		if (!_enabled) {
			write_reg(BATT_PAC17_REG_VOLT_SAMPLE_CONF, 0x88);
			write_reg(BATT_PAC17_REG_SENS1_SAMPLE_CONF, _sens_sample_reg); // first channel range (10-80mV)

			if (_dev_id == PAC17_DEV_E::PAC1720) {
				write_reg(BATT_PAC17_REG_SENS2_SAMPLE_CONF, _sens_sample_reg); // second channel range
			}
		}

		if (try_read_data(now)) {
			// publish to orb
			if (_startupDelay <= 0) {
				//_battery.publish();

			} else {
				_startupDelay--;
				_startRemaining = _battery.getRemainingVoltage();
			}

			// record we are working
			_enabled = true;

		} else {
			if (_enabled) {
				_battery.updateBatteryStatus(now, _voltage_v, _current_a,
							     false, battery_status_s::BATTERY_SOURCE_EXTERNAL, 0,	0.0f);
				// report lost connection to battery
			}

			_enabled = false;
		}
	}
}


int
BATT_PAC17::read_reg(uint8_t reg, uint8_t &val)
{
	uint8_t buff[2];	// 2 bytes of data

	// read from register
	int ret = _interface->transfer(&reg, 1, buff, 2);

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
	int ret = _interface->transfer(buff, 2, nullptr, 0);
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


/* read arming state */
void BATT_PAC17::vehicle_control_mode_poll()
{
	vehicle_control_mode_s vstatus;
	bool vcontrol_mode_updated;
	orb_check(_sub_status, &vcontrol_mode_updated);

	if (vcontrol_mode_updated) {
		orb_copy(ORB_ID(vehicle_control_mode), _sub_status, &vstatus);

		if (_armed != (vstatus.flag_armed > 0)) {
			if (vstatus.flag_armed > 0) {
				_time_arm = hrt_absolute_time();
				_discharged_mah_armed = _battery.getDischarged();
			}
		}

		_armed = vstatus.flag_armed > 0;
	}
}

///////////////////////// shell functions ///////////////////////

void BATT_PAC17::print_usage()
{
	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Driver for power monitor ic pac1710 and pac1720.

### Examples
$ batt_pac17xx -X start -r 0.2 -s 40

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("batt_pac17xx", "driver");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAMS_I2C_SPI_DRIVER(true, false);
	PRINT_MODULE_USAGE_PARAMS_I2C_ADDRESS(BATT_PAC17_ADDR);

	PRINT_MODULE_USAGE_COMMAND_DESCR("suspend", "Suspends the driver from rescheduling the cycle.");
	PRINT_MODULE_USAGE_COMMAND_DESCR("resume", "Resumes the driver from suspension.");
	PRINT_MODULE_USAGE_COMMAND_DESCR("search", "looks on the bus");
	PRINT_MODULE_USAGE_PARAM_FLOAT("r", BATT_PAC17_SENS_R, 0.01f, 1000.0f,"sense resistor in mOhm", true);
	PRINT_MODULE_USAGE_PARAM_INT("s", BATT_PAC17_SENS_RANGE, 10, 80,"full range sense voltage 10,20,40,80mV", true);

	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
/*
	PX4_INFO("missing command: try 'start', 'test', 'stop', 'search'");
	PX4_INFO("options:");
	PX4_INFO("    -b i2cbus (%d)", BATT_PAC17_I2C_BUS);
	PX4_INFO("    -a addr (0x%x)", BATT_PAC17_ADDR);
	PX4_INFO("    -r sense resistor (%.2fmOhm)", (double)BATT_PAC17_SENS_R);
	PX4_INFO("    -s full range sense voltage (%imV) 10,20,40,80", BATT_PAC17_SENS_RANGE);
	*/
}

I2CSPIDriverBase *BATT_PAC17::instantiate(const BusCLIArguments &cli, const BusInstanceIterator &iterator,
				      int runtime_instance)
{
	SMBus *interface = new SMBus(iterator.bus(), cli.i2c_address);
	if (interface == nullptr) {
		PX4_ERR("alloc failed");
		return nullptr;
	}
	BATT_PAC17 *instance = new BATT_PAC17(iterator.configuredBusOption(), iterator.bus(), interface);

	if (instance == nullptr) {
		PX4_ERR("alloc failed");
		return nullptr;
	}

	int result = instance->get_startup_info();

	if (result != PX4_OK) {
		delete instance;
		return nullptr;
	}
	instance->ScheduleOnInterval(BATT_PAC17_MEASUREMENT_INTERVAL_US);
	return instance;
}

void
BATT_PAC17::custom_method(const BusCLIArguments &cli)
{
	switch(cli.custom1) {
		case 1: {
		}
		break;
	}
}
int
batt_pac17xx_main(int argc, char *argv[])
{
//	int i2cdevice = BATT_PAC17_I2C_BUS;
//	int batt_pac17adr = BATT_PAC17_ADDR; // 7bit address
	float	resistor = 0;
	int range = 0;

	using ThisDriver = BATT_PAC17;
	BusCLIArguments cli{true, false};
	cli.default_i2c_frequency = 100000;
	cli.i2c_address = BATT_PAC17_ADDR;

	const char *verb = cli.parseDefaultArguments(argc, argv);
	if (!verb) {
		ThisDriver::print_usage();
		return -1;
	}

	BusInstanceIterator iterator(MODULE_NAME, cli, DRV_BAT_DEVTYPE_SMBUS);

	if (!strcmp(verb, "start")) {
		return ThisDriver::module_start(cli, iterator);
	}

	if (!strcmp(verb, "stop")) {
		return ThisDriver::module_stop(iterator);
	}

	if (!strcmp(verb, "status")) {
		return ThisDriver::module_status(iterator);
	}


	ThisDriver::print_usage();
	return -1;
}
