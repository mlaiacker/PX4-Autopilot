#ifdef __PX4_NUTTX

#include "gpio.h"
#include <cstring>

constexpr uint32_t CameraInterfaceGPIO::_gpios[6];

CameraInterfaceGPIO::CameraInterfaceGPIO():
	CameraInterface(),
	_trigger_invert(false),
	_triggers{0}
{
	_p_polarity = param_find("TRIG_POLARITY");

	// polarity of the trigger (0 = active low, 1 = active high )
	int32_t polarity;
	param_get(_p_polarity, &polarity);
	_trigger_invert = (polarity == 0);

	get_pins();
	setup();
}

CameraInterfaceGPIO::~CameraInterfaceGPIO()
{
}

void CameraInterfaceGPIO::setup()
{
	for (unsigned i = 0, t = 0; i < arraySize(_pins); i++) {

		// Pin range is from 1 to 6, indexes are 0 to 5

		if (_pins[i] >= 0 && _pins[i] < (int)arraySize(_gpios)) {
			uint32_t gpio = _gpios[_pins[i]];
			_triggers[t++] = gpio;
			px4_arch_configgpio(gpio);
			px4_arch_gpiowrite(gpio, false ^ _trigger_invert);
		}
	}
}

void CameraInterfaceGPIO::trigger(bool trigger_on_true)
{
	bool trigger_state = trigger_on_true ^ _trigger_invert;

	for (unsigned i = 0; i < arraySize(_triggers); i++) {

		if (_triggers[i] == 0) { break; }

		px4_arch_gpiowrite(_triggers[i], trigger_state);
	}
}

void CameraInterfaceGPIO::info()
{
	PX4_INFO("GPIO trigger mode, pins enabled : [%d][%d][%d][%d][%d][%d], polarity : %s",
		 _pins[5], _pins[4], _pins[3], _pins[2], _pins[1], _pins[0],
		 _trigger_invert ? "ACTIVE_LOW" : "ACTIVE_HIGH");
}

#endif /* ifdef __PX4_NUTTX */


#ifdef __PX4_POSIX

#include "gpio.h"
#include <cstring>

CameraInterfaceGPIO::CameraInterfaceGPIO():
	CameraInterface(),
	_trigger_invert(false),
	_gpio(4)
{
	_p_polarity = param_find("TRIG_POLARITY");

	// polarity of the trigger (0 = active low, 1 = active high )
	int32_t polarity;
	param_get(_p_polarity, &polarity);
	_trigger_invert = (polarity == 0);

	setup();
}

CameraInterfaceGPIO::~CameraInterfaceGPIO()
{
}

void CameraInterfaceGPIO::setup()
{
	int res = 0;
	res = _gpio.exportPin();

	if (res != 0) {
		PX4_ERR("gpio: failed to export");
	}

	res = _gpio.setDirection(LinuxGPIO::Direction::OUT);

	if (res != 0) {
		PX4_ERR("gpio failed to set direction");
	}

	PX4_INFO("using linux gpio4");
}

void CameraInterfaceGPIO::trigger(bool trigger_on_true)
{
	bool trigger_state = trigger_on_true ^ _trigger_invert;

	_gpio.writeValue((LinuxGPIO::Value)trigger_state);
	//PX4_INFO("gpio4 %i",trigger_state);
}

void CameraInterfaceGPIO::info()
{
	PX4_INFO("GPIO trigger mode, pins enabled : [%d], polarity : %s",
		 _gpio.readValue(),
		 _trigger_invert ? "ACTIVE_LOW" : "ACTIVE_HIGH");
}

#endif /* ifdef __PX4_POSIX */
