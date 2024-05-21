#include "actuators_config.h"

#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(actuators_config);

#define NUM_OF_ACTUATORS 2
#define GPIO_PORT_1      DT_NODELABEL(gpio1)

#define NUM_OF_ACTUATOR_PINS 2 // dir, pwm

struct gpio_pins {
	const struct device *dev;
	gpio_pin_t pins[NUM_OF_ACTUATOR_PINS];
};

struct gpio_pins actuators[NUM_OF_ACTUATORS] = {

	{
		.dev = DEVICE_DT_GET(GPIO_PORT_1), .pins = {8, 10} // P1.08 (dir), P1.10 (pwm)
	},
	{
		.dev = DEVICE_DT_GET(GPIO_PORT_1), .pins = {6, 7} // P1.06 (dir), P1.07 (pwm)
	}};

void set_actuator_pins(uint8_t actuator_index, uint8_t pin_index, uint8_t pin_value)
{

	gpio_pin_set(actuators[actuator_index].dev, actuators[actuator_index].pins[pin_index],
		     pin_value);
}

void actuator_turning_left(uint8_t actuator_index)
{
	set_actuator_pins(actuator_index, 1, 1); // pwm
	set_actuator_pins(actuator_index, 0, 1); // dir
	
	LOG_INF("Actuator %d turning left", actuator_index);
	
}

void actuator_turning_right(uint8_t actuator_index)
{
	set_actuator_pins(actuator_index, 1, 1); // pwm
	set_actuator_pins(actuator_index, 0, 0); // dir

	LOG_INF("Actuator %d turning right", actuator_index);
	
}

void actuator_target_reached(uint8_t actuator_index)
{
	set_actuator_pins(actuator_index, 1, 0);
	set_actuator_pins(actuator_index, 0, 0);
}

int actuators_pins_setup(void)
{

	for (uint8_t j = 0; j < NUM_OF_ACTUATORS; j++) {
		if (!device_is_ready(actuators[j].dev)) {
			LOG_ERR("GPIO device %s is not ready", actuators[j].dev->name);
			return -ENODEV;
		}

		for (uint8_t i = 0; i < NUM_OF_ACTUATOR_PINS; i++) {
			gpio_pin_configure(actuators[j].dev, actuators[j].pins[i],
					   GPIO_OUTPUT | GPIO_PULL_DOWN);
		}
	}

	return 0;
}