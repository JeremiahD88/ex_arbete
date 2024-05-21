#include "actuator_steering.h"
#include "actuators_config.h"
#include "adc_config.h"

#include <stdlib.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(actuator_steering_log);

bool actuator_steering_thread_started = false;

#define DEAD_BAND 1

#define STACK_SIZE      1024
#define THREAD_PRIORITY 7

K_THREAD_STACK_DEFINE(actuator_steering_stack_area, STACK_SIZE);
struct k_thread actuator_steering_thread;

uint8_t adc_values[2];
static uint8_t actuator_steering_target_angle = 0;

uint8_t int_map(uint8_t x, uint8_t in_min, uint8_t in_max, uint8_t out_min, uint8_t out_max)
{
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void actuator_steering(void)
{
	LOG_INF("actuator steering thread started");
	while (1) {

		adc_read_buffers(adc_values);
		uint8_t adc_value_0 = adc_values[0];
		uint8_t adc_value_1 = adc_values[1];

		uint8_t mapped_adc_value_0 = int_map(adc_value_0, 0, 255, 0, 20); 
		uint8_t mapped_target_angle = int_map(actuator_steering_target_angle, 0, 255, 0, 20);

		uint8_t mapped_adc_value_1 = int_map(adc_value_1, 0, 255, 0, 20);

		LOG_INF("ADC 0: %d", mapped_adc_value_0);
		LOG_INF("adc_1: %d", mapped_adc_value_1);

		uint8_t diff_0 = abs(mapped_target_angle - mapped_adc_value_0);
		uint8_t diff_1 = abs(mapped_target_angle - mapped_adc_value_1);

		if (diff_0 > DEAD_BAND) {
			if (mapped_target_angle < mapped_adc_value_0) {
				actuator_turning_left(ACTUATOR_0);
			} else if (mapped_target_angle > mapped_adc_value_0) {
				actuator_turning_right(ACTUATOR_0);
			}
		} else {
			actuator_target_reached(ACTUATOR_0);
		}

		if (diff_1 > DEAD_BAND) {
			if (mapped_target_angle > mapped_adc_value_1) {
				actuator_turning_left(ACTUATOR_1);
			} else if (mapped_target_angle < mapped_adc_value_1) {
				actuator_turning_right(ACTUATOR_1);
			}
		} else {
			actuator_target_reached(ACTUATOR_1);
		}
		k_msleep(2);
	}
}

void actuator_steering_start(uint8_t angle)
{
	actuator_steering_target_angle = angle;

	if (!actuator_steering_thread_started) {
		adc_init();
		actuators_pins_setup();

		k_thread_create(&actuator_steering_thread, actuator_steering_stack_area, STACK_SIZE,
				(k_thread_entry_t)actuator_steering, NULL, NULL, NULL,
				THREAD_PRIORITY, 0, K_NO_WAIT);
	}
	actuator_steering_thread_started = true;
}

void actuator_steering_stop_thread(void)
{
	if (actuator_steering_thread_started) {
		k_thread_abort(&actuator_steering_thread);
		actuator_steering_thread_started = false;
	}
}

bool is_actuator_steering_thread_started(void)
{
	return actuator_steering_thread_started;
}

