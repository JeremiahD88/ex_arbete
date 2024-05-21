#ifndef ACTUATORS_CONFIG_H_
#define ACTUATORS_CONFIG_H_

#include <zephyr/types.h>

typedef enum {
	ACTUATOR_0,
	ACTUATOR_1,
} actuator_index_t;

void actuator_turning_left(uint8_t actuator_index);
void actuator_turning_right(uint8_t actuator_index);
void actuator_target_reached(uint8_t actuator_index);
int actuators_pins_setup(void);

#endif /* ACTUATORS_CONFIG_H_ */