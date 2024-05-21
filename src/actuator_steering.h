#ifndef ACTUATOR_STEERING_H_
#define ACTUATOR_STEERING_H_

#include <zephyr/types.h>
#include <stdbool.h>

void actuator_steering_start(uint8_t angle);
bool is_actuator_steering_thread_started(void);
void actuator_steering_stop_thread(void);

#endif /* ACTUATOR_STEERING_H_ */