#ifndef ODRIVES_H
#define ODRIVES_H

#include <stdint.h>

#define IDLE                0x01
#define CLOSED_LOOP_CONTROL 0x08

void odrives_set_state(const uint8_t requested_state);
void odrives_clear_errors(void);
void odrives_set_input_velocity(uint8_t requested_velocity, uint8_t direction);

int odrives_read_heartbeat(void);
int odrives_read_velocity(void);
void odrives_remove_filters(void);

#endif // ODRIVES_H