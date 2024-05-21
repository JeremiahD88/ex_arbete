#include "lia_protocol_handler.h"
#include "service.h"
#include "odrives.h"
#include "actuator_steering.h"

#include <zephyr/sys/byteorder.h>
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(protocol);

#define LIA_MOVE_MSG_LEN 7
#define MIN_MSG_LEN      2 /*total size of message id and message length fields*/

enum lia_msg_ids {
	IDLE_ID,
	CLOSED_LOOP_CONTROL_ID,
	SET_TORQUE_ID,
	CLEAR_ERRORS_ID,
	LIA_TOTAL_MSGS
};

struct lia_move_msg {
	uint8_t direction;
	uint16_t duration;
	uint8_t speed;
	uint8_t angle;
};

static int lia_move_handle(const uint8_t *data, struct lia_move_msg *msg)
{

	uint8_t msg_len = data[1];
	if (msg_len != LIA_MOVE_MSG_LEN) {
		return -EINVAL;
	}
	msg->direction = data[2];
	msg->duration = sys_get_be16(&data[3]);
	msg->speed = data[5];
	msg->angle = data[6];
	actuator_steering_start(msg->angle);
	LOG_INF("Set angle: %u", msg->angle);
	odrives_set_input_velocity(msg->speed, msg->direction);
	LOG_INF("Set torque: %u, direction: %u", msg->speed, msg->direction);
	return 0;
}

void lia_protocol_handler(struct bt_conn *conn, const uint8_t *data, uint16_t len)
{
	if (len < MIN_MSG_LEN) {
		LOG_ERR("Invalid data length: %d. Expected at least %d bytes", len, MIN_MSG_LEN);
		return;
	}
	uint8_t msg_id = data[0];

	switch (msg_id) {
	case IDLE_ID:
		odrives_set_state(IDLE);
		break;
	case CLOSED_LOOP_CONTROL_ID:
		odrives_set_state(CLOSED_LOOP_CONTROL);
		break;
	case SET_TORQUE_ID:
		struct lia_move_msg msg;
		int err = lia_move_handle(data, &msg);
		if (err) {
			LOG_ERR("Failed to parse message: %d", err);
			return;
		}
		LOG_DBG("Move message parsed: direction: %u, duration: %u, speed: %u",
			msg.direction, msg.duration, msg.speed);
		break;
	case CLEAR_ERRORS_ID:
		odrives_clear_errors();
		break;
	default:
		LOG_WRN("Unknown message id: %u", msg_id);
		break;
	}
}