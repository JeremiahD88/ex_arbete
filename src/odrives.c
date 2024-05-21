#include "odrives.h"
#include "can_config.h"
#include "service.h"

#include <zephyr/drivers/can.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(odrives);

#define NUM_OF_ODRIVES 4

#define ODRIVE_BROADCAST_ID 0x3f
#define ODRIVE_1_ID         0x0b
#define ODRIVE_2_ID         0x0c
#define ODRIVE_3_ID         0x0d
#define ODRIVE_4_ID         0x0e

#define CMD_SET_STATE          0x07
#define CMD_SET_INPUT_VELOCITY 0x0d
#define CMD_SET_CLEAR_ERRORS   0x18
#define CMD_GET_HEARTBEAT      0x01
#define CMD_GET_VELOCITY       0x09

#define FRAME_RTR_FALSE   0x00
#define FRAME_RTR_TRUE    0x01
#define FRAME_SHIFT_5BITS 0x05
#define FRAME_CMD_MASK    0x1f
#define FILTER_ID_MASK    0xfff

#define MAX_BLE_FRAME_SIZE 0x0a

#define MIN_SPEED    0
#define MAX_SPEED    255
#define MIN_velocity 0.0
#define MAX_velocity 1.0

static bool is_disarmed = false;

uint8_t odrives[NUM_OF_ODRIVES] = {ODRIVE_1_ID, ODRIVE_2_ID, ODRIVE_3_ID, ODRIVE_4_ID};

struct can_frame odrive_frame;

int filter_ids[NUM_OF_ODRIVES + 1];

struct can_filter heartbeat_filter[NUM_OF_ODRIVES] = {
	{.id = (ODRIVE_1_ID << FRAME_SHIFT_5BITS | CMD_GET_HEARTBEAT), .mask = FILTER_ID_MASK},
	{.id = (ODRIVE_2_ID << FRAME_SHIFT_5BITS | CMD_GET_HEARTBEAT), .mask = FILTER_ID_MASK},
	{.id = (ODRIVE_3_ID << FRAME_SHIFT_5BITS | CMD_GET_HEARTBEAT), .mask = FILTER_ID_MASK},
	{.id = (ODRIVE_4_ID << FRAME_SHIFT_5BITS | CMD_GET_HEARTBEAT), .mask = FILTER_ID_MASK},
};

struct can_filter velocity_filter = {
	.id = (ODRIVE_2_ID << FRAME_SHIFT_5BITS | CMD_GET_VELOCITY),
	.mask = FILTER_ID_MASK,

};

enum odrives_direction {
	FW = 0x00,
	BW = 0x01,
};

int update_odrive_frame(uint8_t id, uint8_t flags, uint8_t cmd, const void *data_field,
			size_t data_size)
{

	if (data_field == NULL) {
		LOG_ERR("Data_field is NULL");
		return -EINVAL;
	}

	if (data_size > CAN_MAX_DLEN) {
		LOG_ERR("Data_field size is greater than max data lenght of CAN frame");
		return -EINVAL;
	}

	odrive_frame.id = (id << FRAME_SHIFT_5BITS | cmd);
	odrive_frame.flags = flags;
	odrive_frame.dlc = data_size;
	memcpy(odrive_frame.data, data_field, data_size);

	return 0;
}

float int_to_float_map(uint8_t input, uint8_t in_min, uint8_t in_max, float out_min, float out_max)
{
	return (input - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void can_tx_callback(const struct device *dev, int error, void *arg)
{
	if (error) {
		LOG_ERR("CAN message not sent, error: %d", error);
	}
}

void odrives_set_state(const uint8_t requested_state)
{
	int err;

	const struct device *can_dev = get_can_dev();
	err = update_odrive_frame(ODRIVE_BROADCAST_ID, FRAME_RTR_FALSE, CMD_SET_STATE,
				  &requested_state, sizeof(requested_state));
	if (err) {
		LOG_ERR("Failed to update Odrive frame for %d (error code: %d)", requested_state,
			err);
		return;
	}

	err = can_send(can_dev, &odrive_frame, K_MSEC(100), can_tx_callback, NULL);
	if (err) {
		LOG_ERR("Failed to set Odrive state to %d (error code: %d)", requested_state, err);
	}
}

void odrives_clear_errors(void)
{
	int err;
	uint8_t empty_payload = 0;
	const struct device *can_dev = get_can_dev();
	err = update_odrive_frame(ODRIVE_BROADCAST_ID, FRAME_RTR_FALSE, CMD_SET_CLEAR_ERRORS,
				  &empty_payload, sizeof(empty_payload));
	if (err) {
		LOG_ERR("Failed to update Odrive frame for clear errors (error code: %d)", err);
		return;
	}

	err = can_send(can_dev, &odrive_frame, K_MSEC(100), can_tx_callback, NULL);
	if (err) {
		LOG_ERR("Failed to clear Odrive errors (error code: %d)", err);
	}
	is_disarmed = false;
}

void odrives_set_input_velocity(uint8_t requested_velocity, uint8_t direction)
{
	int err;

	enum odrives_direction dir = (enum odrives_direction)direction;

	const struct device *can_dev = get_can_dev();
	float mapped_velocity = int_to_float_map(requested_velocity, MIN_SPEED, MAX_SPEED,
						 MIN_velocity, MAX_velocity);

	switch (dir) {
	case FW:
		mapped_velocity = -mapped_velocity;
		break;
	case BW:
		mapped_velocity = mapped_velocity;
		break;
	default:
		LOG_ERR("Invalid direction");
		return;
	}

	for (int i = 0; i < NUM_OF_ODRIVES; i++) {
		float velocity = (odrives[i] % 2 == 0) ? -mapped_velocity : mapped_velocity;

		err = update_odrive_frame(odrives[i], FRAME_RTR_FALSE, CMD_SET_INPUT_VELOCITY,
					  &velocity, sizeof(velocity));

		err = can_send(can_dev, &odrive_frame, K_MSEC(100), can_tx_callback, NULL);
		if (err) {
			LOG_ERR("Failed to set Odrive_id %d velocity to %d (error code: %d)",
				odrives[i], requested_velocity, err);
		}
	}
}

void odrives_check_disarm_reason(struct can_frame *frame)
{

	uint32_t disarm_reason = frame->data[0] | (frame->data[1] << 8) | (frame->data[2] << 16) |
				 (frame->data[3] << 24);

	uint32_t error_1 = (frame->data[3] << 24);
	uint32_t error_2 = (frame->data[2] << 16);
	uint32_t error_3 = (frame->data[1] << 8);
	uint32_t error_4 = (frame->data[0]);

	if (disarm_reason && !is_disarmed) {
		odrives_set_state(IDLE);
		is_disarmed = true;
		LOG_ERR("Disarm reason: %d, %d, %d, %d", error_1, error_2, error_3, error_4);
	}
}

void odrives_send_ble_frame(struct can_frame *frame)
{
	uint8_t ble_frame[MAX_BLE_FRAME_SIZE];

	if (frame->dlc > MAX_BLE_FRAME_SIZE) {
		LOG_ERR("Frame size is greater than max frame size of BLE");
		return;
	} else {
		ble_frame[0] = frame->id >> FRAME_SHIFT_5BITS;
		ble_frame[1] = frame->id & FRAME_CMD_MASK;
		ble_frame[2] = frame->data[0];
		ble_frame[3] = frame->data[1];
		ble_frame[4] = frame->data[2];
		ble_frame[5] = frame->data[3];
		ble_frame[6] = frame->data[4];
		ble_frame[7] = frame->data[5];
		ble_frame[8] = frame->data[6];
		ble_frame[9] = frame->data[7];

		update_motor_info(ble_frame, MAX_BLE_FRAME_SIZE);
	}
}

void odrives_read_heartbeat_cb(const struct device *dev, struct can_frame *frame, void *arg)
{
	uint8_t node_id = frame->id >> FRAME_SHIFT_5BITS;
	uint8_t cmd = frame->id & 0x1f;
	LOG_INF("Heartbeat ID:%d CMD:0x0%x data %x,%x,%x,%x,%x,%x,%x,%x", node_id, cmd,
		frame->data[0], frame->data[1], frame->data[2], frame->data[3], frame->data[4],
		frame->data[5], frame->data[6], frame->data[7]);

	odrives_send_ble_frame(frame);
	odrives_check_disarm_reason(frame);
}

int odrives_read_heartbeat(void)
{
	const struct device *can_dev = get_can_dev();

	for (int i = 0; i < (NUM_OF_ODRIVES); i++) {
		filter_ids[i] = can_add_rx_filter(can_dev, odrives_read_heartbeat_cb, NULL,
						  &heartbeat_filter[i]);
		if (filter_ids[i] < 0) {
			LOG_ERR("Failed to add CAN filter (error code: %d)", filter_ids[i]);
			return filter_ids[i];
		}
	}

	return 0;
}

void odrives_read_velocity_cb(const struct device *dev, struct can_frame *frame, void *arg)
{
	uint8_t node_id = frame->id >> FRAME_SHIFT_5BITS;
	uint8_t cmd = frame->id & 0x1f;
	LOG_INF("Velocity  ID:%d CMD:0x0%x data %x,%x,%x,%x,%x,%x,%x,%x", node_id, cmd,
		frame->data[0], frame->data[1], frame->data[2], frame->data[3], frame->data[4],
		frame->data[5], frame->data[6], frame->data[7]);

	odrives_send_ble_frame(frame);
}

int odrives_read_velocity(void)
{
	const struct device *can_dev = get_can_dev();

	filter_ids[NUM_OF_ODRIVES] =
		can_add_rx_filter(can_dev, odrives_read_velocity_cb, NULL, &velocity_filter);
	if (filter_ids[NUM_OF_ODRIVES] < 0) {
		LOG_ERR("Failed to add CAN filter (error code: %d)", filter_ids[NUM_OF_ODRIVES]);
		return filter_ids[NUM_OF_ODRIVES];
	}

	return 0;
}

void odrives_remove_filters(void)
{
	const struct device *can_dev = get_can_dev();
	for (int i = 0; i <= NUM_OF_ODRIVES; i++) {
		can_remove_rx_filter(can_dev, filter_ids[i]);
	}
}