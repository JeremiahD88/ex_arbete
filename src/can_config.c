#include "can_config.h"

#include <zephyr/drivers/can.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(can_config);

#define CAN_BAUDRATE     250000
#define CAN_SAMPLE_POINT 750

#define CAN_DEVICE DT_NODELABEL(mcp2515)

const struct device *can_dev = DEVICE_DT_GET(CAN_DEVICE);

const struct device *get_can_dev()
{
	return can_dev;
}

void can_init(void)
{
	int err;

	struct can_timing timing;

	if (!device_is_ready(can_dev)) {
		LOG_ERR("CAN device %s is not ready", can_dev->name);
		return;
	}

	err = can_set_mode(can_dev, CAN_MODE_NORMAL);
	if (err) {
		LOG_ERR("Failed to set CAN in normal mode (error code: %d)", err);
		return;
	}

	err = can_calc_timing(can_dev, &timing, CAN_BAUDRATE, CAN_SAMPLE_POINT);
	if (err) {
		LOG_ERR("Failed to calculate CAN timing (error code: %d)", err);
		return;
	}

	err = can_set_timing(can_dev, &timing);
	if (err) {
		LOG_ERR("Failed to set CAN timing (error code: %d)", err);
		return;
	}

	err = can_start(can_dev);
	if (err) {
		LOG_ERR("Failed to start CAN device (error code: %d)", err);
		return;
	}

	/*TODO: initialize watchdog for heartbeats from Odrives*/
}
