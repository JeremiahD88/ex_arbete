#include "ble_server.h"
#include "service.h"
#include "odrives.h"
#include "actuator_steering.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(ble_server);

#define START_ANGLE 127

static bool was_disconnected = false;

static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_BEAST_SERVICE_VAL)};

static inline void bt_connected(struct bt_conn *conn, uint8_t err)
{
	if (err) {
		LOG_ERR("Connection failed (err %u)", err);
	}
	LOG_INF("BLE client connected!");

	odrives_read_heartbeat();
	odrives_read_velocity();
	if (was_disconnected) {
		actuator_steering_start(START_ANGLE);
		was_disconnected = false;
	}
}

static inline void bt_disconnected(struct bt_conn *conn, uint8_t reason)
{
	LOG_ERR("Disconnected (reason %u)", reason);

	odrives_set_state(IDLE);
	odrives_remove_filters();
	actuator_steering_stop_thread();
	was_disconnected = true;
}

static struct bt_conn_cb conn_callbacks = {
	.connected = bt_connected,
	.disconnected = bt_disconnected,
};

int init_ble_server(void)
{

	int err;

	err = bt_enable(NULL);
	if (err) {
		LOG_ERR("Bluetooth init failed (err %d)\n", err);
		return err;
	}

	LOG_INF("Bluetooth initialized!");

	bt_conn_cb_register(&conn_callbacks);

	err = bt_le_adv_start(BT_LE_ADV_CONN_NAME, ad, ARRAY_SIZE(ad), NULL, 0);

	if (err) {
		LOG_ERR("Advertising failed to start (err %d)", err);
		return err;
	}

	LOG_INF("Advertising successfully started");

	return 0;
}
