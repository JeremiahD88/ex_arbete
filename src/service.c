#include "ble_server.h"
#include "service.h"

#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/bluetooth.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(service);

#define PACKET_SIZE     20
#define MOTOR_INFO_SIZE 10

static uint8_t packet[PACKET_SIZE];
static uint8_t motor_info[MOTOR_INFO_SIZE];

struct ble_callbacks *cb;

void ble_set_cb(struct ble_callbacks *new_cb)
{
	if (new_cb == NULL) {
		LOG_ERR("Invalid callback");
		return;
	}
	cb = new_cb;
}

static ssize_t read_motor_info(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf,
			       uint16_t len, uint16_t offset)
{

	return bt_gatt_attr_read(conn, attr, buf, len, offset, &motor_info, sizeof(motor_info));
}

static ssize_t read_packet(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf,
			   uint16_t len, uint16_t offset)
{

	return bt_gatt_attr_read(conn, attr, buf, len, offset, &packet, sizeof(packet));
}

static ssize_t write_packet(struct bt_conn *conn, const struct bt_gatt_attr *attr, const void *buf,
			    uint16_t len, uint16_t offset, uint8_t flags)
{

	if (offset + len > sizeof(packet)) {

		return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
	}

	memcpy(&packet, buf, len);

	if (cb && cb->data_recieved_cb) {
		cb->data_recieved_cb(conn, (const uint8_t *)buf, len);
	}

	return len;
}

static void ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
	LOG_INF("CCC cfg changed %u", value);

	if (value == 0x0001) {
		LOG_INF("Notifications enabled");
	} else if (value == 0x0000) {
		LOG_INF("Notifications and indications disabled");
	}
}

BT_GATT_SERVICE_DEFINE(beast_srv, BT_GATT_PRIMARY_SERVICE(BT_UUID_THE_BEAST_SERVICE),
		       BT_GATT_CHARACTERISTIC(BT_UUID_THE_BEAST_CONTROL,
					      (BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE),
					      (BT_GATT_PERM_READ | BT_GATT_PERM_WRITE), read_packet,
					      write_packet, &packet),
		       BT_GATT_CHARACTERISTIC(BT_UUID_THE_BEAST_MOTOR_INFO,
					      BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
					      BT_GATT_PERM_READ, read_motor_info, NULL,
					      &motor_info),
		       BT_GATT_CCC(ccc_cfg_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE), );

void update_motor_info(uint8_t *data, uint16_t len)
{
	if (len > MOTOR_INFO_SIZE) {
		LOG_ERR("Invalid motor info size");
		return;
	}

	memcpy(&motor_info, data, len);

	/* Notify the client */
	bt_gatt_notify(NULL, &beast_srv.attrs[4], &motor_info, sizeof(motor_info));
}
