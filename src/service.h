
#ifndef SERVICE_H
#define SERVICE_H

#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/bluetooth.h>

#define BT_UUID_BEAST_SERVICE_VAL                                                                  \
	BT_UUID_128_ENCODE(0x347F1281, 0x56EE, 0x488B, 0xA55A, 0x4AFBF234FA8C)
#define BT_UUID_BEAST_CHRC_VAL                                                                     \
	BT_UUID_128_ENCODE(0x347F1281, 0x56EE, 0x488B, 0xA55A, 0x4AFBF234FA8D)
#define BT_UUID_BEAST_MOTOR_INFO_VAL                                                               \
	BT_UUID_128_ENCODE(0x347F1281, 0x56EE, 0x488B, 0xA55A, 0x4AFBF234FA8E)

#define BT_UUID_THE_BEAST_SERVICE    BT_UUID_DECLARE_128(BT_UUID_BEAST_SERVICE_VAL)
#define BT_UUID_THE_BEAST_CONTROL    BT_UUID_DECLARE_128(BT_UUID_BEAST_CHRC_VAL)
#define BT_UUID_THE_BEAST_MOTOR_INFO BT_UUID_DECLARE_128(BT_UUID_BEAST_MOTOR_INFO_VAL)

struct ble_callbacks {
	void (*data_recieved_cb)(struct bt_conn *conn, const uint8_t *data, uint16_t len);
};

void ble_set_cb(struct ble_callbacks *new_cb);

void update_motor_info(uint8_t *data, uint16_t len);

#endif // SERVICE_H
