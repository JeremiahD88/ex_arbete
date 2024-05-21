#ifndef PROTOCOL_HANDLER_H
#define PROTOCOL_HANDLER_H

#include <zephyr/bluetooth/bluetooth.h>

void lia_protocol_handler(struct bt_conn *conn, const uint8_t *data, uint16_t len);

#endif // PROTOCOL_HANDLER_H