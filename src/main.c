#include "ble_server.h"
#include "service.h"
#include "lia_protocol_handler.h"
#include "can_config.h"
#include "odrives.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(main_app);

struct ble_callbacks callbacks = {
	.data_recieved_cb = lia_protocol_handler,
};

int main(void)
{
	init_ble_server();
	ble_set_cb(&callbacks);
	can_init();

	return 0;
}