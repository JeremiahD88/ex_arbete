#ifndef ADC_CONFIG_H_
#define ADC_CONFIG_H_

#include <zephyr/types.h>

void adc_init(void);
void adc_read_buffers(uint8_t *buffer);

#endif /* ADC_CONFIG_H_ */