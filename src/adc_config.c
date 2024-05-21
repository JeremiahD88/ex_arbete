#include "adc_config.h"

#include <zephyr/drivers/adc.h>
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(adc_config);

#define NUM_OF_ADC_CHANNELS  2
#define ADC_BUFFER_SIZE      (NUM_OF_ADC_CHANNELS * sizeof(uint16_t))
#define ADC_NODE             DT_NODELABEL(adc)
#define ADC_CHANNEL_0        2 // AIN4 (P0.28)
#define ADC_CHANNEL_1        3 // AIN5 (P0.29)
#define ADC_PORT_0           SAADC_CH_PSELP_PSELP_AnalogInput4
#define ADC_PORT_1           SAADC_CH_PSELP_PSELP_AnalogInput5
#define ADC_RESOLUTION       8
#define ADC_GAIN             ADC_GAIN_1_5
#define ADC_REFERENCE        ADC_REF_INTERNAL // double check this! Internal ref is 0.6V
#define ADC_ACQUISITION_TIME ADC_ACQ_TIME_DEFAULT

uint16_t adc_buf[ADC_BUFFER_SIZE];

static const struct device *adc_dev = DEVICE_DT_GET(ADC_NODE);

struct adc_channel_cfg adc_cfgs[NUM_OF_ADC_CHANNELS] = {
	// Array for ease of adding more channels later
	{.gain = ADC_GAIN,
	 .reference = ADC_REFERENCE,
	 .acquisition_time = ADC_ACQUISITION_TIME,
	 .channel_id = ADC_CHANNEL_0,
	 .input_positive = ADC_PORT_0},
	{.gain = ADC_GAIN,
	 .reference = ADC_REFERENCE,
	 .acquisition_time = ADC_ACQUISITION_TIME,
	 .channel_id = ADC_CHANNEL_1,
	 .input_positive = ADC_PORT_1}
	/*TODO: add more adc channels later*/
};

struct adc_sequence adc_seq = {.options = NULL,
			       .channels = BIT(ADC_CHANNEL_0) |
					   BIT(ADC_CHANNEL_1), /*TODO: add more channels later*/
			       .buffer = adc_buf,
			       .buffer_size = ADC_BUFFER_SIZE,
			       .resolution = ADC_RESOLUTION};

void adc_init(void)
{
	if (!device_is_ready(adc_dev)) {
		LOG_ERR("ADC device %s is not ready", adc_dev->name);
		return;
	}
	for (int i = 0; i < ARRAY_SIZE(adc_cfgs); i++) {
		int err = adc_channel_setup(adc_dev, &adc_cfgs[i]);
		if (err) {
			LOG_ERR("Failed to setup ADC channel %d, error: %d", i, err);
			return;
		}
	}
}

void adc_read_buffers(uint8_t *buffer)
{
	int err = adc_read(adc_dev, &adc_seq);
	if (err) {
		LOG_ERR("Failed to read ADC channel, error: %d", err);
	}
	for (int i = 0; i < NUM_OF_ADC_CHANNELS; i++) {
		buffer[i] = (uint8_t)adc_buf[i];
	}
}