#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/sensor.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(adc, LOG_LEVEL_DBG);

uint16_t adc_vin_v;
uint16_t adc_12v_v;
uint16_t adc_vin_i;
uint16_t adc_3v3_v;
uint16_t adc_temp_int;

/* ADC node from the devicetree. */
#define ADC_NODE DT_ALIAS(adc0)

/* Auxiliary macro to obtain channel vref, if available. */
#define CHANNEL_VREF(node_id) DT_PROP_OR(node_id, zephyr_vref_mv, 0)

/* Data of ADC device specified in devicetree. */
static const struct device *adc = DEVICE_DT_GET(ADC_NODE);

/* Data array of ADC channels for the specified ADC. */
static const struct adc_channel_cfg channel_cfgs[] = {
	DT_FOREACH_CHILD_SEP(ADC_NODE, ADC_CHANNEL_CFG_DT, (,))};

/* Data array of ADC channel voltage references. */
static uint32_t vrefs_mv[] = {DT_FOREACH_CHILD_SEP(ADC_NODE, CHANNEL_VREF, (,))};

/* Get the number of channels defined on the DTS. */
#define CHANNEL_COUNT ARRAY_SIZE(channel_cfgs)

static uint32_t count = 0;

static const struct device * die_temp = DEVICE_DT_GET(DT_ALIAS(die_temp0));
static const struct device * vref_voltage = DEVICE_DT_GET(DT_ALIAS(volt_sensor0));

static struct adc_sequence adc_sequence;
static struct adc_sequence_options options; 
static uint16_t channel_reading[CONFIG_SEQUENCE_SAMPLES][CHANNEL_COUNT];

int init_adc(void){
	int rc;

	if (!device_is_ready(die_temp)) {
		LOG_ERR("sensor: device %s not ready.\n", die_temp->name);
		return -2;
	}
	if (!device_is_ready(vref_voltage)) {
		LOG_ERR("sensor: device %s not ready.\n", vref_voltage->name);
		return -2;
	}

	int err;
	LOG_DBG("Samples: %d, channels: %d",CONFIG_SEQUENCE_SAMPLES, CHANNEL_COUNT);

	/* Options for the sequence sampling. */
	options.extra_samplings = CONFIG_SEQUENCE_SAMPLES - 1;
	options.interval_us = 0;

	/* Configure the sampling sequence to be made. */
	adc_sequence.buffer = channel_reading;
		/* buffer size in bytes, not number of samples */
	adc_sequence.buffer_size = sizeof(channel_reading);
	adc_sequence.resolution = CONFIG_SEQUENCE_RESOLUTION;
	adc_sequence.oversampling = CONFIG_SEQUENCE_OVERSAMPLING;
	adc_sequence.options = &options;

	if (!device_is_ready(adc)) {
		printf("ADC controller device %s not ready\n", adc->name);
		return -1;
	}

	/* Configure channels individually prior to sampling. */
	for (size_t i = 0U; i < CHANNEL_COUNT; i++) {
		adc_sequence.channels |= BIT(channel_cfgs[i].channel_id);
		err = adc_channel_setup(adc, &channel_cfgs[i]);
		if (err < 0) {
			LOG_ERR("Could not setup channel #%d (%d)\n", i, err);
			return -1;
		}
		if ((vrefs_mv[i] == 0) && (channel_cfgs[i].reference == ADC_REF_INTERNAL)) {
			vrefs_mv[i] = adc_ref_internal(adc);
		}
	}

	LOG_DBG("Successfully initialized");

	return 0;
}

int read_adc(void){
	int err;
	LOG_DBG("ADC sequence reading [%u]:\n", count++);

	err = adc_read(adc, &adc_sequence);
	if (err < 0) {
		LOG_ERR("Could not read (%d)\n", err);
		return 1;
	}

	for (size_t channel_index = 0U; channel_index < CHANNEL_COUNT; channel_index++) {
		int32_t val_mv;

		printf("- %s, channel %" PRId32 ", %" PRId32 " sequence samples:\n",
		       adc->name, channel_cfgs[channel_index].channel_id,
		       CONFIG_SEQUENCE_SAMPLES);
		for (size_t sample_index = 0U; sample_index < CONFIG_SEQUENCE_SAMPLES;
		     sample_index++) {
			uint8_t res = CONFIG_SEQUENCE_RESOLUTION;

			val_mv = channel_reading[sample_index][channel_index];
			printf("- - %" PRId32, val_mv);
			err = adc_raw_to_millivolts(vrefs_mv[channel_index],
						    channel_cfgs[channel_index].gain,
						    res, &val_mv);

			/* conversion to mV may not be supported, skip if not */
			if ((err < 0) || vrefs_mv[channel_index] == 0) {
				printf(" (value in mV not available)\n");
			} else {
				printf(" = %" PRId32 "mV\n", val_mv);
			}
		}
	}
	// TODO move resistor values to the board config
	int32_t val_mv;
	val_mv = channel_reading[0][0];
	adc_raw_to_millivolts(vrefs_mv[0],channel_cfgs[0].gain,CONFIG_SEQUENCE_RESOLUTION,&val_mv);
	adc_vin_v = val_mv*(24000+1500)/1500;

	val_mv = channel_reading[0][1];
	adc_raw_to_millivolts(vrefs_mv[1],channel_cfgs[1].gain,CONFIG_SEQUENCE_RESOLUTION,&val_mv);
	adc_12v_v = val_mv*(24000+1500)/1500;

	val_mv = channel_reading[0][2];
	k_msleep(100);
	adc_raw_to_millivolts(vrefs_mv[2],channel_cfgs[2].gain,CONFIG_SEQUENCE_RESOLUTION,&val_mv);
	// 50V/V, 6mohm
	//(val_mv/50/1000 / 6/1000)*1000
	//(val_mv/50 / 6)*1000
	//(val_mv*20 / 6)
	adc_vin_i = val_mv*20/6;
	
	LOG_DBG("Vin: %u mV, 12V: %u mV, Iin: %u mA",adc_vin_v, adc_12v_v, adc_vin_i);


	return 0;
}

// Die temp

int print_die_temperature(void)
{
	const struct device *dev = die_temp;
	struct sensor_value val;
	int rc;

	/* fetch sensor samples */
	rc = sensor_sample_fetch(dev);
	if (rc) {
		printk("Failed to fetch sample (%d)\n", rc);
		return rc;
	}

	rc = sensor_channel_get(dev, SENSOR_CHAN_DIE_TEMP, &val);
	if (rc) {
		printk("Failed to get data (%d)\n", rc);
		return rc;
	}

	printk("CPU Die temperature[%s]: %d °C\n", dev->name, val.val1);
	adc_temp_int = val.val1;
	return 0;
}

// Internal reference voltage

int print_vref_voltage(void)
{
	const struct device *dev = vref_voltage;
	struct sensor_value val;
	int rc;

	/* fetch sensor samples */
	rc = sensor_sample_fetch(dev);
	if (rc) {
		printk("Failed to fetch sample (%d)\n", rc);
		return rc;
	}

	rc = sensor_channel_get(dev, SENSOR_CHAN_VOLTAGE, &val);
	if (rc) {
		printk("Failed to get data (%d)\n", rc);
		return rc;
	}

	printk("VREF voltage[%s]: %d mV\n", dev->name, sensor_value_to_milli(&val));
	adc_3v3_v = sensor_value_to_milli(&val);
	return 0;
}