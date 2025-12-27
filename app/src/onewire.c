
#include <zephyr/drivers/sensor/w1_sensor.h>
#include <zephyr/drivers/w1.h>
#include <zephyr/device.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(onewire, LOG_LEVEL_DBG);

#include "onewire.h"

const struct device const * dev_ds18b20_0 = DEVICE_DT_GET(DT_NODELABEL(ds18b20_0));
const struct device const * dev_ds18b20_1 = DEVICE_DT_GET(DT_NODELABEL(ds18b20_1));

K_MSGQ_DEFINE(ds18b20_msgq, sizeof(struct ds18b20_result), 10, 1);

struct ds18b20_sensor collected_sensors_onewire[] = {
	{
		.dev = DEVICE_DT_GET(DT_NODELABEL(ds18b20_0)),
		.valid = false
	},
	{
		.dev = DEVICE_DT_GET(DT_NODELABEL(ds18b20_1)),
		.valid = false
	}
};

const size_t num_collected_sensors_onewire = ARRAY_SIZE(collected_sensors_onewire);

static char ds18b20_names[][17] = {
	"",
	"",
};

static const struct device *ds18b20_devices[] = {
	DEVICE_DT_GET(DT_NODELABEL(ds18b20_0)),
	DEVICE_DT_GET(DT_NODELABEL(ds18b20_1)),
};

int trigger_sensors(struct ds18b20_sensor *sensors, size_t num_sensors)
{
	int total_ret = 0;
	int ret = 0;

	LOG_INF("Fetching sensors");


	for (size_t i = 0; i < num_sensors; i++) {
		ret = sensor_sample_fetch(sensors[i].dev);
		if (ret < 0) {
			LOG_ERR("Failed to fetch sample from %s", sensors[i].dev->name);
			total_ret = ret;
			sensors[i].valid = false;
		} else {
			sensors[i].valid = true;
		}
	}

	for (size_t i = 0; i < num_sensors; i++) {
		if (sensors[i].valid == false) {
			continue;
		}
		struct sensor_value temp_value;
		ret = sensor_channel_get(sensors[i].dev, SENSOR_CHAN_AMBIENT_TEMP, &temp_value);
		if (ret < 0) {
			LOG_ERR("Failed to get temperature from %s", sensors[i].dev->name);
			total_ret = ret;
			sensors[i].valid = false;
		} else {
			sensors[i].temperature_centi = sensor_value_to_centi(&temp_value);
			struct ds18b20_result result = {
				.timestamp = k_uptime_seconds(),
				.id = {0},
				.temperature_centi = sensors[i].temperature_centi,
				.idx = i
			};
			memcpy(result.id, sensors[i].id, sizeof(sensors[i].id));
			k_msgq_put(&ds18b20_msgq, &result, K_NO_WAIT);
			sensors[i].valid = true;
			LOG_INF("Sensor %s temperature: %d cC, addr: %04lx %04lx %04lx %04lx",
					sensors[i].dev	->name,
					sensors[i].temperature_centi,
					sensors[i].id[0],
					sensors[i].id[1],
					sensors[i].id[2],
					sensors[i].id[3]);
		}
	}

	return total_ret;
}

static void onewire_search_callback(struct w1_rom rom, void *user_data)
{
	LOG_INF("Found thermometer");
	if (rom.family != 0x28) {
		LOG_WRN("Found 1-wire device with unknown family %02x, skipping", rom.family);
		return;
	}

	// Find an empty slot to configure this address
	size_t i = 0;
	for (; i < ARRAY_SIZE(ds18b20_names); i++) {
		if (ds18b20_names[i][0] == '\0') {
			break;
		}
	}

	if (i == ARRAY_SIZE(ds18b20_names)) {
		LOG_ERR("Found more than %d DS18B20 sensors, skipping", ARRAY_SIZE(ds18b20_names));
		return;
	}

	// Set the name and ROM of the sensor
	snprintf(ds18b20_names[i], sizeof(ds18b20_names[i]),
			"%016llx", w1_rom_to_uint64(&rom));
	LOG_INF("Thermometer found: %s", ds18b20_names[i]);
	struct sensor_value rom_value;
	w1_rom_to_sensor_value(&rom, &rom_value);
	LOG_INF("Sensor addr: %08lx,%08lx ",rom_value.val1, rom_value.val2);
	uint64_t rom_uint64 = w1_rom_to_uint64(&rom);
	memcpy(collected_sensors_onewire[i].id, &rom_uint64, sizeof(collected_sensors_onewire[i].id));

	if (sensor_attr_set(collected_sensors_onewire[i].dev, SENSOR_CHAN_ALL,
					(enum sensor_attribute)SENSOR_ATTR_W1_ROM,
					&rom_value) < 0) {
		LOG_ERR("Failed to set ROM for sensor %s", collected_sensors_onewire[i].dev->name);
	}
}

int rescan_onewire()
{
	const struct device *onewire = DEVICE_DT_GET(DT_NODELABEL(w1));
	int ret;

	// TODO: We should be able to cache the ROMs and only rescan if we 
	// fail to read from a sensor. This is needlessly slow.

	LOG_DBG("Rescanning the 1-wire bus");

	// First, we clear the currently assigned addresses
	for (size_t i = 0; i < ARRAY_SIZE(ds18b20_names); i++) {
		struct w1_rom blank_rom = W1_ROM_INIT_ZERO;
		struct sensor_value rom_value;
		w1_rom_to_sensor_value(&blank_rom, &rom_value);

		ds18b20_names[i][0] = '\0';
		sensor_attr_set(ds18b20_devices[i], SENSOR_CHAN_ALL,
						(enum sensor_attribute)SENSOR_ATTR_W1_ROM,
						&rom_value);
	}

	// Next, we rescan the bus
	ret = w1_lock_bus(onewire);
	if (ret < 0) {
		LOG_ERR("Failed to lock the 1-wire bus");
		return ret;
	}
	ret = w1_search_rom(onewire, onewire_search_callback, NULL);
	w1_unlock_bus(onewire);

	if (ret < 0) {
		LOG_ERR("Failed to search for 1-wire devices");
		return ret;
	}

	return 0;
}
