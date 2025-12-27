#ifndef ONEWIRE_H
#define ONEWIRE_H

#include <zephyr/device.h>

struct ds18b20_sensor {
	uint8_t id[8];
    const struct device *dev;
    int16_t temperature_centi;
    bool valid;
};

struct ds18b20_result {
    uint64_t timestamp;
    uint16_t id[4];
    int16_t temperature_centi;
    uint8_t idx;
};

extern struct k_msgq ds18b20_msgq;
extern struct ds18b20_sensor collected_sensors_onewire[];
extern const size_t num_collected_sensors_onewire;

int rescan_onewire();
int trigger_sensors(struct ds18b20_sensor *sensors, size_t num_sensors);

#endif // ONEWIRE_H