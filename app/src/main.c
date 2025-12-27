/*
 * Copyright (c) 2020 PHYTEC Messtechnik GmbH
 * Copyright (c) 2022 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/modbus/modbus.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/drivers/sensor.h>

#include "modbus_cb.h"
#include "config.h"
#include "adc.h"
#include "onewire.h"
#include "pwm.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(main, LOG_LEVEL_DBG);

static const struct gpio_dt_spec led_dev[] = {
	GPIO_DT_SPEC_GET(DT_ALIAS(led1), gpios),
	GPIO_DT_SPEC_GET(DT_ALIAS(led2), gpios),
};

uint8_t coils_state;


const static struct modbus_iface_param server_param = {
	.mode = MODBUS_MODE_RTU,
	.server = {
		.user_cb = &mbs_cbs,
		.unit_id = MODBUS_ADDR,
	},
	.serial = {
		.baud = 115200,
		.parity = UART_CFG_PARITY_NONE,
	},
};

#define MODBUS_NODE DT_COMPAT_GET_ANY_STATUS_OKAY(zephyr_modbus_serial)

static int init_modbus_server(void)
{
	const char iface_name[] = {DEVICE_DT_NAME(MODBUS_NODE)};
	int iface;

	iface = modbus_iface_get_by_name(iface_name);

	if (iface < 0) {
		LOG_ERR("Failed to get iface index for %s", iface_name);
		return iface;
	}

	return modbus_init_server(iface, server_param);
}

int main(void)
{
	int err;

	for (int i = 0; i < ARRAY_SIZE(led_dev); i++) {
		if (!gpio_is_ready_dt(&led_dev[i])) {
			LOG_ERR("LED%u GPIO device not ready", i);
			return 0;
		}

		err = gpio_pin_configure_dt(&led_dev[i], GPIO_OUTPUT_INACTIVE);
		if (err != 0) {
			LOG_ERR("Failed to configure LED%u pin", i);
			return 0;
		}
	}


	int ret;
	if (init_pwm()){
		LOG_ERR("PWM initialization failed");
	}


	if (init_modbus_server()) {
		LOG_ERR("Modbus RTU server initialization failed");
	}

	if (init_adc()){
		LOG_ERR("ADC initialization failed");
	}

	while (true){
		LOG_INF("Waiting...");
		int rc;
		rc = print_die_temperature();
		if (rc < 0) {
			return 0;
		}
		rc = print_vref_voltage();
		if (rc < 0) {
			return 0;
		}
		read_adc();

		rescan_onewire();
		trigger_sensors(collected_sensors_onewire, num_collected_sensors_onewire);
		k_msleep(1000);
	}
	return 0;
}
