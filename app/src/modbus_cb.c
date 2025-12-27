#include <stdint.h>

#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/modbus/modbus.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/drivers/sensor.h>

#include "config.h"
#include "adc.h"
#include "pwm.h"
#include "onewire.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(modbus_cb, LOG_LEVEL_DBG);

static const struct pwm_dt_spec pwm_o1 =
	PWM_DT_SPEC_GET(DT_ALIAS(pwmled1));
static const struct pwm_dt_spec pwm_o2 =
	PWM_DT_SPEC_GET(DT_ALIAS(pwmled2));
static const struct pwm_dt_spec pwm_o3 =
	PWM_DT_SPEC_GET(DT_ALIAS(pwmled3));

extern uint16_t pwm_reg[3];
extern uint8_t coils_state;

static struct ds18b20_result ds18b20_results[MODBUS_N_THERMOMETERS];

static const struct gpio_dt_spec led_dev[] = {
	GPIO_DT_SPEC_GET(DT_ALIAS(led1), gpios),
	GPIO_DT_SPEC_GET(DT_ALIAS(led2), gpios),
};

static int coil_rd(uint16_t addr, bool *state)
{
	LOG_INF("Coil rd");
	if (addr >= ARRAY_SIZE(led_dev)) {
		return -ENOTSUP;
	}

	if (coils_state & BIT(addr)) {
		*state = true;
	} else {
		*state = false;
	}

	LOG_INF("Coil read, addr %u, %d", addr, (int)*state);

	return 0;
}

static int coil_wr(uint16_t addr, bool state)
{
	LOG_INF("Coil write");
	bool on;

	if (addr >= ARRAY_SIZE(led_dev)) {
		return -ENOTSUP;
	}


	if (state == true) {
		coils_state |= BIT(addr);
		on = true;
	} else {
		coils_state &= ~BIT(addr);
		on = false;
	}

	gpio_pin_set(led_dev[addr].port, led_dev[addr].pin, (int)on);

	LOG_INF("Coil write, addr %u, %d", addr, (int)state);

	return 0;
}

static int input_reg_rd(uint16_t addr, uint16_t *reg)
{
	struct sensor_value temp_value;
	int ret;
	//LOG_INF("Reading input reg");
	if (addr >= MODBUS_TEMP0_REG && addr < MODBUS_TEMP0_REG + MODBUS_N_THERMOMETERS*5) {
		struct ds18b20_result qresult;
		while(k_msgq_get(&ds18b20_msgq, &qresult, K_NO_WAIT) == 0) {
			LOG_INF("Got temperature result: %d", qresult.temperature_centi);
			ds18b20_results[qresult.idx].timestamp = qresult.timestamp;
			ds18b20_results[qresult.idx].temperature_centi = qresult.temperature_centi;
			memcpy(ds18b20_results[qresult.idx].id, qresult.id, sizeof(qresult.id));
		}
		int therm_idx = (addr - MODBUS_TEMP0_REG) / 5;
		int reg_offset = (addr - MODBUS_TEMP0_REG) % 5;
		if (ds18b20_results[therm_idx].timestamp + MODBUS_TEMP_MAX_AGE < k_uptime_seconds()) {
			return -ENOTSUP;
		}
		if (reg_offset < 4) {
			*reg = ds18b20_results[therm_idx].id[3-reg_offset];
		} else {
			*reg = ds18b20_results[therm_idx].temperature_centi;
		}
		return 0;
	}
	switch (addr) {
		case MODBUS_BASE_VIN_REG:
			*reg = adc_vin_v;
			break;
		case MODBUS_BASE_IIN_REG:
			*reg = adc_vin_i;
			break;
		case MODBUS_BASE_3V3_REG:
			*reg = adc_3v3_v;
			break;
		case MODBUS_BASE_TEMPINT_REG:
			*reg = adc_temp_int;
			break;
	    default:
		    return -ENOTSUP;
	}
	return 0;
}

static int holding_reg_rd(uint16_t addr, uint16_t *reg)
{
	LOG_INF("Reading holding reg");
    switch (addr)
	{
		case MODBUS_BASE_STATUS_REG:
			*reg = 0;
			break;
		case MODBUS_BASE_UPTIME_REG:
			*reg = k_uptime_seconds();
			break;
		case MODBUS_PWM0_REG:
			*reg = pwm_reg[0];
			break;
		case MODBUS_PWM1_REG:
		    *reg = pwm_reg[1];
			break;
		case MODBUS_PWM2_REG:
			*reg = pwm_reg[2];
			break;
	default:
		return -ENOTSUP;
	}
	return 0;
}

static int holding_reg_wr(uint16_t addr, uint16_t reg)
{
	LOG_INF("Writing holding reg");
    switch (addr)
	{
		case MODBUS_PWM0_REG:
			pwm_reg[0] = reg;
			set_pwm_outputs();
			break;
		case MODBUS_PWM1_REG:
			pwm_reg[1] = reg;
			set_pwm_outputs();
			break;
		case MODBUS_PWM2_REG:
			pwm_reg[2] = reg;
			set_pwm_outputs();
			break;
	default:
        LOG_INF("Unknown register %u",addr);
        return -ENOTSUP;
	}
	return 0;
}

struct modbus_user_callbacks mbs_cbs = {
	.coil_rd = coil_rd,
	.coil_wr = coil_wr,
    .input_reg_rd = input_reg_rd,
	.holding_reg_rd = holding_reg_rd,
	.holding_reg_wr = holding_reg_wr,
};
