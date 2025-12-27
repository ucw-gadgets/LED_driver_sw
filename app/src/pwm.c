#include "pwm.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(pwm, LOG_LEVEL_DBG);

static const struct pwm_dt_spec pwm_o1 =
	PWM_DT_SPEC_GET(DT_ALIAS(pwmled1));
static const struct pwm_dt_spec pwm_o2 =
	PWM_DT_SPEC_GET(DT_ALIAS(pwmled2));
static const struct pwm_dt_spec pwm_o3 =
	PWM_DT_SPEC_GET(DT_ALIAS(pwmled3));

uint16_t pwm_reg[3] = {0, 0, 0 }; // Initial duty cycles (0-1000)

int init_pwm(void)
{
    LOG_INF("Initializing PWM devices");
    if (!pwm_is_ready_dt(&pwm_o1) ||
        !pwm_is_ready_dt(&pwm_o2) ||
        !pwm_is_ready_dt(&pwm_o3)) {
        LOG_ERR("One or more PWM devices not ready");
        return -ENODEV;
    }
    return 0;
}

int set_pwm_outputs(void)
{
    int ret;
    uint32_t raw_brightness;

    // Set PWM output 1
    raw_brightness = pwm_reg[0] * 65536 / 1000; // scale 0-1000 to 0-65536
    ret = pwm_set_pulse_dt(&pwm_o1, PWM_NSEC(raw_brightness));
    if (ret) {
        LOG_ERR("Error %d setting PWM output 1", ret);
        return ret;
    }

    // Set PWM output 2
    raw_brightness = pwm_reg[1] * 65536 / 1000; // scale 0-1000 to 0-65536
    ret = pwm_set_pulse_dt(&pwm_o2, PWM_NSEC(raw_brightness));
    if (ret) {
        LOG_ERR("Error %d setting PWM output 2", ret);
        return ret;
    }

    // Set PWM output 3
    raw_brightness = pwm_reg[2] * 65536 / 1000; // scale 0-1000 to 0-65536
    ret = pwm_set_pulse_dt(&pwm_o3, PWM_NSEC(raw_brightness));
    if (ret) {
        LOG_ERR("Error %d setting PWM output 3", ret);
        return ret;
    }

    return 0;
}