#ifndef PWM_H
#define PWM_H

#include <zephyr/drivers/pwm.h>

extern uint16_t pwm_reg[3]; // Duty cycles (0-1000)
#endif // PWM_H