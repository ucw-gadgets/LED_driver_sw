#ifndef ADC_H
#define ADC_H
#include <stdint.h>

extern uint16_t adc_vin_v;
extern uint16_t adc_12v_v;
extern uint16_t adc_3v3_v;
extern uint16_t adc_vin_i;
extern uint16_t adc_temp_int;

int init_adc(void);
int read_adc(void);
int print_die_temperature(void);

#endif // ADC_H