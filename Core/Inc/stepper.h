#ifndef STEPPER_H
#define STEPPER_H

#include "config.h"

void stepper_init(void);
void stepper_stop_axis(uint8_t id);
void stepper_stop_all(void);
void stepper_start_homing(uint8_t id);
void stepper_start_homing_all(void);
void stepper_10us_interrupt(void);
uint8_t stepper_limit_switch_status_bits(void);

#endif
