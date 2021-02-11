#ifndef motor_pins_arrays_h
#define motor_pins_arrays_h

#include <avr/pgmspace.h>
#include <utility/motor_pins.h>

const PROGMEM unsigned int motor1_pin_array[] = {MOT1_IN1, MOT1_IN2, MOT1_EN};
//PROGMEM uint8_t    encoder1_pin_array[] = {MOT1_ENCOD1, MOT1_ENCOD2};

const PROGMEM unsigned int motor2_pin_array[] = {MOT2_IN1, MOT2_IN2, MOT2_EN};
//PROGMEM uint8_t    encoder2_pin_array[] = {MOT2_ENCOD1, MOT2_ENCOD2};
#endif
