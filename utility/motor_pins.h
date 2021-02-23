#ifndef motor_pins_h_
#define motor_pins_h_

//#if defined(__AVR__) || defined(__SAM3X8E__)
// FOR NOW ARDUINO MEGA IS ONLY CONSIDERED
// ARDUINO MEGA INT PINS: 2,3,18,19,20,21
// ARDUINO MEGA PWM PINS: 2-13

#define MOT1_IN1	    22
#define MOT1_IN2        23
#define MOT1_EN        	5   // PWM!
#define MOT1_ENCOD1	    2   // INTERRUPT!
#define MOT1_ENCOD2	    24  // NO INTERRUPT!

#define MOT2_IN1	    26
#define MOT2_IN2        27
#define MOT2_EN        	6   // PWM!
#define MOT2_ENCOD1	    3   // INTERRUPT!
#define MOT2_ENCOD2	    28  // NO INTERRUPT!

#define MOT3_IN1	    30
#define MOT3_IN2        31
#define MOT3_EN         7   // PWM!
#define MOT3_ENCOD1	    18  // INTERRUPT!
#define MOT3_ENCOD2	    32  // NO INTERRUPT!

#define MOT4_IN1	    34
#define MOT4_IN2        35
#define MOT4_EN         8   // PWM!
#define MOT4_ENCOD1	    19  // INTERRUPT!
#define MOT4_ENCOD2	    36  // NO INTERRUPT!

//#endif

#endif
