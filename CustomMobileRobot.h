 /*
  * DynamixelProPlusOvidiusShield.cpp  - Library for controlling Dynamixels Pro+ of a Metamorphic Manipulator using DunamixelShield-Dynamixel2Arduino ROBOTIS libraries
  * Created by N.A. Stravopodis, December, 2020.
  */

#ifndef CustomMobileRobot_h
#define CustomMobileRobot_h

#include "Arduino.h"
#include <L298N.h>
#include <Encoder.h>
using namespace std;

typedef unsigned char PWM_type;
typedef unsigned char debug_error_type;

class MecanumEncoderWheel: public L298N, public Encoder
{
    private:

        uint8_t _MotorID;
        int _Motor_IN1;
        int _Motor_IN2;
        int _Motor_EN;
        int _Encoder_Pin1;
        int _Encoder_Pin2;

        int32_t encoder_write_value;
        long encoder_current_pulse;
        long pulses_total;
        long pulses_remain;

        bool fn_return_state;

        bool rotateWheel(uint8_t DIR, debug_error_type * debug_error);

        void calculatePulses2Move(short revs_d, long * pulses2move);

    public:
        //L298N Motor(const unsigned int IN1, const unsigned int IN2, const unsigned int EN );

        //Encoder MotorEncoder(uint8_t INT_PIN1, uint8_t INT_PIN2);

        MecanumEncoderWheel(uint8_t MotorID, const unsigned int Motor_EN, const unsigned int Motor_IN1, const unsigned int Motor_IN2, uint8_t Encoder_Pin1,  uint8_t Encoder_Pin2);

        bool runFor_FixedRevsSpeed(short revs_d, PWM_type speed_d, uint8_t DIR, debug_error_type * debug_error );

        
};

/*
class CustomMobileRobot: public MecanumEncoderWheel
{
private:
public:
    CustomMobileRobot();
};

*/



 #endif
