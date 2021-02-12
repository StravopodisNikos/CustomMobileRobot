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
enum motion_states {success, failed};
extern volatile bool KILL_MOTION_TRIGGERED;

class MecanumEncoderWheel: public L298N, public Encoder
//class MecanumEncoderWheel: public Encoder
{
    private:
        uint8_t _MotorID;
        int _Motor_IN1;
        int _Motor_IN2;
        int _Motor_EN;
        int _Encoder_Pin1;
        int _Encoder_Pin2;
        uint8_t _DIR;
        unsigned short _PWM_SPEED;

        int32_t encoder_write_value;
        long encoder_current_pulse;
        long pulses_total;
        long pulses_remain;

        bool KILL_MOTION;
        bool fn_return_state;
        motion_states _MOTION_STATE;

        //using L298N::forward;

        //bool rotateWheel(L298N * ptr2clacc, debug_error_type * debug_error);
        bool rotateWheel(debug_error_type * debug_error);

        void calculatePulses2Move(short revs_d, long * pulses2move);

        motion_states MecanumEncoderWheel::getMotionState();

        void setPwmSpeedWheel(unsigned short speed);

        void forwardWheel();

        void backwardWheel();

        void stopWheel();

    public:
        MecanumEncoderWheel(uint8_t MotorID, uint8_t Motor_EN, uint8_t Motor_IN1, uint8_t Motor_IN2, uint8_t Encoder_Pin1,  uint8_t Encoder_Pin2);

        bool runFor_FixedRevsSpeed(short revs_d, unsigned short speed_d, uint8_t DIR, volatile bool *KILL_MOTION_TRIGGERED, debug_error_type * debug_error );        
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
