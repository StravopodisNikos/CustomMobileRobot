 /*
  * CustomMobileRobot.cpp  - Library for controlling DC+Encoder Wheel System for Mobile Robot
  * Created by N.A. Stravopodis, February, 2021.
  */

#ifndef CustomMobileRobot_h
#define CustomMobileRobot_h

#include "Arduino.h"
#include <L298N.h>
#include <Encoder.h>
#include <utility/motor_pins.h>

typedef unsigned char PWM_type;
typedef unsigned char debug_error_type;

extern volatile bool KILL_MOTION_TRIGGERED;

namespace MobileWheel
{
    typedef enum wheel_motion_states {success, failed, ready, is_moving, stopped};
    typedef enum wheel_rot_dir {CCW, CW};

class MecanumEncoderWheel: public L298N, public Encoder
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

        wheel_motion_states _MOTION_STATE;

        //bool rotateWheel(L298N * ptr2l298n, debug_error_type * debug_error);
        void rotateWheel(debug_error_type * debug_error);

        void calculatePulses2Move(short revs_d, long * pulses2move);

        wheel_motion_states getMotionState();
/*
        void setPwmSpeedWheel(unsigned short speed);

        void forwardWheel();

        void backwardWheel();

        void stopWheel();
*/
    public:
        //L298N WheelMotor;
        //L298N WheelMotor(Motor_EN,Motor_IN1,Motor_IN2);
        //Encoder WheelEncoder;
        //Encoder WheelEncoder(Encoder_Pin1,Encoder_Pin2);

        MecanumEncoderWheel(uint8_t MotorID, uint8_t Motor_EN, uint8_t Motor_IN1, uint8_t Motor_IN2,  uint8_t Encoder_Pin1,  uint8_t Encoder_Pin2);

        //bool runFor_FixedRevsSpeed(L298N * ptr2l298n, Encoder * ptr2encoder, short revs_d, unsigned short speed_d, uint8_t DIR, volatile bool *KILL_MOTION_TRIGGERED, debug_error_type * debug_error );        
        bool runFor_FixedRevsSpeed(short revs_d, unsigned short speed_d, wheel_rot_dir DIR, volatile bool *KILL_MOTION_TRIGGERED, debug_error_type * debug_error );        

};

}
/*
class CustomMobileRobot: public MecanumEncoderWheel
{
private:
public:
    CustomMobileRobot();
};

*/



 #endif
