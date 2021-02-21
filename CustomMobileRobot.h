 /*
  * CustomMobileRobot.cpp  - Library for controlling DC+Encoder Wheel System for Mobile Robot
  * Created by N.A. Stravopodis, February, 2021.
  */

#ifndef CustomMobileRobot_h
#define CustomMobileRobot_h

#include "Arduino.h"
#include <L298N.h>
#include <Encoder.h>
#include <PID_v2.h>
#include <utility/motor_pins.h>

typedef unsigned char PWM_type;
typedef unsigned char debug_error_type;

extern volatile bool KILL_MOTION_TRIGGERED;

namespace MobileWheel
{
    typedef enum wheel_motion_states {success, failed, ready, is_moving, stopped};
    typedef enum wheel_rot_dir {CCW, CW};

class MecanumEncoderWheel: public L298N, public Encoder, public PID_v2
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
        double _Kp;
        double _Kd;
        double _Ki;
        double _output_speed_pid;

        int32_t encoder_write_value;
        long encoder_current_pulse;
        long encoder_current_pulse_abs;
        long pulses_total;
        long pulses_remain;

        bool KILL_MOTION;
        bool fn_return_state;

        wheel_motion_states _MOTION_STATE;

        void rotateWheel(debug_error_type * debug_error);

        void calculatePulses2Move(short revs_d, long * pulses2move);

        wheel_motion_states getMotionState();

    public:
        //MecanumEncoderWheel(uint8_t MotorID, uint8_t Motor_EN, uint8_t Motor_IN1, uint8_t Motor_IN2,  uint8_t Encoder_Pin1,  uint8_t Encoder_Pin2);
    
        MecanumEncoderWheel(uint8_t MotorID, uint8_t Motor_EN, uint8_t Motor_IN1, uint8_t Motor_IN2,  uint8_t Encoder_Pin1,  uint8_t Encoder_Pin2, double Kp, double Ki, double Kd, PID::Direction CONT_DIR);

        bool runFor_FixedRevsSpeed(short revs_d, unsigned short speed_d, wheel_rot_dir DIR, volatile bool *KILL_MOTION_TRIGGERED, debug_error_type * debug_error ); 

        bool runFor_FixedRevsPID(double revs_d, wheel_rot_dir DIR, volatile bool *KILL_MOTION_TRIGGERED, debug_error_type * debug_error);
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
