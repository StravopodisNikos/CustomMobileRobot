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
#include <utility/config.h>

typedef unsigned char PWM_type;
typedef unsigned char debug_error_type;

extern volatile bool KILL_MOTION_TRIGGERED;

namespace MobileWheel
{
    typedef enum wheel_motion_states {success, failed, ready, is_moving, stopped};
    typedef enum wheel_rot_dir {ccw, cw};

class MecanumEncoderWheel: public L298N, public Encoder, public PID_v2
{
    private:
        uint8_t _MotorID;
        int _Motor_IN1;
        int _Motor_IN2;
        int _Motor_EN;
        int _Encoder_Pin1;
        int _Encoder_Pin2;
        wheel_rot_dir _DIR;
        unsigned short _PWM_SPEED;
        double _Kp;
        double _Kd;
        double _Ki;
        double _output_speed_pid;

        int32_t encoder_write_value;
        int32_t encoder_current_pulse;
        long encoder_current_pulse_abs;
        long pulses_total;
        long pulses_remain;

        bool KILL_MOTION;
        bool MOTION_FINISHED;
        bool fn_return_state;

        wheel_motion_states _MOTION_STATE;

        int _updateInterval;
        unsigned long _lastUpdate;

        void rotateWheel(debug_error_type * debug_error);

        void calculatePulses2Move(double revs_d, long * pulses2move);

        wheel_motion_states getMotionState();

        void setMotionState(wheel_motion_states new_wheel_state);

    public:    
        MecanumEncoderWheel(uint8_t MotorID, uint8_t Motor_EN, uint8_t Motor_IN1, uint8_t Motor_IN2,  uint8_t Encoder_Pin1,  uint8_t Encoder_Pin2, double Kp, double Ki, double Kd, PID::Direction CONT_DIR);

        bool runFor_FixedRevsSpeed(double revs_d, unsigned short speed_d, wheel_rot_dir DIR, volatile bool *KILL_MOTION_TRIGGERED, debug_error_type * debug_error ); 

        bool runFor_FixedRevsPID(double revs_d, wheel_rot_dir DIR, volatile bool *KILL_MOTION_TRIGGERED, debug_error_type * debug_error);

        // STATE MACHINE IMPLEMENTATION
        void initialize_FixedRevsPID(double revs_d, wheel_rot_dir DIR, volatile bool *KILL_MOTION_TRIGGERED, debug_error_type * debug_error);

        void start_FixedRevsPID( debug_error_type * debug_error);

        void update_FixedRevsPID( volatile bool *KILL_MOTION_TRIGGERED, wheel_motion_states * current_wheel_state , debug_error_type * debug_error);

};

}

namespace MecanumMobileRobot
{
    typedef enum robot_motion_states {SUCCESS, FAILED, READY, IS_MOVING, STOPPED};
    typedef enum robot_dir {BWD, FWD, DIAG_LEFT_FWD, DIAG_RIGHT_FWD, DIAG_LEFT_BWD, DIAG_RIGHT_BWD, ROT_CENTER, ROT_FRONT, ROT_BACK};

    class CustomMobileRobot
    {
        private:

        public:

            CustomMobileRobot();

            MobileWheel::MecanumEncoderWheel *RobotWheel; 
    };
}



/*
namespace MecanumMobileRobot
{
    typedef enum robot_motion_states {SUCCESS, FAILED, READY, IS_MOVING, STOPPED};
    typedef enum robot_wheel_states {success, failed, ready, is_moving, stopped};

    typedef enum robot_dir {BWD, FWD, DIAG_LEFT_FWD, DIAG_RIGHT_FWD, DIAG_LEFT_BWD, DIAG_RIGHT_BWD, ROT_CENTER, ROT_FRONT, ROT_BACK};
    typedef enum robot_wheel_dir{CCW, CW};

//class CustomMobileRobot: public MecanumEncoderWheel
/*
class CustomMobileRobot
{
    private:

        L298N *ptr2l298n_object;

        robot_motion_states _robot_state;
        bool fn_return_state;

        double _Kp[num_ROBOT_WHEELS];
        double _Kd[num_ROBOT_WHEELS];
        double _Ki[num_ROBOT_WHEELS];
        double _output_speed_pid[num_ROBOT_WHEELS];

        double input_pid[num_ROBOT_WHEELS];
        double output_pid[num_ROBOT_WHEELS];
        double setpoint_pid[num_ROBOT_WHEELS];

        int32_t encoder_write_value[num_ROBOT_WHEELS];
        long encoder_current_pulse[num_ROBOT_WHEELS];
        long encoder_current_pulse_abs[num_ROBOT_WHEELS];
        long pulses_total[num_ROBOT_WHEELS];
        long pulses_remain[num_ROBOT_WHEELS];

        uint8_t _DIR[num_ROBOT_WHEELS];

        robot_wheel_states _ROBOT_WHEEL_STATE[num_ROBOT_WHEELS];
        robot_motion_states _ROBOT_STATE;

        unsigned long lastUpdate;

        void calculatePulses2Move2(double * revs_d, long * pulses2move);

        void syncRotateWheel(L298N *ptr2motor, debug_error_type * debug_error, int i);

        void initializeWheelPosition(L298N *ptr2motor, Encoder *ptr2encoder, PID_v2 *ptr2controller, double * revs_d, int i);

        void updateWheelPosition(L298N *ptr2motor, Encoder *ptr2encoder, PID_v2 *ptr2controller, int updateInterval);

    public:
        CustomMobileRobot();

        // this function cannot implement synced 4WD
        bool moveFwd_FixedRevsPID(MobileWheel::MecanumEncoderWheel * ptr2wheel_object, double revs_d, MobileWheel::wheel_rot_dir DIR, volatile bool *KILL_MOTION_TRIGGERED, debug_error_type * debug_error);

        // for synced 4WD
        bool syncRunFor_FixedRevsPID(L298N *ptr2motor, Encoder *ptr2encoder, PID_v2 *ptr2controller, double * revs_d, robot_wheel_dir DIR, volatile bool *KILL_MOTION_TRIGGERED, debug_error_type * debug_error);

};

}
*/
 #endif
