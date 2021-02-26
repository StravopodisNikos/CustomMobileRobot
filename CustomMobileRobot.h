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
#include <Wire.h>
#include <MPU6050.h>

typedef unsigned char PWM_type;
typedef unsigned char debug_error_type;

extern volatile bool KILL_MOTION_TRIGGERED;

namespace MecanumMobileRobot{
    typedef enum robot_motion_states {SUCCESS, FAILED, READY, IS_MOVING, STOPPED};
    typedef enum robot_dir {BWD, FWD, DIAG_LEFT_FWD, DIAG_RIGHT_FWD, DIAG_LEFT_BWD, DIAG_RIGHT_BWD, LEFT, RIGHT, ROT_CENTER_CW, ROT_CENTER_CCW, ROT_FRONT_CW, ROT_FRONT_CCW, ROT_BACK_CW, ROT_BACK_CCW, CORNER_BACK_RIGHT, CORNER_BACK_LEFT, CORNER_FRONT_RIGHT, CORNER_FRONT_LEFT};

    class CustomMobileRobot;
}


namespace MobileWheel
{
    typedef enum wheel_motion_states {success, failed, ready, is_moving, is_stall, stopped};
    typedef enum wheel_rot_dir {ccw, cw, stall=-1};

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

        void setMotionState(wheel_motion_states * new_wheel_state);

        //float updateHeadingAngle(MPU6050 * ptr2mpu);

        // IMU 
        float _desired_heading_angle;
        //float _heading_angle;           // this is the yaw angle
        //float _timeStepIMU;             // [sec]

        //int _updateIMUinterval;         // = (int) _timeStepIMU*1000; [millis]
        //unsigned long _lastIMUupdate;

        //Vector _normGyro;

    public:  

        //MPU6050 * mpu_sensor;       // pointer to single imu sensor object
  
        MecanumEncoderWheel(uint8_t MotorID, uint8_t Motor_EN, uint8_t Motor_IN1, uint8_t Motor_IN2,  uint8_t Encoder_Pin1,  uint8_t Encoder_Pin2, double Kp, double Ki, double Kd, PID::Direction CONT_DIR);

        bool runFor_FixedRevsSpeed(double revs_d, unsigned short speed_d, wheel_rot_dir DIR, volatile bool *KILL_MOTION_TRIGGERED, debug_error_type * debug_error ); 

        bool runFor_FixedRevsPID(double revs_d, wheel_rot_dir DIR, volatile bool *KILL_MOTION_TRIGGERED, debug_error_type * debug_error);

        // STATE MACHINE IMPLEMENTATION
        void initialize_FixedRevsPID(double revs_d, wheel_rot_dir DIR, volatile bool *KILL_MOTION_TRIGGERED, debug_error_type * debug_error);

        void start_PID( debug_error_type * debug_error);

        void update_FixedRevsPID( volatile bool *KILL_MOTION_TRIGGERED, wheel_motion_states * current_wheel_state , debug_error_type * debug_error);

        //void initialize_HeadingPID(MPU6050 * ptr2mpu, float desired_heading_angle, wheel_rot_dir DIR, volatile bool *KILL_MOTION_TRIGGERED, debug_error_type * debug_error);
        void initialize_HeadingPID(float desired_heading_angle, wheel_rot_dir DIR, volatile bool *KILL_MOTION_TRIGGERED, debug_error_type * debug_error);

        //void update_HeadingPID(MPU6050 * ptr2mpu,  volatile bool *KILL_MOTION_TRIGGERED, wheel_motion_states * current_wheel_state , debug_error_type * debug_error);
        void update_HeadingPID(MecanumMobileRobot::CustomMobileRobot & robot_obj, float * current_yaw_measured, volatile bool *KILL_MOTION_TRIGGERED, wheel_motion_states * current_wheel_state , debug_error_type * debug_error);

        void access_yaw_angle(MecanumMobileRobot::CustomMobileRobot & robot_obj, float * current_robot_yaw);
};

}

//namespace MecanumMobileRobot
namespace MecanumMobileRobot
{
    //typedef enum robot_motion_states {SUCCESS, FAILED, READY, IS_MOVING, STOPPED};
    //typedef enum robot_dir {BWD, FWD, DIAG_LEFT_FWD, DIAG_RIGHT_FWD, DIAG_LEFT_BWD, DIAG_RIGHT_BWD, LEFT, RIGHT, ROT_CENTER_CW, ROT_CENTER_CCW, ROT_FRONT_CW, ROT_FRONT_CCW, ROT_BACK_CW, ROT_BACK_CCW, CORNER_BACK_RIGHT, CORNER_BACK_LEFT, CORNER_FRONT_RIGHT, CORNER_FRONT_LEFT};

    class CustomMobileRobot //public MecanumEncoderWheel
    {
        private:
            bool _TERMINATE_MOTION;
            bool _MOTOR_STILL_MOVING;
        
            robot_motion_states _ROBOT_MOTION_STATE;

            float _desired_heading_angle;
            float _heading_angle;           // this is the yaw angle
            float _timeStepIMU;             // [sec]

            int _updateIMUinterval;         // = (int) _timeStepIMU*1000; [millis]
            unsigned long _lastIMUupdate;

            Vector _normGyro;

            

        public:
            MobileWheel::MecanumEncoderWheel * RobotWheel;  // pointer to wheel objects array

            MPU6050 mpu_sensor;       // object of mpu6050 as public member of robot object

            CustomMobileRobot * RobotObject;

            CustomMobileRobot();

            void setRobotDir(robot_dir DESIRED_DIR, MobileWheel::wheel_rot_dir * WHEEL_DIRS, debug_error_type * debug_error);

            bool driveFor_FixedRevsPID(MobileWheel::MecanumEncoderWheel * ptr2RobotWheel, double * WHEEL_REVS, MobileWheel::wheel_rot_dir * WHEEL_DIRS, MobileWheel::wheel_motion_states * WHEEL_STATES, volatile bool *KILL_MOTION_TRIGGERED, debug_error_type * debug_error);       
    
            bool rotateFor_FixedYawPID(CustomMobileRobot * ptr2RobotObject, MPU6050 * ptr2mpu, MobileWheel::MecanumEncoderWheel * ptr2RobotWheel, float  desired_heading_angle, MobileWheel::wheel_rot_dir * WHEEL_DIRS, MobileWheel::wheel_motion_states * WHEEL_STATES, volatile bool *KILL_MOTION_TRIGGERED, debug_error_type * debug_error);

            float updateHeadingAngle(MPU6050 * ptr2mpu);

            void initializeMPU(MPU6050 * ptr2mpu, debug_error_type * debug_error);

            friend class MobileWheel::MecanumEncoderWheel;
    };
}

#endif
