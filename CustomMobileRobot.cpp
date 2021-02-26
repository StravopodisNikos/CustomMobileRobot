#include "Arduino.h"
#include <L298N.h>
#include <Encoder.h>
#include <PID_v2.h>
#include <utility/config.h>
#include <utility/debug_code.h>
#include <CustomMobileRobot.h>

using namespace MobileWheel;
// Constructors
/*
MecanumEncoderWheel::MecanumEncoderWheel(uint8_t MotorID, uint8_t Motor_EN, uint8_t Motor_IN1, uint8_t Motor_IN2,  uint8_t Encoder_Pin1,  uint8_t Encoder_Pin2): L298N(Motor_EN,Motor_IN1,Motor_IN2),Encoder(Encoder_Pin1,Encoder_Pin2)
{
    _MotorID        = MotorID;
    _Motor_EN       = Motor_EN;
    _Motor_IN1      = Motor_IN1;
    _Motor_IN2      = Motor_IN2;
    _Encoder_Pin1   = Encoder_Pin1;
    _Encoder_Pin2   = Encoder_Pin2;
}
*/
MecanumEncoderWheel::MecanumEncoderWheel(uint8_t MotorID, uint8_t Motor_EN, uint8_t Motor_IN1, uint8_t Motor_IN2,  uint8_t Encoder_Pin1,  uint8_t Encoder_Pin2, double Kp, double Ki, double Kd, PID::Direction CONT_DIR): L298N(Motor_EN,Motor_IN1,Motor_IN2),Encoder(Encoder_Pin1,Encoder_Pin2), PID_v2(Kp, Ki, Kd, CONT_DIR)
{
    _MotorID        = MotorID;
    _Motor_EN       = Motor_EN;
    _Motor_IN1      = Motor_IN1;
    _Motor_IN2      = Motor_IN2;
    _Encoder_Pin1   = Encoder_Pin1;
    _Encoder_Pin2   = Encoder_Pin2;
    _Kp            = Kp;
    _Kd            = Kd;
    _Ki            = Kd;

    _MOTION_STATE  = ready;


    _updateInterval = UPDATE_INTERVAL; // milliseconds
    _lastUpdate     = 0;
/*
    _timeStepIMU = 0.1;         // [sec]
    _updateIMUinterval = 100;   // [milllis]
    _heading_angle = 0;
    _lastIMUupdate = 0;*/
}
// =========================================================================================================== //
bool MecanumEncoderWheel::runFor_FixedRevsSpeed(double revs_d, unsigned short speed_d, wheel_rot_dir DIR, volatile bool *KILL_MOTION_TRIGGERED, debug_error_type * debug_error)
{
    /*
     * Runs motor for specified number of revolutions and desired speed.
     * No inner dc motor control loop. Only encoder pulses counter.
     * THIS IS ABANDONED. USE runFor_FixedRevsPID instead, but first make
     * sure PID parameters are tuned!
     */
    if (*KILL_MOTION_TRIGGERED)
    {
        KILL_MOTION = true;
        *debug_error = KILL_TRIGGER_PRESSED;
    }
    else
    {
        KILL_MOTION = false;
        *debug_error = NO_ERROR;
    }

    // set encoder to zero
    this->encoder_write_value = ZERO_ENC;

    Encoder::write(encoder_write_value);

    //set pulse counter to zero
    this->encoder_current_pulse = 0;

    //set motor speed
    L298N::setSpeed(speed_d);

    // calculate pulses for desired revs
    this->pulses_total = 0;
    calculatePulses2Move(revs_d, &pulses_total);

    // run motor until total pulses reached
    this->pulses_remain = 0;
    _DIR = DIR;

    delay(500);     // TO BE OPTIMIZED

    MecanumEncoderWheel::rotateWheel(debug_error);

    // motor started rotating
    do
    {
        encoder_current_pulse = Encoder::read();

        Serial.print("CURRENT PULSE="); Serial.println(encoder_current_pulse);

        pulses_remain = abs(pulses_total) - abs( encoder_current_pulse);

        if (*KILL_MOTION_TRIGGERED)
        {
            KILL_MOTION = true;
            // stop motor
            L298N::stop();
        }

        delay(5);       // TO BE OPTIMIZED
           
    } while ( (pulses_remain > 0) && (!KILL_MOTION) );
    // motor finished but stops in l.94 OR stopped by kill switch

    if (!KILL_MOTION)
    {
        *debug_error = NO_ERROR;

        _MOTION_STATE = success;

        // stop motor
        L298N::stop();
    }
    else
    {
        *debug_error = MOTION_FAILED;

        _MOTION_STATE = failed;
    }

    if (*debug_error == NO_ERROR)
    {
        return true;
    }
    else
    {
        return false;
    }
}

// =========================================================================================================== //
bool MecanumEncoderWheel::runFor_FixedRevsPID(double revs_d, wheel_rot_dir DIR, volatile bool *KILL_MOTION_TRIGGERED, debug_error_type * debug_error)
{
    /*
     * Runs motor for specified number of revolutions and desired speed.
     * Implements PID. Need to tune the PID parameters!
     */
    double input_pid, output_pid, setpoint_pid;

    // Checks if kill switch is HIGH->ABORTS
    if (*KILL_MOTION_TRIGGERED)
    {
        KILL_MOTION = true;
        *debug_error = KILL_TRIGGER_PRESSED;
    }
    else
    {
        KILL_MOTION = false;
        MOTION_FINISHED = false;
        *debug_error = NO_ERROR;
    }

    // Checks if motor state==is_moving->stops the motor
     if (_MOTION_STATE == is_moving)
    {
        KILL_MOTION  = true;
        *debug_error = MOTOR_IS_MOVING_AT_STARTUP;
    }
    else
    {
        KILL_MOTION = false;
        MOTION_FINISHED = false;
        *debug_error = NO_ERROR;
    }   

    // set encoder to zero
    this->encoder_write_value = ZERO_ENC;

    Encoder::write(encoder_write_value);

    //set pulse counter to zero
    this->encoder_current_pulse = 0;

    // calculate pulses for desired revs
    this->pulses_total = 0;
    calculatePulses2Move(revs_d, &pulses_total);

    // run motor until total pulses reached
    this->pulses_remain = 0;

    // set desired rotation direction
    _DIR = DIR;

    // start pid
    input_pid    = encoder_current_pulse;
    setpoint_pid = pulses_total;

    PID_v2::Start(input_pid, 75.00, setpoint_pid);    // long->double?
    _output_speed_pid = PID_v2::Run(input_pid);
    // motor started rotating
    Serial.print("PID initial speed="); Serial.println(_output_speed_pid);

    Serial.println("START");

    L298N::setSpeed(_output_speed_pid);
    MecanumEncoderWheel::rotateWheel(debug_error);
    delay(100);

    do
    {
        encoder_current_pulse = Encoder::read();
        encoder_current_pulse_abs = abs(encoder_current_pulse);

        Serial.print("CURRENT PULSE="); Serial.println(encoder_current_pulse);

        pulses_remain = abs(pulses_total) - abs( encoder_current_pulse);        // this is the error signal:controller input
        
        Serial.print("PULSES REMAIN="); Serial.println(pulses_remain);
        
        //PID_v2::Start((double) encoder_current_pulse, _output_speed_pid, (double) pulses_total);

        input_pid         = encoder_current_pulse_abs;
        _output_speed_pid = PID_v2::Run(input_pid);
        
        Serial.print("PID new speed="); Serial.println(_output_speed_pid);

        L298N::setSpeed(_output_speed_pid);

        MecanumEncoderWheel::rotateWheel(debug_error);

        delay(5);           // THIS DELAY IS VERY IMPORTANT IN ORDER ENCODER TO ACTUALLY CAN READ PULSES

        if (*KILL_MOTION_TRIGGERED)
        {
            // bad thing happened
            KILL_MOTION = true;
            L298N::stop();
        }

        if (pulses_remain <= 0)
        {
            // finished
            MOTION_FINISHED = true;
            L298N::stop();
        }
        
    } while ( (!KILL_MOTION ) && (!MOTION_FINISHED)  );

    if ( (!KILL_MOTION) && MOTION_FINISHED)
    {
        *debug_error = NO_ERROR;

        _MOTION_STATE = success;

        return true;
    }
    else
    {
        *debug_error = MOTION_FAILED;

        _MOTION_STATE = failed;

        return false;
    }
}

// =========================================================================================================== //
/*
 *                             S T A T E -- M A C H I N E  -- F U N C T I O N S
 */
// =========================================================================================================== //
void MecanumEncoderWheel::initialize_FixedRevsPID(double revs_d, wheel_rot_dir DIR, volatile bool *KILL_MOTION_TRIGGERED, debug_error_type * debug_error)
{
    /*
     *  This function initializes wheel for sync run fixed revs pid.
     */

    double input_pid, output_pid, setpoint_pid;

    // Checks if kill switch is HIGH->ABORTS
    if (*KILL_MOTION_TRIGGERED)
    {
        KILL_MOTION = true;
        MOTION_FINISHED = true;
        *debug_error = KILL_TRIGGER_PRESSED;
    }
    else if (_MOTION_STATE == is_moving)
    {
        KILL_MOTION  = true;
        MOTION_FINISHED = true;
        *debug_error = MOTOR_IS_MOVING_AT_STARTUP;    
    }
    else
    {
        KILL_MOTION = false;
        MOTION_FINISHED = false;
        *debug_error = NO_ERROR;    
    }

    if ( !KILL_MOTION  && (!MOTION_FINISHED))
    {
        // set encoder to zero
        this->encoder_write_value = ZERO_ENC;

        Encoder::write(encoder_write_value);

        //set pulse counter to zero
        this->encoder_current_pulse = 0;

        // calculate pulses for desired revs
        this->pulses_total = 0;
        calculatePulses2Move(revs_d, &pulses_total);

        // run motor until total pulses reached
        this->pulses_remain = 0;

        // set desired rotation direction
        _DIR = DIR;

        // start pid
        input_pid    = encoder_current_pulse;
        setpoint_pid = pulses_total;

        PID_v2::Start(input_pid, 75.00, setpoint_pid);    // long->double?
        _output_speed_pid = PID_v2::Run(input_pid);

        // set new state
        this->_MOTION_STATE = ready;
    }
    else
    {
        // set new state
        this->_MOTION_STATE = failed;
    }
    
}
// =========================================================================================================== //

void MecanumEncoderWheel::start_PID( debug_error_type * debug_error)
{
    if ( _MOTION_STATE == ready)
    {
        // Start motion!
        L298N::setSpeed(this->_output_speed_pid);
        MecanumEncoderWheel::rotateWheel(debug_error);
        *debug_error = NO_ERROR;

        this->_MOTION_STATE = is_moving;
    }
    else
    {
        Serial.print("PID CANNOT START DUE TO STATE ERROR");

        *debug_error = STATE_WHEEL_NOT_READY;

        this->_MOTION_STATE = failed;
    }
    
}
// =========================================================================================================== //

void MecanumEncoderWheel::update_FixedRevsPID( volatile bool *KILL_MOTION_TRIGGERED, wheel_motion_states * current_wheel_state , debug_error_type * debug_error)
{
    // check if state is right
    if ( this->_MOTION_STATE == is_moving)
    { 
        *current_wheel_state = is_moving;

        Serial.println("_MOTION_STATE == is_moving");

        // check if it is time to update
        if( (millis() - _lastUpdate) > _updateInterval )
        {
            this->encoder_current_pulse = Encoder::read();
            //encoder_current_pulse_abs = abs(encoder_current_pulse);

            Serial.print("CURRENT PULSE ["); Serial.print(_MotorID); Serial.print("] = "); Serial.println(this->encoder_current_pulse);

            this->pulses_remain = abs(this->pulses_total) - abs( this->encoder_current_pulse);        // this is the error signal:controller input
            
            Serial.print("PULSES REMAIN ["); Serial.print(_MotorID); Serial.print("] = "); Serial.println(this->pulses_remain);
            
            //input_pid         = encoder_current_pulse_abs;
            this->_output_speed_pid = PID_v2::Run(abs(this->encoder_current_pulse));
            
            Serial.print("PID new speed ["); Serial.print(_MotorID); Serial.print("] = "); Serial.println(this->_output_speed_pid);

            L298N::setSpeed(this->_output_speed_pid);

            MecanumEncoderWheel::rotateWheel(debug_error);

            if (*KILL_MOTION_TRIGGERED)
            {
                Serial.println("Mphka KILL_MOTION_TRIGGERED");
                // bad thing happened
                this->KILL_MOTION = true;
                L298N::stop();
                this->_MOTION_STATE = stopped;

                * debug_error = KILL_TRIGGER_PRESSED;
                
                * current_wheel_state = failed;
            }

            if (this->pulses_remain <= 0)
            {
                Serial.println("Mphka FINISHED");
                // finished!
                this->MOTION_FINISHED = true;
                L298N::stop();
                this->_MOTION_STATE = stopped;

                * debug_error =  NO_ERROR;
                
                * current_wheel_state = success;
            }

            this->_lastUpdate = millis();
        }
    }
    else
    {
        Serial.println("_MOTION_STATE == is not moving");

        *debug_error = STATE_WHEEL_NOT_MOVING;

        this->_MOTION_STATE = failed;   

        * current_wheel_state = failed; 
    }
    
}

// =========================================================================================================== //

//void MecanumEncoderWheel::initialize_HeadingPID(MPU6050 * ptr2mpu, float desired_heading_angle, wheel_rot_dir DIR, volatile bool *KILL_MOTION_TRIGGERED, debug_error_type * debug_error)
void MecanumEncoderWheel::initialize_HeadingPID(float desired_heading_angle, wheel_rot_dir DIR, volatile bool *KILL_MOTION_TRIGGERED, debug_error_type * debug_error)
{
    /*
     *  This function initializes wheel for sync run rotate for fixed angle
     */

    //mpu_sensor = ptr2mpu;   // pointer to single imu sensor
    float current_yaw;

    double input_pid, output_pid, setpoint_pid;

    // Checks if kill switch is HIGH->ABORTS
    if (*KILL_MOTION_TRIGGERED)
    {
        KILL_MOTION = true;
        MOTION_FINISHED = true;
        *debug_error = KILL_TRIGGER_PRESSED;
    }
    else if (_MOTION_STATE == is_moving)
    {
        KILL_MOTION  = true;
        MOTION_FINISHED = true;
        *debug_error = MOTOR_IS_MOVING_AT_STARTUP;    
    }
    else
    {
        KILL_MOTION = false;
        MOTION_FINISHED = false;
        *debug_error = NO_ERROR;    
    }

    if ( !KILL_MOTION  && (!MOTION_FINISHED))
    {
        // measure current IMU angle
        //updateHeadingAngle(this->mpu_sensor);
        current_yaw = 0;

        _desired_heading_angle = desired_heading_angle;

        Serial.print("CURRENT YAW ANGLE = "); Serial.println(current_yaw);

        // set desired rotation direction
        _DIR = DIR;

        // start pid
        input_pid    = current_yaw;
        setpoint_pid = _desired_heading_angle;

        Serial.print("SETPOINT YAW ANGLE = "); Serial.println(setpoint_pid);

        PID_v2::Start(input_pid, 75.00, setpoint_pid);    
        _output_speed_pid = PID_v2::Run(input_pid);

        // set new state
        this->_MOTION_STATE = ready;
    }
    else
    {
        // set new state
        this->_MOTION_STATE = failed;
    }
    
}

// =========================================================================================================== //
void MecanumEncoderWheel::access_yaw_angle(MecanumMobileRobot::CustomMobileRobot & robot_obj, float * current_robot_yaw)
{
    *current_robot_yaw = robot_obj._heading_angle;
}


// =========================================================================================================== //

//void MecanumEncoderWheel::update_HeadingPID(MPU6050 * ptr2mpu, volatile bool *KILL_MOTION_TRIGGERED, wheel_motion_states * current_wheel_state , debug_error_type * debug_error)
void MecanumEncoderWheel::update_HeadingPID(MecanumMobileRobot::CustomMobileRobot & robot_obj, float * current_yaw_measured, volatile bool *KILL_MOTION_TRIGGERED, wheel_motion_states * current_wheel_state , debug_error_type * debug_error)
{
    float delta_angle,current_yaw,desired_yaw;
    Vector normGyroMPU;
    //float timeStepMPU = 0.020;

    //mpu_sensor = ptr2mpu;   // pointer to single imu sensor
    
    // check if state is right
    if ( this->_MOTION_STATE == is_moving)
    { 
        *current_wheel_state = is_moving;

        //Serial.println("_MOTION_STATE == is_moving");

        // check if it is time to update
        if( (millis() - _lastUpdate) > _updateInterval )
        {
            // measure current IMU angle -> ALL CRASH!!!

            // 1 will give error
            //current_yaw = robot_obj.updateHeadingAngle();

            // 2 
            //current_yaw = robot_obj._heading_angle;
            //desired_yaw = robot_obj._desired_heading_angle;
            // 3
            current_yaw = *current_yaw_measured;

            Serial.print("CURRENT YAW ANGLE ["); Serial.print(_MotorID); Serial.print("] = "); Serial.println(current_yaw);
            //Serial.print("DESIRED YAW ANGLE ["); Serial.print(_MotorID); Serial.print("] = "); Serial.println(_desired_heading_angle);

            // calculate delta yaw
            //delta_angle = abs(_desired_heading_angle) - abs(current_yaw);
            delta_angle = _desired_heading_angle - current_yaw;
            Serial.print("delta_angle ["); Serial.print(_MotorID); Serial.print("] = "); Serial.println(delta_angle);

            this->_output_speed_pid = PID_v2::Run(current_yaw);
            
            Serial.print("PID new speed ["); Serial.print(_MotorID); Serial.print("] = "); Serial.println(this->_output_speed_pid);

            L298N::setSpeed(this->_output_speed_pid);

            MecanumEncoderWheel::rotateWheel(debug_error);

            if (*KILL_MOTION_TRIGGERED)
            {
                Serial.println("Mphka KILL_MOTION_TRIGGERED");
                // bad thing happened
                this->KILL_MOTION = true;
                L298N::stop();
                this->_MOTION_STATE = stopped;

                * debug_error = KILL_TRIGGER_PRESSED;
                
                * current_wheel_state = failed;
            }

            if (delta_angle <= 0)
            {
                Serial.println("Mphka FINISHED");
                // finished!
                this->MOTION_FINISHED = true;
                L298N::stop();
                this->_MOTION_STATE = stopped;

                * debug_error =  NO_ERROR;
                
                * current_wheel_state = success;
            }

            this->_lastUpdate = millis();

            //*last_yaw = current_yaw;
        }
    }
    else
    {
        Serial.println("_MOTION_STATE == is not moving");

        *debug_error = STATE_WHEEL_NOT_MOVING;

        this->_MOTION_STATE = failed;   

        * current_wheel_state = failed; 
    }
    
}

// =========================================================================================================== //
/*
 *                              P R I V A T E -- C L A S S -- F U N C T I O N S
 */
// =========================================================================================================== //

void MecanumEncoderWheel::rotateWheel(debug_error_type * debug_error)
{

    if (_DIR == cw)
    {
        L298N::forward();
        _MOTION_STATE = is_moving;
    }
    else if (_DIR == ccw)
    {
        L298N::backward();
        _MOTION_STATE = is_moving;
    }
    else if (_DIR == stall)
    {
        L298N::stop();
        _MOTION_STATE = is_stall;
    }
    else
    {
        *debug_error = ERROR_IN_rotateWheel;
        _MOTION_STATE = failed;
    }
}

// =========================================================================================================== //
void MecanumEncoderWheel::calculatePulses2Move(double revs_d, long * pulses2move)
{
    *pulses2move = (long) PULS4REV * revs_d; 
}

// =========================================================================================================== //
wheel_motion_states MecanumEncoderWheel::getMotionState()
{
    return _MOTION_STATE;
}
// =========================================================================================================== //
void MecanumEncoderWheel::setMotionState(wheel_motion_states * new_wheel_state)
{
    this->_MOTION_STATE = * new_wheel_state;
}


// =========================================================================================================== //
// NEW CLASS FOR ROBOT SYSTEM -> IMPLEMENTS STATE MACHINE FOR SYNC MOTION - S I M P L E
// =========================================================================================================== //

using namespace MecanumMobileRobot;

CustomMobileRobot::CustomMobileRobot()
{
    _ROBOT_MOTION_STATE  = READY;

    _timeStepIMU = 0.1;
    _updateIMUinterval = 100;
    _lastIMUupdate = 0;
    _heading_angle = 0;
    _normGyro.XAxis = 0;
    _normGyro.YAxis = 0;
    _normGyro.ZAxis = 0;
};

void CustomMobileRobot::setRobotDir(robot_dir DESIRED_DIR, MobileWheel::wheel_rot_dir * WHEEL_DIRS, debug_error_type * debug_error)
{
    // ALL DIRS ARE PRESENTED IN /jpg/robot-4wd-dir.jpg

    switch (DESIRED_DIR)
    {
    case BWD:
        WHEEL_DIRS[0] = MobileWheel::wheel_rot_dir::ccw;
        WHEEL_DIRS[1] = MobileWheel::wheel_rot_dir::ccw;
        //WHEEL_DIRS[2] = MobileWheel::wheel_rot_dir::ccw;
        //WHEEL_DIRS[3] = MobileWheel::wheel_rot_dir::ccw;
        *debug_error = NO_ERROR;
        break;
    case FWD:
        WHEEL_DIRS[0] = MobileWheel::wheel_rot_dir::cw;
        WHEEL_DIRS[1] = MobileWheel::wheel_rot_dir::cw;
        //WHEEL_DIRS[2] = MobileWheel::wheel_rot_dir::cw;
        //WHEEL_DIRS[3] = MobileWheel::wheel_rot_dir::cw;
        *debug_error = NO_ERROR;
        break;
    case DIAG_LEFT_FWD:
        WHEEL_DIRS[0] = MobileWheel::wheel_rot_dir::stall;
        WHEEL_DIRS[1] = MobileWheel::wheel_rot_dir::cw;
        //WHEEL_DIRS[2] = MobileWheel::wheel_rot_dir::cw;
        //WHEEL_DIRS[3] = MobileWheel::wheel_rot_dir::stall;
        *debug_error = NO_ERROR;
        break;
    case DIAG_RIGHT_FWD:
        WHEEL_DIRS[0] = MobileWheel::wheel_rot_dir::cw;
        WHEEL_DIRS[1] = MobileWheel::wheel_rot_dir::stall;
        //WHEEL_DIRS[2] = MobileWheel::wheel_rot_dir::stall;
        //WHEEL_DIRS[3] = MobileWheel::wheel_rot_dir::cw;
        *debug_error = NO_ERROR;
        break;
    case DIAG_LEFT_BWD:
        WHEEL_DIRS[0] = MobileWheel::wheel_rot_dir::ccw;
        WHEEL_DIRS[1] = MobileWheel::wheel_rot_dir::stall;
        //WHEEL_DIRS[2] = MobileWheel::wheel_rot_dir::stall;
        //WHEEL_DIRS[3] = MobileWheel::wheel_rot_dir::ccw;
        *debug_error = NO_ERROR;
        break;
    case DIAG_RIGHT_BWD:
        WHEEL_DIRS[0] = MobileWheel::wheel_rot_dir::stall;
        WHEEL_DIRS[1] = MobileWheel::wheel_rot_dir::ccw;
        //WHEEL_DIRS[2] = MobileWheel::wheel_rot_dir::ccw;
        //WHEEL_DIRS[3] = MobileWheel::wheel_rot_dir::stall;
        *debug_error = NO_ERROR;
        break;
    case LEFT:
        WHEEL_DIRS[0] = MobileWheel::wheel_rot_dir::ccw;
        WHEEL_DIRS[1] = MobileWheel::wheel_rot_dir::cw;
        //WHEEL_DIRS[2] = MobileWheel::wheel_rot_dir::cw;
        //WHEEL_DIRS[3] = MobileWheel::wheel_rot_dir::ccw;
        *debug_error = NO_ERROR;
        break;
    case RIGHT:
        WHEEL_DIRS[0] = MobileWheel::wheel_rot_dir::cw;
        WHEEL_DIRS[1] = MobileWheel::wheel_rot_dir::ccw;
        //WHEEL_DIRS[2] = MobileWheel::wheel_rot_dir::ccw;
        //WHEEL_DIRS[3] = MobileWheel::wheel_rot_dir::cw;
        *debug_error = NO_ERROR;
        break;
    case ROT_CENTER_CW:
        WHEEL_DIRS[0] = MobileWheel::wheel_rot_dir::cw;
        WHEEL_DIRS[1] = MobileWheel::wheel_rot_dir::ccw;
        //WHEEL_DIRS[2] = MobileWheel::wheel_rot_dir::cw;
        //WHEEL_DIRS[3] = MobileWheel::wheel_rot_dir::ccw;
        *debug_error = NO_ERROR;
        break;
    case ROT_CENTER_CCW:
        WHEEL_DIRS[0] = MobileWheel::wheel_rot_dir::ccw;
        WHEEL_DIRS[1] = MobileWheel::wheel_rot_dir::cw;
        //WHEEL_DIRS[2] = MobileWheel::wheel_rot_dir::ccw;
        //WHEEL_DIRS[3] = MobileWheel::wheel_rot_dir::cw;
        *debug_error = NO_ERROR;
        break;
    case ROT_FRONT_CW:
        WHEEL_DIRS[0] = MobileWheel::wheel_rot_dir::stall;
        WHEEL_DIRS[1] = MobileWheel::wheel_rot_dir::stall;
        //WHEEL_DIRS[2] = MobileWheel::wheel_rot_dir::cw;
        //WHEEL_DIRS[3] = MobileWheel::wheel_rot_dir::ccw;
        *debug_error = NO_ERROR;
        break;
    case ROT_FRONT_CCW:
        WHEEL_DIRS[0] = MobileWheel::wheel_rot_dir::stall;
        WHEEL_DIRS[1] = MobileWheel::wheel_rot_dir::stall;
        //WHEEL_DIRS[2] = MobileWheel::wheel_rot_dir::ccw;
        //WHEEL_DIRS[3] = MobileWheel::wheel_rot_dir::cw;
        *debug_error = NO_ERROR;
        break;
    case ROT_BACK_CW:
        WHEEL_DIRS[0] = MobileWheel::wheel_rot_dir::cw;
        WHEEL_DIRS[1] = MobileWheel::wheel_rot_dir::ccw;
        //WHEEL_DIRS[2] = MobileWheel::wheel_rot_dir::stall;
        //WHEEL_DIRS[3] = MobileWheel::wheel_rot_dir::stall;
        *debug_error = NO_ERROR;
        break;
    case ROT_BACK_CCW:
        WHEEL_DIRS[0] = MobileWheel::wheel_rot_dir::ccw;
        WHEEL_DIRS[1] = MobileWheel::wheel_rot_dir::cw;
        //WHEEL_DIRS[2] = MobileWheel::wheel_rot_dir::stall;
        //WHEEL_DIRS[3] = MobileWheel::wheel_rot_dir::stall;
        *debug_error = NO_ERROR;
        break;      
    case CORNER_BACK_RIGHT:
        WHEEL_DIRS[0] = MobileWheel::wheel_rot_dir::cw;
        WHEEL_DIRS[1] = MobileWheel::wheel_rot_dir::stall;
        //WHEEL_DIRS[2] = MobileWheel::wheel_rot_dir::cw;
        //WHEEL_DIRS[3] = MobileWheel::wheel_rot_dir::stall;
        *debug_error = NO_ERROR;
        break;  
    case CORNER_BACK_LEFT:
        WHEEL_DIRS[0] = MobileWheel::wheel_rot_dir::stall;
        WHEEL_DIRS[1] = MobileWheel::wheel_rot_dir::cw;
        //WHEEL_DIRS[2] = MobileWheel::wheel_rot_dir::stall;
        //WHEEL_DIRS[3] = MobileWheel::wheel_rot_dir::cw;
        *debug_error = NO_ERROR;
        break;  
    case CORNER_FRONT_RIGHT:
        WHEEL_DIRS[0] = MobileWheel::wheel_rot_dir::ccw;
        WHEEL_DIRS[1] = MobileWheel::wheel_rot_dir::stall;
        //WHEEL_DIRS[2] = MobileWheel::wheel_rot_dir::ccw;
        //WHEEL_DIRS[3] = MobileWheel::wheel_rot_dir::stall;
        *debug_error = NO_ERROR;
        break;  
    case CORNER_FRONT_LEFT:
        WHEEL_DIRS[0] = MobileWheel::wheel_rot_dir::stall;
        WHEEL_DIRS[1] = MobileWheel::wheel_rot_dir::ccw;
        //WHEEL_DIRS[2] = MobileWheel::wheel_rot_dir::stall;
        //WHEEL_DIRS[3] = MobileWheel::wheel_rot_dir::ccw;
        *debug_error = NO_ERROR;
        break;       
    default:
        *debug_error = WRONG_DIR_GIVEN;
        break;
    }
}

// =========================================================================================================== //

bool CustomMobileRobot::driveFor_FixedRevsPID(MobileWheel::MecanumEncoderWheel * ptr2RobotWheel, double * WHEEL_REVS, MobileWheel::wheel_rot_dir * WHEEL_DIRS, MobileWheel::wheel_motion_states * WHEEL_STATES, volatile bool *KILL_MOTION_TRIGGERED, debug_error_type * debug_error)
{
    // this drives the robot for specified number of revs and specified wheel directions

    RobotWheel = ptr2RobotWheel;    // this is a pointer to an array of wheel objects

    _TERMINATE_MOTION = false;

    _MOTOR_STILL_MOVING = true;

    // initialize
    for (size_t i = 0; i < num_ROBOT_WHEELS; i++)
    {
        (ptr2RobotWheel+i)->initialize_FixedRevsPID(WHEEL_REVS[i], WHEEL_DIRS[i], KILL_MOTION_TRIGGERED, debug_error);
    }
    // start
    for (size_t i = 0; i < num_ROBOT_WHEELS; i++)
    {
        (ptr2RobotWheel+i)->start_PID( debug_error );
    }
    // update
    do
    {
        // update the objects state
        for (size_t i = 0; i < num_ROBOT_WHEELS; i++)
        {
            (ptr2RobotWheel+i)->update_FixedRevsPID(KILL_MOTION_TRIGGERED, (WHEEL_STATES+i),debug_error );
        }

        // inside state machine loop always check the current state of motors
        for (size_t i = 0; i < num_ROBOT_WHEELS; i++)
        {
            if (WHEEL_STATES[i] == MobileWheel::wheel_motion_states::is_moving)
            {
                _MOTOR_STILL_MOVING = true;
                break;
            }
            else
            {
                _MOTOR_STILL_MOVING = false;
            } 
        }
            
        // if not even 1 motor turning terminate the state-machine loop
        if( (!_MOTOR_STILL_MOVING) ) 
        {
            _TERMINATE_MOTION = true;
        }
    }while(!_TERMINATE_MOTION);

    return true;
}

// =========================================================================================================== //

bool CustomMobileRobot::rotateFor_FixedYawPID(CustomMobileRobot * ptr2RobotObject, MPU6050 * ptr2mpu, MobileWheel::MecanumEncoderWheel * ptr2RobotWheel, float  desired_heading_angle, MobileWheel::wheel_rot_dir * WHEEL_DIRS, MobileWheel::wheel_motion_states * WHEEL_STATES, volatile bool *KILL_MOTION_TRIGGERED, debug_error_type * debug_error)
{
    // this rotates the robot for the desired yaw angle

    float yaw_measured_here;

    ptr2mpu = & mpu_sensor;           // pointer to mpu object

    RobotWheel = ptr2RobotWheel;    // this is a pointer to an array of wheel objects

    //RobotObject = ptr2RobotObject;  // this is ptr to object of robot and is passed to friend class
    ptr2RobotObject = RobotObject;

    _TERMINATE_MOTION = false;

    //_MOTOR_STILL_MOVING = true;
    //*
    // open imu sensor;
    //initializeMPU(mpu_sensor);
    //*/

    // initialize
    for (size_t i = 0; i < num_ROBOT_WHEELS; i++)
    {
        (ptr2RobotWheel+i)->initialize_HeadingPID(desired_heading_angle, WHEEL_DIRS[i], KILL_MOTION_TRIGGERED, debug_error);
    }
    // start
    for (size_t i = 0; i < num_ROBOT_WHEELS; i++)
    {
        (ptr2RobotWheel+i)->start_PID( debug_error );
    }
    // update
    //float yaw = 0;
    do
    {
        // update the objects state
        for (size_t i = 0; i < num_ROBOT_WHEELS; i++)
        {
            //_heading_angle = updateHeadingAngle(ptr2mpu);
            yaw_measured_here = updateHeadingAngle(ptr2mpu);

            (ptr2RobotWheel+i)->update_HeadingPID(*ptr2RobotObject, &yaw_measured_here, KILL_MOTION_TRIGGERED, (WHEEL_STATES+i),debug_error );
        }

        // inside state machine loop always check the current state of motors
        for (size_t i = 0; i < num_ROBOT_WHEELS; i++)
        {
            if (WHEEL_STATES[i] == MobileWheel::wheel_motion_states::is_moving)
            {
                _MOTOR_STILL_MOVING = true;
                break;
            }
            else
            {
                _MOTOR_STILL_MOVING = false;
            } 
        }

        Serial.print("_MOTOR_STILL_MOVING = "); Serial.println(_MOTOR_STILL_MOVING);

        // if not even 1 motor turning terminate the state-machine loop
        if( (!_MOTOR_STILL_MOVING) ) 
        {
            _TERMINATE_MOTION = true;
        }

    }while(!_TERMINATE_MOTION);

    return true;
}

// =========================================================================================================== //

float CustomMobileRobot::updateHeadingAngle(MPU6050 * ptr2mpu)
{
    float current_heading_angle,var_time_step;

    ptr2mpu = & mpu_sensor;

    if ( (millis() - _lastIMUupdate) > _updateIMUinterval )
    {
        _normGyro  = ptr2mpu->readNormalizeGyro(); 

        var_time_step = (float) ((millis() - _lastIMUupdate) / 1000.0);
        _heading_angle = _heading_angle + _normGyro.ZAxis * var_time_step;

        current_heading_angle = _heading_angle;

        _lastIMUupdate = millis();
    }
    else
    {
        current_heading_angle = _heading_angle;
    }

    return current_heading_angle;
}

// =========================================================================================================== //
/*
 *                              P R I V A T E -- C L A S S -- F U N C T I O N S
 */
// =========================================================================================================== //

void CustomMobileRobot::initializeMPU(MPU6050 * ptr2mpu, debug_error_type * debug_error)
{
    //mpu_sensor = & ptr2mpu;

    ptr2mpu = & mpu_sensor;

    while(!ptr2mpu->begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G))
    {
        *debug_error = MPU_NOT_CONNECTED;
        delay(100);
    }

    ptr2mpu->calibrateGyro();
    ptr2mpu->setThreshold(3);

    *debug_error = NO_ERROR;
}