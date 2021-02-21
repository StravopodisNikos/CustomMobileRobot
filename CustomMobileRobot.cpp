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
}
// =========================================================================================================== //
bool MecanumEncoderWheel::runFor_FixedRevsSpeed(short revs_d, unsigned short speed_d, wheel_rot_dir DIR, volatile bool *KILL_MOTION_TRIGGERED, debug_error_type * debug_error)
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
        *debug_error = NO_ERROR;
    }

    // Checks if motor state==is_moving->stops the motor

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
    _DIR = DIR;

    // start pid
    input_pid    = encoder_current_pulse;
    setpoint_pid = pulses_total;

    PID_v2::Start(input_pid, 75.00, setpoint_pid);    // long->double?
    _output_speed_pid = PID_v2::Run(input_pid);
    // motor started rotating
    Serial.print("PID initial speed="); Serial.println(_output_speed_pid);
    L298N::setSpeed(_output_speed_pid);
    MecanumEncoderWheel::rotateWheel(debug_error);
    delay(100);

    do
    {
        encoder_current_pulse = Encoder::read();
        encoder_current_pulse_abs = abs(encoder_current_pulse);

        Serial.print("CURRENT PULSE="); Serial.println(encoder_current_pulse);

        pulses_remain = abs(pulses_total) - abs( encoder_current_pulse);        // this is the error signal:controller input

        //PID_v2::Start((double) encoder_current_pulse, _output_speed_pid, (double) pulses_total);

        input_pid         = encoder_current_pulse_abs;
        _output_speed_pid = PID_v2::Run(input_pid);
        
        Serial.print("PID new speed="); Serial.println(_output_speed_pid);

        L298N::setSpeed(_output_speed_pid);

        MecanumEncoderWheel::rotateWheel(debug_error);

        //delay(100);

        if (*KILL_MOTION_TRIGGERED)
        {
            // bad thing happened
            KILL_MOTION = true;
            L298N::stop();
        }

        if (pulses_remain <= 0)
        {
            // finished
            KILL_MOTION = true;
            L298N::stop();
        }
        
    } while ( !KILL_MOTION );

    if (!KILL_MOTION)
    {
        *debug_error = NO_ERROR;

        _MOTION_STATE = success;
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
/*
 *                              P R I V A T E -- C L A S S -- F U N C T I O N S
 */
// =========================================================================================================== //

void MecanumEncoderWheel::rotateWheel(debug_error_type * debug_error)
{

    if (_DIR == CW)
    {
        L298N::forward();
        _MOTION_STATE = is_moving;
    }
    else if (_DIR == CCW)
    {
        L298N::backward();
        _MOTION_STATE = is_moving;
    }
    else
    {
        *debug_error = ERROR_IN_rotateWheel;
        _MOTION_STATE = failed;
    }
}

// =========================================================================================================== //
void MecanumEncoderWheel::calculatePulses2Move(short revs_d, long * pulses2move)
{
    *pulses2move = (long) PULS4REV * revs_d; 
}

// =========================================================================================================== //
wheel_motion_states MecanumEncoderWheel::getMotionState()
{
    return _MOTION_STATE;
}