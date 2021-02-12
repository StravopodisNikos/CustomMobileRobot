#include "Arduino.h"
#include <L298N.h>
#include <Encoder.h>

#include <utility/config.h>
#include <utility/debug_code.h>
#include <CustomMobileRobot.h>

using namespace std;

//typedef unsigned short PWM_type;
typedef unsigned char debug_error_type;

// Constructor
MecanumEncoderWheel::MecanumEncoderWheel(uint8_t MotorID, uint8_t Motor_EN, uint8_t Motor_IN1, uint8_t Motor_IN2,  uint8_t Encoder_Pin1,  uint8_t Encoder_Pin2) : L298N(Motor_EN,Motor_IN1,Motor_IN2),Encoder(Encoder_Pin1,Encoder_Pin2)
//MecanumEncoderWheel::MecanumEncoderWheel(uint8_t MotorID, uint8_t Motor_EN, uint8_t Motor_IN1, uint8_t Motor_IN2,  uint8_t Encoder_Pin1,  uint8_t Encoder_Pin2) : Encoder(Encoder_Pin1,Encoder_Pin2)
{
    _MotorID        = MotorID;
    _Motor_EN       = Motor_EN;
    _Motor_IN1      = Motor_IN1;
    _Motor_IN2      = Motor_IN2;
    _Encoder_Pin1   = Encoder_Pin1;
    _Encoder_Pin2   = Encoder_Pin2;

    
/*
    pinMode(_Motor_EN, OUTPUT);
    pinMode(_Motor_IN1, OUTPUT);
    pinMode(_Motor_IN2, OUTPUT);

    digitalWrite(_Motor_IN1, LOW);
    digitalWrite(_Motor_IN2, LOW);
    */
}
// =========================================================================================================== //

//bool MecanumEncoderWheel::runFor_FixedRevsSpeed(short revs_d, unsigned short speed_d, uint8_t DIR, volatile bool *KILL_MOTION_TRIGGERED, L298N * ptr2clacc, debug_error_type * debug_error)
bool MecanumEncoderWheel::runFor_FixedRevsSpeed(short revs_d, unsigned short speed_d, uint8_t DIR, volatile bool *KILL_MOTION_TRIGGERED, debug_error_type * debug_error)
{
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
    //ptr2clacc->setSpeed(speed_d);
    //MecanumEncoderWheel::setPwmSpeedWheel(speed_d);

    // calculate pulses for desired revs
    this->pulses_total = 0;
    calculatePulses2Move(revs_d, &pulses_total);

    // run motor until total pulses reached
    this->pulses_remain = 0;

    if (!KILL_MOTION)
    {
        _DIR = DIR;
        //fn_return_state = MecanumEncoderWheel::rotateWheel(ptr2clacc, debug_error);
        fn_return_state = MecanumEncoderWheel::rotateWheel(debug_error);
    }
    else
    {
        fn_return_state = false;
    }


    if (fn_return_state)
    {   // motor rotates
        do
        {
            encoder_current_pulse = Encoder::read();
            Serial.print("CURRENT PULSE="); Serial.println(encoder_current_pulse);

            pulses_remain = pulses_total - encoder_current_pulse;

            if (*KILL_MOTION_TRIGGERED)
            {
                KILL_MOTION = true;
                // stop motor
                L298N::stop();
                //ptr2clacc->stop();
                //MecanumEncoderWheel::stopWheel();
            }
            
        } while ( (pulses_remain > 0) && (!KILL_MOTION) );

        if (!KILL_MOTION)
        {
            *debug_error = NO_ERROR;

            _MOTION_STATE = success;

            // stop motor
            L298N::stop();
            //ptr2clacc->stop();
            //MecanumEncoderWheel::stopWheel();
        }
        else
        {
            *debug_error = MOTION_FAILED;

            _MOTION_STATE = failed;
        }

    }
    else
    {
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

//bool MecanumEncoderWheel::rotateWheel(L298N * ptr2clacc, debug_error_type * debug_error)
bool MecanumEncoderWheel::rotateWheel(debug_error_type * debug_error)
{
    if (_DIR == 0)
    {
        Serial.println("FWD");
        L298N::forward();
        //ptr2clacc->forward();
        //MecanumEncoderWheel::forwardWheel();
        //delay(1000);
        //MecanumEncoderWheel::stopWheel();

    }
    else if (_DIR == 1)
    {
        Serial.println("BWD");
        L298N::backward();
        //MecanumEncoderWheel::backwardWheel();
    }
    else
    {
        Serial.println("NO DIR GIVEN");
        *debug_error = ERROR_IN_rotateWheel;
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
void MecanumEncoderWheel::calculatePulses2Move(short revs_d, long * pulses2move)
{
    *pulses2move = (long) PULS4REV * revs_d; 
}

// =========================================================================================================== //
motion_states MecanumEncoderWheel::getMotionState()
{
    return _MOTION_STATE;
}

// =========================================================================================================== //

void MecanumEncoderWheel::setPwmSpeedWheel(unsigned short speed)
{
    _PWM_SPEED = speed;

    analogWrite(_Motor_EN, _PWM_SPEED);
}

// =========================================================================================================== //

void MecanumEncoderWheel::forwardWheel()
{
    digitalWrite(_Motor_IN1, HIGH);
    digitalWrite(_Motor_IN2, LOW);
}

// =========================================================================================================== //

void MecanumEncoderWheel::backwardWheel()
{
    digitalWrite(_Motor_IN1, LOW);
    digitalWrite(_Motor_IN2, HIGH);
}

// =========================================================================================================== //

void MecanumEncoderWheel::stopWheel()
{
    digitalWrite(_Motor_IN1, LOW);
    digitalWrite(_Motor_IN2, LOW);
}