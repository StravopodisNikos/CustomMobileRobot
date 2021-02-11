#include "Arduino.h"
#include <L298N.h>
#include <Encoder.h>

#include <utility/config.h>
#include <utility/debug_code.h>
#include <CustomMobileRobot.h>

using namespace std;

typedef unsigned char PWM_type;
typedef unsigned char debug_error_type;

// Constructor
MecanumEncoderWheel::MecanumEncoderWheel(uint8_t MotorID, const unsigned int Motor_EN, const unsigned int Motor_IN1, const unsigned int Motor_IN2,  uint8_t Encoder_Pin1,  uint8_t Encoder_Pin2) : L298N(Motor_EN,Motor_IN1,Motor_IN2),Encoder(Encoder_Pin1,Encoder_Pin2)
{
    _MotorID        = MotorID;
    _Motor_EN       = Motor_EN;
    _Motor_IN1      = Motor_IN1;
    _Motor_IN2      = Motor_IN2;
    _Encoder_Pin1   = Encoder_Pin1;
    _Encoder_Pin2   = Encoder_Pin2;
}
// =========================================================================================================== //

bool MecanumEncoderWheel::runFor_FixedRevsSpeed(short revs_d, PWM_type speed_d, uint8_t DIR, debug_error_type * debug_error)
{
bool KILL_MOTION = false;

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
fn_return_state = MecanumEncoderWheel::rotateWheel(DIR, debug_error);
do
{
    if (fn_return_state)
    {
        encoder_current_pulse = Encoder::read();

        pulses_remain = pulses_total - encoder_current_pulse;
    }
    else
    {
        KILL_MOTION = true;
        Serial.println("MOTION KILLED");
    }

    Serial.println(pulses_remain);

} while ( (pulses_remain > 0) && (!KILL_MOTION) );

// stop motor
L298N::stop();

}
// =========================================================================================================== //
/*
 *                              P R I V A T E -- C L A S S -- F U N C T I O N S
 */
// =========================================================================================================== //

bool MecanumEncoderWheel::rotateWheel(uint8_t DIR, debug_error_type * debug_error)
{

    if (DIR == 0)
    {
        L298N::forward();
    }
    else if (DIR == 1)
    {
        L298N::backward();
    }
    else
    {
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