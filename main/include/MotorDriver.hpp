/*

    A class for motor control.

*/

#pragma once

#include <driver/gpio.h>
#include <driver/ledc.h>

#include "PID.hpp"

#include "SensorReader.hpp"
#include "ConfigLoader.hpp"

class MotorDriver
{
    private:

    void init_gpio();

    void init_ledc();

    // a yaw that class try robot to set to
    float target_yaw;

    // set a max power of both engines, from 0 to 4095
    uint16_t power;

    // a power for turning
    uint16_t turning_power;

    PID<float> motorA;
    PID<float> motorB;

    bool automaticMode;


    public:

    MotorDriver()
    : target_yaw(0.f),
    power(0),
    turning_power(0),
    motorA(1.f,0.f,0.f),
    motorB(-1.f,0.f,0.f),
    automaticMode(true)
    {
        this->motorA.setMin(0.f);
        this->motorA.setMax(1.f);
        this->motorA.setTimeStep(SAMPLE_TIME);

        this->motorB.setMin(0.f);
        this->motorB.setMax(1.f);
        this->motorB.setTimeStep(SAMPLE_TIME);
    }

    void init();

    void setPIDA(const config::MotorPID& pid);

    void setPIDB(const config::MotorPID& pid);

    void setMotorConfig(const config::MotorCFG& cfg);

    config::MotorPID PIDA();

    config::MotorPID PIDB();

    config::MotorCFG MotorConfig();

    static bool channelADirection();

    static bool channelBDirection();

    // main drive function
    void loop();

    void setTargetAngel(const float& target)
    {
        this->target_yaw=target;
    }

    const float& TargetAngel() const
    {
        return this->target_yaw;
    }

    void setPower(const uint16_t& power)
    {
        this->power=power;
    }

    const uint16_t& Power() const
    {
        return this->power;
    }

    void setTurningPower(const uint16_t& power)
    {
        this->turning_power=power;
    }

    const uint16_t& TurningPower() const
    {
        return this->turning_power;
    }

    void setAutomaticMode(bool mode)
    {
        this->automaticMode=mode;
    }

    const bool& AutomaticMode()
    {
        return this->automaticMode;
    }

    // set motor A power and spinning direction
    void set_channelA(const int32_t& pwr);

    // set motor B power and spinning direction
    void set_channelB(const int32_t& pwr);

    // stop engines
    void stop();

};