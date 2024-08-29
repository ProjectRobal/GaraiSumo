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
    float target_speed;

    // a power for turning
    float turning_power;

    PID<float> motorA;

    PID<float> motorLeft;
    PID<float> motorRight;
    
    bool automaticMode;

    StackType_t* motor_stack;
    StaticTask_t motor_task;

    bool last_A_direction;
    bool last_B_direction;

    public:

    MotorDriver()
    : target_yaw(0.f),
    target_speed(0),
    turning_power(0),
    motorA(1.f,0.f,0.f),
    motorLeft(1.f,0.f,0.f),
    motorRight(1.f,0.f,0.f),
    automaticMode(true)
    {
        this->motorA.setMin(0.f);
        this->motorA.setMax(1.f);
        this->motorA.setTimeStep(SAMPLE_TIME);

        this->motorLeft.setMin(0.f);
        this->motorLeft.setMax(1.f);
        this->motorLeft.setTimeStep(SAMPLE_TIME);

        this->motorRight.setMin(0.f);
        this->motorRight.setMax(1.f);
        this->motorRight.setTimeStep(SAMPLE_TIME);

        this->last_A_direction = false;
        this->last_B_direction = false;
    }

    void init();

    void setPIDA(const config::MotorPID& pid);

    void setPIDLeft(const config::MotorPID& pid);

    void setPIDRight(const config::MotorPID& pid);

    void setMotorConfig(const config::MotorCFG& cfg);

    config::MotorPID PIDA();

    config::MotorPID PIDLeft();

    config::MotorPID PIDRight();

    config::MotorCFG MotorConfig();

    bool channelADirection();

    bool channelBDirection();

    // main drive function
    void loop();

    void setTargetAngel(float target)
    {
        this->target_yaw=target;
    }

    float TargetAngel() const
    {
        return this->target_yaw;
    }

    void setTargetSpeed(float target_speed)
    {
        this->target_speed=target_speed;
    }

    float TargetSpeed() const
    {
        return this->target_speed;
    }

    void setTurningPower(float power)
    {
        this->turning_power=power;
    }

    float TurningPower() const
    {
        return this->turning_power;
    }

    void setAutomaticMode(bool mode)
    {
        this->automaticMode=mode;
    }

    bool AutomaticMode()
    {
        return this->automaticMode;
    }

    // set motor A power and spinning direction
    void set_channelA(int32_t pwr);

    // set motor B power and spinning direction
    void set_channelB(int32_t pwr);

    // stop engines
    void stop();

};