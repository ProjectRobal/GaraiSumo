/*

    A base class for implementing AI for a robot.

*/

#pragma once

#include "SensorReader.hpp"
#include "MotorDriver.hpp"


class StrategyBase
{
    protected:


    public:

    StrategyBase()
    {}

    // a main decision function that take readings from SensorReader and decide what action to take, it would also take a refrence to motor controller
    virtual void loop(const sensors::Readings& readings,MotorDriver& driver)=0;

};