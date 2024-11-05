#pragma once

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/event_groups.h>

#include "MotorDriver.hpp"

#include "RuraPlayer.hpp"

#include "SensorReader.hpp"

#include "shared.hpp"

#include "Tactics_type.hpp"


/*
    A base class for Tactis type.
*/
namespace tactics
{
    class Tactics
    {
        public:

        virtual void loop() = 0;

        virtual Tactics_type type() = 0;

        virtual ~Tactics(){};
    };
}
