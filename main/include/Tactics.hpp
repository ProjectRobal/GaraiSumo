#pragma once

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/event_groups.h>

#include "sprites.hpp"

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

        virtual const char* descritpion()
        {
            return "Dummy";
        }

        virtual const char* name()
        {
            return "None";
        }

        // size 64 x 32
        virtual const uint8_t* picture()
        {
            return sprites::konar_logo_64x32;
        }

        virtual void loop() = 0;

        virtual Tactics_type type() = 0;

        virtual ~Tactics(){};
    };
}
