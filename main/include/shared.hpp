#pragma once

#include "MotorDriver.hpp"

#include "RuraPlayer.hpp"

#include "SensorReader.hpp"

#include "LockResource.hpp"

#include "Tactics_type.hpp"

namespace shared
{

struct Modules
{
    sensors::SensorReader* sensors;
    RuraPlayer* audio_play;
    MotorDriver* driver;
    LockRes<Tactics_type> current_tactics;
    
};

extern Modules mods;

};