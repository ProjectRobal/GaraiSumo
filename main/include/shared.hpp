#pragma once

#include "MotorDriver.hpp"

#include "RuraPlayer.hpp"

#include "SensorReader.hpp"

namespace shared
{

struct Modules
{
    sensors::SensorReader* sensors;
    RuraPlayer* audio_play;
    MotorDriver* driver;

};

extern Modules mods;

};