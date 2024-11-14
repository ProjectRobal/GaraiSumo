#pragma once

#include "Tactics.hpp"

#include "Tactics_type.hpp"

namespace tactics
{
    class DoNothing : public Tactics
    {
        public:

        DoNothing()
        : Tactics()
        {}

        const char* name()
        {
            return "Nothing";
        }

        const char* descritpion()
        {
            return "It does nothing!";
        }

        // size 64 x 32
        const uint8_t* picture()
        {
            return sprites::do_nothing_64x32;
        }

        void loop(){}

        Tactics_type type()
        {
            return Tactics_type::DoNothing;
        }

        ~DoNothing()
        {}
    };
}