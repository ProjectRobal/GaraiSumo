#pragma once

#include "Tactics.hpp"

#include "Tactics_type.hpp"

namespace tactics
{
    class FullSpeed : public Tactics
    {
        public:

        FullSpeed()
        : Tactics()
        {}

        void loop(){}

        Tactics_type type()
        {
            return Tactics_type::FullSpeed;
        }

        ~FullSpeed()
        {}
    };
}