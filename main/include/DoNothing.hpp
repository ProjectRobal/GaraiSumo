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

        void loop(){}

        Tactics_type type()
        {
            return Tactics_type::DoNothing;
        }

        ~DoNothing()
        {}
    };
}