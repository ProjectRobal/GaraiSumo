#pragma once

#include "OLED.hpp"

namespace oled_modes
{

    class Page
    {

        public:

        virtual int loop(oled::OLED& screen,bool fun,bool mode) = 0;

    };

}