#pragma once

#include "sprites.hpp"

#include "oledPage.hpp"

#include "OLED.hpp"

namespace oled_modes
{
    // page for basic sensor readings
    class Sensors : public Page
    {
        public:

        Sensors()
        {
            
        }

        int loop(oled::OLED& screen,bool fun,bool mode)
        {

            if( mode )
            {
                return 1;
            }

            return 0;
        }

    };

}