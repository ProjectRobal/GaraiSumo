#pragma once

#include "sprites.hpp"

#include "oledPage.hpp"

#include "OLED.hpp"

#include "shared.hpp"

namespace oled_modes
{
    // page wtih tactics selection
    class Tactics : public Page
    {
        public:

        Tactics()
        {
            
        }

        int loop(oled::OLED& screen,bool fun,bool mode)
        {

            if( mode )
            {
                return 3;
            }

            return 0;
        }

    };

}