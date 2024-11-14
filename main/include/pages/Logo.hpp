#pragma once

#include "sprites.hpp"

#include "oledPage.hpp"

#include "OLED.hpp"

namespace oled_modes
{

    class Logo : public Page
    {
        public:

        uint8_t step;
        bool moveup;

        Logo()
        {
            this->step = 0;
            this->moveup = true;
        }

        int loop(oled::OLED& screen,bool fun,bool mode)
        {
            screen.drawBitmap(step*16,0,64,32,sprites::konar_logo_64x32);

            if(moveup)
            {
                ++step;

                if( step == 4 )
                {
                    moveup = false;
                }
            }
            else
            {
                --step;

                if( step == 0 )
                {
                    moveup = true;
                }
            }

            if( mode )
            {
                return 2;
            }

            // if(gpio_get_level(MODE_BUTTON)!=mode_button_state)
            // {
            //     if(gpio_get_level(MODE_BUTTON))
            //     {
            //         return 2;
            //     }
            // }

            return 0;
        }

    };

}