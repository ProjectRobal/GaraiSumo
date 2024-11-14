#pragma once

#include "sprites.hpp"

#include "SensorReader.hpp"

#include "oledPage.hpp"

#include "OLED.hpp"

#include "shared.hpp"

#include "Tactics_list.hpp"

namespace oled_modes
{
    // page wtih tactics selection
    class Tactics : public Page
    {
        public:

        Tactics()
        {
            
        }

        const char* name()
        {
            return "Tactics";
        }

        int loop(oled::OLED& screen,bool fun,bool mode)
        {
            
            uint32_t current_tactics = shared::mods.current_tactics.get();

            // show diffrent pictures and description depending on selected tactics

            
            tactics::Tactics* tactics = tactics_list[current_tactics];

            screen.drawBitmap(0,0,64,32,tactics->picture());  

            screen.setFont(ssd1306xled_font6x8);

            // screen.drawText(65,8,1,tactics->name());

            const char* description = tactics->descritpion();   

            size_t i=0;

            uint8_t x=0;
            uint8_t y=0;

            while(description[i])
            {
                screen.drawChar(65+(x*6),16+(y*8),1,description[i]);

                x++;

                if( x == 8 )
                {
                    y++;
                    x = 0;
                }

                i++;
            }


            // change tactic
            if( fun )
            {
                current_tactics++;

                current_tactics = current_tactics - static_cast<uint32_t>( current_tactics / TACTICS_COUNT ) * TACTICS_COUNT;

                shared::mods.current_tactics.set(current_tactics);
            }

            if( mode )
            {
                return 3;
            }

            return 0;
        }

    };

}