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