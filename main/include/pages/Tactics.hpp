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

        size_t scroll_iterator;

        uint8_t sleep_counter;

        public:

        Tactics()
        {
            this->scroll_iterator = 0;
            this->sleep_counter = 3;
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

            screen.drawText(65,8,1,tactics->name());

            const char* description = tactics->descritpion();   

            // add text scrolling

            this->sleep_counter++;

            if( this->sleep_counter >= 3 )
            {

                for(uint8_t i=0;i<8;++i)
                {
                    if( !description[this->scroll_iterator+i] )
                    {
                        this->scroll_iterator = 0;
                        break;
                    }

                    screen.drawChar(65+(i*6),20,1,description[this->scroll_iterator+i]);
                
                }

                this->scroll_iterator ++;

                this->sleep_counter = 0;

            }


            // change tactic
            if( fun )
            {
                this->scroll_iterator = 0;
                this->sleep_counter = 3;

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