#pragma once

#include <cmath>

#include "shared.hpp"

#include "OLED.hpp"

#include "sprites.hpp"

#include "config.hpp"

namespace oled_modes
{

// returns page id to switch to ( returns 0 to not switch )
typedef uint8_t(*oled_fun)(oled::OLED& screen,shared::Modules* mods,const bool& fun_button_state,const bool& mode_button_state);

// show main page ( KoNaR logo )
uint8_t main_page(oled::OLED& screen,shared::Modules* mods,const bool& fun_button_state,const bool& mode_button_state)
{

    screen.drawBitmap(0,0,128,64,sprites::konar_v_128_64);

    if(gpio_get_level(MODE_BUTTON)!=mode_button_state)
    {
        if(gpio_get_level(MODE_BUTTON))
        {
            return 2;
        }
    }

    return 0;
}

// show readings from all tof, rotation and position
uint8_t vl_page(oled::OLED& screen,shared::Modules* mods,const bool& fun_button_state,const bool& mode_button_state)
{
    screen.setFont(ssd1306xled_font6x8);
   

    char text[32]={0};

    itoa(mods->sensors->read().distances[17],text,32);
    screen.drawText(48,48,1,text);

    memset(text,0,sizeof(char)*32);
    itoa(mods->sensors->read().distances[16],text,32);
    screen.drawText(64,48,1,text);

    memset(text,0,sizeof(char)*32);
    itoa(mods->sensors->read().distances[15],text,32);
    screen.drawText(80,48,1,text);

    memset(text,0,sizeof(char)*32);
    itoa(mods->sensors->read().distances[14],text,32);
    screen.drawText(96,48,1,text);

    memset(text,0,sizeof(char)*32);
    itoa(mods->sensors->read().distances[13],text,32);
    screen.drawText(112,48,1,text);

    memset(text,0,sizeof(char)*32);
    itoa(mods->sensors->read().distances[12],text,32);
    screen.drawText(112,32,1,text);

    memset(text,0,sizeof(char)*32);
    itoa(mods->sensors->read().distances[11],text,32);
    screen.drawText(112,16,1,text);

    memset(text,0,sizeof(char)*32);
    itoa(mods->sensors->read().distances[10],text,32);
    screen.drawText(112,8,1,text);

    memset(text,0,sizeof(char)*32);
    itoa(mods->sensors->read().distances[9],text,32);
    screen.drawText(80,8,1,text);

    memset(text,0,sizeof(char)*32);
    itoa(mods->sensors->read().distances[8],text,32);
    screen.drawText(64,8,1,text);

    memset(text,0,sizeof(char)*32);
    itoa(mods->sensors->read().distances[7],text,32);
    screen.drawText(32,8,1,text);

    memset(text,0,sizeof(char)*32);
    itoa(mods->sensors->read().distances[6],text,32);
    screen.drawText(16,8,1,text);

    memset(text,0,sizeof(char)*32);
    itoa(mods->sensors->read().distances[5],text,32);
    screen.drawText(0,8,1,text);
    
    memset(text,0,sizeof(char)*32);
    itoa(mods->sensors->read().distances[4],text,32);
    screen.drawText(0,16,1,text);

    memset(text,0,sizeof(char)*32);
    itoa(mods->sensors->read().distances[3],text,32);
    screen.drawText(0,32,1,text);

    memset(text,0,sizeof(char)*32);
    itoa(mods->sensors->read().distances[2],text,32);
    screen.drawText(0,48,1,text);

    memset(text,0,sizeof(char)*32);
    itoa(mods->sensors->read().distances[1],text,32);
    screen.drawText(16,48,1,text);

    memset(text,0,sizeof(char)*32);
    itoa(mods->sensors->read().distances[0],text,32);
    screen.drawText(32,48,1,text);



    if(gpio_get_level(MODE_BUTTON)!=mode_button_state)
    {
        if(gpio_get_level(MODE_BUTTON))
        {
            return 2;
        }
    }

    return 0;
}

// select modes
uint8_t mode_select(oled::OLED& screen,shared::Modules* mods,const bool& fun_button_state,const bool& mode_button_state)
{


    return 0;
}

oled_fun modes[3]={
    main_page,
    vl_page,
    mode_select
};

} // namespace oled_modes


#define OLED_MODES_COUNT sizeof(oled_modes::modes)/sizeof(oled_modes::oled_fun)