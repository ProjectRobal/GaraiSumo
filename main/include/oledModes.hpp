#pragma once

#include <cmath>

#include "shared.hpp"

#include "OLED.hpp"

#include "sprites.hpp"

#include "config.hpp"

namespace oled_modes
{

// returns page id to switch to ( returns 0 to not switch )

// show main page ( KoNaR logo )
uint8_t main_page(oled::OLED& screen)
{

    screen.drawBitmap(0,0,128,32,sprites::konar_logo_64x32);

    // if(gpio_get_level(MODE_BUTTON)!=mode_button_state)
    // {
    //     if(gpio_get_level(MODE_BUTTON))
    //     {
    //         return 2;
    //     }
    // }

    return 0;
}

} // namespace oled_modes