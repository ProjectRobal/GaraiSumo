#include "oledModes.hpp"

namespace oled_modes
{

static uint8_t step = 0;
static bool moveup = true;

// returns page id to switch to ( returns 0 to not switch )

// show main page ( KoNaR logo )
uint8_t main_page(oled::OLED& screen)
{

    
    screen.drawBitmap(step*16,0,64,32,sprites::konar_logo_64x32);

    if(moveup)
    {
        ++step;

        if( step == 8 )
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