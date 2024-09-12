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
uint8_t main_page(oled::OLED& screen);

} // namespace oled_modes