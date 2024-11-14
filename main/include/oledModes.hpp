#pragma once

#include <cmath>

#include "shared.hpp"

#include "OLED.hpp"

#include "sprites.hpp"

#include "config.hpp"

#include "oledPage.hpp"

#define OLED_PAGES_COUNT 3

namespace oled_modes
{

    extern Page* pages[OLED_PAGES_COUNT];

} // namespace oled_modes