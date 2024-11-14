#include "oledModes.hpp"


#include "pages/Logo.hpp"
#include "pages/Tactics.hpp"
#include "pages/Sensors.hpp"


namespace oled_modes
{

    Page* pages[OLED_PAGES_COUNT] = {
        new Logo(),
        new Tactics(),
        new Sensors()
    };

} // namespace oled_modes