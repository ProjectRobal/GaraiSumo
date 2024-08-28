#include "starter.hpp"

#include "config.hpp"

static bool overrideFlag = false;

void init_starter()
{
    gpio_set_direction(START_PIN,GPIO_MODE_INPUT);
    gpio_pulldown_en(START_PIN);
}

bool starter_state()
{
    return gpio_get_level(START_PIN) || overrideFlag;
}

void starter_override(bool status)
{
    overrideFlag = status;
}