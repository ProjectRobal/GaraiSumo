#pragma once
/*

    A module used to play a song using buzzer.

*/

#include <driver/gpio.h>
#include <driver/ledc.h>
#include <driver/timer.h>

#include "config.hpp"

const uint16_t tones[]=
{
    50,
    55,
    61,
    65,
    73,
    82,
    87,
    97,
    110,
    123,
    130,
    146,
    164,
    174,
    196,
    220,
    246,
    261,
    293,
    329,
    349,
    440,
    493,
    523,
    587,
    659,
    698,
    783,   
    880,
    987,
    1046,
    1174,
    1318,
    1568,
    1760,
    1975,
    2093,
    2349,
    2637,
    2793,
    3136,
    3520,
    3951,
    4186
};


class RuraPlayer
{
    const uint8_t *audio;

    const size_t size;

    size_t ptr;

    void IRAM_ATTR timer_callback();

    void init_pwm();

    void init_timer(uint32_t frameTime);

    static IRAM_ATTR bool _timer_callback_wrapper(void* arg)
    {

        ((RuraPlayer*)arg)->timer_callback();

        return false;
    }

    public:

    RuraPlayer(const uint8_t* audio,size_t size,uint32_t frameTime);

    void play();

    void pause();

};