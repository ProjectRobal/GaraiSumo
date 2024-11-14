#pragma once
/*

    A module used to play a song using buzzer.

*/

#include <driver/gpio.h>
#include <driver/ledc.h>
#include <driver/timer.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

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
    uint8_t *audio;

    size_t size;

    uint32_t frameTime;

    size_t ptr;

    void init_pwm();

    TaskHandle_t timerTask;

    StackType_t* rura_stack;
    StaticTask_t rura_task;

    SemaphoreHandle_t semp;

    void lock()
    {
        xSemaphoreTake(this->semp,portMAX_DELAY);
    }

    void unlock()
    {
        xSemaphoreGive(this->semp);
    }

    public:

    RuraPlayer(uint8_t* audio,size_t size,uint32_t frameTime);

    void set_audio(uint8_t* audio,size_t size)
    {
        this->pause();
        this->audio = audio;
        this->size = size;
    }

    void timer_callback();

    void play();

    void pause();

};