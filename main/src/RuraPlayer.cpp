#include "RuraPlayer.hpp"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <esp_log.h>

static void timer_task(void* arg)
{
    ((RuraPlayer*)arg)->timer_callback();

}

void RuraPlayer::timer_callback()
{
    //ledc_set_duty(LEDC_LOW_SPEED_MODE,LEDC_CHANNEL_4,this->audio[this->ptr++]);

    while(true)
    {
        this->lock();

        uint8_t sample=this->audio[this->ptr++];

        if(sample==0)
        {

            ledc_set_duty(LEDC_LOW_SPEED_MODE,LEDC_CHANNEL_4,0);
            ledc_update_duty(LEDC_LOW_SPEED_MODE,LEDC_CHANNEL_4);

        }
        else
        {
            ledc_set_freq(LEDC_LOW_SPEED_MODE,LEDC_TIMER_1,tones[sample-1]);
            ledc_set_duty(LEDC_LOW_SPEED_MODE,LEDC_CHANNEL_4,127);
            ledc_update_duty(LEDC_LOW_SPEED_MODE,LEDC_CHANNEL_4);
        }

        if(this->ptr>=this->size)
        {
            this->ptr=0;
        }

        this->unlock();

        vTaskDelay(this->frameTime/portTICK_PERIOD_MS);

    }

}

void RuraPlayer::init_pwm()
{
    ESP_ERROR_CHECK(gpio_set_direction(BUZZER_PIN,GPIO_MODE_OUTPUT));

    ledc_timer_config_t timer_cfg;
    timer_cfg.speed_mode=LEDC_LOW_SPEED_MODE;
    timer_cfg.duty_resolution=LEDC_TIMER_12_BIT;
    timer_cfg.freq_hz=1000;
    timer_cfg.timer_num=LEDC_TIMER_1;
    timer_cfg.clk_cfg=LEDC_USE_RC_FAST_CLK;
    timer_cfg.deconfigure=false;

    ESP_ERROR_CHECK(ledc_timer_config(&timer_cfg));

    ledc_channel_config_t channel_cfg;

    channel_cfg.speed_mode=LEDC_LOW_SPEED_MODE;
    channel_cfg.intr_type=LEDC_INTR_DISABLE;
    channel_cfg.duty=0;
    channel_cfg.timer_sel=LEDC_TIMER_1;
    channel_cfg.flags.output_invert=false;
    channel_cfg.hpoint=4095;

    channel_cfg.channel=LEDC_CHANNEL_4;
    channel_cfg.gpio_num=BUZZER_PIN;

    ESP_ERROR_CHECK(ledc_channel_config(&channel_cfg));

}

RuraPlayer::RuraPlayer(uint8_t* audio,size_t size,uint32_t frameTime)
: audio(audio),
size(size)
{
    this->semp = xSemaphoreCreateMutex();
    this->frameTime = frameTime;
    this->init_pwm();

    ptr=0;

    this->rura_stack = (StackType_t*)malloc(MIN_TASK_STACK_SIZE);

    this-> timerTask = xTaskCreateStaticPinnedToCore(timer_task,"Rura",MIN_TASK_STACK_SIZE,this,configMAX_PRIORITIES-1,this->rura_stack,&this->rura_task,1-xPortGetCoreID());

    if( this->timerTask != NULL)
    {
        vTaskSuspend(this->timerTask);
    }
    else
    {
        ESP_LOGE("MAIN","Cannot create Rura task!");
    }
}

void RuraPlayer::play()
{
    vTaskResume(this->timerTask);
}

void RuraPlayer::pause()
{
    this->lock();

    vTaskSuspend(this->timerTask);
    this->ptr=0;
    ledc_set_duty(LEDC_LOW_SPEED_MODE,LEDC_CHANNEL_4,0);
    ledc_update_duty(LEDC_LOW_SPEED_MODE,LEDC_CHANNEL_4);

    this->unlock();
}