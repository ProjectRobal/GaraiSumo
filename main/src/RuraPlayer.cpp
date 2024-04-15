#include "RuraPlayer.hpp"

void RuraPlayer::timer_callback()
{

    //ledc_set_duty(LEDC_LOW_SPEED_MODE,LEDC_CHANNEL_4,this->audio[this->ptr++]);

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
}

void RuraPlayer::init_timer(uint32_t frameTime)
{
    timer_config_t config{};

    config.alarm_en=TIMER_ALARM_DIS;
    config.counter_en=TIMER_PAUSE;
    config.intr_type=TIMER_INTR_LEVEL;
    config.counter_dir=TIMER_COUNT_UP;
    config.auto_reload=TIMER_AUTORELOAD_EN;
    // Timer count with 1Mhz frequency
    config.divider=80;
    // 80Mhz clock
    config.clk_src=TIMER_SRC_CLK_APB;


    timer_init(TIMER_GROUP_0,TIMER_1,&config);
    timer_set_alarm_value(TIMER_GROUP_0,TIMER_1,1000000/frameTime);
    timer_set_alarm(TIMER_GROUP_0,TIMER_1,TIMER_ALARM_EN);
    timer_enable_intr(TIMER_GROUP_0,TIMER_1);

    timer_isr_callback_add(TIMER_GROUP_0,TIMER_1,this->_timer_callback_wrapper,this,0);
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

RuraPlayer::RuraPlayer(const uint8_t* audio,size_t size,uint32_t frameTime)
: audio(audio),
size(size)
{
    this->init_pwm();
    this->init_timer(frameTime);
    ptr=0;
}

void RuraPlayer::play()
{
    timer_start(TIMER_GROUP_0,TIMER_1);
}

void RuraPlayer::pause()
{
    timer_pause(TIMER_GROUP_0,TIMER_1);
}