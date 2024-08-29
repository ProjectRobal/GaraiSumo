#pragma once

#include "cstdint"
#include "limits.h"

extern "C"
{

#include "driver/pulse_cnt.h"
#include "driver/gpio.h"
#include "esp_log.h"

}

class SingleCounter
{
    private:

    pcnt_channel_handle_t channel=NULL;
    pcnt_unit_handle_t unit=NULL;

    public:

    SingleCounter()
    {}

    void init(gpio_num_t pin)
    {
    
    gpio_reset_pin(pin);
    
    gpio_set_direction(pin,GPIO_MODE_INPUT);

    gpio_pulldown_en(pin);

    pcnt_unit_config_t config={0};

    config.low_limit=-1;
    config.high_limit=SHRT_MAX;

    ESP_ERROR_CHECK(pcnt_new_unit(&config, &this->unit));

    pcnt_chan_config_t chan_config={0};

    chan_config.edge_gpio_num=pin;
    chan_config.level_gpio_num=-1;
    chan_config.flags.virt_level_io_level=1;


    ESP_ERROR_CHECK(pcnt_new_channel(this->unit,&chan_config,&this->channel));

    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(this->channel, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_HOLD));
    
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(this->channel, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_KEEP));

    pcnt_glitch_filter_config_t filter_config = {
        .max_glitch_ns = 10,
        };

    ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(this->unit, &filter_config));

    ESP_ERROR_CHECK(pcnt_unit_enable(this->unit));

    
    }

    void start()
    {
        ESP_ERROR_CHECK(pcnt_unit_start(this->unit));
    }

    void stop()
    {
        ESP_ERROR_CHECK(pcnt_unit_stop(this->unit));
    }

    void clear()
    {
        ESP_ERROR_CHECK(pcnt_unit_clear_count(this->unit));
    }

    int32_t get()
    {
        int32_t pulse_count = 0;
        ESP_ERROR_CHECK(pcnt_unit_get_count(this->unit, (int*)(&pulse_count)));

        return pulse_count;
    }

    void free()
    {
        ESP_ERROR_CHECK(pcnt_unit_disable(this->unit));

        ESP_ERROR_CHECK(pcnt_del_channel(this->channel));
    }

    ~SingleCounter()
    {
    
    }


};
