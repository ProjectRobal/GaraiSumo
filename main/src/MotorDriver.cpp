#include "MotorDriver.hpp"

#include "shared.hpp"

#include "config.hpp"

#include "starter.hpp"

using shared::mods;

static void freeRTOS_task(void* arg)
{
    MotorDriver* driver = (MotorDriver*)arg;

    while(true)
    {
        driver->loop();

        vTaskDelay(MOTOR_UPDATE_TIME_MS/portTICK_PERIOD_MS);
    }
}


bool MotorDriver::channelADirection()
{
    return this->last_A_direction;
}

bool MotorDriver::channelBDirection()
{
    return this->last_B_direction;
}

void MotorDriver::init_gpio()
{

    for(const gpio_num_t& pin : MotorsPin)
    {
        ESP_ERROR_CHECK(gpio_set_direction(pin,GPIO_MODE_OUTPUT));
        ESP_ERROR_CHECK(gpio_pullup_en(pin));
    }

}

void MotorDriver::init_ledc()
{
    ledc_timer_config_t timer_cfg;
    timer_cfg.speed_mode=LEDC_LOW_SPEED_MODE;
    timer_cfg.duty_resolution=LEDC_TIMER_12_BIT;
    timer_cfg.freq_hz=MOTOR_FREQ;
    timer_cfg.timer_num=LEDC_TIMER_0;
    timer_cfg.clk_cfg=LEDC_USE_RTC8M_CLK;
    timer_cfg.deconfigure=false;

    ESP_ERROR_CHECK(ledc_timer_config(&timer_cfg));

    ledc_channel_config_t channel_cfg;

    uint8_t channel=0;

    channel_cfg.speed_mode=LEDC_LOW_SPEED_MODE;
    channel_cfg.intr_type=LEDC_INTR_DISABLE;
    channel_cfg.duty=0;
    channel_cfg.timer_sel=LEDC_TIMER_0;
    channel_cfg.flags.output_invert=true;
    channel_cfg.hpoint=(1<<12)-1;

    for(const gpio_num_t& pin : MotorsPin)
    {
        channel_cfg.gpio_num=pin;
        channel_cfg.channel=static_cast<ledc_channel_t>(channel++);

        ESP_ERROR_CHECK(ledc_channel_config(&channel_cfg));
    }

}

void MotorDriver::set_channelA(int32_t pwr)
{
    if(pwr>0)
    {
        ledc_set_duty(LEDC_LOW_SPEED_MODE,LEDC_CHANNEL_0,pwr);
        ledc_set_duty(LEDC_LOW_SPEED_MODE,LEDC_CHANNEL_1,0);
        this->last_A_direction = false;
    }
    else if(pwr<0)
    {
        ledc_set_duty(LEDC_LOW_SPEED_MODE,LEDC_CHANNEL_0,0);
        ledc_set_duty(LEDC_LOW_SPEED_MODE,LEDC_CHANNEL_1,-pwr);
        this->last_A_direction = true;
    }
    else
    {
        ledc_set_duty(LEDC_LOW_SPEED_MODE,LEDC_CHANNEL_0,0);
        ledc_set_duty(LEDC_LOW_SPEED_MODE,LEDC_CHANNEL_1,0);
    }

    ledc_update_duty(LEDC_LOW_SPEED_MODE,LEDC_CHANNEL_0);
    ledc_update_duty(LEDC_LOW_SPEED_MODE,LEDC_CHANNEL_1);
}

void MotorDriver::set_channelB(int32_t pwr)
{
    if(pwr>0)
    {
        ledc_set_duty(LEDC_LOW_SPEED_MODE,LEDC_CHANNEL_2,0);
        ledc_set_duty(LEDC_LOW_SPEED_MODE,LEDC_CHANNEL_3,pwr);
        this->last_B_direction = false;
    }
    else if(pwr<0)
    {
        ledc_set_duty(LEDC_LOW_SPEED_MODE,LEDC_CHANNEL_2,-pwr);
        ledc_set_duty(LEDC_LOW_SPEED_MODE,LEDC_CHANNEL_3,0);
        this->last_B_direction = true;
    }
    else
    {
        ledc_set_duty(LEDC_LOW_SPEED_MODE,LEDC_CHANNEL_2,0);
        ledc_set_duty(LEDC_LOW_SPEED_MODE,LEDC_CHANNEL_3,0);
    }

    ledc_update_duty(LEDC_LOW_SPEED_MODE,LEDC_CHANNEL_2);
    ledc_update_duty(LEDC_LOW_SPEED_MODE,LEDC_CHANNEL_3);
}


void MotorDriver::init()
{
    this->init_gpio();

    this->init_ledc();

    this->stop();

    // start Motor Task

    this->motor_stack = (StackType_t*)malloc(MIN_TASK_STACK_SIZE);

    if( xTaskCreateStaticPinnedToCore(freeRTOS_task,"Motors",MIN_TASK_STACK_SIZE,this,configMAX_PRIORITIES-1,this->motor_stack,&this->motor_task,xPortGetCoreID()) == NULL )
    {
        ESP_LOGE("MAIN","Failed to create Motor task");
    }

}

void MotorDriver::setPIDA(const config::MotorPID& pid)
{
    this->motorA.setParams(pid.P,pid.I,pid.D);
}

void MotorDriver::setPIDLeft(const config::MotorPID& pid)
{
    this->motorLeft.setParams(pid.P,pid.I,pid.D);
    // this->motorRight.setParams(pid.P,pid.I,pid.D);
}

void MotorDriver::setPIDRight(const config::MotorPID& pid)
{
    // this->motorLeft.setParams(pid.P,pid.I,pid.D);
    this->motorRight.setParams(pid.P,pid.I,pid.D);
}

void MotorDriver::setMotorConfig(const config::MotorCFG& cfg)
{
    this->setPIDA(cfg.motor1);
    this->setPIDLeft(cfg.motor2);
    this->setPIDRight(cfg.motor3);
}

config::MotorPID MotorDriver::PIDA()
{
    return {
        .P=this->motorA.P(),
        .I=this->motorA.I(),
        .D=this->motorA.D()
    };
}

config::MotorPID MotorDriver::PIDLeft()
{
    return {
        .P=this->motorLeft.P(),
        .I=this->motorLeft.I(),
        .D=this->motorLeft.D()
    };
}

config::MotorPID MotorDriver::PIDRight()
{
    return {
        .P=this->motorRight.P(),
        .I=this->motorRight.I(),
        .D=this->motorRight.D()
    };
}

config::MotorCFG MotorDriver::MotorConfig()
{
    return {
        .motor1=this->PIDA(),
        .motor2=this->PIDLeft(),
        .motor3=this->PIDRight()
    };
}

void MotorDriver::loop()
{
    mods.sensors->Lock();

    const sensors::Readings& readings = mods.sensors->read();

    ESP_LOGD("MAIN","Motor task");

    if(( readings.stoped )||( !starter_state() ))
    {
        mods.sensors->Unlock();
        this->stop();
        return;
    }

    // add speed control for engines
    if(automaticMode)
    {

        /*

            PID A will set angular velocity. Which will be then translated to velocity of
            each wheels.

            Then PID Left and PID Right is going to adjust a speed on each motor.
        
        */

        // error between target angel and current angel 
        float d0 = this->target_yaw-readings.yaw;   

        float angular_speed = this->turning_power*this->motorA.step(d0);

        float target_speed_left = this->target_speed + D_WHEELS*angular_speed;
        float target_speed_right = this->target_speed - D_WHEELS*angular_speed;

        float dSpeedLeft = abs(target_speed_left) - abs(readings.motorSpeed[0]); 
        float dSpeedRight = abs(target_speed_right) - abs(readings.motorSpeed[1]); 

        int32_t PowerLeft = this->motorLeft.step(dSpeedLeft)*MAX_ENGINE_POWER;
        int32_t PowerRight = this->motorRight.step(dSpeedRight)*MAX_ENGINE_POWER;

        if( target_speed_left < 0 )
        {
            PowerLeft = -PowerLeft;
        }

        if( target_speed_right < 0 )
        {
            PowerRight = -PowerRight;
        }

        this->set_channelA(PowerLeft);
        this->set_channelB(PowerRight);

    }

    mods.sensors->Unlock();
}

void MotorDriver::stop()
{
    this->set_channelA(0);
    this->set_channelB(0);
}
