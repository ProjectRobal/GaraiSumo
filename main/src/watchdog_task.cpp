
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <esp_log.h>
#include <esp_sleep.h>

#include "config.hpp"

#include "shared.hpp"

#include "watchdog_task.hpp"


using shared::mods;

static StaticTask_t watchdog_task;

void watchdog(void *arg)
{
    while(true)
    {
        mods.sensors->Lock();

        const sensors::Readings& readings = mods.sensors->read();

        // go into sleep mode
        if( readings.battery_voltage < 13.2 )
        {
            ESP_LOGI("MAIN","Low battery voltage, going into sleep mode!!");

            esp_deep_sleep_start();
        }

        mods.sensors->Unlock();

        vTaskDelay(WATCHDOG_TIME_MS/portTICK_PERIOD_MS);
    }
}

void init_watchdog()
{
    StackType_t* watchdog_stack = (StackType_t*)malloc(MIN_TASK_STACK_SIZE);
    
    if( xTaskCreateStaticPinnedToCore(watchdog,"WatchDog",MIN_TASK_STACK_SIZE,NULL,tskIDLE_PRIORITY+1,watchdog_stack,&watchdog_task,xPortGetCoreID()) == NULL )
    {
        ESP_LOGE("MAIN","Cannot create watchdog task");
    }   
}