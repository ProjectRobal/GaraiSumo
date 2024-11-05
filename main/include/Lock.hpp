#pragma once

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>


class Lock
{
    SemaphoreHandle_t mutex;

    public:

    Lock(SemaphoreHandle_t mutex)
    : mutex(mutex)
    {
        xSemaphoreTake(this->mutex,portMAX_DELAY);
    }

    ~Lock()
    {
        xSemaphoreGive(this->mutex);
    }
};