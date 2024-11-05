#pragma once

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>

#include <Lock.hpp>

template<typename T>
class LockRes
{
    T resource;
    SemaphoreHandle_t mutex;

    public:

    LockRes()
    {
        this->mutex = xSemaphoreCreateMutex();
    }

    LockRes(T _resource)
    : LockRes()
    {
        this->resource = _resource;
    }

    void set(T resource)
    {
        Lock lock(this->mutex);

        this->resource = resource;
    }

    T get()
    {
        Lock lock(this->mutex);

        return this->resource;
    }

};