/*

    A class that store N elements of specified type in queue.
    Used then for mean calculation.

*/

#pragma once

#include <algorithm>
#include <cstdint>
#include "esp_heap_caps.h"

template<typename T,uint16_t Size>
class MeanBuffer
{
    protected:

    T buffer;
    uint16_t iter;

    public:

    MeanBuffer()
    {
        this->buffer=0;
        // point at first element of buffer array
        this->iter=0;
    }

    void push(const T& element)
    {
        if(this->iter<Size)
        {
            this->buffer+=element;
            this->iter++;
        }
        else
        {
            this->reset();

            // T mean=this->mean();
            // this->push(mean);
            this->push(element);
        }

    }

    void reset()
    {
        this->iter=0;
        this->buffer=0;
    }

    bool isFull() const
    {
        return this->iter == Size;
    }

    T mean() const
    {
        if(this->iter==0)
        {
            return 0;
        }

        T out=this->buffer/this->iter;

        return out;
    }

};