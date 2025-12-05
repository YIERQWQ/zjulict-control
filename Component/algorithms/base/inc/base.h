#ifndef _BASE_H_
#define _BASE_H_

#include "string.h"
#include "stm32f4xx_hal.h"
#include "math.h"
#include "stdint.h"
#include "cstdint"
#include "iostream"
#include "vector"

template <typename T>
T limit(T data_t, T limit_t)
{
    if (limit_t < 0)
    {
        limit_t = -limit_t;
    }

    if (data_t > limit_t)
    {
        return limit_t;
    }
    else if (data_t < -limit_t)
    {
        return -limit_t;
    }
    else
    {
        return data_t;
    }
}

template <typename T>
T normalize(T data_t, T cycle_low, T cycle_high)
{
    if (cycle_low > cycle_high)
    {
        float t = cycle_low;
        cycle_low = cycle_high;
        cycle_high = t;
    }

    float cycle = cycle_high - cycle_low;

    while (data_t > cycle_high)
    {
        data_t -= cycle;
    }

    while (data_t < cycle_low)
    {
        data_t += cycle;
    }

    return data_t;
}

#endif