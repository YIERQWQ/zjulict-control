#ifndef _TD_H_
#define _TD_H_

#include "base.h"

namespace td
{
    typedef struct _parameter
    {
        float r;
        float h;
        float dt;
        bool is_cycle;
        float cycle_low;
        float cycle_high;
    } parameter;

    class td
    {
    public:
        td(parameter parameter_t, float init);
        ~td() = default;
        void calc(float raw_data);
        float getData() { return data; };
        float getDiff() { return diff; };

    private:
        parameter parameter_;
        float x1;
        float x2;
        float x1k;
        float x2k;
        float d;
        float d0;
        float data;
        float diff;
        float last_raw_data_;
        float raw_data_;
        float cycle;
    };

}

#endif