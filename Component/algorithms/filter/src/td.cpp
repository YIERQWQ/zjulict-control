#include "td.h"

namespace td
{

    td::td(parameter parameter_t, float init)
    {
        parameter_ = parameter_t;
        x1 = init;
        x1k = init;
        x2 = 0;
        x2k = 0;
        d = parameter_.r * parameter_.h;
        d0 = d * parameter_.h;
        cycle = parameter_.cycle_high - parameter_.cycle_low;
    }

    void td::calc(float raw_data)
    {

        if (parameter_.is_cycle)
        {
            float delta_raw_data = raw_data - last_raw_data_;
            if (delta_raw_data > cycle / 2.0)
            {
                raw_data_ += delta_raw_data - cycle;
            }
            else if (delta_raw_data < -cycle / 2.0)
            {
                raw_data_ += delta_raw_data + cycle;
            }
            else
            {
                raw_data_ += delta_raw_data;
            }
        }
        else
        {
            raw_data_ = raw_data;
        }

        x1k = x1;
        x2k = x2;

        float tdy = x1k - raw_data_ + parameter_.h * x2k;
        float a0 = sqrt(d * d + 8 * parameter_.r * abs(tdy));

        float a = 0;

        if (abs(tdy) <= d0)
        {
            a = x2k + tdy / parameter_.h;
        }
        else
        {
            if (tdy > 0)
            {
                a = x2k + 0.5 * (a0 - d);
            }
            else
            {
                a = x2k - 0.5 * (a0 - d);
            }
        }

        float f = 0;
        if (abs(a) <= d)
        {
            f = -parameter_.r * a / d;
        }
        else
        {
            if (a > 0)
            {
                f = -parameter_.r;
            }
            else
            {
                f = parameter_.r;
            }
        }

        x1 = x1k + parameter_.dt * x2k;
        x2 = x2k + parameter_.dt * f;

        if (parameter_.is_cycle)
        {
            data = normalize<float>(x1, parameter_.cycle_low, parameter_.cycle_high);
        }
        else
        {
            data = x1;
        }

        diff = x2;

        last_raw_data_ = raw_data;
    }
}