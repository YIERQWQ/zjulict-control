#include "motor_base.h"

namespace motor
{
    // 电流输入
    void Motor::setCurrInput(float curr_input_t)
    {
        curr_input_t = limit<float>(curr_input_t, motor_info.broad_curr_limit);

        cmd_raw_input_ = motor_parameter.dir * (int16_t)(curr_input_t / motor_info.motor_curr_limit * motor_info.raw_input_limit);

        if (motor_parameter.need_limit_vel)
        {
            actual_raw_input_ = limitVel();
        }
        else
        {
            actual_raw_input_ = cmd_raw_input_;
        }
    }

    void Motor::setTorqInput(float torq_input_t)
    {
        // 移除减速箱相当于多了一个 （1 / 原减速比） 的减速箱，这里的力矩是相对于带有减速箱的，和decode那里不同
        if (motor_parameter.remove_build_in_reducer)
        {
            setCurrInput(torq_input_t / (((float)motor_parameter.ex_redu_rat )/ motor_info.raw_redu_rat) / motor_info.motor_torq_limit * motor_info.motor_curr_limit);
        }
        else
        {
            setCurrInput(torq_input_t / ((float)motor_parameter.ex_redu_rat ) / motor_info.motor_torq_limit * motor_info.motor_curr_limit);
        }
    }

    int16_t Motor::limitVel(void)
    {
        if ((vel_ > motor_parameter.max_vel) || (vel_ < -motor_parameter.max_vel))
        {
            return actual_raw_input_ * motor_parameter.limitVel_CurrProportion;
        }
        else
        {
            return cmd_raw_input_;
        }
    }

} // namespace motor
