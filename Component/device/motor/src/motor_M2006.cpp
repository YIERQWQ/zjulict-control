#include "motor_M2006.h"

namespace motor
{

    Motor_M2006::Motor_M2006(const MotorParameter motor_parameter_t) : Motor()
    {
        motor_info = MotorInfo_M2006;
        motor_info.rx_id += motor_parameter_t.id;
        motor_parameter = motor_parameter_t;
        // �Ƴ��������൱�ڶ���һ�� ��1 / ԭ���ٱȣ� �ļ�����
        if (motor_parameter.remove_build_in_reducer)
        {
            redu_rat = ((float)motor_parameter.ex_redu_rat);
        }
        else
        {
            redu_rat = ((float)motor_parameter.ex_redu_rat) * motor_info.raw_redu_rat;
        }
    }

    void Motor_M2006::encode(uint8_t *tx_data)
    {
        uint8_t data_H = 0x00;
        uint8_t data_L = 0x00;

        data_H = actual_raw_input_ >> 8;
        data_L = actual_raw_input_;

        if (motor_parameter.id <= 8)
        {
            tx_data[(motor_parameter.id - 5) * 2] = data_H;
            tx_data[(motor_parameter.id - 5) * 2 + 1] = data_L;
        }
    }

    void Motor_M2006::decode(const uint8_t rx_data[8])
    {
        last_raw_angle_ = raw_angle_;

        raw_angle_ = 360.0 * ((((float)((((int16_t)rx_data[0]) << 8) | (int16_t)rx_data[1]))) / 8191);
        raw_vel_ = (int16_t)(((rx_data[2]) << 8) | rx_data[3]);
        raw_curr_ = (int16_t)((rx_data[4] << 8) | rx_data[5]);

        float delta_raw_angle = raw_angle_ - last_raw_angle_;

        if (delta_raw_angle < -180.0)
        {
            delta_raw_angle = delta_raw_angle + 360.0;
        }

        if (delta_raw_angle > 180.0)
        {
            delta_raw_angle = delta_raw_angle - 360.0;
        }

        angle_ = (float)motor_parameter.dir * normalize<float>((float)motor_parameter.dir * angle_ + delta_raw_angle / ((float)redu_rat), -180, 180);
        vel_ = (float)(motor_parameter.dir * raw_vel_ / ((float)redu_rat));
        curr_ = (float)(motor_parameter.dir * raw_curr_);
        torq_ = (float)(curr_ / motor_info.motor_curr_limit * motor_info.motor_torq_limit * ((float)redu_rat));
    }

} // namespace motor
