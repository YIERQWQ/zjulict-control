#include "task.h"

float wheelInput[4] = {0};
float dribblerInput = 0;
int time;

float debug_pose = 0;
float debug_vel = 0;

float debug_K = 0;
float debug_D = 0;
float debug_M = 0;

float debug_angle_ref = 0;

bool debug_enable = false;

bool wheel_enable = false;

uint32_t cs = 10;

int timer_time = 0;

uint8_t dribbler_mode = 0;

float debug_torq = 0;

float debug_motor_vel[4] = {0};
float debug_motor_acc[4] = {0};

pid::parameter wheel_pidParameter = {

    .kp = 0,
    .ki = 0,
    .kd = 0,
    .outputLimit = motor::MotorInfo_DM3519.broad_curr_limit,
    .integLimit = motor::MotorInfo_DM3519.broad_curr_limit,
    .dt = 1.0 / 1000.0,
};

pid::parameter wheelVel_pidParameter = {

    .kp = 0,
    .ki = 0,
    .kd = 0,
    .outputLimit = 400,
    .integLimit = 400,
    .dt = 1.0 / 1000.0,
};

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim == &htim2)
    {
        timer_time = HAL_GetTick();

        robot.motion_planner(1000);
        robot.ik_solve();

        // for (uint8_t i = 0; i < 4; i++)
        // {
        //     // wheel_pidParameter.kp = robot.wheel_PID[0];
        //     // wheel_pidParameter.ki = robot.wheel_PID[1];
        //     // wheel_pidParameter.kd = robot.wheel_PID[2];
        //     // robot.wheelPID[i]->setParameter(wheel_pidParameter);
        //     // wheelInput[i] = robot.wheelPID[i]->calc(robot.motor_vel[i], robot.wheelMotor[i]->vel());
        //     // robot.wheelMotor[i]->setInput(wheelInput[i]);
        //     // robot.wheelMotor[i]->encode(can2TxData);
        //     // robot.wheelFilter[i]->calc(robot.wheelMotor[i]->vel() * 0.1047);
        //     // robot.motor_vel[i] = robot.wheelFilter[i]->getData();
        //     // robot.motor_acc[i] = robot.wheelFilter[i]->getDiff();
        //     // robot.motor_vel[i] = robot.wheelMotor[i]->vel() * 0.1047;
        //     // robot.motor_acc[i] = (robot.motor_vel[i] - robot.last_motor_vel[i])/0.001;
        //     // robot.last_motor_vel[i] = robot.motor_vel[i];
        //     // robot.wheelMotor[i]->setTorqInput(0);
        // }

        for (uint8_t i = 0; i < 4; i++)
        {

            robot.wheelFilter[i]->calc(robot.wheelMotor[i]->vel());
            debug_motor_vel[i] = robot.wheelFilter[i]->getData();
            debug_motor_acc[i] = robot.wheelFilter[i]->getDiff() * PI / 30.0;

            if (!wheel_enable)
            {
                robot.wheelMotor[i]->setTorqInput(0);
                robot.wheelPID[i]->reset();
            }
            else
            {
                if (robot.motor_acc[i] < 0.1 && robot.motor_acc[i] > -0.1)
                {
                    robot.wheelPID[i]->reset();
                }
                else
                {
                    robot.wheelVelPID[i]->reset();
                }

                wheelVel_pidParameter.kp = robot.wheelVel_PID[0];
                wheelVel_pidParameter.ki = robot.wheelVel_PID[1];
                wheelVel_pidParameter.kd = robot.wheelVel_PID[2];
                robot.wheelVelPID[i]->setParameter(wheelVel_pidParameter);

                // if (robot.motor_acc[i] != 0 )
                // {
                //     robot.motor_acc_t[i] = robot.motor_acc[i];
                // }
                // else
                // {
                // robot.motor_acc_t[i] = robot.wheelVelPID[i]->calc(robot.motor_vel[i], robot.wheelMotor[i]->vel());
                // }

                wheel_pidParameter.kp = robot.wheel_PID[0];
                wheel_pidParameter.ki = robot.wheel_PID[1];
                wheel_pidParameter.kd = robot.wheel_PID[2];
                robot.wheelPID[i]->setParameter(wheel_pidParameter);
                wheelInput[i] = robot.wheelPID[i]->calc(robot.motor_acc[i], debug_motor_acc[i]) + robot.motor_torq[i];
                robot.wheelMotor[i]->setTorqInput(wheelInput[i]);
                // robot.wheelMotor[1]->setTorqInput(debug_torq);
            }

            // if (robot.motor_acc[i] < 0.1 && robot.motor_acc[i] > -0.1)
            // {
            //     robot.wheelPID[i]->reset();
            // }

            // wheelVel_pidParameter.kp = robot.wheelVel_PID[0];
            // wheelVel_pidParameter.ki = robot.wheelVel_PID[1];
            // wheelVel_pidParameter.kd = robot.wheelVel_PID[2];
            // robot.wheelVelPID[i]->setParameter(wheelVel_pidParameter);

            // wheel_pidParameter.kp = robot.wheel_PID[0];
            // wheel_pidParameter.ki = robot.wheel_PID[1];
            // wheel_pidParameter.kd = robot.wheel_PID[2];
            // robot.wheelPID[i]->setParameter(wheel_pidParameter);
            // wheelInput[i] = robot.wheelPID[i]->calc(robot.motor_acc[i], debug_motor_acc[i]) + robot.motor_torq[i] + robot.wheelVelPID[i]->calc(robot.motor_vel[i], robot.wheelMotor[i]->vel());
            // robot.wheelMotor[i]->setTorqInput(wheelInput[i]);

            // if (robot.motor_acc[i] < 0.1 && robot.motor_acc[i] > -0.1)
            // {
            //     robot.wheelPID[i]->reset();
            // }

            // wheel_pidParameter.kp = robot.wheel_PID[0];
            // wheel_pidParameter.ki = robot.wheel_PID[1];
            // wheel_pidParameter.kd = robot.wheel_PID[2];
            // robot.wheelPID[i]->setParameter(wheel_pidParameter);
            // wheelInput[i] = robot.wheelPID[i]->calc(robot.motor_acc[i], debug_motor_acc[i]) + robot.motor_torq[i];
            // robot.wheelMotor[i]->setTorqInput(wheelInput[i]);

            // robot.wheelMotor[i]->setTorqInput(robot.motor_torq[i]);
        }

        for (uint8_t i = 0; i < 4; i++)
        {
            robot.wheelMotor[i]->encode(can2TxData);
        }
        // sendCan(&hcan2, robot.wheelMotor[0]->tx_id(), can2TxData);

        robot.dribblerFilter->calc(robot.dribbler->vel());
        debug_pose = robot.dribblerFilter->getData();
        debug_vel = robot.dribblerFilter->getDiff();
        // dribblerInput = robot.dribblerPID->calc(60, robot.dribblerFilter->getData());

        if (debug_enable)
        {
            dribblerInput = debug_K * (debug_angle_ref - robot.dribbler->angle()) + debug_D * debug_pose + debug_M * debug_vel;
        }
        else
        {
            dribblerInput = 0;
        }
        robot.dribbler->setTorqInput(dribblerInput);
        robot.dribbler->encode(can2TxData);
        can2TxData[2] = dribbler_mode;
        // sendCan(&hcan2, robot.dribbler->tx_id(), can2TxData);

        // getADC();

        time = HAL_GetTick();

        if (time % 4 == 0)
        {
        }

        robot.imu->decode(robot.imuRxData);
        robot.piEncodeSpi();
        robot.piDecodeSpi();
        robot.piDecodeUart();
        robot.piEncodeUart();
    }
}