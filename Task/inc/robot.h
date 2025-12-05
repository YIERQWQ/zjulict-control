#ifndef _ROBOT_H_
#define _ROBOT_H_

#include "motor.h"
#include "imu.h"
#include "pid.h"
#include "td.h"

const uint8_t piTxDataUartLength = 3;
const uint8_t piRxDataUartLength = 7;
const uint8_t spi_length = 38;

const float PI = 3.1415926;
const float infraredThreshold = 3.3;

const uint8_t piRxFrameHeader = 0xbb;
const float bat_k = 12.27;
const float cap_k = 113.73;

enum CM4_to_stm32_spi_flag : uint8_t
{
    kUseImu = 0
};

enum stm32_to_CM4_spi_flag : uint8_t
{
    kInfrare = 0,
    kGetBall = 1,
    kImuOnline = 2
};

struct __attribute__((packed)) CM4_to_stm32_spi
{
    uint8_t drib_power;
    int16_t vel[3];
    int16_t angle_pid[3];
    int16_t wheel_pid[3];
    bool use_imu;
    // uint8_t flag;
};

struct __attribute__((packed)) stm32_to_CM4_spi
{
    int16_t imu_data[9];
    bool infrare_flag;
    bool getBall;
    bool imu_online;
    int8_t battery_vol;
    int16_t cap_vol;
    int16_t wheel[4];
    int16_t wheel_ref[4];
};

enum vel : uint8_t
{
    kVelX = 0,
    kVelY = 1,
    kVelW = 2
};

class robot_
{
public:
    robot_();
    ~robot_();

    void piEncodeUart(void);
    void piDecodeUart(void);

    void piEncodeSpi(void);
    void piDecodeSpi(void);

    void ik_solve(void);
    void motion_planner(const double _dt);

    float INFRA_ADC1_Value = 0;
    float BATVOL_ADC2_Value = 0;
    float CAPVOL_ADC3_Value = 0;

    motor::Motor *wheelMotor[4] = {nullptr};
    motor::Motor *dribbler = nullptr;
    imu::Imu *imu = nullptr;
    pid::PID *wheelPID[4] = {nullptr};
    pid::PID *wheelVelPID[4] = {nullptr};
    pid::PID *dribblerPID = nullptr;
    td::td *dribblerFilter = nullptr;
    td::td *wheelFilter[4] = {nullptr};

    // rpm
    float motor_vel[4] = {0};
    float last_motor_vel[4] = {0};
    float motor_torq[4] = {0};
    float motor_acc[4] = {0};
    float motor_acc_t[4] = {0};
    float motor_F[4] = {0};
    float motor_Ff[4] = {0};

    // huart1 <---> linux
    uint8_t piRxDataUart[piRxDataUartLength] = {0};
    uint8_t piTxDataUart[piTxDataUartLength] = {0};

    uint8_t spiRxData[spi_length] = {0};
    uint8_t spiTxData[spi_length] = {0};

    uint8_t imuRxData[imu::imuRxDataLength] = {0};
    uint8_t imuTxData[imu::imuTxDataLength] = {0};

    CM4_to_stm32_spi SpiRx;
    stm32_to_CM4_spi SpiTx;

    float wheel_PID[3] = {0.000, 0.005, 0};
    float wheelVel_PID[3] = {0.5, 0.1, 0};

    // m/s
    float robot_vel[3] = {0};

    float robot_real_vel[3] = {0};
    float last_robot_real_vel[3] = {0};
    float robot_acc[3] = {0};
    float ikSolveBasis[3] = {0, 1, 2};
    float ikSolveInvB[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};


private:
};

extern robot_ robot;

#endif