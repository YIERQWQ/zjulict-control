#ifndef _IMU_H_
#define _IMU_H_

#include "base.h"

namespace imu
{
    enum imuData : uint8_t
    {
        kAccX = 0,
        kAccY,
        kAccZ,
        kTemperature,
        kOmegaX,
        kOmegaY,
        kOmegaZ,
        kVoltage,
        kAngleX,
        kAngleY,
        kAngleZ,
        kVersion
    };

    enum getImuMode : uint8_t
    {
        kAcc = 0,
        kOmega,
        kAngle,
        kAuto
    };

    const uint8_t getAcc[5] = {0xFF, 0xAA, 0x27, 0x34, 0x00};
    const uint8_t getOmega[5] = {0xFF, 0xAA, 0x27, 0x37, 0x00};
    const uint8_t getAngle[5] = {0xFF, 0xAA, 0x27, 0x3D, 0x00};

    const uint8_t imuRxDataLength = 11*3*2;
    const uint8_t imuTxDataLength = 5;

    class Imu
    {
    public:
        Imu() {};
        ~Imu() = default;

        void decode(uint8_t raw_data[imuRxDataLength]);
        float getData(imuData imu_data_t) const { return data[imu_data_t]; };
        void getData(float imu_data_t[9]) const;
        // void update(uint8_t imuTxData[5]);
        // void setUpdateMode(getImuMode mode_t) { mode = mode_t; };

    private:
        float data[12] = {0};
        bool sumcrc(const uint8_t raw_data[11]);
        // getImuMode mode = kAngle;
        // uint8_t auto_mode = kAngle;
    };

}

#endif