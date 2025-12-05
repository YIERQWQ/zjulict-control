#include "imu.h"

namespace imu
{
    const float imu_k[3] = {16.0f * 9.8f / 32768.0f, 2000.0f / 32768.0f, 180.0f / 32768.0f};

    void Imu::decode(uint8_t raw_data[imuRxDataLength])
    {
        for (uint8_t j = 0; j < 33; j++)
        {
            if (raw_data[j] != 0x55)
                continue;

            for (uint8_t i = 0; i < 3; i++)
            {
                if (raw_data[j + 0 + i * 11] == 0x55 && raw_data[j + 1 + i * 11] == (0x51 + i))
                {

                    if (sumcrc(&(raw_data[j + 0 + i * 11])))
                    {
                        data[0 + i * 4] = (short)(((short)raw_data[j + 3 + i * 11] << 8) | raw_data[j + 2 + i * 11]) * imu_k[i];
                        data[1 + i * 4] = (short)(((short)raw_data[j + 5 + i * 11] << 8) | raw_data[j + 4 + i * 11]) * imu_k[i];
                        data[2 + i * 4] = (short)(((short)raw_data[j + 7 + i * 11] << 8) | raw_data[j + 6 + i * 11]) * imu_k[i];
                        // data[kVoltage] = (short)(((short)raw_data[9] << 8) | raw_data[8]) / 100.0;
                    }
                }
            }
        }
    }

    bool Imu::sumcrc(const uint8_t raw_data[11])
    {
        uint16_t sum = 0x0;
        for (size_t i = 0; i < 10; i++)
        {
            sum += raw_data[i]; // �ۼ�����
        }
        uint8_t crc = sum & 0xFF; // ȡУ��͵ĵ�8λ
        return (crc == raw_data[10]);
    }

    void Imu::getData(float imu_data_t[9]) const
    {
        imu_data_t[0] = data[kAccX];
        imu_data_t[1] = data[kAccY];
        imu_data_t[2] = data[kAccZ];

        // correct mapping: omega X/Y/Z
        imu_data_t[3] = data[kOmegaX];
        imu_data_t[4] = data[kOmegaY];
        imu_data_t[5] = data[kOmegaZ];

        imu_data_t[6] = data[kAngleX];
        imu_data_t[7] = data[kAngleY];
        imu_data_t[8] = data[kAngleZ];
    }

    // void Imu::update(uint8_t imuTxData[5])
    // {
    //     switch (mode)
    //     {
    //     case kAcc:
    //         memcpy(imuTxData, getAcc, 5);
    //         break;
    //     case kOmega:
    //         memcpy(imuTxData, getOmega, 5);
    //         break;
    //     case kAngle:
    //         memcpy(imuTxData, getAngle, 5);
    //         break;
    //     case kAuto:
    //         switch (auto_mode)
    //         {
    //         case kAcc:
    //             memcpy(imuTxData, getAcc, 5);
    //             break;
    //         case kOmega:
    //             memcpy(imuTxData, getOmega, 5);
    //             break;
    //         case kAngle:
    //             memcpy(imuTxData, getAngle, 5);
    //             break;
    //         default:
    //             auto_mode = 0;
    //             break;
    //         }
    //         break;
    //     default:
    //         memcpy(imuTxData, getAngle, 5);
    //         break;
    //     }
    // }

} // namespace imu
