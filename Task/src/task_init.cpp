#include "task_init.h"
#include "hardware.h"
#include "robot.h"
#include "task.h"

void task_init()
{
    HAL_SPI_TransmitReceive_DMA(&hspi1, robot.spiTxData, robot.spiRxData, spi_length);
    for (uint8_t i = 0; i < 4; i++)
    {
        robot.wheelMotor[i]->setCurrInput(0);
        robot.dribbler->setCurrInput(0);
    }
    for (int8_t i = 0; i < 10; i++)
    {
        sendCan(&hcan2, robot.wheelMotor[0]->tx_id(), can2TxData);
        HAL_Delay(2);
    }

    HAL_Delay(2000);

    hardware_init();

    HAL_TIM_Base_Start_IT(&htim2);
}