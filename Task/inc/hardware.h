#ifndef _HARDWARE_H_
#define _HARDWARE_H_

#include "adc.h"
#include "can.h"
#include "dma.h"
#include "i2c.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"
#include "tim.h"
#include "iwdg.h"

#include "robot.h"

void hardware_init(void);
void sendCan(CAN_HandleTypeDef *hcan, uint32_t id, uint8_t tx_data[8]);
void getADC(void);

extern uint8_t can1TxData[8];
extern uint8_t can2TxData[8];

#endif


