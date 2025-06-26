#ifndef __SIM808_H__
#define __SIM808_H__

#include "stm32f1xx_hal.h"

HAL_StatusTypeDef SIM808_Init(UART_HandleTypeDef *huart);

void SIM808_SendSMS(UART_HandleTypeDef *huart, const char *phoneNumber, const char *message);

#endif /* __SIM808_H__ */
