#include "sim808.h"
#include <string.h>
#include <stdio.h>

#define SIM808_TIMEOUT 1000

HAL_StatusTypeDef SIM808_Init(UART_HandleTypeDef *huart) {

    return HAL_OK;
}

static void SIM808_SendATCommand(UART_HandleTypeDef *huart, const char *command) {
    HAL_UART_Transmit(huart, (uint8_t*)command, strlen(command), SIM808_TIMEOUT);
}

void SIM808_SendSMS(UART_HandleTypeDef *huart, const char *phoneNumber, const char *message) {
    char command[50];
		

    SIM808_SendATCommand(huart, "AT+CMGF=1\r\n");
		HAL_Delay(1000);
    sprintf(command, "AT+CMGS=\"%s\"\r\n", phoneNumber);
    SIM808_SendATCommand(huart, command);
		HAL_Delay(1000);
    HAL_UART_Transmit(huart, (uint8_t*)message, strlen(message), SIM808_TIMEOUT);
		HAL_Delay(1000);
    HAL_UART_Transmit(huart, (uint8_t*)"\x1A", 1, SIM808_TIMEOUT);

//    HAL_Delay(5000);
}
