#ifndef HEAVY_HOST_CAN_BSP_H
#define HEAVY_HOST_CAN_BSP_H
#include "main.h"
extern FDCAN_HandleTypeDef hfdcan1;
uint8_t fdcanx_send_data(FDCAN_HandleTypeDef *port, uint16_t id, uint8_t *data, uint32_t length);
#endif
