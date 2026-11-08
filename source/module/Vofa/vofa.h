#ifndef VOFA_H
#define VOFA_H

#include "main.h"
#include <stdint.h>

/*
 * VOFA+ JustFloat 调试模块。
 * 每帧由 N 个 little-endian float 和 4 字节帧尾组成：00 00 80 7F。
 * 当前最多支持 32 个通道，实际发送通道数由 Vofa_SendArray() 的 count 决定。
 */
#define VOFA_MAX_CHANNEL_COUNT 32u

/**
 * @brief 绑定用于发送的 UART，并清空通道缓存。
 * @param huart 已由 CubeMX 初始化的 UART 句柄，模块只使用 TX DMA。
 */
void Vofa_Init(UART_HandleTypeDef *huart);

/**
 * @brief 设置一个通道的值。
 * @param channel 通道号，从 0 开始。
 * @param value 要观察的浮点值。
 */
void Vofa_SetChannel(uint8_t channel, float value);

/**
 * @brief 清空所有通道。
 */
void Vofa_ClearChannels(void);

/**
 * @brief 发送当前通道缓存。
 * @return HAL_OK 发送成功，HAL_BUSY 上一帧尚未完成，HAL_ERROR 未初始化或参数错误。
 */
HAL_StatusTypeDef Vofa_Send(void);

/**
 * @brief 直接发送一组浮点数据。
 * @param values 浮点数组。
 * @param count 发送的通道数量，不能超过 VOFA_MAX_CHANNEL_COUNT。
 * @return HAL_OK、HAL_BUSY 或 HAL_ERROR。
 */
HAL_StatusTypeDef Vofa_SendArray(const float *values, uint16_t count);

/**
 * @brief 查询 UART DMA 发送是否空闲。
 * @return 1 表示可以发送，0 表示未初始化或上一帧仍在发送。
 */
uint8_t Vofa_IsReady(void);

#endif /* VOFA_H */
