#ifndef HEAVY_INTERBOARD_H
#define HEAVY_INTERBOARD_H
#include "heavy_protocol.h"
#include "heavy_actuators.h"
#ifdef __cplusplus
extern "C" {
#endif
uint8_t HeavyInterboard_Init(void);
void HeavyInterboard_Process(uint32_t now_ms);
const heavy_link_t *HeavyInterboard_GetLink(void);
void HeavyInterboard_SendStatus(uint32_t now_ms, const heavy_actuator_status_t *act,
                               uint8_t referee_online, uint8_t sbus_online, uint16_t heat);
#ifdef __cplusplus
}
#endif
#endif
