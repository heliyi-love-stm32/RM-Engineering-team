#ifndef HEAVY_REFEREE_H
#define HEAVY_REFEREE_H

#include <stdint.h>

/* This wrapper deliberately exposes only the information needed by the heavy
 * shooter.  The inherited protocol does not yet expose the 2027 remaining
 * 42-mm ammunition field, so its value is supplied by the board-1 command
 * until the official frame definition is integrated. */
typedef struct
{
    uint16_t shooter_heat;
    uint16_t shooter_heat_limit;
    uint8_t shooter_power_enabled;
    uint8_t link_online;
    uint8_t allowed_42mm_rounds;
} heavy_referee_state_t;

void HeavyReferee_Init(void);
void HeavyReferee_Refresh(void);
void HeavyReferee_SetAllowed42mmRounds(uint8_t rounds);
const heavy_referee_state_t *HeavyReferee_GetState(void);
uint8_t HeavyReferee_IsLinkOnline(void);
uint8_t HeavyReferee_IsFirePermitted(void);

#endif /* HEAVY_REFEREE_H */
