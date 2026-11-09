#include "heavy_board2.h"

#include <string.h>

#include "cmsis_os2.h"
#include "heavy_config.h"
#include "heavy_interboard.h"
#include "heavy_referee.h"
#include "heavy_sbus.h"
#include "heavy_shoot.h"
#include "heavy_watchdog.h"

void HeavyBoard2_Init(void)
{
    HeavySbus_Init();
    HeavyReferee_Init();
    HeavyInterboard_Init();
    HeavyShoot_Init();
    HeavyWatchdog_Init();
}

void HeavyControl_Task(void *argument)
{
    (void)argument;

    for (;;) {
        const heavy_board1_command_t *command;
        const heavy_referee_state_t *referee;
        const heavy_shoot_status_t *shoot;
        heavy_board2_status_t status;
        uint8_t control_link_online;
        uint8_t fire_gate;

        HeavyInterboard_Update();
        HeavySbus_Refresh();
        HeavyReferee_Refresh();
        command = HeavyInterboard_GetCommand();
        referee = HeavyReferee_GetState();
        control_link_online = HeavyInterboard_IsOnline();

        /* The old referee protocol has no usable 2027 42-mm permit field.
         * Board 1 provides the temporary value; its default is zero and
         * therefore fail-safe until the owner confirms the official mapping. */
        HeavyReferee_SetAllowed42mmRounds(command->allowed_42mm_rounds);
        referee = HeavyReferee_GetState();
        fire_gate = ((HeavyInterboard_IsFireLinkHealthy() != 0U) &&
                     (HeavyReferee_IsFirePermitted() != 0U)) ? 1U : 0U;

        HeavyShoot_SetCommand(control_link_online,
                              ((command->command_flags & HEAVY_CMD_FRICTION_ARM) != 0U) ? 1U : 0U,
                              ((command->command_flags & HEAVY_CMD_GIMBAL_ENABLE) != 0U) ? 1U : 0U,
                              command->friction_speed_deci_mps,
                              command->yaw_rate_dps,
                              command->pitch_target_cdeg,
                              fire_gate);
        HeavyShoot_Update();
        shoot = HeavyShoot_GetStatus();

        memset(&status, 0, sizeof(status));
        status.shoot_state = shoot->state;
        status.packet_loss_percent = HeavyInterboard_GetPacketLossPercent();
        status.shooter_heat = referee->shooter_heat;
        status.allowed_42mm_rounds = referee->allowed_42mm_rounds;
        if (shoot->friction_ready != 0U) {
            status.status_flags |= HEAVY_STATUS_FRICTION_READY;
        }
        if (shoot->fire_permitted != 0U) {
            status.status_flags |= HEAVY_STATUS_FIRE_PERMITTED;
        }
        if (HeavyReferee_IsLinkOnline() != 0U) {
            status.status_flags |= HEAVY_STATUS_REFEREE_LINK;
        }
        if (HeavySbus_IsOnline() != 0U) {
            status.status_flags |= HEAVY_STATUS_SBUS_ONLINE;
        }
        if (control_link_online != 0U) {
            status.status_flags |= HEAVY_STATUS_INTERBOARD_ONLINE;
        }
        if (shoot->watchdog_fault != 0U) {
            status.status_flags |= HEAVY_STATUS_WATCHDOG_FAULT;
        }
        HeavyInterboard_SendStatus(&status);

        osDelay(HEAVY_CONTROL_PERIOD_MS);
    }
}
