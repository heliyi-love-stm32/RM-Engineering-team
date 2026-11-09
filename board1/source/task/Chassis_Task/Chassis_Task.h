#ifndef CHASSIS_TASK_H
#define CHASSIS_TASK_H

/**
 * @file Chassis_Task.h
 * @brief Chassis-task public state and command interface.
 *
 * The task arbitrates between DBUS and keyboard control, publishes the active
 * chassis mode, and starts the timed rising/reverse/auto-normal sequences.
 * Callers should change state through these functions rather than writing the
 * exported variables directly; the task synchronizes its state every cycle.
 */

#include "cmsis_os2.h"
#include "chassis_config.h"

#include <stdint.h>

typedef enum
{
    /** All chassis and rising motor outputs are disabled. */
    CHASSIS_MODE_STATE_PowerOff = 0,
    CHASSIS_MODE_STATE_Normal = 1,
    CHASSIS_MODE_STATE_Rising = 2,
    CHASSIS_MODE_STATE_Downstairs = 3,
} Chassis_Mode_State_t;

typedef enum
{
    /** Commands originate from the physical DBUS remote controller. */
    CHASSIS_CONTROL_SOURCE_STATE_DBUS = 0,
    CHASSIS_CONTROL_SOURCE_STATE_Keyboard = 1,
} Chassis_Control_Source_State_t;

typedef enum
{
    /** Execute the short, single-lift timed rising sequence. */
    CHASSIS_RISING_BEHAVIOR_STATE_SingleLift = 0,
    CHASSIS_RISING_BEHAVIOR_STATE_DoubleLift = 1,
} Chassis_Rising_Behavior_State_t;

typedef enum
{
    /** Keyboard W/S maps to the chassis forward/backward axis. */
    CHASSIS_KEYBOARD_DIRECTION_STATE_Front = 0,
    CHASSIS_KEYBOARD_DIRECTION_STATE_Right = 1,
} Chassis_Keyboard_Direction_State_t;

/** Latest mode resolved by the chassis task; volatile because it is shared. */
extern volatile Chassis_Mode_State_t g_chassis_mode_state;
extern volatile Chassis_Control_Source_State_t g_chassis_control_source_state;
extern volatile Chassis_Rising_Behavior_State_t g_chassis_rising_behavior_state;
extern volatile Chassis_Keyboard_Direction_State_t g_chassis_keyboard_direction_state;

/** @brief FreeRTOS chassis task entry point. @param argument Unused task argument. */
void Chassis_Task(void *argument);

/** @brief Latch or release the highest-priority software power-off request. */
void Chassis_ForcePowerOff(uint8_t enable);
/** @brief Return the software power-off latch (0 or 1). */
uint8_t Chassis_IsForcePowerOff(void);
/** @brief Select DBUS or keyboard as the persistent control source. */
void Chassis_SetControlSourceState(Chassis_Control_Source_State_t source_state);
/** @brief Request a chassis mode; the task may subsequently override it from its source input. */
void Chassis_SetModeState(Chassis_Mode_State_t mode_state);
/** @brief Select the keyboard-test rising behavior used by the timed sequence. */
void Chassis_SetRisingBehaviorState(Chassis_Rising_Behavior_State_t behavior_state);
/** @brief Select the keyboard translation reference direction. */
void Chassis_SetKeyboardDirectionState(Chassis_Keyboard_Direction_State_t direction_state);
/** @brief Toggle keyboard translation reference between front and right. */
void Chassis_ToggleKeyboardDirectionState(void);
/** @brief Process an R-key press; Ctrl+R starts rising, R alone selects lift behavior where permitted. */
void Chassis_HandleRisingKeyPressed(uint8_t ctrl_pressed);
/** @brief Queue the fixed-duration reverse sequence for the next task cycle. */
void Chassis_RequestKeyboardReverseSequence(void);
/** @brief Queue the keyboard auto-normal sequence for the next task cycle. */
void Chassis_RequestKeyboardAutoNormalSequence(void);
/** @brief Return the mode currently published by the chassis task. */
Chassis_Mode_State_t Chassis_GetModeState(void);
/** @brief Return the currently active control source. */
Chassis_Control_Source_State_t Chassis_GetControlSourceStatePublic(void);
/** @brief Return the selected timed rising behavior. */
Chassis_Rising_Behavior_State_t Chassis_GetRisingBehaviorState(void);
/** @brief Return the active keyboard translation reference direction. */
Chassis_Keyboard_Direction_State_t Chassis_GetKeyboardDirectionState(void);

#endif // !CHASSIS_TASK_H
