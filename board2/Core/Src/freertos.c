/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * File Name          : freertos.c
 * Description        : Board-2 FreeRTOS task registration.
 ******************************************************************************
 */
/* USER CODE END Header */

#include "FreeRTOS.h"
#include "cmsis_os2.h"
#include "main.h"

#include "heavy_board2.h"
#include "heavy_watchdog.h"

typedef StaticTask_t osStaticThreadDef_t;

/* Each task deliberately has its own control block, stack buffer and
 * osThreadAttr_t.  Keep the three declarations together when resizing a task. */
static osThreadId_t defaultTaskHandle;
static const osThreadAttr_t defaultTask_attributes = {
    .name = "defaultTask",
    .stack_size = 128U * 4U,
    .priority = (osPriority_t)osPriorityLow,
};

static osThreadId_t HeavyControlTaskHandle;
static uint32_t HeavyControlTaskBuffer[512];
static osStaticThreadDef_t HeavyControlTaskControlBlock;
static const osThreadAttr_t HeavyControlTask_attributes = {
    .name = "HeavyControl",
    .cb_mem = &HeavyControlTaskControlBlock,
    .cb_size = sizeof(HeavyControlTaskControlBlock),
    .stack_mem = &HeavyControlTaskBuffer[0],
    .stack_size = sizeof(HeavyControlTaskBuffer),
    .priority = (osPriority_t)osPriorityAboveNormal,
};

static osThreadId_t HeavyWatchdogTaskHandle;
static uint32_t HeavyWatchdogTaskBuffer[256];
static osStaticThreadDef_t HeavyWatchdogTaskControlBlock;
static const osThreadAttr_t HeavyWatchdogTask_attributes = {
    .name = "HeavyWatchdog",
    .cb_mem = &HeavyWatchdogTaskControlBlock,
    .cb_size = sizeof(HeavyWatchdogTaskControlBlock),
    .stack_mem = &HeavyWatchdogTaskBuffer[0],
    .stack_size = sizeof(HeavyWatchdogTaskBuffer),
    .priority = (osPriority_t)osPriorityNormal,
};

static void StartDefaultTask(void *argument)
{
    UNUSED(argument);
    for (;;) {
        osDelay(10U);
    }
}

void MX_FREERTOS_Init(void)
{
    defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);
    HeavyControlTaskHandle = osThreadNew(HeavyControl_Task, NULL, &HeavyControlTask_attributes);
    HeavyWatchdogTaskHandle = osThreadNew(HeavyWatchdog_Task, NULL, &HeavyWatchdogTask_attributes);

    /* Handles are intentionally retained for debugger visibility. */
    (void)defaultTaskHandle;
    (void)HeavyControlTaskHandle;
    (void)HeavyWatchdogTaskHandle;
}
