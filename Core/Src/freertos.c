/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * File Name          : freertos.c
 * Description        : Code for freertos applications
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "cmsis_os2.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include <string.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

// #include "DBusSys.h"
// #include "dma.h"
// #include "usart.h"
// #include "PIDtool.h"
#include "arm_debug.h"
#include "Shoot_Task.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef StaticTask_t osStaticThreadDef_t;
typedef StaticSemaphore_t osStaticSemaphoreDef_t;
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/*
 * FreeRTOS 任务总览
 * ----------------
 * 本文件是调度器入口，不实现具体机器人业务；各任务实现在 source/task/ 下。
 *
 * 推荐阅读顺序：
 *   1. 从 MX_FREERTOS_Init() 了解对象创建与任务注册。
 *   2. 跟踪每个 osThreadNew() 的任务入口函数。
 *   3. 再跟踪任务对 source/module/ 与 source/bsp/ 的调用。
 *
 * 下方任务均采用静态分配的控制块和栈空间；新增或调整任务栈时，需同步维护
 * Buffer、ControlBlock 和 attributes 三部分定义。
 */
/* USER CODE BEGIN Variables */
/* 发射/云台任务位于source/task/Shoot_Task/，统一协调CAN1输出。
 * 静态栈1024个uint32_t=4096字节；栈、控制块、属性应一起维护。
 * 保持原2ms控制周期和Normal优先级，机械臂等任务注册沿用原配置。 */
static uint32_t Shoot_TaskBuffer[1024];
static osStaticThreadDef_t Shoot_TaskControlBlock;
static osThreadId_t Shoot_TaskHandle;
static const osThreadAttr_t Shoot_Task_attributes = {
  .name = "Shoot_Task",
  .cb_mem = &Shoot_TaskControlBlock,
  .cb_size = sizeof(Shoot_TaskControlBlock),
  .stack_mem = Shoot_TaskBuffer,
  .stack_size = sizeof(Shoot_TaskBuffer),
  .priority = (osPriority_t)osPriorityNormal,
};
/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for IMU_TempCtrl */
/* 板载 IMU 温控 / 姿态更新任务。 */
osThreadId_t IMU_TempCtrlHandle;
uint32_t IMU_TempCtrlBuffer[ 128 ];
osStaticThreadDef_t IMU_TempCtrlControlBlock;
const osThreadAttr_t IMU_TempCtrl_attributes = {
  .name = "IMU_TempCtrl",
  .cb_mem = &IMU_TempCtrlControlBlock,
  .cb_size = sizeof(IMU_TempCtrlControlBlock),
  .stack_mem = &IMU_TempCtrlBuffer[0],
  .stack_size = sizeof(IMU_TempCtrlBuffer),
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Remoter */
/* 遥控器接收任务：向其他任务发布 DBUS 输入数据。 */
osThreadId_t RemoterHandle;
uint32_t RemoterBuffer[ 256 ];
osStaticThreadDef_t RemoterControlBlock;
const osThreadAttr_t Remoter_attributes = {
  .name = "Remoter",
  .cb_mem = &RemoterControlBlock,
  .cb_size = sizeof(RemoterControlBlock),
  .stack_mem = &RemoterBuffer[0],
  .stack_size = sizeof(RemoterBuffer),
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for jointFollowAngle */

osThreadId_t jointFollowAngleHandle;
uint32_t jointFollowAngleBuffer[512];
osStaticThreadDef_t jointFollowAngleControlBlock;

const osThreadAttr_t jointFollowAngle_attributes = {
    .name       = "jointFollowAngle",// 关节跟随角度任务名称
    .cb_mem     = &jointFollowAngleControlBlock,// 控制块地址（静态创建）
    .cb_size    = sizeof(jointFollowAngleControlBlock),// 控制块大小
    .stack_mem  = &jointFollowAngleBuffer[0],// 栈空间地址
    .stack_size = sizeof(jointFollowAngleBuffer),// 栈大小
    .priority   = (osPriority_t)osPriorityNormal,// 优先级（与默认任务相同）
};

 /* Definitions for Arm_State_Machine_Task */
 /* 可选的机械臂状态机任务定义；当前未在下方注册运行。 */
 osThreadId_t Arm_State_Machine_TaskHandle;
 uint32_t Arm_State_Machine_TaskBuffer[512];
 osStaticThreadDef_t Arm_State_Machine_TaskControlBlock;
 const osThreadAttr_t Arm_State_Machine_Task_attributes = {
     .name = "Arm_State_Machine_Task",
     .cb_mem = &Arm_State_Machine_TaskControlBlock,
     .cb_size = sizeof(Arm_State_Machine_TaskControlBlock),
     .stack_mem = &Arm_State_Machine_TaskBuffer[0],
     .stack_size = sizeof(Arm_State_Machine_TaskBuffer),
     .priority = (osPriority_t) osPriorityNormal,
 };

uint32_t Ctrller_TaskBuffer[1024];  // 栈大小：1024 * 4字节 = 4096字节
/* 自定义控制器数据接收任务。 */
osStaticThreadDef_t Ctrller_TaskControlBlock;  // 静态任务控制块
osThreadId_t Ctrller_TaskHandle;  // 任务句柄
const osThreadAttr_t Ctrller_Task_attributes = {
  .name = "Ctrller_Task",
  .cb_mem = &Ctrller_TaskControlBlock,
  .cb_size = sizeof(Ctrller_TaskControlBlock),
  .stack_mem = &Ctrller_TaskBuffer[0],
  .stack_size = sizeof(Ctrller_TaskBuffer),
  .priority = (osPriority_t)osPriorityNormal,
};

/* Definitions for imuBinarySem01 */
osSemaphoreId_t imuBinarySem01Handle;
osStaticSemaphoreDef_t imuBinarySemControlBlock;
const osSemaphoreAttr_t imuBinarySem01_attributes = {
  .name = "imuBinarySem01",
  .cb_mem = &imuBinarySemControlBlock,
  .cb_size = sizeof(imuBinarySemControlBlock),
};
 /* Definitions for Trajectory_Publisher */
 osThreadId_t Trajectory_PublisherHandle;
 uint32_t Trajectory_PublisherBuffer[256];
 osStaticThreadDef_t Trajectory_PublisherControlBlock;
 const osThreadAttr_t Trajectory_Publisher_attributes = {
     .name = "Trajectory_Publisher",
     .cb_mem = &Trajectory_PublisherControlBlock,
     .cb_size = sizeof(Trajectory_PublisherControlBlock),
     .stack_mem = &Trajectory_PublisherBuffer[0],
     .stack_size = sizeof(Trajectory_PublisherBuffer),
     .priority = (osPriority_t) osPriorityNormal,
 };
//  /* Definitions for test_CAN3_ARM */
//  osThreadId_t test_CAN3_ARMHandle;
//  uint32_t test_CAN3_ARMBuffer[256];// 栈空间
//  osStaticThreadDef_t test_CAN3_ARMControlBlock;
//  const osThreadAttr_t test_CAN3_ARM_attributes = {
//      .name = "test_CAN3_ARM",
//      .cb_mem = &test_CAN3_ARMControlBlock,// 控制块地址（静态创建）
//      .cb_size = sizeof(test_CAN3_ARMControlBlock),// 控制块大小
//      .stack_mem = &test_CAN3_ARMBuffer[0],// 栈空间地址
//      .stack_size = sizeof(test_CAN3_ARMBuffer),// 栈大小
//      .priority = (osPriority_t) osPriorityNormal,// 优先级（与默认任务相同）
//  };

 /* Definitions for Arm_Reset */
 /* 机械臂回零 / 复位任务。 */
 osThreadId_t Arm_ResetHandle;
 uint32_t Arm_ResetBuffer[1024];
 osStaticThreadDef_t Arm_ResetControlBlock;
 const osThreadAttr_t Arm_Reset_attributes = {
     .name = "Arm_Reset",
     .cb_mem = &Arm_ResetControlBlock,
     .cb_size = sizeof(Arm_ResetControlBlock),
     .stack_mem = &Arm_ResetBuffer[0],
     .stack_size = sizeof(Arm_ResetBuffer),
     .priority = (osPriority_t) osPriorityNormal,
 };

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void jointFollowAngle(void *arguments);
void Ctrller_Task(void *arguments);
void arm_reset_task(void *argument);
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void IMU_TempCtrlTask(void *argument);
void Remoter_Task(void *argument);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /*
   * 本函数在 osKernelStart() 前仅执行一次。
   * 依赖 RTOS 对象的初始化放在这里，周期性控制逻辑放在对应任务中。
   */
  /* USER CODE BEGIN Init */
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of imuBinarySem01 */
  /* IMU 相关生产者 / 消费者同步用的事件型信号量。 */
  imuBinarySem01Handle = osSemaphoreNew(1, 0, &imuBinarySem01_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /*
   * 当前启用的机械臂、发射/云台任务与所需基础服务。
   */
  /* creation of defaultTask */
  /* 空闲占位任务；必须主动让出 CPU，保证控制任务正常运行。 */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of IMU_TempCtrl */
  //IMU_TempCtrlHandle = osThreadNew(IMU_TempCtrlTask, NULL, &IMU_TempCtrl_attributes);

  /* creation of Remoter */
  //RemoterHandle = osThreadNew(Remoter_Task, NULL, &Remoter_attributes);

  
  /* 六轴机械臂关节跟随与末端执行器控制循环。 */
  jointFollowAngleHandle = osThreadNew(jointFollowAngle,NULL, &jointFollowAngle_attributes);
  /* 自定义控制器数据接收。 */
  Ctrller_TaskHandle = osThreadNew(Ctrller_Task, NULL, &Ctrller_Task_attributes);

  /* 机械臂复位 / 回零流程。 */
#if !DEBUG_READ_DATA_ONLY
  Arm_ResetHandle = osThreadNew(arm_reset_task, NULL, &Arm_Reset_attributes);
#endif

  /* USER CODE BEGIN RTOS_EVENTS */
  /* 只创建一次：任务内部初始化新增电机、板间通信及裁判接收，然后周期控制。
   * 不再额外创建独立yaw/摩擦轮发送任务，避免相互覆盖同一个DJI组帧。 */
  Shoot_TaskHandle = osThreadNew(Shoot_Task, NULL, &Shoot_Task_attributes);
  if (Shoot_TaskHandle == NULL) Error_Handler();
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
 * @brief  Function implementing the defaultTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  UNUSED(argument);
  /* Infinite loop */
  for (;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_IMU_TempCtrlTask */
/**
* @brief Function implementing the IMU_TempCtrl thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_IMU_TempCtrlTask */
__weak void IMU_TempCtrlTask(void *argument)
{
  /* 弱符号兜底实现会被 source/task/ 中的同名强符号实现替换。 */
  /* USER CODE BEGIN IMU_TempCtrlTask */
  UNUSED(argument);
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END IMU_TempCtrlTask */
}

/* USER CODE BEGIN Header_Remoter_Task */
/**
* @brief Function implementing the Remoter thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Remoter_Task */
__weak void Remoter_Task(void *argument)
{
  /* 仅在未链接真实 Remoter_Task 实现时使用此兜底函数。 */
  /* USER CODE BEGIN Remoter_Task */
  UNUSED(argument);
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Remoter_Task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/*
 * 下方弱符号函数是 CubeMX 生成的链接期兜底实现。
 * 若 source/task/ 中存在同名真实实现，会优先链接真实实现。
 * 不应在这里编写正式业务逻辑，否则可能被同名强符号静默覆盖。
 */

__weak void jointFollowAngle(void *argument)
{
  UNUSED(argument);
  for (;;) {
    osDelay(1);
  }
}
__weak void Ctrller_Task(void *argument)
{
  UNUSED(argument);
  for(;;)
  {
    
    osDelay(1);
  }
}

/* USER CODE END Application */
