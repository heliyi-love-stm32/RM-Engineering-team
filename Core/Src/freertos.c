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
#include "auto_get_timer_init.h"
#include "arm_debug.h"
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
/* Definitions for uartTest */
osThreadId_t uartTestHandle;
uint32_t uartTestBuffer[512];
osStaticThreadDef_t uartTestControlBlock;
const osThreadAttr_t uartTest_attributes = {
    .name       = "uartTest",
    .cb_mem     = &uartTestControlBlock,
    .cb_size    = sizeof(uartTestControlBlock),
    .stack_mem  = &uartTestBuffer[0],
    .stack_size = sizeof(uartTestBuffer),
    .priority   = (osPriority_t)osPriorityNormal,
};
/* Definitions for SerialPlot */
osThreadId_t SerialPlotHandle;
uint32_t SerialPlotBuffer[512];
osStaticThreadDef_t SerialPlotControlBlock;
const osThreadAttr_t SerialPlot_attributes = {
    .name       = "SerialPlot",
    .cb_mem     = &SerialPlotControlBlock,
    .cb_size    = sizeof(SerialPlotControlBlock),
    .stack_mem  = &SerialPlotBuffer[0],
    .stack_size = sizeof(SerialPlotBuffer),
    .priority   = (osPriority_t)osPriorityNormal,
};
/* Definitions for motor_test */
osThreadId_t motorTestHandle;
uint32_t motorTestBuffer[512];
osStaticThreadDef_t motorTestControlBlock;

const osThreadAttr_t motorTest_attributes = {
    .name       = "motorTest",
    .cb_mem     = &motorTestControlBlock,
    .cb_size    = sizeof(motorTestControlBlock),
    .stack_mem  = &motorTestBuffer[0],
    .stack_size = sizeof(motorTestBuffer),
    .priority   = (osPriority_t)osPriorityNormal,
};

/* Definitions for jointFollowAngle */

osThreadId_t jointFollowAngleHandle;
uint32_t jointFollowAngleBuffer[512];
osStaticThreadDef_t jointFollowAngleControlBlock;

const osThreadAttr_t jointFollowAngle_attributes = {
    .name       = "jointFollowAngle",
    .cb_mem     = &jointFollowAngleControlBlock,
    .cb_size    = sizeof(jointFollowAngleControlBlock),
    .stack_mem  = &jointFollowAngleBuffer[0],
    .stack_size = sizeof(jointFollowAngleBuffer),
    .priority   = (osPriority_t)osPriorityNormal,
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
 /* Definitions for SerialPort */
 osThreadId_t SerialPortHandle;
 uint32_t SerialPortBuffer[256];
 osStaticThreadDef_t SerialPortControlBlock;
 const osThreadAttr_t SerialPort_attributes = {
     .name = "SerialPort",
     .cb_mem = &SerialPortControlBlock,
     .cb_size = sizeof(SerialPortControlBlock),
     .stack_mem = &SerialPortBuffer[0],
     .stack_size = sizeof(SerialPortBuffer),
     .priority = (osPriority_t) osPriorityNormal,
 };
uint32_t Chassis_TaskBuffer[512];  // 栈大小：128 * 4字节 = 512字节
/* 底盘主控制循环：模式选择、行驶及抬升逻辑。 */
osStaticThreadDef_t Chassis_TaskControlBlock;  // 静态任务控制块
osThreadId_t Chassis_TaskHandle;  // 任务句柄
const osThreadAttr_t Chassis_Task_attributes = {
  .name = "Chassis_Task",        // 任务名称（调试用）
  .cb_mem = &Chassis_TaskControlBlock,  // 控制块地址（静态创建）
  .cb_size = sizeof(Chassis_TaskControlBlock),  // 控制块大小
  .stack_mem = &Chassis_TaskBuffer[0],  // 栈空间地址
  .stack_size = sizeof(Chassis_TaskBuffer),  // 栈大小
  .priority = (osPriority_t)osPriorityNormal,  // 优先级（与默认任务相同）
};

uint32_t Referee_TaskBuffer[1024];  // 栈大小：1024 * 4字节 = 4096字节
/* 裁判系统 / 自定义控制器数据处理任务。 */
osStaticThreadDef_t Referee_TaskControlBlock;  // 静态任务控制块
osThreadId_t Referee_TaskHandle;  // 任务句柄
const osThreadAttr_t Referee_Task_attributes = {
  .name = "Referee_Task",        // 任务名称（调试用）
  .cb_mem = &Referee_TaskControlBlock,  // 控制块地址（静态创建）
  .cb_size = sizeof(Referee_TaskControlBlock),  // 控制块大小
  .stack_mem = &Referee_TaskBuffer[0],  // 栈空间地址
  .stack_size = sizeof(Referee_TaskBuffer),  // 栈大小
  .priority = (osPriority_t)osPriorityNormal,  // 优先级（与默认任务相同）
};

uint32_t Watchdog_TaskBuffer[512];
/* 健康监测任务：电机、遥控器超时检测及告警。 */
osStaticThreadDef_t Watchdog_TaskControlBlock;
osThreadId_t Watchdog_TaskHandle;
const osThreadAttr_t Watchdog_Task_attributes = {
  .name = "Watchdog_Task",
  .cb_mem = &Watchdog_TaskControlBlock,
  .cb_size = sizeof(Watchdog_TaskControlBlock),
  .stack_mem = &Watchdog_TaskBuffer[0],
  .stack_size = sizeof(Watchdog_TaskBuffer),
  .priority = (osPriority_t)osPriorityLow,
};

uint32_t IMU_TaskBuffer[512];
/* 外置 IMU 通信及姿态数据更新任务。 */
osStaticThreadDef_t IMU_TaskControlBlock;
osThreadId_t IMU_TaskHandle;
const osThreadAttr_t IMU_Task_attributes = {
  .name = "IMU_Task",
  .cb_mem = &IMU_TaskControlBlock,
  .cb_size = sizeof(IMU_TaskControlBlock),
  .stack_mem = &IMU_TaskBuffer[0],
  .stack_size = sizeof(IMU_TaskBuffer),
  .priority = (osPriority_t)osPriorityLow,
};

/* Definitions for uart_Transmit_Angle */
osThreadId_t uart_Transmit_AngleHandle;
uint32_t uart_Transmit_AngleBuffer[1024];
osStaticThreadDef_t uart_Transmit_AngleControlBlock;
const osThreadAttr_t uart_Transmit_Angle_attributes = {
    .name       = "uart_Transmit_Angle",
    .cb_mem     = &uart_Transmit_AngleControlBlock,
    .cb_size    = sizeof(uart_Transmit_AngleControlBlock),
    .stack_mem  = &uart_Transmit_AngleBuffer[0],
    .stack_size = sizeof(uart_Transmit_AngleBuffer),
    .priority   = (osPriority_t)osPriorityNormal,
};
/* Definitions for imuBinarySem01 */
osSemaphoreId_t imuBinarySem01Handle;
osStaticSemaphoreDef_t imuBinarySemControlBlock;
const osSemaphoreAttr_t imuBinarySem01_attributes = {
  .name = "imuBinarySem01",
  .cb_mem = &imuBinarySemControlBlock,
  .cb_size = sizeof(imuBinarySemControlBlock),
};
/* Definitions for controlBinaryIMU */
osSemaphoreId_t controlBinaryIMUHandle;
osStaticSemaphoreDef_t controlBinaryIMUControlBlock;
const osSemaphoreAttr_t controlBinaryIMU_attributes = {
  .name = "controlBinaryIMU",
  .cb_mem = &controlBinaryIMUControlBlock,
  .cb_size = sizeof(controlBinaryIMUControlBlock),
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
//  uint32_t test_CAN3_ARMBuffer[256];
//  osStaticThreadDef_t test_CAN3_ARMControlBlock;
//  const osThreadAttr_t test_CAN3_ARM_attributes = {
//      .name = "test_CAN3_ARM",
//      .cb_mem = &test_CAN3_ARMControlBlock,
//      .cb_size = sizeof(test_CAN3_ARMControlBlock),
//      .stack_mem = &test_CAN3_ARMBuffer[0],
//      .stack_size = sizeof(test_CAN3_ARMBuffer),
//      .priority = (osPriority_t) osPriorityNormal,
//  };

 /* Definitions for Joint1_Move_Task */
 osThreadId_t Joint1_Move_TaskHandle;
 uint32_t Joint1_Move_TaskBuffer[128];
 osStaticThreadDef_t Joint1_Move_TaskControlBlock;
 const osThreadAttr_t Joint1_Move_Task_attributes = {
     .name = "Joint1_Move_Task",
     .cb_mem = &Joint1_Move_TaskControlBlock,
     .cb_size = sizeof(Joint1_Move_TaskControlBlock),
     .stack_mem = &Joint1_Move_TaskBuffer[0],
     .stack_size = sizeof(Joint1_Move_TaskBuffer),
     .priority = (osPriority_t) osPriorityNormal,
 };
 /* Definitions for Joint2_Move_Task  */
 osThreadId_t Joint2_Move_TaskHandle;
 uint32_t Joint2_Move_TaskBuffer[128];
 osStaticThreadDef_t Joint2_Move_TaskControlBlock;
 const osThreadAttr_t Joint2_Move_Task_attributes = {
     .name = "Joint2_Move_Task ",
     .cb_mem = &Joint2_Move_TaskControlBlock,
     .cb_size = sizeof(Joint2_Move_TaskControlBlock),
     .stack_mem = &Joint2_Move_TaskBuffer[0],
     .stack_size = sizeof(Joint2_Move_TaskBuffer),
     .priority = (osPriority_t) osPriorityNormal,
 };
 /* Definitions for Joint3_Move_Task */
 osThreadId_t Joint3_Move_TaskHandle;
 uint32_t Joint3_Move_TaskBuffer[128];
 osStaticThreadDef_t Joint3_Move_TaskControlBlock;
 const osThreadAttr_t Joint3_Move_Task_attributes = {
     .name = "Joint3_Move_Task",
     .cb_mem = &Joint3_Move_TaskControlBlock,
     .cb_size = sizeof(Joint3_Move_TaskControlBlock),
     .stack_mem = &Joint3_Move_TaskBuffer[0],
     .stack_size = sizeof(Joint3_Move_TaskBuffer),
     .priority = (osPriority_t) osPriorityNormal,
 };
 /* Definitions for Joint4_Move_Task */
 osThreadId_t Joint4_Move_TaskHandle;
 uint32_t Joint4_Move_TaskBuffer[128];
 osStaticThreadDef_t Joint4_Move_TaskControlBlock;
 const osThreadAttr_t Joint4_Move_Task_attributes = {
     .name = "Joint4_Move_Task",
     .cb_mem = &Joint4_Move_TaskControlBlock,
     .cb_size = sizeof(Joint4_Move_TaskControlBlock),
     .stack_mem = &Joint4_Move_TaskBuffer[0],
     .stack_size = sizeof(Joint4_Move_TaskBuffer),
     .priority = (osPriority_t) osPriorityNormal,
 };
 /* Definitions for Joint5_Move_Task */
 osThreadId_t Joint5_Move_TaskHandle;
 uint32_t Joint5_Move_TaskBuffer[128];
 osStaticThreadDef_t Joint5_Move_TaskControlBlock;
 const osThreadAttr_t Joint5_Move_Task_attributes = {
     .name = "Joint5_Move_Task",
     .cb_mem = &Joint5_Move_TaskControlBlock,
     .cb_size = sizeof(Joint5_Move_TaskControlBlock),
     .stack_mem = &Joint5_Move_TaskBuffer[0],
     .stack_size = sizeof(Joint5_Move_TaskBuffer),
     .priority = (osPriority_t) osPriorityAboveNormal,
 };
 /* Definitions for Joint6_Move_Task */
 osThreadId_t Joint6_Move_TaskHandle;
 uint32_t Joint6_Move_TaskBuffer[128];
 osStaticThreadDef_t Joint6_Move_TaskControlBlock;
 const osThreadAttr_t Joint6_Move_Task_attributes = {
     .name = "Joint6_Move_Task",
     .cb_mem = &Joint6_Move_TaskControlBlock,
     .cb_size = sizeof(Joint6_Move_TaskControlBlock),
     .stack_mem = &Joint6_Move_TaskBuffer[0],
     .stack_size = sizeof(Joint6_Move_TaskBuffer),
     .priority = (osPriority_t) osPriorityAboveNormal,
 };

 /* Definitions for View_Gimbal_Task */
 /* 视觉云台 / 舵机控制任务。 */
 osThreadId_t View_Gimbal_TaskHandle;
 uint32_t View_Gimbal_TaskBuffer[128];
 osStaticThreadDef_t View_Gimbal_TaskControlBlock;
 const osThreadAttr_t View_Gimbal_Task_attributes = {
     .name = "View_Gimbal_Task",
     .cb_mem = &View_Gimbal_TaskControlBlock,
     .cb_size = sizeof(View_Gimbal_TaskControlBlock),
     .stack_mem = &View_Gimbal_TaskBuffer[0],
     .stack_size = sizeof(View_Gimbal_TaskBuffer),
     .priority = (osPriority_t) osPriorityNormal,
 };
 /* Definitions for vofa */
 osThreadId_t vofaHandle;
 uint32_t vofaBuffer[512];
 osStaticThreadDef_t vofaControlBlock;
 const osThreadAttr_t vofa_attributes = {
     .name = "vofa",
     .cb_mem = &vofaControlBlock,
     .cb_size = sizeof(vofaControlBlock),
     .stack_mem = &vofaBuffer[0],
     .stack_size = sizeof(vofaBuffer),
     .priority = (osPriority_t) osPriorityNormal,
 };
/* Definitions for debug_msg */
 osThreadId_t debug_msgHandle;
 uint32_t debug_msgBuffer[256];
 osStaticThreadDef_t debug_msgControlBlock;
 const osThreadAttr_t debug_msg_attributes = {
     .name = "debug_msg",
     .cb_mem = &debug_msgControlBlock,
     .cb_size = sizeof(debug_msgControlBlock),
     .stack_mem = &debug_msgBuffer[0],
     .stack_size = sizeof(debug_msgBuffer),
     .priority = (osPriority_t) osPriorityNormal,
 };
 /* Definitions for auto_get_task */
 /* 自动取矿轨迹 / 控制任务。 */
 osThreadId_t auto_get_taskHandle;
 uint32_t auto_get_taskBuffer[128];
 osStaticThreadDef_t auto_get_taskControlBlock;
 const osThreadAttr_t auto_get_task_attributes = {
     .name = "auto_get_task",
     .cb_mem = &auto_get_taskControlBlock,
     .cb_size = sizeof(auto_get_taskControlBlock),
     .stack_mem = &auto_get_taskBuffer[0],
     .stack_size = sizeof(auto_get_taskBuffer),
     .priority = (osPriority_t) osPriorityNormal,
 };
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
void uart_test(void *argument);
void motor_test(void *arguments);
void jointFollowAngle(void *arguments);
void uart_Transmit_Angle(void *arguments);
void Chassis_Task(void *arguments);
void Referee_Task(void *arguments);
void Watchdog_Task(void *arguments);
void SerialPlot(void *argument);
void IMU_Task(void *argument);
// void test_CAN3_ARM(void *argument);
void Joint1_Move_Task(void *argument);
void Joint2_Move_Task(void *argument);
void Joint3_Move_Task(void *argument);
void Joint4_Move_Task(void *argument);
void Joint5_Move_Task(void *argument);
void Joint6_Move_Task(void *argument);

void arm_reset_task(void *argument);
/* USER CODE END FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void auto_get_task(void *argument);
void debug_msg_task(void *argument);
void View_Gimbal_Task(void *argument);
void StartDefaultTask(void *argument);
void IMU_TempCtrlTask(void *argument);
void Remoter_Task(void *argument);
void Trajectory_Publisher_Task(void *argument) ;

void vofa_send(void *argument);
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
  /* 初始化自动轨迹子系统使用的定时器。 */
  traj_timer_init();
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of imuBinarySem01 */
  /* IMU 相关生产者 / 消费者同步用的事件型信号量。 */
  imuBinarySem01Handle = osSemaphoreNew(1, 0, &imuBinarySem01_attributes);

  /* creation of controlBinaryIMU */
  /* 协调 IMU 控制与数据处理的信号量。 */
  controlBinaryIMUHandle = osSemaphoreNew(1, 0, &controlBinaryIMU_attributes);

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
   * 当前启用的任务集合。
   * 被注释的 osThreadNew() 是可选测试或调试任务，不会在当前固件中创建线程。
   */
  /* creation of defaultTask */
  /* 空闲占位任务；必须主动让出 CPU，保证控制任务正常运行。 */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of IMU_TempCtrl */
  IMU_TempCtrlHandle = osThreadNew(IMU_TempCtrlTask, NULL, &IMU_TempCtrl_attributes);

  /* creation of Remoter */
  RemoterHandle = osThreadNew(Remoter_Task, NULL, &Remoter_attributes);

  
  // uartTestHandle = osThreadNew(uart_test, NULL, &uartTest_attributes);
  // motorTestHandle = osThreadNew(motor_test, NULL,&motorTest_attributes);
  /* 六轴机械臂关节跟随与末端执行器控制循环。 */
  jointFollowAngleHandle = osThreadNew(jointFollowAngle,NULL, &jointFollowAngle_attributes);
  // debug_msgHandle = osThreadNew(debug_msg_task, NULL, &debug_msg_attributes);
  /* 自动取矿行为任务。 */
  auto_get_taskHandle = osThreadNew(auto_get_task, NULL, &auto_get_task_attributes);
  // vofaHandle = osThreadNew(vofa_send, NULL, &vofa_attributes);
  // Joint1_Move_TaskHandle = osThreadNew(Joint1_Move_Task, NULL, &Joint1_Move_Task_attributes);
  // Joint2_Move_TaskHandle = osThreadNew(Joint2_Move_Task, NULL, &Joint2_Move_Task_attributes);
  // Joint3_Move_TaskHandle = osThreadNew(Joint3_Move_Task, NULL, &Joint3_Move_Task_attributes);
  // Joint4_Move_TaskHandle = osThreadNew(Joint4_Move_Task, NULL, &Joint4_Move_Task_attributes);
  // Joint5_Move_TaskHandle = osThreadNew(Joint5_Move_Task, NULL, &Joint5_Move_Task_attributes);
  // Joint6_Move_TaskHandle = osThreadNew(Joint6_Move_Task, NULL, &Joint6_Move_Task_attributes);
  //uart_Transmit_AngleHandle = osThreadNew(uart_Transmit_Angle, NULL, &uart_Transmit_Angle_attributes);
  /* 底盘驱动、模式切换与抬升状态机。 */
  Chassis_TaskHandle = osThreadNew(Chassis_Task, NULL, &Chassis_Task_attributes);
  // SerialPortHandle = osThreadNew(SerialPlot, NULL, &SerialPort_attributes);
  /* 自定义控制器 / 裁判系统输入处理。 */
  Referee_TaskHandle = osThreadNew(Referee_Task, NULL, &Referee_Task_attributes);
  /* 获取并发布外置 IMU 数据。 */
  IMU_TaskHandle = osThreadNew(IMU_Task, NULL, &IMU_Task_attributes);
  // Trajectory_PublisherHandle = osThreadNew(Trajectory_Publisher_Task, NULL, &Trajectory_Publisher_attributes);
  /* 视觉云台舵机控制循环。 */
  View_Gimbal_TaskHandle = osThreadNew(View_Gimbal_Task, NULL, &View_Gimbal_Task_attributes);

  /* 机械臂复位 / 回零流程。 */
#if !DEBUG_READ_DATA_ONLY
  Arm_ResetHandle = osThreadNew(arm_reset_task, NULL, &Arm_Reset_attributes);
#endif

  // SerialPlotHandle = osThreadNew(SerialPlot, NULL, &SerialPlot_attributes);
  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* 最后启动看门狗，使其监测前面已创建的任务和设备。 */
  Watchdog_TaskHandle = osThreadNew(Watchdog_Task, NULL, &Watchdog_Task_attributes);
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
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

__weak void uart_test(void *argument)
{
  UNUSED(argument);
  for (;;) {
    osDelay(1);
  }
}
__weak void motor_test(void *argument)
{
  UNUSED(argument);
  for (;;) {
    osDelay(1);
  }
}
__weak void jointFollowAngle(void *argument)
{
  UNUSED(argument);
  for (;;) {
    osDelay(1);
  }
}
__weak void uart_Transmit_Angle(void *argument)
{
  UNUSED(argument);
  for (;;) {
    osDelay(1);
  }
}

__weak void Chassis_Task(void *argument)
{
  /* USER CODE Chassis_Task */
  UNUSED(argument);
  /* Infinite loop */
  for(;;)
  {
  
    osDelay(2);
  }
  /* USER CODE END Chassis_Task */
}

__weak void Referee_Task(void *argument)
{
  UNUSED(argument);
  for(;;)
  {
    
    osDelay(1);
  }
}

__weak void SerialPort(void *argument)
{
  UNUSED(argument);
  for(;;)
  {
    
    osDelay(1);
  }
}

__weak void Trajectory_Publisher_Task (void *argument){
  UNUSED(argument);
  for(;;){
    osDelay(1);
  }
}

__weak void IMU_Task(void *argument)
{
  UNUSED(argument);
  for(;;){
    osDelay(1);
  }
}
/* USER CODE END Application */


__weak void SerialPlot(void *argument)
{
  UNUSED(argument);
  for(;;)
  {
    
    osDelay(1);
  }
}
