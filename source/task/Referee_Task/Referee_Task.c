#include "Referee_Task.h"

#include "cmsis_os2.h"
#include "referee_api.h"
#include "referee_ui.h"
#include "referee_keyboard.h"

/**
 * @brief 工程机器人模式全局变量
 * @note 初始化为DBUS模式，Normal底盘控制，CustomCtrl机械臂控制
 *       可通过按键B切换Ctrl_Logic_Mode（DBUS ↔ Keyboard）
 */
Engineer_Mode_t Engineer_Mode = {
    .Chassis_Ctrl_Mode = CHASSIS_CTRL_MODE_Normal,
    .Arm_Ctrl_Mode = ARM_CTRL_MODE_CustomCtrl,
    .Ctrl_Logic_Mode = CTRL_LOGIC_MODE_DBUS
};

/**
 * @brief 裁判系统任务
 * @note 执行键盘边沿检测，键盘数据源由 USE_REMOTER_KEYBOARD 宏控制
 */
/**
 * @brief 裁判系统与自定义控制器服务任务。
 *
 * 初始化两路 UART 接收，解析裁判数据和控制器数据，执行键盘边沿分发，
 * 并周期维护客户端图形 UI。
 */
void Referee_Task(void *argument)
{
    UNUSED(argument);

    /* 根据当前工程代码分工：
     * 1. UART7 配置为 921600，对应图传/自定义控制器链路；
     * 2. USART10 已被 SerialPlot_Task 占用做调试串口；
     * 3. USART1 是当前工程中空闲且配置为 115200 的常规链路串口。
     * 因此裁判系统常规链路和 UI 发送统一挂到 USART1。
     */
    referee_init(&huart1);
    ctrller_init(&huart7);
    Referee_UI_Init();

    for (;;) {
        (void)get_custom_controller_msg();
        (void)get_referee_msg();

        // 获取键盘数据并进行边沿检测
        Referee_KeyboardEdgeDetect(Referee_GetActiveKeyboard());
        Referee_UI_Service();

        osDelay(2);
    }
}
