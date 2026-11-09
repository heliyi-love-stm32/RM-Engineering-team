#include "cmsis_os2.h"
#include "referee_api.h"
#include "usart.h"

/**
 * @brief 自定义控制器数据接收任务。
 *
 * 初始化 UART7 (921600) 的控制器接收链路，并周期解析环形缓冲区中的数据帧，
 * 解析结果供机械臂跟随模式读取。
 */
void Ctrller_Task(void *argument)
{
    UNUSED(argument);

    ctrller_init(&huart7);

    for (;;) {
        (void)get_custom_controller_msg();
        osDelay(2);
    }
}
