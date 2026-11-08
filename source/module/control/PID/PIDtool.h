/**
 * @file PIDtool.h
 * @brief 通用 PID 控制器数据结构和计算接口。
 */
#ifndef PIDTOOL
#define PIDTOOL

/** @brief PID 控制器运行时状态。 */
typedef struct
{
    float Kp; /**< 比例系数。 */
    float Ki; /**< 积分系数。 */
    float Kd; /**< 微分系数。 */

    float max_out;  /**< 输出限幅。 */
    float max_iout; /**< 积分输出限幅。 */

    float set; /**< 当前目标值。 */
    float fdb; /**< 当前反馈值。 */

    float out; /**< PID 总输出。 */
    float Pout; /**< 比例项输出。 */
    float Iout; /**< 积分项输出。 */
    float Dout; /**< 微分项输出。 */
    float Dbuf[3];  /**< 微分项缓存：0 最新，1 上一次，2 上上次。 */
    float error[3]; /**< 误差缓存：0 最新，1 上一次，2 上上次。 */

} pid_type_def;

/**
 * @brief 初始化 PID 控制器。
 * @param pid PID 实例。
 * @param kp 比例系数。
 * @param ki 积分系数。
 * @param kd 微分系数。
 * @param max_out 输出限幅。
 * @param max_iout 积分输出限幅。
 */
void PID_Init(pid_type_def *pid, const float kp, const float ki, const float kd, float max_out, float max_iout);
/** @brief 计算增量式 PID 输出。 */
float PID_Calc_Add(pid_type_def *pid, float now, float target);
/** @brief 计算位置式 PID 输出。 */
float PID_Calc_Pos(pid_type_def *pid, float now, float target);



#endif // !PIDTOOL
