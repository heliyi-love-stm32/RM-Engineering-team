#include "auto.h"
#include "auto_traj_data.h"
#include "auto_keyboard.h"
#include "arm_debug.h"

huangjiazhi::TrajectoryExecutor traj_exec;
static volatile auto_key_cmd_t cmd_req = CMD_NONE;
static auto_key_cmd_t last_cmd = CMD_NONE;

/**
 * @brief 切换机械臂轨迹组并从首点开始定时回放。
 * @param group 新轨迹的采样点数组。
 * @param size 采样点数量。
 */
void Auto_Switch_Group(huangjiazhi::traj_group_point_t *group, uint32_t size)
{
    traj_exec.init(group, size, 5);
    traj_exec.build_time_acc();
    traj_exec.reset();
    auto_traj_idx = 0;
    osTimerStart(auto_traj_timer_id, 5);
}

/** @brief 提交由按键触发的自动取放命令，供轨迹任务异步执行。 */
extern "C" void auto_key_cmd_exec(auto_key_cmd_t cmd){
    cmd_req = cmd;
}

extern "C" void auto_key_get_cmd_exec(auto_key_get_cmd_t cmd){
    switch (cmd) {
        case CMD_AUTO_GET_RIGHT_BACK:
            Auto_Switch_Group(traj_back_get, traj_back_get_size);
            break;
        case CMD_AUTO_GET_RIGHT_MID:
            Auto_Switch_Group(statsh_get, statsh_get_size);
            break;
        case CMD_AUTO_GET_RIGHT_FRONT:
            Auto_Switch_Group(statsh_put_L_A, statsh_put_L_A_size);
            break;
        case CMD_AUTO_GET_LEFT_FORNT:
            Auto_Switch_Group(statsh_get_front_L,statsh_get_front_L_size);
            break;
        default:
            break;
    }
}

/**
 * @brief 自动取放任务。
 *
 * 启动默认轨迹定时器，并消费按键命令以切换预置取放、放置或紧急收纳轨迹。
 */
extern "C" void auto_get_task(void *argument) {
  UNUSED(argument);
#if TRAJ_DEBUG
  traj_exec.init(debug_traj, debug_traj_size, 5);
#elif ARM_CHECK_IN
  traj_exec.init(checkin_traj, checkin_traj_size, 5);
#else
  traj_exec.init(traj_group_auto_A_step1, traj_group_auto_A_step1_size, 5);
#endif
  traj_exec.build_time_acc();
  traj_exec.reset();
  auto_traj_idx = 0;
  osTimerStart(auto_traj_timer_id, 5);
  while (true) {
    if (cmd_req != CMD_NONE && cmd_req != last_cmd) {

        switch (cmd_req) {

            case CMD_AUTO_GET_A_POS:
                Auto_Switch_Group(traj_group_auto_A_step1,
                                  traj_group_auto_A_step1_size);
                break;

            case CMD_AUTO_GET_A_SET:
                Auto_Switch_Group(traj_group_auto_A_step2,
                                  traj_group_auto_A_step2_size);
                break;

            case CMD_AUTO_GET_B_POS:
                Auto_Switch_Group(traj_group_auto_B_step1,
                                  traj_group_auto_B_step1_size);
                break;

            case CMD_AUTO_GET_B_SET:
                Auto_Switch_Group(traj_group_auto_B_step2,
                                  traj_group_auto_B_step2_size);
                break;

            case CMD_AUTO_GET_C_POS:
                Auto_Switch_Group(traj_group_auto_C_step1,
                                  traj_group_auto_C_step1_size);
                break;

            case CMD_AUTO_GET_C_SET:
                Auto_Switch_Group(traj_group_auto_C_step2,
                                  traj_group_auto_C_step2_size);
                break;
            case CMD_AUTO_PUT_D:
                Auto_Switch_Group(traj_back_put, traj_back_put_size);
                break;
            case CMD_EMERENCY_STASH_R_B:
                Auto_Switch_Group(traj_back_put, traj_back_put_size);
                break;
            case CMD_EMERENCY_STASH_R_M:
                Auto_Switch_Group(emerency_auto_stash_R_M, emerency_auto_stash_R_M_size);
                break;
            case CMD_EMERENCY_STASH_R_F:
                Auto_Switch_Group(emerency_auto_stash_R_F, emerency_auto_stash_R_F_size);
                break;
            case CMD_EMERENCY_STASH_L_F:
                Auto_Switch_Group(emerency_auto_stash_L_F, emerency_auto_stash_L_F_size);
                break;
            case CMD_AUTO_CHECKIN:
                Auto_Switch_Group(checkin_traj, checkin_traj_size);
                break;

            default:
                break;
        }

        last_cmd = cmd_req;
        cmd_req = CMD_NONE; 
    }
    traj_exec.update(auto_traj_idx, Target_Point);

    if (traj_exec.is_finished()) {
        osTimerStop(auto_traj_timer_id);
        auto_traj_idx = 0;
        last_cmd = CMD_NONE;
    }

    osDelay(5);
  }
}
