#include "arm_referee.h"
#include "arm_handle.h"
#include "arm_state_machine.h"
#include "servo_drv.h"
#include "auto_keyboard.h"
#include "arm_debug.h"
#include <stdint.h>

extern rc_info_t remoter;
static const auto_key_cmd_t pos_cmds[4] = {
    CMD_AUTO_PUT_D,
    CMD_AUTO_GET_A_POS,
    CMD_AUTO_GET_B_POS,
    CMD_AUTO_GET_C_POS,
};

static uint8_t get_idx = 0;
static uint8_t emerency_idx = 0;

static const auto_key_get_cmd_t get_cmds[4] = {
    CMD_AUTO_GET_LEFT_FORNT,
    CMD_AUTO_GET_RIGHT_FRONT,
    CMD_AUTO_GET_RIGHT_MID,
    CMD_AUTO_GET_RIGHT_BACK,
};

/* 紧急存矿模式标志 */
static uint8_t emerency_stash_active = 0;
uint8_t Emerency_flag = 0;
static const auto_key_cmd_t emerency_cmds[4] = {
    CMD_EMERENCY_STASH_R_B,   /* 0: 右后 */
    CMD_EMERENCY_STASH_R_M,   /* 1: 右中 */
    CMD_EMERENCY_STASH_R_F,   /* 2: 右前 */
    CMD_EMERENCY_STASH_L_F,   /* 3: 左前 */
};

void Arm_Keyboard_E_Exec(void) {
  Arm_Current_Control_Mode = Arm_Auto_Mode;
  auto_key_get_cmd_exec(auto_key_get_cmd);

}
uint8_t emerency_pos_index = 0;
void Arm_Keyboard_Manager(uint8_t key) {
  static uint8_t pos_idx = 0;

  if (key == (uint8_t)'F') {
    Arm_Current_Control_Mode = Arm_Custom_Controller_Follow_Mode;
  }
  if (key == (uint8_t)'Q') {
    if (emerency_stash_active != 0) {
      /* 紧急存矿模式 */
      Arm_Current_Control_Mode = Arm_Auto_Mode;
      auto_key_cmd_exec(emerency_cmds[emerency_idx]);
      emerency_idx = (emerency_idx + 1) % 4;
      emerency_pos_index = emerency_idx;
    } else {
      /* 正常 Q：A/B/C 取矿 */
      Arm_Current_Control_Mode = Arm_Auto_Mode;
      auto_key_cmd_exec(pos_cmds[pos_idx]);
      auto_key_cmd = pos_cmds[pos_idx];
      pos_idx = (pos_idx + 1) % 4;
    }
  }
   if (key == (uint8_t)'E') {

       auto_key_get_cmd = get_cmds[get_idx];
       get_idx = (get_idx + 1) % 4;
       Arm_Keyboard_E_Exec();
   }   
   // if (key == (uint8_t)'V') {
   //  Arm_Current_Control_Mode = Arm_Rising_Mode;
   // }

   if (key == (uint8_t)'Z')
   {

     emerency_stash_active = !emerency_stash_active;
     
     Emerency_flag = emerency_stash_active;

   }
}

RESET_SAVEZERO_STATUS KEYBOARD_RESET_SAVE_ZERO_HANDLE(uint8_t key) {
  if (key == (uint8_t)'V') {
      Arm_Current_Control_Mode = ARM_RESET_ZERO_MODE;
    return RESET_SAVE_ZERO_OK;
  }
  return RESET_SAVE_ZERO_NONE;
}
RESET_SAVEZERO_STATUS reset_zero_status = RESET_SAVE_ZERO_NONE;
void Arm_Keyboard_ctrl_Manager(uint8_t key) {
   if (key == (uint8_t)'Z')
   {
       if (emerency_stash_active!=0) {
            emerency_idx++;
            emerency_idx = (emerency_idx) % 4;
       }
      emerency_pos_index = emerency_idx;

   }

  if (key == 'Q') {
    Arm_Current_Control_Mode = Arm_Auto_Mode;

    if (auto_key_cmd == CMD_AUTO_GET_A_POS) {
      auto_key_cmd_exec(CMD_AUTO_GET_A_SET);
    }

    if (auto_key_cmd == CMD_AUTO_GET_B_POS) {
      auto_key_cmd_exec(CMD_AUTO_GET_B_SET);
    }

    if (auto_key_cmd == CMD_AUTO_GET_C_POS) {
      auto_key_cmd_exec(CMD_AUTO_GET_C_SET);
    }
  }

  if (key == 'E') {
    auto_key_get_cmd = get_cmds[get_idx];
    get_idx = (get_idx + 1) % 4;
  }
  reset_zero_status = KEYBOARD_RESET_SAVE_ZERO_HANDLE(key);
}
EXIT_RESET_STATUS KEYBOARD_EXIT_RESET_STATUS_HANDLE(uint8_t key)
{
    if (key == (uint8_t)'V') {
        if (Arm_Current_Control_Mode == ARM_RESET_ZERO_MODE) {
            Arm_Current_Control_Mode = Arm_IDLE_Mode;
            return RESET_EXIT;
        }
        else {
            return RESET_LOGIC_ERROR;
        }
    }

        return RESET_NONE; 
}
EXIT_RESET_STATUS exit_reset_status = RESET_NONE;
void Arm_Keyboard_shift_Manager(uint8_t key) {
  if (key == (uint8_t)'F') {
    Arm_Current_Control_Mode = Arm_Frozen_Mode;
  }
  if (key == (uint8_t)'Z') {
    if (Arm_Current_Control_Mode != Arm_Auto_Mode) {
      Arm_Current_Control_Mode = ARM_SAFE_MODE;
    }
  }
  exit_reset_status = KEYBOARD_EXIT_RESET_STATUS_HANDLE(key);
}
