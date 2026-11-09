#ifndef AUTO_KEY_BOARD_H
#define AUTO_KEY_BOARD_H

#include <stdint.h>
typedef enum auto_key_cmd_t{
    CMD_NONE,
    CMD_AUTO_GET_A_POS,
    CMD_AUTO_GET_A_SET,
    CMD_AUTO_GET_B_POS,
    CMD_AUTO_GET_B_SET,
    CMD_AUTO_GET_C_POS,
    CMD_AUTO_GET_C_SET,
    CMD_AUTO_PUT_D,
    CMD_EMERENCY_STASH_R_B,  /* 右后紧急存矿 */
    CMD_EMERENCY_STASH_R_M,  /* 右中紧急存矿 */
    CMD_EMERENCY_STASH_R_F,  /* 右前紧急存矿 */
    CMD_EMERENCY_STASH_L_F,  /* 左前紧急存矿 */
    CMD_AUTO_CHECKIN,          /* 检录：遥控器触发一次执行一次 */
}auto_key_cmd_t;

typedef enum auto_key_get_cmd_t {
    CMD_GET_NONE,
    CMD_AUTO_GET_RIGHT_BACK,
    CMD_AUTO_GET_RIGHT_MID,
    CMD_AUTO_GET_RIGHT_FRONT,
    CMD_AUTO_GET_LEFT_FORNT,
} auto_key_get_cmd_t;

#ifdef __cplusplus
extern "C" {
#endif

extern auto_key_cmd_t auto_key_cmd;
extern auto_key_get_cmd_t auto_key_get_cmd;

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus
extern "C"
#endif
void auto_key_cmd_exec(auto_key_cmd_t cmd);

#ifdef __cplusplus
extern "C"
#endif
void auto_key_get_cmd_exec(auto_key_get_cmd_t cmd);

#endif
