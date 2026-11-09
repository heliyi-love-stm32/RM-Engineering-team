#ifndef AUTO_TRAJ_DATA_H

#define AUTO_TRAJ_DATA_H

#include "auto.h"



extern huangjiazhi::traj_group_point_t traj_group_A[];
extern huangjiazhi::traj_group_point_t traj_group_B[];
extern huangjiazhi::traj_group_point_t traj_group_C[];
extern huangjiazhi::traj_group_point_t traj_group_D[];
extern huangjiazhi::traj_group_point_t statsh_put[];
extern huangjiazhi::traj_group_point_t statsh_get[]; //右中
extern huangjiazhi::traj_group_point_t statsh_put_L[]; 
extern huangjiazhi::traj_group_point_t statsh_put_R[]; //左前
extern huangjiazhi::traj_group_point_t statsh_put_L_A[]; //右前
extern huangjiazhi::traj_group_point_t traj_back_get[]; //右后
extern huangjiazhi::traj_group_point_t traj_back_put[];
extern huangjiazhi::traj_group_point_t traj_get_put[];

extern huangjiazhi::traj_group_point_t debug_traj[];

// 紧急存矿轨迹
// 已夹紧矿→移动到存矿位→张开放下→下放J5→回位
// 对应 E 键四个取矿位置的逆向存矿操作
extern huangjiazhi::traj_group_point_t emerency_auto_stash_R_B[]; // 右后
extern huangjiazhi::traj_group_point_t emerency_auto_stash_R_M[]; // 右中
extern huangjiazhi::traj_group_point_t emerency_auto_stash_R_F[]; // 右前
extern huangjiazhi::traj_group_point_t emerency_auto_stash_L_F[]; // 左前
extern huangjiazhi::traj_group_point_t emerency_auto_stash_L_M[]; // 左中
extern huangjiazhi::traj_group_point_t emerency_auto_stash_L_B[]; // 左后

extern huangjiazhi::traj_group_point_t statsh_get_front_L[];
extern huangjiazhi::traj_group_point_t traj_group_auto_A_step1[];
extern huangjiazhi::traj_group_point_t traj_group_auto_A_step2[];
extern huangjiazhi::traj_group_point_t traj_group_auto_B_step1[];
extern huangjiazhi::traj_group_point_t traj_group_auto_B_step2[];
extern huangjiazhi::traj_group_point_t traj_group_auto_C_step1[];
extern huangjiazhi::traj_group_point_t traj_group_auto_C_step2[];
extern huangjiazhi::traj_group_point_t traj_back_put[];
extern huangjiazhi::traj_group_point_t checkin_traj[];

extern const uint32_t traj_get_put_size;
extern const uint32_t traj_group_auto_A_step1_size;
extern const uint32_t traj_group_auto_A_step2_size;
extern const uint32_t traj_group_auto_B_step1_size;
extern const uint32_t traj_group_auto_B_step2_size;
extern const uint32_t traj_group_auto_C_step1_size;
extern const uint32_t traj_group_auto_C_step2_size;

extern const uint32_t statsh_get_size;
extern const uint32_t statsh_put_L_size;
extern const uint32_t statsh_put_R_size;
extern const uint32_t statsh_put_L_A_size;
extern const uint32_t traj_back_get_size;
extern const uint32_t emerency_auto_stash_R_B_size;
extern const uint32_t emerency_auto_stash_R_M_size;
extern const uint32_t emerency_auto_stash_R_F_size;
extern const uint32_t emerency_auto_stash_L_F_size;
extern const uint32_t emerency_auto_stash_L_M_size;
extern const uint32_t emerency_auto_stash_L_B_size;

extern const uint32_t statsh_get_front_L_size;

extern const uint32_t debug_traj_size;
extern const uint32_t traj_back_put_size;
extern const uint32_t checkin_traj_size;
#endif
