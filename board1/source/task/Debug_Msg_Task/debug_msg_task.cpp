#include "cmsis_os2.h"
#include "debug_msg.h"
#include "servo_drv.h"
#include "usart.h"
#include <cstdint>
#include <string.h>

UART_CLASS debug_msg;

extern servo_t view_gimbal_yaw, view_gimbal_pitch;

struct view_gimbal_msg_t {
  const char *title;
  uint16_t angle[2];
};
static inline bool if_data_update(float last_pos, float pos) {
  return (last_pos != pos);
}
static int float_to_str(float num, char *buf, int precision) {
  int len = 0;

  // 处理负数
  if (num < 0) {
    buf[len++] = '-';
    num = -num;
  }

  int int_part = (int)num;
  float frac_part = num - int_part;

  char int_buf[12];
  int i = 0;

  if (int_part == 0) {
    int_buf[i++] = '0';
  } else {
    while (int_part > 0) {
      int_buf[i++] = '0' + (int_part % 10);
      int_part /= 10;
    }
  }

  for (int j = i - 1; j >= 0; j--) {
    buf[len++] = int_buf[j];
  }

  if (precision > 0) {
    buf[len++] = '.';

    while (precision--) {
      frac_part *= 10;
      int digit = (int)frac_part;
      buf[len++] = '0' + digit;
      frac_part -= digit;
    }
  }

  return len;
}
static inline void
transmit_debug_servo_data(char *msg, view_gimbal_msg_t view_gimbal_msg) {

  int len = 0;

  // 头
  const char *head = "舵机gimbal目标角度为: ";
  memcpy(msg + len, head, strlen(head));
  len += strlen(head);

  len += float_to_str(view_gimbal_msg.angle[0], msg + len, 2);

  msg[len++] = ' ';

  len += float_to_str(view_gimbal_msg.angle[1], msg + len, 2);

  msg[len++] = '\r';
  msg[len++] = '\n';

  debug_msg.set_tx_data((uint8_t *)msg, len);
  debug_msg.uart_transmit(1000);
}
enum servo_error_outRange_t {
  OK,
  BIGGER,
  LOWER,
};
static inline servo_error_outRange_t servo_out_of_range(uint16_t pos) {
  if (pos > 180) {
    return BIGGER;
  }
  return OK;
}
extern "C" void debug_servo_set_angle_hook(uint8_t *pData, uint32_t size) {
  uint16_t yaw = 0;
  uint16_t pitch = 0;

  uint32_t i = 0;

  while (i < size && pData[i] != ' ') {
    if (pData[i] >= '0' && pData[i] <= '9') {
      yaw = yaw * 10 + (pData[i] - '0');
    }
    i++;
  }

  i++;

  while (i < size) {
    if (pData[i] >= '0' && pData[i] <= '9') {
      pitch = pitch * 10 + (pData[i] - '0');
    }
    i++;
  }
  if (servo_out_of_range(yaw) == OK && servo_out_of_range(pitch) == OK) {
    view_gimbal_yaw.position = yaw;
    view_gimbal_pitch.position = pitch;
  }
}
// extern "C" void debug_msg_task(void *argument) {
//   view_gimbal_msg_t view_gimbal_msg;

//   uint8_t debug_msg_tx_init_buf[64] = {0};
//   uint8_t debug_msg_rx_init_buf[64] = {0};

//   debug_msg.tx_init(&huart10, debug_msg_tx_init_buf, 64);
//   debug_msg.rx_init(&huart10, debug_msg_rx_init_buf, 64,
//                     debug_servo_set_angle_hook);
//   const char *servo_debug_msg_title = "舵机gimbal目标角度为:";
//   view_gimbal_msg.title = servo_debug_msg_title;
//   view_gimbal_msg.angle[0] = view_gimbal_yaw.position;
//   view_gimbal_msg.angle[1] = view_gimbal_pitch.position;
//   debug_msg.set_tx_data((uint8_t *)servo_debug_msg_title,
//                         strlen(servo_debug_msg_title));
//   char msg[64];
//   uint16_t yaw_last_pos = 0;
//   uint16_t pitch_last_pos = 0;
//   // debug_msg.set_tx_data((uint8_t* )"hello from huart10",sizeof("hello from
//   // huart10") - 1);
//   while (true) {
//     view_gimbal_msg.angle[0] = view_gimbal_yaw.position;
//     view_gimbal_msg.angle[1] = view_gimbal_pitch.position;
//     if (if_data_update(pitch_last_pos, view_gimbal_msg.angle[1]) ||
//         if_data_update(yaw_last_pos, view_gimbal_msg.angle[0])) {
//       transmit_debug_servo_data(msg, view_gimbal_msg);
//       yaw_last_pos = view_gimbal_yaw.position;
//       pitch_last_pos = view_gimbal_pitch.position;
//     }
//     osDelay(100);
//   }
// }
