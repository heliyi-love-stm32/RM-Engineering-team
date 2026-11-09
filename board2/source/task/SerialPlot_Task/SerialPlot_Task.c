#include "cmsis_os2.h"
#include "joint_control_drv.h"
#include "uart_api.h"
#include "usart.h"
#include <stdint.h>
#include <string.h>

uart_msg_t SerialPlot_tx_msg;
extern float joint_radian[6];
extern float Current_Radian[6];
#define SerialPlotLen 25
void SerialPlot_tx_msg_init(void) {
  static uint8_t temp_zero_buffer[SerialPlotLen] = {0};
  SerialPlot_tx_msg.huart = &huart10;
  SerialPlot_tx_msg.pBuffer = temp_zero_buffer;
  SerialPlot_tx_msg.Len = SerialPlotLen;
}

static inline void Data_Cpy(uint8_t* frame, float *Data)
{
  memcpy(&frame[1], Data, JOINT_NUM * sizeof(float));
}
static inline void radian_cpy(uint8_t *frame) {
  memcpy(&frame[1], &Current_Radian[0], sizeof(float));
  memcpy(&frame[5], &Current_Radian[1], sizeof(float));
  memcpy(&frame[9], &Current_Radian[2], sizeof(float));
  memcpy(&frame[13], &Current_Radian[3], sizeof(float));
  memcpy(&frame[17], &Current_Radian[4], sizeof(float));
  memcpy(&frame[21], &Current_Radian[5], sizeof(float));
}
static inline float Motor_Get_Tor(const DM_motor_t *motor) {
  return motor->motor_msg.torque_current;
}

void Joint_Get_Tor(Joint_t *Joint,float *tor){
  for (int joint_index=0; joint_index<JOINT_NUM; joint_index++) {
    tor[joint_index]=Motor_Get_Tor(Joint[joint_index].joint_motor);
  }
}
void SerialPlot(void *argument) {
  UNUSED(argument);
  osDelay(10);
  SerialPlot_tx_msg_init();

  uint8_t frame[SerialPlotLen];
  memset(frame, 0, sizeof(frame));
  frame[0] = 0xAB;
  while (1) {
    osDelay(10);
    Data_Cpy(frame, Current_Radian);
    SerialPlot_tx_msg.pBuffer = frame;
    SerialPlot_tx_msg.Len = SerialPlotLen;
    uart_tx_send(&SerialPlot_tx_msg, HAL_MAX_DELAY);
  }
}
