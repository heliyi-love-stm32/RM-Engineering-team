#include "joint_control_drv.h"
#include "vofa.h"

Vofa_UART vofa;

extern "C"  void vofa_rx_hook(uint8_t* pData, uint32_t size) {

}

extern Joint_t Joint[JOINT_NUM];

float tor_data[JOINT_NUM] = {};
static inline float getJointTor(Joint_t joint)
{
  return joint.joint_motor->motor_msg.torque_current;
}
static inline void Joint_Tor(void)
{
  for (int i = 0; i<JOINT_NUM; i++) {
    tor_data[i] = getJointTor(Joint[i]);
  }
}
extern "C" void vofa_send(void *argument) {
  UNUSED(argument);
  vofa.init(&huart10,vofa_rx_hook);
  for (int i =0 ; i<CH_COUNT; i++) {
    vofa.setFrameData(i, tor_data[i]);
  }
  while (1) {
    Joint_Tor();
    vofa.sendFrame();
    osDelay(100);
  }
}
