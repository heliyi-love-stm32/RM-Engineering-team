extern "C"{
#include "cmsis_os2.h"
#include "uart_api.h"
#include "usart.h"
}
#include <string.h>
#include <math.h>

#define CH_COUNT 6
#define RX_COUNT 2

#pragma pack(1)
struct VofaFrame {
    float ch[CH_COUNT];
    uint8_t tail[4] = {0x00, 0x00, 0x80, 0x7f};
};
#pragma pack()

class Vofa_UART {
  public:
    using RxCallback = void(*)(uint8_t* pData, uint32_t size);
    void init(UART_HandleTypeDef* huart, RxCallback callback = nullptr);
    void setFrameData(uint8_t idx, float val);
    void sendFrame(void);
    
  private:
    VofaFrame  frame;
    uart_msg_t tx_msg;
    uart_msg_t rx_msg;
    uart_rx_t  rx;
    uint8_t rx_buf[32];
};
void Vofa_UART::init(UART_HandleTypeDef *huart, RxCallback callback) {
  memset(&this->frame.ch, 0, sizeof(this->frame.ch));
  frame.tail[0] = 0x00;
  frame.tail[1] = 0x00;
  frame.tail[2] = 0x80;
  frame.tail[3] = 0x7f;
  this->tx_msg.huart = huart;
  this->tx_msg.pBuffer = (uint8_t *)&this->frame;
  this->tx_msg.Len = sizeof(VofaFrame);
  this->rx_msg.huart = huart;
  this->rx_msg.pBuffer = rx_buf;
  this->rx_msg.Len = sizeof(rx_buf);
  this->rx.rx_msg = &this->rx_msg;
  uart_rx_hook_reg(&this->rx, callback);
  uart_rx_init(&this->rx);
}
void Vofa_UART::setFrameData(uint8_t ch_idx, float val) {
  if (ch_idx < CH_COUNT) {
    frame.ch[ch_idx] = val;
  }
}
void Vofa_UART::sendFrame(void)
{
  uart_tx_send(&this->tx_msg, 1000);
}
