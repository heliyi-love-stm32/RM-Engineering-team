#include "debug_msg.h"
#include <cstdint>

void UART_CLASS::tx_init(UART_HandleTypeDef *huart, uint8_t *tx_msg, uint16_t Len) {
  this->tx_msg.huart = huart;
  this->tx_msg.Len = Len;
  this->tx_msg.pBuffer = tx_msg;
}
void UART_CLASS::rx_init(UART_HandleTypeDef *huart, uint8_t *rx_buf, uint16_t Len,
               RxCallback callback) {
  this->rx_msg.huart = huart;
  this->rx_msg.pBuffer = rx_buf;
  this->rx_msg.Len = Len;
  this->rx.rx_msg = &this->rx_msg;
  uart_rx_hook_reg(&this->rx, callback);
  uart_rx_init(&this->rx);
}
void UART_CLASS::set_tx_data(uint8_t* data, uint16_t Len)
{
  this->tx_msg.pBuffer = data;
  this->tx_msg.Len = Len;
}

void UART_CLASS::uart_transmit(uint32_t timeout){
  uart_tx_send(&this->tx_msg,timeout);
}

