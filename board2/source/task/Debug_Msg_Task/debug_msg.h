#ifndef DEBUG_MSG_H

#include <cstdint>
extern "C"{
  #include "uart_api.h"
}
class UART_CLASS {
public:
  using RxCallback = void (*)(uint8_t *pData, uint32_t size);
  void tx_init(UART_HandleTypeDef *huart, uint8_t *tx_msg, uint16_t Len);
  void rx_init(UART_HandleTypeDef *huart, uint8_t *rx_buf, uint16_t Len,
               RxCallback callback = nullptr);
  void set_tx_data(uint8_t* data, uint16_t Len);
  void uart_transmit(uint32_t timeout);
private:
  uart_msg_t tx_msg;
  uart_msg_t rx_msg;
  uart_rx_t rx;
};
#endif // !DEBUG_MSG_H

