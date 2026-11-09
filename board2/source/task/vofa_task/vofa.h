#ifndef VOFA_UART_H
#define VOFA_UART_H

#ifdef __cplusplus
extern "C" {
#endif
#include "cmsis_os2.h"
#include "uart_api.h"
#include "usart.h"
#ifdef __cplusplus
}
#endif

#include <stdint.h>

#define CH_COUNT 1

#pragma pack(1)
struct VofaFrame {
    float ch[CH_COUNT];
    uint8_t tail[4];
};
#pragma pack()

class Vofa_UART {
public:
    using RxCallback = void(*)(uint8_t* pData, uint32_t size);

    void init(UART_HandleTypeDef* huart, RxCallback callback = nullptr);
    void setFrameData(uint8_t idx, float val);
    void sendFrame(void);
    void parseCommand(uint8_t* pData, uint32_t size);

private:
    VofaFrame  frame;
    uart_msg_t tx_msg;
    uart_msg_t rx_msg;
    uart_rx_t  rx;
    uint8_t rx_buf[32];
};

#endif
