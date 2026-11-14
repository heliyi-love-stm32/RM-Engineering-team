#include "can_struct.h"
#include "can_api.h"
#include "can_bsp.h"
// #include "MotorTool.h"
#include "list.h"

#include "string.h"
/**
************************************************************************
* @brief:      	can_bsp_init(void)
* @param:       void
* @retval:     	void
* @details:    	CAN 使能
************************************************************************
**/
void can_bsp_init(void)
{
	can_filter_init();
	HAL_FDCAN_Start(&hfdcan1); // 开启FDCAN
	HAL_FDCAN_Start(&hfdcan2);
	HAL_FDCAN_Start(&hfdcan3);
	HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
	HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
	HAL_FDCAN_ActivateNotification(&hfdcan3, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
}
/**
************************************************************************
* @brief:      	can_filter_init(void)
* @param:       void
* @retval:     	void
* @details:    	CAN滤波器初始化
************************************************************************
**/
void can_filter_init(void)
{
	FDCAN_FilterTypeDef fdcan1_filter = {0};

	fdcan1_filter.IdType = FDCAN_STANDARD_ID;			  // 标准ID
	fdcan1_filter.FilterIndex = 0;						  // 滤波器索引
	fdcan1_filter.FilterType = FDCAN_FILTER_RANGE;		  // 允许接收两个ID之间
	fdcan1_filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0; // 过滤器0关联到FIFO0
	fdcan1_filter.FilterID1 = 0x000;
	fdcan1_filter.FilterID2 = 0x7FF; // 标准帧ID只有11位
	HAL_FDCAN_ConfigFilter(&hfdcan1, &fdcan1_filter);
	HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);
	HAL_FDCAN_ConfigFifoWatermark(&hfdcan1, FDCAN_CFG_RX_FIFO0, 1);

	FDCAN_FilterTypeDef fdcan2_filter = {0};

	fdcan2_filter.IdType = FDCAN_STANDARD_ID;			  // 标准ID
	fdcan2_filter.FilterIndex = 0; // 索引相对本实例；StdFiltersNbr=1时只有0合法
	fdcan2_filter.FilterType = FDCAN_FILTER_RANGE;		  // 允许接收两个ID之间
	fdcan2_filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0; // 过滤器0关联到FIFO0
	fdcan2_filter.FilterID1 = 0x000;
	fdcan2_filter.FilterID2 = 0x7FF;
	HAL_FDCAN_ConfigFilter(&hfdcan2, &fdcan2_filter);
	HAL_FDCAN_ConfigGlobalFilter(&hfdcan2, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);
	HAL_FDCAN_ConfigFifoWatermark(&hfdcan2, FDCAN_CFG_RX_FIFO0, 1);

	FDCAN_FilterTypeDef fdcan3_filter = {0};
	fdcan3_filter.IdType = FDCAN_STANDARD_ID;			  // 标准ID
	fdcan3_filter.FilterIndex = 0; // 不能把CAN1/CAN2的全局偏移当成本实例索引
	fdcan3_filter.FilterType = FDCAN_FILTER_RANGE;		  // 允许接收两个ID之间
	fdcan3_filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0; // 过滤器0关联到FIFO0
	fdcan3_filter.FilterID1 = 0x000;
	fdcan3_filter.FilterID2 = 0x7FF;
	HAL_FDCAN_ConfigFilter(&hfdcan3, &fdcan3_filter);
	HAL_FDCAN_ConfigGlobalFilter(&hfdcan3, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);
	HAL_FDCAN_ConfigFifoWatermark(&hfdcan3, FDCAN_CFG_RX_FIFO0, 1);
}
/**
************************************************************************
* @brief:      	fdcanx_send_data(FDCAN_HandleTypeDef *hfdcan, uint16_t id, uint8_t *data, uint32_t len)
* @param:       hfdcan：FDCAN句柄
* @param:       id：CAN设备ID
* @param:       data：发送的数据
* @param:       len：发送的数据长度 用Drivers\STM32H7xx_HAL_Driver\Inc\stm32h7xx_hal_fdcan.h第967行的宏定义！！！
* @retval:     	void
* @details:    	发送数据
************************************************************************
**/
uint8_t fdcanx_send_data(FDCAN_HandleTypeDef *hfdcan, uint16_t id, uint8_t *data, uint32_t len)
{
	if ((hfdcan == NULL) || (data == NULL) || (id > 0x7FFU) || (len > 8U)) {
		return 1U;
	}
	FDCAN_TxHeaderTypeDef TxHeader;

	TxHeader.Identifier = id;
	TxHeader.IdType = FDCAN_STANDARD_ID;			  // 标准ID
	TxHeader.TxFrameType = FDCAN_DATA_FRAME;		  // 数据帧
	TxHeader.DataLength = len;						  // 发送数据长度
	TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;  // 设置错误状态指示
	TxHeader.BitRateSwitch = FDCAN_BRS_OFF;			  // 不开启可变波特率
	TxHeader.FDFormat = FDCAN_CLASSIC_CAN;			  // 普通CAN格式
	TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS; // 用于发送事件FIFO控制, 不存储
	TxHeader.MessageMarker = 0x00;					  // 用于复制到TX EVENT FIFO的消息Maker来识别消息状态，范围0到0xFF

	if (HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &TxHeader, data) != HAL_OK)
		return 1; // 发送
	return 0;
}

/**
 * @brief :      	fdcanx_send_data_ext(FDCAN_HandleTypeDef *hfdcan, uint16_t id, uint8_t *data, uint32_t len)
 * 
 * @param hfdcan 	hfdcan：FDCAN句柄
 * @param id 		id：CAN设备ID
 * @param data 		data：发送的数据
 * @param len 		len：发送的数据长度 用Drivers\STM32H7xx_HAL_Driver\Inc\stm32h7xx_hal_fdcan.h第967行的宏定义！！
 * @return uint8_t 
 * @details:    	发送数据
 */
uint8_t fdcanx_send_data_ext(FDCAN_HandleTypeDef *hfdcan, uint16_t id, uint8_t *data, uint32_t len)
{
	FDCAN_TxHeaderTypeDef TxHeader;

	TxHeader.Identifier = id;
	TxHeader.IdType = FDCAN_EXTENDED_ID;			  // 扩展ID
	TxHeader.TxFrameType = FDCAN_DATA_FRAME;		  // 数据帧
	TxHeader.DataLength = len;						  // 发送数据长度
	TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;  // 设置错误状态指示
	TxHeader.BitRateSwitch = FDCAN_BRS_OFF;			  // 不开启可变波特率
	TxHeader.FDFormat = FDCAN_CLASSIC_CAN;			  // 普通CAN格式
	TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS; // 用于发送事件FIFO控制, 不存储
	TxHeader.MessageMarker = 0x00;					  // 用于复制到TX EVENT FIFO的消息Maker来识别消息状态，范围0到0xFF

	if (HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &TxHeader, data) != HAL_OK)
		return 1; // 发送
	return 0;
}

uint8_t fdcanx_send_data_fd(FDCAN_HandleTypeDef *hfdcan, uint16_t id, uint8_t *data, uint32_t len)
{
	FDCAN_TxHeaderTypeDef TxHeader;

	TxHeader.Identifier = id;
	TxHeader.IdType = FDCAN_STANDARD_ID;			  // 标准ID
	TxHeader.TxFrameType = FDCAN_DATA_FRAME;		  // 数据帧
	TxHeader.DataLength = len;						  // 发送数据长度
	TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;  // 设置错误状态指示
	TxHeader.BitRateSwitch = FDCAN_BRS_ON;			  // 不开启可变波特率
	TxHeader.FDFormat = FDCAN_FD_CAN;				  // 扩展CAN格式
	TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS; // 用于发送事件FIFO控制, 不存储
	TxHeader.MessageMarker = 0x00;					  // 用于复制到TX EVENT FIFO的消息Maker来识别消息状态，范围0到0xFF

	if (HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &TxHeader, data) != HAL_OK)
		return 1; // 发送
	return 0;
}

/**
************************************************************************
* @brief:      	fdcanx_receive(FDCAN_HandleTypeDef *hfdcan, uint8_t *buf)
* @param:       hfdcan：FDCAN句柄
* @param:       buf：接收数据缓存
* @retval:     	接收的数据长度
* @details:    	接收数据
************************************************************************
**/
FDCAN_RxHeaderTypeDef *fdcanx_receive(FDCAN_HandleTypeDef *hfdcan, uint8_t *buf)
{
	/* 保留旧API给外部模块使用；正式中断路径改用下方局部header。
	 * 先接到64字节临时区，再验证帧类型，防止非法DLC覆盖调用者8字节缓存。
	 */
	static FDCAN_RxHeaderTypeDef saved_header[3];
	FDCAN_RxHeaderTypeDef header;
	uint8_t data[64];
	uint8_t index;
	if ((hfdcan == NULL) || (buf == NULL)) return NULL;
	if (hfdcan == &hfdcan1) index = 0U;
	else if (hfdcan == &hfdcan2) index = 1U;
	else if (hfdcan == &hfdcan3) index = 2U;
	else return NULL;
	if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &header, data) != HAL_OK)
		return NULL;
	if ((header.IdType != FDCAN_STANDARD_ID) ||
	    (header.RxFrameType != FDCAN_DATA_FRAME) ||
	    (header.FDFormat != FDCAN_CLASSIC_CAN) || (header.DataLength > 8U))
		return NULL;
	memcpy(buf, data, header.DataLength);
	saved_header[index] = header;
	return &saved_header[index];
}
/**
************************************************************************
* @brief:      	HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
* @param:       hfdcan；FDCAN句柄
* @param:       RxFifo0ITs：中断标志位
* @retval:     	void
* @details:    	HAL库的FDCAN中断回调函数
************************************************************************
**/
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
	if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET)
	{
		if (hfdcan == &hfdcan1)
		{
			fdcan1_rx_callback();
		}
		if (hfdcan == &hfdcan2)
		{
			fdcan2_rx_callback();
		}
		if (hfdcan == &hfdcan3)
		{
			fdcan3_rx_callback();
		}
	}
}
/**
************************************************************************
* @brief:      	fdcan_rx_callback(void)
* @param:       void
* @retval:     	void
* @details:    	供用户调用的接收弱函数
************************************************************************
**/
/* 新模块可提供同名强符号，把板间报文逐帧写入自己的ISR队列。
 * 默认什么也不做，既有机械臂仍沿用can_msg链表读取反馈。
 * Hook运行在ISR中：只能快速复制，禁止阻塞、动态分配或在此做电机控制。
 */
__weak void HeavyInterboard_OnCanRx(uint8_t port, uint32_t id,
                                    const uint8_t *data, uint8_t len)
{
	(void)port;
	(void)id;
	(void)data;
	(void)len;
}

static void fdcan_drain_rx_fifo(FDCAN_HandleTypeDef *hfdcan, uint8_t port)
{
	FDCAN_RxHeaderTypeDef header;
	uint8_t data[64];
	/* 一次中断排空FIFO；只取一帧会在突发反馈时留下旧帧，增加控制延迟。
	 * header/data属于本次调用，各CAN同时来中断不会覆盖另一条总线的解析。
	 */
	while (HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO0) > 0U) {
		if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &header, data) != HAL_OK)
			break;
		if ((header.IdType != FDCAN_STANDARD_ID) ||
		    (header.RxFrameType != FDCAN_DATA_FRAME) ||
		    (header.FDFormat != FDCAN_CLASSIC_CAN) || (header.DataLength > 8U))
			continue;

		can_msg_t *msg = can_msg_find_item(can_msg_find_list((can_port_t)port), header.Identifier);
		if (msg != NULL) {
			/* 仍更新原对象，队友的DM刷新逻辑无需修改。cnt最后发布。 */
			memset(msg->data, 0, sizeof(msg->data));
			memcpy(msg->data, data, header.DataLength);
			msg->len = (uint8_t)header.DataLength;
			__DMB();
			msg->cnt++;
		}
		/* 未注册ID也交给Hook，使板间队列无需重复占用电机反馈节点。 */
		HeavyInterboard_OnCanRx(port, header.Identifier, data, (uint8_t)header.DataLength);
	}
}

void fdcan1_rx_callback(void)
{
	fdcan_drain_rx_fifo(&hfdcan1, CAN1_PORT);
}

void fdcan2_rx_callback(void)
{
	fdcan_drain_rx_fifo(&hfdcan2, CAN2_PORT);
}

void fdcan3_rx_callback(void)
{
	fdcan_drain_rx_fifo(&hfdcan3, CAN3_PORT);
}
