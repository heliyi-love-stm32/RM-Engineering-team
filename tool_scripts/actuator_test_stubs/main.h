#ifndef HEAVY_HOST_MAIN_H
#define HEAVY_HOST_MAIN_H
/* 桌面测试仅替换硬件类型/中断原语，不参与执行器控制决策。 */
#include <stdint.h>
#include <stddef.h>
typedef struct { uint32_t instance; } FDCAN_HandleTypeDef;
#define FDCAN_DLC_BYTES_4 4U
#define FDCAN_DLC_BYTES_8 8U
static inline uint32_t __get_PRIMASK(void) { return 0U; }
static inline void __disable_irq(void) {}
static inline void __set_PRIMASK(uint32_t value) { (void)value; }
static inline void __DMB(void) {}
#endif
