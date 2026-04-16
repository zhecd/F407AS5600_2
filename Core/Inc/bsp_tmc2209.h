#ifndef __BSP_TMC2209_H__
#define __BSP_TMC2209_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>
#include <stdbool.h>

// TMC2209 寄存器地址
#define TMC2209_REG_GCONF       0x00
#define TMC2209_REG_IHOLD_IRUN  0x10
#define TMC2209_REG_CHOPCONF    0x6C

/**
 * @brief 初始化所有 TMC2209 驱动 (开启静音和休眠降流)
 * @param huart 用于通信的半双工串口句柄 (比如 &huart6)
 */
void BSP_TMC2209_InitAll(UART_HandleTypeDef *huart);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_TMC2209_H__ */