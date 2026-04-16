/**
  * @file    bsp_tmc2209.h
  * @brief   TMC2209 步进电机智能驱动解耦封装层
  */

#ifndef __BSP_TMC2209_H__
#define __BSP_TMC2209_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

// ==============================================================
// 常用参数推荐宏定义 (人类友好型)
// ==============================================================

// 电流设置 (有效范围: 0 ~ 31) 
#define TMC_IRUN_HIGH    24  // 高扭矩 (适合承重的大臂)
#define TMC_IRUN_MED     16  // 中扭矩 (适合常规移动)
#define TMC_IRUN_LOW     8   // 低扭矩 (适合末端轻负载关节)

#define TMC_IHOLD_STRONG 12  // 强力保持 (悬停锁死)
#define TMC_IHOLD_COOL   4   // 低功耗保持 (电机温凉)

// ==============================================================
// 暴露给主函数的 API 层
// ==============================================================

/**
 * @brief  初始化 UART 总线上的所有驱动节点 (一键傻瓜式)
 * @param  huart 单线半双工配置的串口句柄 (例如 &huart6)
 */
void BSP_TMC2209_InitBus(UART_HandleTypeDef *huart);

/**
 * @brief  对单个 TMC2209 节点进行精细化配置 (高级玩家专用)
 * @param  huart 串口句柄
 * @param  node_addr  硬件跳线帽地址 (0, 1, 2, 3)
 * @param  microsteps 目标细分数 (支持 1, 2, 4, 8, 16, 32, 64, 128, 256)
 * @param  irun       运行电流系数 (0~31)
 * @param  ihold      静止保持电流系数 (0~31)
 */
void BSP_TMC2209_ConfigNode(UART_HandleTypeDef *huart, uint8_t node_addr, uint16_t microsteps, uint8_t irun, uint8_t ihold);


#ifdef __cplusplus
}
#endif

#endif /* __BSP_TMC2209_H__ */