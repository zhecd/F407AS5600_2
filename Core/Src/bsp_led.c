/**
  ******************************************************************************
  * @file    bsp_led.c
  * @author  Lingma (灵码)
  * @brief   LED板级支持包(BSP)驱动实现文件
  * @details 本文件实现了bsp_led.h中声明的所有LED控制函数。
  *          采用状态管理机制，确保软件状态与硬件状态保持同步。
  *          所有公共接口都包含参数验证，保证系统的健壮性。
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "bsp_led.h"
#include "gpio.h"

/* Private variables ---------------------------------------------------------*/

/**
 * @brief LED软件状态数组
 * @details 存储每个LED的当前软件状态，用于状态查询和切换操作。
 *          初始化为全熄灭状态（LED_OFF），与硬件初始状态保持一致。
 *          数组索引0~3分别对应LED_0~LED_3。
 */
static LedState_t s_led_states[LED_COUNT] = {LED_OFF, LED_OFF, LED_OFF, LED_OFF};

/**
 * @brief LED引脚映射表
 * @details 将逻辑LED编号映射到具体的GPIO引脚。
 *          此设计实现了硬件无关性，如果将来修改LED的物理连接，
 *          只需要修改此映射表，无需改动上层应用代码。
 */
static const uint16_t s_led_pins[LED_COUNT] = {
    LED0_Pin,
    LED1_Pin,
    LED2_Pin,
    LED3_Pin
};

/**
 * @brief LED端口映射表
 * @details 将逻辑LED编号映射到具体的GPIO端口。
 *          目前所有LED都连接在GPIOC端口，但此设计保留了扩展性，
 *          支持未来LED分布在不同GPIO端口的情况。
 */
static GPIO_TypeDef* const s_led_ports[LED_COUNT] = {
    LED0_GPIO_Port,
    LED1_GPIO_Port,
    LED2_GPIO_Port,
    LED3_GPIO_Port
};

/* Private function prototypes -----------------------------------------------*/
/* None */

/* Exported functions --------------------------------------------------------*/

/**
 * @brief LED BSP驱动初始化函数
 * @details 执行LED驱动的完整初始化流程：
 *          1. 调用STM32CubeMX生成的GPIO初始化函数
 *          2. 将所有LED设置为初始熄灭状态（高电平）
 *          3. 更新内部软件状态数组
 * @note 此函数必须在使用任何LED控制功能之前调用
 */
void BSP_LED_Init(void)
{
    // 调用底层GPIO初始化，配置引脚模式、速度、上下拉等参数
    MX_GPIO_Init();
    
    // 初始化所有LED为熄灭状态（高电平）
    // 由于LED采用共阳极设计，高电平使LED熄灭，低电平使LED点亮
    for (int i = 0; i < LED_COUNT; i++) {
        s_led_states[i] = LED_OFF;  // 更新软件状态
        HAL_GPIO_WritePin(s_led_ports[i], s_led_pins[i], GPIO_PIN_SET);  // 设置硬件状态
    }
}

/**
 * @brief 设置指定LED的状态
 * @details 核心LED控制函数，包含完整的参数验证和状态同步逻辑
 * @param led_number LED编号 (LED_0 ~ LED_3)
 * @param state 目标状态 (LED_OFF 或 LED_ON)
 * @retval true 操作成功
 * @retval false 参数错误
 */
bool BSP_LED_SetState(LedNumber_t led_number, LedState_t state)
{
    // 参数边界检查：确保led_number在有效范围内 [0, LED_COUNT-1]
    if (led_number >= LED_COUNT) {
        return false;  // 返回错误码，避免越界访问
    }
    
    // 更新内部软件状态，保持与即将设置的硬件状态一致
    s_led_states[led_number] = state;
    
    // 执行实际的硬件操作
    // 注意：由于硬件电路设计，LED_ON需要输出低电平，LED_OFF需要输出高电平
    if (state == LED_ON) {
        // 点亮LED：输出低电平
        HAL_GPIO_WritePin(s_led_ports[led_number], s_led_pins[led_number], GPIO_PIN_RESET);
    } else {
        // 熄灭LED：输出高电平  
        HAL_GPIO_WritePin(s_led_ports[led_number], s_led_pins[led_number], GPIO_PIN_SET);
    }
    
    return true;  // 操作成功
}

/**
 * @brief 获取指定LED的当前状态
 * @details 安全的状态查询函数，包含参数验证
 * @param led_number LED编号 (LED_0 ~ LED_3)
 * @retval LED状态，参数无效时返回LED_OFF
 */
LedState_t BSP_LED_GetState(LedNumber_t led_number)
{
    // 参数边界检查
    if (led_number >= LED_COUNT) {
        return LED_OFF;  // 安全返回默认值，避免系统异常
    }
    
    // 返回存储的软件状态（与硬件状态保持同步）
    return s_led_states[led_number];
}

/**
 * @brief 切换指定LED的状态
 * @details 实现LED状态翻转功能
 * @param led_number LED编号 (LED_0 ~ LED_3)
 * @retval true 操作成功
 * @retval false 参数错误
 */
bool BSP_LED_Toggle(LedNumber_t led_number)
{
    // 参数边界检查
    if (led_number >= LED_COUNT) {
        return false;
    }
    
    // 计算新的状态：当前状态为ON则切换为OFF，当前为OFF则切换为ON
    LedState_t new_state = (s_led_states[led_number] == LED_ON) ? LED_OFF : LED_ON;
    
    // 调用状态设置函数完成实际操作
    return BSP_LED_SetState(led_number, new_state);
}

/**
 * @brief 批量设置所有LED的状态
 * @details 高效的批量操作函数，适用于需要同时控制多个LED的场景
 * @param states 状态数组指针，包含LED_COUNT个元素
 * @retval true 操作成功
 * @retval false 参数错误（空指针）
 */
bool BSP_LED_SetAllStates(const LedState_t states[LED_COUNT])
{
    // 空指针检查，防止解引用空指针导致系统崩溃
    if (states == NULL) {
        return false;
    }
    
    // 批量更新所有LED状态
    for (int i = 0; i < LED_COUNT; i++) {
        // 更新软件状态
        s_led_states[i] = states[i];
        
        // 执行硬件操作
        if (states[i] == LED_ON) {
            HAL_GPIO_WritePin(s_led_ports[i], s_led_pins[i], GPIO_PIN_RESET);
        } else {
            HAL_GPIO_WritePin(s_led_ports[i], s_led_pins[i], GPIO_PIN_SET);
        }
    }
    
    return true;
}

/**
 * @brief 获取所有LED的当前状态
 * @details 批量状态查询函数
 * @param states 输出数组指针，用于存储LED状态
 * @retval true 操作成功
 * @retval false 参数错误（空指针）
 */
bool BSP_LED_GetAllStates(LedState_t states[LED_COUNT])
{
    // 空指针检查
    if (states == NULL) {
        return false;
    }
    
    // 批量复制软件状态到输出数组
    for (int i = 0; i < LED_COUNT; i++) {
        states[i] = s_led_states[i];
    }
    
    return true;
}